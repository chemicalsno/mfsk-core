//! Log fanout: console + LCD scroll panel + flash file.
//!
//! 動機: USB-OTG host モード中 (= IC-705 接続中) はシリアルコンソールが
//! 物理的に使えない。電源を切って USB ケーブルを母艦 PC に挿し直さない
//! と `espflash --monitor` は読めない。そこで:
//!
//!   log::info!(...) ──► [Fanout]
//!                         ├─ EspLogger (USB-CDC、生きていれば)
//!                         ├─ LcdPanel  (mipidsi に常時 12 行の scroll)
//!                         └─ FlashLog  (/littlefs/run.log に append)
//!
//! どれか落ちても残りは継続。LCD/Flash が初期化前に呼ばれた `log::info!`
//! は内部 staging buffer に貯めておき、初期化後に flush する。

use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::{Deque, String};

/// 1 ログ行の最大長 (LCD 6x10 フォントで 22 文字幅 + 余裕)。
pub const LINE_MAX: usize = 80;
/// LCD scroll panel に保持する行数。120 px / 10 px = 12 行。
pub const LCD_LINES: usize = 12;
/// 起動直後のステージング (LCD/Flash 未初期化時) 行数。
pub const STAGING_LINES: usize = 32;

pub type LogLine = String<LINE_MAX>;

/// LCD scroll panel — `Deque` で末尾追加、容量超過で先頭削除。
pub struct LcdPanel {
    lines: Deque<LogLine, LCD_LINES>,
    dirty: bool,
}

impl LcdPanel {
    pub const fn new() -> Self {
        Self {
            lines: Deque::new(),
            dirty: false,
        }
    }

    pub fn push(&mut self, line: &str) {
        let mut s: LogLine = String::new();
        let truncated = if line.len() > LINE_MAX {
            &line[..LINE_MAX]
        } else {
            line
        };
        let _ = s.push_str(truncated);
        if self.lines.is_full() {
            let _ = self.lines.pop_front();
        }
        let _ = self.lines.push_back(s);
        self.dirty = true;
    }

    pub fn drain_dirty(&mut self) -> Option<&Deque<LogLine, LCD_LINES>> {
        if !self.dirty {
            return None;
        }
        self.dirty = false;
        Some(&self.lines)
    }

    /// 最古 → 最新の順でイテレート (描画時に上から下へ流せるように).
    pub fn iter_chronological(&self) -> impl Iterator<Item = &LogLine> {
        self.lines.iter()
    }

    pub fn len(&self) -> usize {
        self.lines.len()
    }

    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }
}

/// Append-only flash log writer (LittleFS).
///
/// Phase 0.5 の LittleFS bring-up で `/littlefs/run.log` を open する。
/// `write_line` は ASCII 1 行 + `\n` を fsync 付き append。1 MB を超えた
/// ら `run.log` → `run.log.1` にローテート。
pub trait FlashLog: Send {
    fn write_line(&mut self, line: &str);
}

/// Fanout sink — `log::Log` 実装。
pub struct LogFanout {
    pub lcd: Mutex<CriticalSectionRawMutex, LcdPanel>,
    /// LCD/Flash 未初期化期間のステージング。
    pub staging: Mutex<CriticalSectionRawMutex, Deque<LogLine, STAGING_LINES>>,
    /// Flash sink (Phase 5 に近い形で確立)。`Option` で boot 時に late-bind。
    pub flash: Mutex<CriticalSectionRawMutex, Option<&'static mut dyn FlashLog>>,
}

impl LogFanout {
    pub const fn new() -> Self {
        Self {
            lcd: Mutex::new(LcdPanel::new()),
            staging: Mutex::new(Deque::new()),
            flash: Mutex::new(None),
        }
    }

    /// 1 行投函 (caller は format 済み &str を渡す)。
    pub fn push(&self, line: &str) {
        // LCD: lock 取れれば push、取れなければ staging に積む
        if let Ok(mut lcd) = self.lcd.try_lock() {
            lcd.push(line);
        } else if let Ok(mut staging) = self.staging.try_lock() {
            let mut s: LogLine = String::new();
            let truncated = if line.len() > LINE_MAX {
                &line[..LINE_MAX]
            } else {
                line
            };
            let _ = s.push_str(truncated);
            if staging.is_full() {
                let _ = staging.pop_front();
            }
            let _ = staging.push_back(s);
        }
        // Flash: 同様。Mutex を握れない最悪ケースは drop (boot シーケンス
        // 中の竞合のみ想定で、運用時は lock 競合は起きない)。
        if let Ok(mut flash) = self.flash.try_lock() {
            if let Some(sink) = flash.as_deref_mut() {
                sink.write_line(line);
            }
        }
    }
}

/// `log::Log` 実装。`init()` で `log::set_logger` する。
pub struct FanoutLogger {
    inner: &'static LogFanout,
    level: log::LevelFilter,
}

impl FanoutLogger {
    pub const fn new(inner: &'static LogFanout, level: log::LevelFilter) -> Self {
        Self { inner, level }
    }

    pub fn install(&'static self) {
        let _ = log::set_logger(self);
        log::set_max_level(self.level);
    }
}

impl log::Log for FanoutLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= self.level
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        // LCD/flash 用の短い行 (target prefix を捨てる)。
        let mut line: LogLine = String::new();
        let _ = write!(
            &mut line,
            "{} {}",
            level_short(record.level()),
            record.args()
        );
        self.inner.push(&line);

        // UART にも吐き出す (EspLogger を init していないので自前で)。
        // C-side ESP_LOG のタイムスタンプ付きフォーマットには合わせず、
        // Rust 側ログは簡素に。
        println!(
            "{} {}: {}",
            level_short(record.level()),
            record.target(),
            record.args()
        );
    }

    fn flush(&self) {}
}

const fn level_short(l: log::Level) -> &'static str {
    match l {
        log::Level::Error => "E",
        log::Level::Warn => "W",
        log::Level::Info => "I",
        log::Level::Debug => "D",
        log::Level::Trace => "T",
    }
}
