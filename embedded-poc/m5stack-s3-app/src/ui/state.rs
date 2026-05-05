//! Shared UI state across `decode_pipeline` (writer) and `display`
//! (reader). Single `Mutex<UiState>` — coarse-grained but the lock
//! holds for ≪1 ms each side (push 1 row / paint 4 fields).
//!
//! Phase 3 scope: `decoded` ring + `status` only. Waterfall row
//! injection lands in a follow-up iteration once stage 1 surfaces a
//! per-slot averaged spectrum.

use core::sync::atomic::{AtomicU32, Ordering};

use heapless::String;
use std::sync::Mutex;

/// Single decoded FT8 message row, ready to render.
#[derive(Clone, Debug)]
pub struct DecodedRow {
    /// Audio offset (Hz). 200..3000 in practice.
    pub df_hz: u16,
    /// SNR (dB), clamped to i8 range. WSJT-X-style 2-digit signed.
    pub snr_db: i8,
    /// Hard-error count from BP — useful for marking borderline
    /// decodes (`!` if ≥ 24).
    pub hard_errors: u8,
    /// Decoded text. WSJT-X 77-bit packed messages fit in 22 chars.
    pub msg: String<22>,
    /// Monotonic slot number (decode_pipeline tick), for sort + dedup.
    pub slot_seq: u32,
}

/// Status-bar fields. All optional so the bar renders during boot
/// before peripherals are up.
#[derive(Clone, Debug, Default)]
pub struct StatusInfo {
    /// Rig audio band centre (Hz). e.g. 7_074_000 for FT8 40 m.
    pub rig_freq_hz: Option<u32>,
    /// "USB" / "USB-D" / "FM" — IC-705 mode string.
    pub rig_mode: Option<String<8>>,
    /// UTC second-of-day (0..86400). Updated by time_sync.
    pub utc_sod: Option<u32>,
    /// Current free heap (KB) — sanity gauge during dev.
    pub free_heap_kb: u32,
}

/// Bounded ring of decoded rows. Newest at the back. Capacity 16
/// covers >2 slots of qso3-busy density (= 7 decodes/slot) without
/// dropping; UI picks the trailing 7 to render.
#[derive(Default)]
pub struct UiState {
    decoded: heapless::Deque<DecodedRow, 16>,
    pub status: StatusInfo,
    /// Bumped by writers when state changes; readers compare against
    /// their last-rendered seq to skip the LCD push when nothing new.
    /// `AtomicU32` so the dirty check itself doesn't need the mutex.
    dirty_seq: AtomicU32,
}

impl UiState {
    pub const fn new() -> Self {
        Self {
            decoded: heapless::Deque::new(),
            status: StatusInfo {
                rig_freq_hz: None,
                rig_mode: None,
                utc_sod: None,
                free_heap_kb: 0,
            },
            dirty_seq: AtomicU32::new(0),
        }
    }

    /// Push a fresh decode. Drops the oldest row when full.
    pub fn push_decode(&mut self, row: DecodedRow) {
        if self.decoded.is_full() {
            let _ = self.decoded.pop_front();
        }
        let _ = self.decoded.push_back(row);
        self.bump();
    }

    /// Newest-last view of all retained rows.
    pub fn decoded_iter(&self) -> impl Iterator<Item = &DecodedRow> {
        self.decoded.iter()
    }

    pub fn decoded_len(&self) -> usize {
        self.decoded.len()
    }

    /// Render-side dirty check. Returns the current dirty seq;
    /// readers should compare with their last-seen value and skip the
    /// LCD push when equal.
    pub fn dirty_seq(&self) -> u32 {
        self.dirty_seq.load(Ordering::Acquire)
    }

    fn bump(&self) {
        self.dirty_seq.fetch_add(1, Ordering::AcqRel);
    }

    /// Update status bar from any thread.
    pub fn update_status(&mut self, f: impl FnOnce(&mut StatusInfo)) {
        f(&mut self.status);
        self.bump();
    }
}

/// Process-wide single instance. `Mutex` not `RwLock` since both
/// sides hold the lock briefly and esp-idf's `RwLock` has no async
/// advantages here.
pub static UI: Mutex<UiState> = Mutex::new(UiState::new());
