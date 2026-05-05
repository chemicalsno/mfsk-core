//! M5StickS3 IC-705 FT8 controller — entry point.

#![allow(dead_code)]

mod adif;
mod audio;
mod board;
mod buttons;
mod civ;
mod decode_pipeline;
mod display;
mod flash_log;
mod log_sink;
mod pmic;
mod qso;
mod snr_norm;
mod time_sync;
mod tx_picker;
mod uac;
mod ui;

use esp_idf_hal::peripherals::Peripherals;
use log::LevelFilter;

use crate::log_sink::{FanoutLogger, LogFanout};

static FANOUT: LogFanout = LogFanout::new();
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // EspLogger を init すると log::set_logger を奪われ、自前の
    // FanoutLogger が install 失敗 → LCD に何も流れなくなる。
    // C-side ESP_LOG (タイムスタンプ付き UART 出力) は init せずとも
    // 自動で動作するのでこのままでよい。
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-s3-app boot ===");
    log::info!("phase 3: WAV-fed UI (qso3_busy)");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    // 別スレッドで decode pipeline を走らせる。デコード結果は log::info!
    // → FanoutLogger 経由で LCD scroll panel に流れる。
    std::thread::Builder::new()
        .stack_size(32 * 1024)
        .spawn(|| decode_pipeline::run())
        .expect("spawn decode pipeline");

    // メインタスクは LCD render loop (返らない)。
    display::run_log_panel(peripherals, &FANOUT)
}
