//! LCD bring-up + LcdPanel scroll renderer (Phase 0.5).
//!
//! M5StickS3 内蔵 LCD: ST7789P3 (135x240)。`mipidsi::models::ST7789` の
//! 標準 init で動く想定。SPI ピンは `board.rs` で確定済み。
//!
//! Phase 0.5 段階の方針: display と描画ループを 1 つの関数に閉じ込めて
//! 型パラメータを外部に露出しない。Phase 3 で本来 UI へ進化する際に
//! 構造体化する。

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

use display_interface_spi::SPIInterface;
use esp_idf_hal::{
    delay::Ets,
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    units::FromValueType,
};
use mipidsi::{models::ST7789, options::ColorInversion, Builder};

use crate::log_sink::LogFanout;
use crate::ui::{decoded_list, state::UI, status_bar, waterfall};

/// 1 行高 (FONT_6X10)。
pub const LINE_H: u16 = 10;

/// TX-line placeholder (Phase 4 will fill with QSO FSM state).
const TX_REGION_Y: i32 = 226;
const TX_REGION_H: u32 = 14;

/// LCD bring-up + 永続描画ループ。**戻らない**。
///
/// `Peripherals` を all-take し、SPI2 + LCD 関連 GPIO + 内蔵 ES8311
/// codec を遠ざけて初期化、`fanout.lcd` を 1 秒間隔で flush する。
///
/// 実装メモ:
/// - SPI clock 40 MHz。歪んだら 26 MHz 等に下げる。
/// - ST7789 panel は内部 RAM 240x320 から表示窓を切り出すので
///   `display_offset` を panel 実装に合わせる。M5StickS3 の 135x240 は
///   実機で (52, 40) または (40, 53) のどちらか。一回 flash して縞 or
///   ずれを観察して調整する。
pub fn run_log_panel(peripherals: Peripherals, fanout: &'static LogFanout) -> ! {
    // ── PMIC: M5PM1 経由で LCD 電源 ON。これを欠くと SPI/GPIO が完璧でも
    //   panel は永久に黒。M5GFX board_M5StickS3 と同シーケンス。
    let _i2c = match crate::pmic::init_lcd_power(
        peripherals.i2c1,
        peripherals.pins.gpio47,
        peripherals.pins.gpio48,
    ) {
        Ok(d) => Some(d),
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            None
        }
    };

    // Backlight ON (gpio38、PMIC 電源 ON 後に有効化)
    let mut bl = PinDriver::output(peripherals.pins.gpio38).expect("BL gpio38");
    bl.set_high().ok();
    core::mem::forget(bl);

    // ── SPI3 host (M5GFX が SPI3_HOST を使用)。SPI2 ではない。
    let driver = SpiDriver::new(
        peripherals.spi3,
        peripherals.pins.gpio40, // SCK
        peripherals.pins.gpio39, // MOSI
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new(),
    )
    .expect("SPI3 driver");
    let spi_cfg = SpiConfig::new().baudrate(40_u32.MHz().into());
    let spi_dev = SpiDeviceDriver::new(driver, Some(peripherals.pins.gpio41), &spi_cfg)
        .expect("SPI device (CS=41)");

    let dc = PinDriver::output(peripherals.pins.gpio45).expect("DC gpio45");
    let rst = PinDriver::output(peripherals.pins.gpio21).expect("RST gpio21");

    let di = SPIInterface::new(spi_dev, dc);

    let mut delay = Ets;
    let mut display = match Builder::new(ST7789, di)
        .reset_pin(rst)
        .display_size(crate::board::LCD_WIDTH, crate::board::LCD_HEIGHT)
        .display_offset(52, 40)
        .invert_colors(ColorInversion::Inverted) // M5GFX cfg.invert = true
        .init(&mut delay)
    {
        Ok(d) => d,
        Err(e) => {
            log::error!("display init failed: {:?}", e);
            // LCD 不在でもログだけ吐き続けるループへ。
            loop {
                log::info!("alive (no LCD)");
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        }
    };

    log::info!("LCD init OK (ST7789 135x240 offset 52,40 invert)");
    display.clear(Rgb565::BLACK).ok();

    let tx_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::new(0, 0, 8)) // dim blue strip
        .build();

    // Paint the TX placeholder strip once (refreshed per redraw cycle).
    let tx_bg = Rgb565::new(0, 0, 8);

    let mut tick: u32 = 0;
    let mut last_ui_seq: u32 = u32::MAX; // force first paint
    loop {
        let heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
        log::info!("alive tick={tick} free_heap={heap}");

        // ── status bar + heap update from UI state. We refresh the
        //    status struct here so the UTC / heap stay current even
        //    when no new decodes land.
        let ui_seq;
        let status_snapshot;
        let decoded_snapshot;
        let wf_snapshot: heapless::Vec<crate::ui::state::WfLine, { crate::ui::state::WF_DEPTH }>;
        {
            let mut ui = UI.lock().expect("UI mutex poisoned");
            ui.status.free_heap_kb = (heap / 1024) as u32;
            ui_seq = ui.dirty_seq();
            status_snapshot = ui.status.clone();
            // Take snapshots so we don't hold the lock while drawing.
            decoded_snapshot = ui
                .decoded_iter()
                .cloned()
                .collect::<heapless::Vec<_, 16>>();
            wf_snapshot = ui.waterfall_iter().cloned().collect();
        }

        // Always repaint status bar (cheap, 14 px tall).
        status_bar::render(&mut display, &status_snapshot).ok();

        // Waterfall + decoded list — repaint only when state changed.
        // Both are gated by the same `dirty_seq` since the decoder
        // writes a WF row + N decode rows under one lock per slot.
        if ui_seq != last_ui_seq {
            // `waterfall::render` wants `&[&WfLine]` — borrow each.
            let wf_refs: heapless::Vec<&crate::ui::state::WfLine, { crate::ui::state::WF_DEPTH }> =
                wf_snapshot.iter().collect();
            waterfall::render(&mut display, &wf_refs).ok();
            decoded_list::render(&mut display, &decoded_snapshot).ok();
            last_ui_seq = ui_seq;
        }

        // Boot-time log scroll has been retired now that the WF region
        // is live. C-side ESP_LOG output and Rust `log::info!` still
        // reach UART via the FanoutLogger; the on-LCD panel is only
        // useful when the cable is unplugged from the host.
        let _ = fanout;

        // ── TX placeholder strip — Phase 4 (QSO FSM) will replace
        //    this with live `TxIntent` text. For now it shows a
        //    ‐ marker so the bottom of the screen isn't black.
        Rectangle::new(
            Point::new(0, TX_REGION_Y),
            Size::new(crate::board::LCD_WIDTH as u32, TX_REGION_H),
        )
        .into_styled(PrimitiveStyle::with_fill(tx_bg))
        .draw(&mut display)
        .ok();
        Text::with_baseline(
            "TX: ---",
            Point::new(2, TX_REGION_Y + 2),
            tx_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .ok();

        std::thread::sleep(std::time::Duration::from_millis(100));
        tick = tick.wrapping_add(1);
    }
}
