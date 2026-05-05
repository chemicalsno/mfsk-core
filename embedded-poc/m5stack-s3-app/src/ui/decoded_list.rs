//! Decoded message list — 7 rows × 16 px = 112 px tall, drawn at
//! y ∈ [114, 226). Newest decode at the bottom (= row 6); older rows
//! scroll up as new decodes land.
//!
//! Each row is `"-NN ffff  message ..."` at FONT_6X10 (22 chars max,
//! fits 135 px width with 3 px slack). Borderline decodes
//! (`hard_errors ≥ 24`) get a trailing `!` plus amber tint so users
//! can spot CRC-luck candidates without reading the error column.
//! Cursor / "Call this station" wiring is deferred to Phase 6 (button
//! integration); for Phase 3 this is read-only.

use core::fmt::Write as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

use crate::ui::state::DecodedRow;

/// Region geometry on the 135 × 240 panel.
pub const ORIGIN_Y: i32 = 114;
pub const HEIGHT: u32 = 112;
pub const ROW_PX: u32 = 16;
/// Visible rows in the region (HEIGHT / ROW_PX).
pub const ROWS: usize = (HEIGHT / ROW_PX) as usize;
/// FONT_6X10 char width.
pub const CHAR_W: u32 = 6;
/// Max characters per row at 135 px width.
const ROW_CHARS: usize = (135 / CHAR_W) as usize;

/// Clear + repaint the decoded list. Idempotent — caller gates by
/// `UiState::dirty_seq`.
pub fn render<D>(display: &mut D, rows: &[DecodedRow]) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let bg = Rgb565::BLACK;
    let fg = Rgb565::WHITE;
    let warn = Rgb565::CSS_ORANGE;

    Rectangle::new(Point::new(0, ORIGIN_Y), Size::new(135, HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(bg))
        .draw(display)?;

    let style_ok = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(fg)
        .background_color(bg)
        .build();
    let style_warn = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(warn)
        .background_color(bg)
        .build();

    let n = rows.len();
    let take = n.min(ROWS);
    let start = n - take;

    for (i, row) in rows[start..].iter().enumerate() {
        let y = ORIGIN_Y + (i as i32) * ROW_PX as i32 + 3;
        let mut s: String<32> = String::new();
        let snr = row.snr_db.clamp(-30, 30);
        let _ = write!(&mut s, "{snr:>+3} {:>4} ", row.df_hz);
        let msg_room = ROW_CHARS.saturating_sub(s.len());
        let msg = row.msg.as_str();
        let msg_take = msg.len().min(msg_room.saturating_sub(1));
        let _ = s.push_str(&msg[..msg_take]);
        let style = if row.hard_errors >= 24 {
            let _ = s.push('!');
            style_warn
        } else {
            style_ok
        };
        Text::with_baseline(s.as_str(), Point::new(0, y), style, Baseline::Top)
            .draw(display)?;
    }

    Ok(())
}
