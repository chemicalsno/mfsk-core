// SPDX-License-Identifier: GPL-3.0-or-later
//! Puncturing patterns for the four uvpacket modes.
//!
//! All four modes share the same `Ldpc240_101` mother code. Puncturing
//! the parity bits maps the native rate 0.421 (101 / 240) onto higher
//! rates without changing the encoder or the decoder algorithm; the
//! decoder handles punctured positions by treating them as erasure
//! LLRs (value 0) before BP / OSD.
//!
//! Phase 1c lands the actual puncture index tables. Phase 1a only
//! exposes the `Mode` enum and the per-mode codeword length so the
//! rest of the scaffolding can refer to them.

/// Per-mode rate / FEC posture.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    /// Unpunctured `Ldpc240_101`. Native rate ≈ 0.42.
    UltraRobust,
    /// Punctured to rate 1/2. AFSK 1200 throughput parity, fading-robust.
    Robust,
    /// Punctured to rate 2/3.
    Standard,
    /// Punctured to rate 3/4.
    Fast,
}

impl Mode {
    /// 2-bit encoding for the frame-header `mode` field.
    pub const fn header_code(self) -> u8 {
        match self {
            Mode::UltraRobust => 0,
            Mode::Robust => 1,
            Mode::Standard => 2,
            Mode::Fast => 3,
        }
    }

    /// Inverse of [`Self::header_code`]. Returns `None` for unknown
    /// codes (would only happen on header CRC mismatch since the
    /// field is 2 bits wide and all 4 codes are valid).
    pub const fn from_header_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Mode::UltraRobust),
            1 => Some(Mode::Robust),
            2 => Some(Mode::Standard),
            3 => Some(Mode::Fast),
            _ => None,
        }
    }

    /// Channel bits transmitted per LDPC block at this mode. The
    /// difference between this and the mother codeword length (240)
    /// is the count of punctured parity bits.
    pub const fn ch_bits_per_block(self) -> usize {
        match self {
            Mode::UltraRobust => 240,
            Mode::Robust => 202,
            Mode::Standard => 152,
            Mode::Fast => 134,
        }
    }
}
