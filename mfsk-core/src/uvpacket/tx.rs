// SPDX-License-Identifier: GPL-3.0-or-later
//! TX path: bytes → 12 kHz f32 PCM audio.
//!
//! Pipeline (Phase 1e implementation):
//!
//! ```text
//! bytes + (mode, app_type, sequence)
//!   ↓ framing::pack             4-byte header + payload bytes
//!   ↓ slice into 12-byte chunks one chunk per LDPC block
//!   ↓ pad each chunk to 101 bits each chunk → 12 byte data + 5 zero bits
//!   ↓ Ldpc240_101::encode       each chunk → 240-bit codeword
//!   ↓ puncture per mode         each codeword → ch_bits_per_block(mode)
//!   ↓ block-interleave          across all codewords in the frame
//!   ↓ for each LDPC block:
//!       prepend a Costas-4 sync pattern
//!       map bits → 4-FSK tone indices
//!       GFSK-shape and synthesise PCM
//!   → output Vec<f32> at 12 kHz
//! ```
//!
//! Reuses [`crate::core::dsp::gfsk::synth_f32`] and
//! [`crate::core::tx::codeword_to_itone`].

use super::framing::FrameHeader;

/// Encode a uvpacket frame to 12 kHz f32 audio at the given centre
/// frequency.
///
/// Returns the audio as a `Vec<f32>` ready to feed an audio output
/// device. The buffer length depends on `mode`, `header.block_count`,
/// and the pre/post Costas count.
///
/// **TODO (Phase 1e)**: implement.
pub fn encode(_header: &FrameHeader, _payload: &[u8], _audio_centre_hz: f32) -> Vec<f32> {
    unimplemented!("Phase 1e: tx.rs::encode")
}
