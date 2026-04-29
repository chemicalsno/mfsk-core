// SPDX-License-Identifier: GPL-3.0-or-later
//! RX path: 12 kHz f32 PCM audio → decoded `(app_type, payload)`
//! tuples.
//!
//! Pipeline (Phase 1f implementation):
//!
//! ```text
//! input audio (12 kHz f32)
//!   ↓ Costas search via core::sync::coarse_sync<P>
//!   ↓     candidates: (freq_hz, t_offset, score)
//!   ↓ for each candidate (a Costas-prefixed LDPC block start):
//!   ↓   group consecutive in-track candidates → multi-block frame
//!   ↓   per-block 4-tone soft demod → LLR vector (channel-bit order)
//!   ↓ block-deinterleave LLR vectors back into per-codeword shape
//!   ↓ de-puncture: insert 0-LLR at punctured positions per mode
//!   ↓ Ldpc240_101::decode_soft for each block
//!   ↓ extract 12 byte info bits per block, concatenate
//!   ↓ framing::unpack (CRC-16 verify)
//!   ↓ return Vec<(app_type, payload_bytes)>
//! ```
//!
//! The returned vector is empty if no frames decode. Multiple
//! independent frames at distinct audio centres in the same buffer
//! all decode in one call (multi-peak NMS in `coarse_sync` finds
//! them all).

/// Decode every uvpacket frame found in a buffer of 12 kHz f32 PCM.
///
/// **TODO (Phase 1f)**: implement.
pub fn decode(_audio: &[f32]) -> Vec<(u8, Vec<u8>)> {
    unimplemented!("Phase 1f: rx.rs::decode")
}
