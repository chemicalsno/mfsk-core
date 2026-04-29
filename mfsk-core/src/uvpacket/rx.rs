// SPDX-License-Identifier: GPL-3.0-or-later
//! RX path: 12 kHz f32 PCM audio → decoded `(app_type, payload)`
//! tuples.
//!
//! Two layers:
//!
//! - [`decode_known_layout`] takes `(mode, n_blocks, sample_offset)`
//!   and decodes the frame at that exact location. Used by unit
//!   tests for round-trip verification and as the inner kernel of
//!   the auto-detecting receiver.
//! - [`decode`] scans an arbitrary-length audio buffer for every
//!   uvpacket frame it can find: per-mode Costas search,
//!   Costas-spacing-based mode disambiguation, trailing-Costas
//!   length determination, then a `decode_known_layout` call per
//!   candidate frame.
//!
//! Pipeline (per-frame):
//!
//! ```text
//! audio @ 12 kHz f32, sample_offset → frame start
//!   ↓ for each data symbol in the N blocks:
//!       4-tone non-coherent power detect over NSPS = 10 samples
//!       → LLR(b1), LLR(b0)  (max-log, WSJT sign convention)
//!   ↓ block-deinterleave LLRs into per-codeword vectors
//!   ↓ de-puncture per mode (insert 0 LLR at punctured positions)
//!   ↓ Ldpc240_101::decode_soft for each codeword
//!   ↓ extract 12 byte/block info → concatenate
//!   ↓ framing::unpack → header + (payload + padding)
//!   ↓ verify mode / block_count match the layout we decoded for
//!   → (app_type, payload-bytes-incl-padding)
//! ```
//!
//! The returned payload buffer is `n_blocks × INFO_BYTES_PER_BLOCK
//! − HEADER_BYTES` long; trailing zero bytes correspond to the TX
//! padding. Application code that needs an exact length carries it
//! at the application protocol layer (e.g. JSON brace matching for
//! signed-QSL).
//!
//! ## Header recovery (D-iii fast / slow path)
//!
//! Phase 1f implements the **fast path** only: block 0 must decode
//! and its payload bits 0..32 must constitute a CRC-valid header.
//! The **slow path** that reconstructs the header from the per-block
//! 5-bit spread copies (when block 0 fails) is left for a follow-up
//! commit; the spread bits are emitted by TX correctly so the slow
//! path can be added later without TX-side changes.

use crate::core::{FecCodec, FecOpts};
use crate::fec::Ldpc240_101;

use super::framing::{INFO_BYTES_PER_BLOCK, UnpackError, unpack as unpack_frame};
use super::interleaver::deinterleave_llr;
use super::puncture::{Mode, de_puncture_llr};

/// LDPC info-bit count.
const K_LDPC: usize = 101;

/// 4-FSK Gray map (bit pair → tone index). Same permutation as TX
/// (`[0, 1, 3, 2]`).
const GRAY_4: [u8; 4] = [0, 1, 3, 2];

/// Costas head occupies 4 symbols at the start of each block.
const COSTAS_LEN: usize = 4;

/// Sample rate the modem operates at.
const SAMPLE_RATE_HZ: f32 = 12_000.0;
/// 1200 baud → 10 samples per symbol at 12 kHz.
const NSPS: usize = 10;
/// Tone spacing (Hz). h = 0.5 × R_s = 600 Hz.
const TONE_SPACING_HZ: f32 = 600.0;

/// Errors returned by [`decode_known_layout`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Audio buffer ended before the layout's `n_blocks` worth of
    /// samples could be extracted.
    Truncated,
    /// At least one LDPC block failed to decode (BP did not
    /// converge within the iteration budget, even with OSD-2).
    FecFailed,
    /// The frame data unpacked but its CRC-16 did not match —
    /// either the channel mangled it past the FEC's correction
    /// capacity or the layout assumed (mode, n_blocks) was wrong.
    Crc(UnpackError),
    /// The decoded frame's header's mode / block_count differ from
    /// the layout the caller requested. Indicates either a layout
    /// mismatch or a bit-flip past the FEC's correction.
    LayoutMismatch {
        wanted_mode: Mode,
        got_mode: Mode,
        wanted_blocks: u8,
        got_blocks: u8,
    },
}

/// Result of a successful frame decode.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecodedFrame {
    /// Application dispatch tag from the frame header.
    pub app_type: u8,
    /// Sequence number from the frame header.
    pub sequence: u8,
    /// Mode the frame was decoded with.
    pub mode: Mode,
    /// Number of LDPC blocks in the frame.
    pub block_count: u8,
    /// Payload bytes — exactly `block_count × 12 − 4` bytes, including
    /// any zero padding the TX side added to fill the last LDPC block.
    /// Application code is responsible for trimming based on its own
    /// length / framing.
    pub payload: Vec<u8>,
}

/// Decode a uvpacket frame at a known location with known layout.
///
/// `audio` is 12 kHz f32 PCM. `sample_offset` is the index of the
/// first sample of the head Costas. `audio_centre_hz` is the modem
/// audio centre frequency (typically [`crate::uvpacket::AUDIO_CENTRE_HZ`],
/// 1700 Hz). `mode` and `n_blocks` describe the frame's layout.
///
/// Returns `Err(DecodeError::Truncated)` if the audio is shorter
/// than the layout requires; otherwise either a successful decode
/// or a Crc / LayoutMismatch / FecFailed error.
pub fn decode_known_layout(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    n_blocks: u8,
) -> Result<DecodedFrame, DecodeError> {
    let n_blocks = n_blocks as usize;
    let block_ch_bits = mode.ch_bits_per_block();
    let block_data_syms = block_ch_bits / 2;
    let block_total_syms = COSTAS_LEN + block_data_syms;
    let total_syms = n_blocks * block_total_syms + COSTAS_LEN;
    let total_samples = total_syms * NSPS;

    if sample_offset + total_samples > audio.len() {
        return Err(DecodeError::Truncated);
    }

    // 1. Demodulate every data symbol (skip per-block head Costas
    // and trailing Costas) into channel-order LLR pairs.
    let tone_freqs = tone_frequencies(audio_centre_hz);
    let mut llrs_channel = Vec::with_capacity(n_blocks * block_ch_bits);

    for block_idx in 0..n_blocks {
        let block_start_sym = block_idx * block_total_syms;
        let data_start_sym = block_start_sym + COSTAS_LEN;
        for sym_offset in 0..block_data_syms {
            let sym_idx = data_start_sym + sym_offset;
            let sample_start = sample_offset + sym_idx * NSPS;
            let samples = &audio[sample_start..sample_start + NSPS];
            let powers = symbol_powers(samples, &tone_freqs);
            let (llr_b1, llr_b0) = symbol_powers_to_llrs(&powers);
            llrs_channel.push(llr_b1);
            llrs_channel.push(llr_b0);
        }
    }

    // 2. Block-de-interleave channel LLRs back to per-codeword
    // vectors (one per LDPC block, each block_ch_bits long).
    let llrs_per_block = deinterleave_llr(&llrs_channel, n_blocks);

    // 3. De-puncture and decode each block.
    let fec = Ldpc240_101;
    let opts = FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
    };
    let mut decoded_info: Vec<Vec<u8>> = Vec::with_capacity(n_blocks);
    for block_llrs in &llrs_per_block {
        let full_llrs = de_puncture_llr(block_llrs, mode);
        let result = fec
            .decode_soft(&full_llrs, &opts)
            .ok_or(DecodeError::FecFailed)?;
        decoded_info.push(result.info);
    }

    // 4. Pack info bits back to bytes (12 bytes per block; the
    // trailing 5-bit spread-header chunk is dropped here — it's the
    // slow-path fallback for header recovery, not used in the fast
    // path).
    let mut frame_data = Vec::with_capacity(n_blocks * INFO_BYTES_PER_BLOCK);
    for block_info in &decoded_info {
        debug_assert_eq!(block_info.len(), K_LDPC);
        for byte_idx in 0..INFO_BYTES_PER_BLOCK {
            let mut byte = 0u8;
            for bit_idx in 0..8 {
                if block_info[byte_idx * 8 + bit_idx] != 0 {
                    byte |= 1 << (7 - bit_idx);
                }
            }
            frame_data.push(byte);
        }
    }

    // 5. Unpack header + verify CRC-16 over the full frame_data
    // (header + payload + padding), then sanity-check the layout.
    let (header, payload) = unpack_frame(&frame_data).map_err(DecodeError::Crc)?;
    if header.mode != mode || header.block_count as usize != n_blocks {
        return Err(DecodeError::LayoutMismatch {
            wanted_mode: mode,
            got_mode: header.mode,
            wanted_blocks: n_blocks as u8,
            got_blocks: header.block_count,
        });
    }

    Ok(DecodedFrame {
        app_type: header.app_type,
        sequence: header.sequence,
        mode: header.mode,
        block_count: header.block_count,
        payload: payload.to_vec(),
    })
}

/// Tone-0 / tone-3 frequency layout for a given audio centre.
fn tone_frequencies(audio_centre_hz: f32) -> [f32; 4] {
    let f0 = audio_centre_hz - 1.5 * TONE_SPACING_HZ;
    [
        f0,
        f0 + TONE_SPACING_HZ,
        f0 + 2.0 * TONE_SPACING_HZ,
        f0 + 3.0 * TONE_SPACING_HZ,
    ]
}

/// Compute per-tone power for one symbol (`NSPS` samples) by direct
/// length-`NSPS` DFT at each of the 4 tone frequencies. Returns the
/// magnitude-squared per tone — the natural input to the max-log
/// non-coherent FSK LLR formula.
fn symbol_powers(samples: &[f32], tone_freqs: &[f32; 4]) -> [f32; 4] {
    debug_assert_eq!(samples.len(), NSPS);
    let mut out = [0.0f32; 4];
    for (t, &freq) in tone_freqs.iter().enumerate() {
        let mut re = 0.0f32;
        let mut im = 0.0f32;
        for (n, &s) in samples.iter().enumerate() {
            let phase = 2.0 * std::f32::consts::PI * freq * n as f32 / SAMPLE_RATE_HZ;
            re += s * phase.cos();
            im -= s * phase.sin();
        }
        out[t] = re * re + im * im;
    }
    out
}

/// Convert per-tone powers to (LLR(b1), LLR(b0)) using the max-log
/// non-coherent 4-FSK formula and the Gray map `GRAY_4 = [0,1,3,2]`.
///
/// Bit assignment (after Gray-decoding): tone 0→00, tone 1→01,
/// tone 2→11, tone 3→10. So:
/// - `b1 = 1` for tones {2, 3}
/// - `b0 = 1` for tones {1, 2}
///
/// Sign convention: `LLR > 0` → bit 1 is the more likely value
/// (matches `bp_decode_generic`'s convention).
fn symbol_powers_to_llrs(p: &[f32; 4]) -> (f32, f32) {
    let _ = GRAY_4; // documentation pin; the constants below assume this map
    let llr_b1 = p[2].max(p[3]) - p[0].max(p[1]);
    let llr_b0 = p[1].max(p[2]) - p[0].max(p[3]);
    (llr_b1, llr_b0)
}

/// Auto-detecting receiver — placeholder for now.
///
/// **TODO**: full Costas search + per-mode disambiguation. The
/// known-layout decoder above is sufficient for unit tests of the
/// modem core (TX→channel→RX round-trip with known layout) and for
/// Phase 2 characterisation harnesses; the full unconstrained
/// receiver lands in a follow-up commit and uses
/// [`crate::core::sync::coarse_sync`] to drive [`decode_known_layout`].
pub fn decode(_audio: &[f32], _audio_centre_hz: f32) -> Vec<DecodedFrame> {
    Vec::new()
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvpacket::AUDIO_CENTRE_HZ;
    use crate::uvpacket::framing::{FrameHeader, HEADER_BYTES};
    use crate::uvpacket::tx::encode;

    fn header_for(mode: Mode, n_blocks: u8, app_type: u8, seq: u8) -> FrameHeader {
        FrameHeader {
            mode,
            block_count: n_blocks,
            app_type,
            sequence: seq,
        }
    }

    /// Round-trip every mode at a representative frame size.
    #[test]
    fn roundtrip_clean_channel_all_modes() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            let n_blocks = 4u8;
            let header = header_for(mode, n_blocks, 1, 7);
            let payload: Vec<u8> = (0..40).map(|i| (i ^ 0x5A) as u8).collect();
            let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
            let decoded = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, n_blocks)
                .unwrap_or_else(|e| panic!("{mode:?}: {e:?}"));
            assert_eq!(decoded.app_type, 1);
            assert_eq!(decoded.sequence, 7);
            assert_eq!(decoded.mode, mode);
            assert_eq!(decoded.block_count, n_blocks);
            assert_eq!(&decoded.payload[..payload.len()], &payload[..]);
            // Trailing bytes are zero padding from the TX side.
            for &b in &decoded.payload[payload.len()..] {
                assert_eq!(b, 0, "{mode:?} non-zero padding byte");
            }
        }
    }

    /// Round-trip a 19-block Standard frame at QSL size (214 byte
    /// payload). This is the design's flagship use case.
    #[test]
    fn roundtrip_qsl_size_standard() {
        let header = header_for(Mode::Standard, 19, 1, 0);
        let payload: Vec<u8> = (0..214).map(|i| (i ^ 0xAA) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let decoded = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, Mode::Standard, 19).unwrap();
        assert_eq!(&decoded.payload[..214], &payload[..]);
    }

    /// Round-trip a 32-block Robust frame (the maximum frame size
    /// at the most fade-tolerant mode).
    #[test]
    fn roundtrip_max_blocks_robust() {
        let header = header_for(Mode::Robust, 32, 5, 31);
        let payload: Vec<u8> = (0..(32 * INFO_BYTES_PER_BLOCK - HEADER_BYTES))
            .map(|i| ((i * 31) & 0xFF) as u8)
            .collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let decoded = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, Mode::Robust, 32).unwrap();
        assert_eq!(&decoded.payload[..payload.len()], &payload[..]);
    }

    /// A single-block frame should round-trip in every mode.
    #[test]
    fn roundtrip_single_block_all_modes() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            let header = header_for(mode, 1, 0, 0);
            let payload: Vec<u8> = vec![0xC3; INFO_BYTES_PER_BLOCK - HEADER_BYTES];
            let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
            let decoded = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, 1)
                .unwrap_or_else(|e| panic!("{mode:?}: {e:?}"));
            assert_eq!(&decoded.payload[..payload.len()], &payload[..]);
        }
    }

    /// Audio shorter than the layout demands must produce
    /// `Truncated`, not panic.
    #[test]
    fn truncated_audio_is_reported() {
        let header = header_for(Mode::Robust, 4, 1, 0);
        let audio = encode(&header, b"hi", AUDIO_CENTRE_HZ).unwrap();
        let short = &audio[..audio.len() / 2];
        let err = decode_known_layout(short, 0, AUDIO_CENTRE_HZ, Mode::Robust, 4).unwrap_err();
        assert_eq!(err, DecodeError::Truncated);
    }

    /// Trying to decode a frame as the wrong mode must NOT silently
    /// succeed: either FEC fails to converge, the CRC mismatches,
    /// or the layout-mismatch check fires.
    #[test]
    fn wrong_mode_rejects() {
        let header = header_for(Mode::Robust, 4, 1, 0);
        let audio = encode(&header, b"abc", AUDIO_CENTRE_HZ).unwrap();
        // The audio is 4 Robust blocks; trying to decode as Standard
        // gives a mismatched layout — but the Standard layout fits
        // within the buffer (smaller block_ch_bits), so we'll get
        // garbage LLRs and either FEC fail or CRC fail.
        let err = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, Mode::Standard, 4).unwrap_err();
        assert!(
            matches!(err, DecodeError::FecFailed | DecodeError::Crc(_)),
            "expected FecFailed or Crc, got {err:?}",
        );
    }
}
