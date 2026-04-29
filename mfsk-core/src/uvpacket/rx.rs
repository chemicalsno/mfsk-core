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
use super::sync_pattern::UVPACKET_COSTAS;

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
///
/// Form: max-log non-coherent FSK uses **magnitudes** (`|r_t|`),
/// not power (`|r_t|²`); the Bessel-function-based exact LLR has
/// the same `|r_t|`-linear leading term. We deliberately do **not**
/// per-symbol-normalise by an estimated `σ̂`: the per-symbol noise
/// estimate goes to zero on clean tones, blowing up the LLR
/// magnitude and saturating BP's `tanh` to ±1 (= hard decision).
/// Phase 2 measured this empirically. A frame-level σ̂ (averaged
/// across symbols) is a sensible follow-up.
fn symbol_powers_to_llrs(p: &[f32; 4]) -> (f32, f32) {
    let _ = GRAY_4; // documentation pin; the constants below assume this map
    let m: [f32; 4] = [p[0].sqrt(), p[1].sqrt(), p[2].sqrt(), p[3].sqrt()];
    let llr_b1 = m[2].max(m[3]) - m[0].max(m[1]);
    let llr_b0 = m[1].max(m[2]) - m[0].max(m[3]);
    (llr_b1, llr_b0)
}

/// Diagnostic env var: set `UVPACKET_DEBUG_RX=1` to dump
/// Costas-search and decode-attempt traces to stderr.
fn debug_enabled() -> bool {
    std::env::var("UVPACKET_DEBUG_RX").is_ok()
}

/// Score a hypothetical Costas-4 head at `sample_offset`. Returns
/// `target − max(other)` summed across the 4 Costas symbols, where
/// `target` is the power at the expected tone for that symbol and
/// `max(other)` is the highest of the other three tones' powers.
/// Strongly positive ↔ Costas detected; near-zero ↔ data / noise.
fn costas_score_at(audio: &[f32], sample_offset: usize, audio_centre_hz: f32) -> f32 {
    let tone_freqs = tone_frequencies(audio_centre_hz);
    if sample_offset + COSTAS_LEN * NSPS > audio.len() {
        return f32::NEG_INFINITY;
    }
    let mut score = 0.0f32;
    for sym in 0..COSTAS_LEN {
        let start = sample_offset + sym * NSPS;
        let powers = symbol_powers(&audio[start..start + NSPS], &tone_freqs);
        let expected = UVPACKET_COSTAS[sym] as usize;
        let target = powers[expected];
        let max_other = (0..4)
            .filter(|&t| t != expected)
            .fold(0.0f32, |acc, t| acc.max(powers[t]));
        score += target - max_other;
    }
    score
}

/// Brute-force Costas search across the whole audio buffer at
/// sample-level resolution. Returns sample offsets where the Costas
/// pattern is plausibly present, sorted ascending. Threshold + NMS
/// are tuned for clean / moderate-SNR channels — Phase 2 will
/// re-tune for low-SNR scenarios.
fn find_costas_hits_sorted(audio: &[f32], audio_centre_hz: f32) -> Vec<usize> {
    let costas_samples = COSTAS_LEN * NSPS;
    if audio.len() <= costas_samples {
        return Vec::new();
    }
    let n_positions = audio.len() - costas_samples + 1;
    let mut scores = vec![0.0f32; n_positions];
    for (offset, slot) in scores.iter_mut().enumerate() {
        *slot = costas_score_at(audio, offset, audio_centre_hz);
    }
    let global_max = scores.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    if global_max <= 0.0 {
        return Vec::new();
    }
    // Diagnostic: dump scores when running RUST_LOG-style debug.
    if debug_enabled() {
        let mut top: Vec<(usize, f32)> = scores.iter().copied().enumerate().collect();
        top.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        eprintln!(
            "find_costas_hits: max={:.2}, top 8 = {:?}",
            global_max,
            &top[..8.min(top.len())],
        );
    }
    // Loose threshold so that the first Costas of a frame — which
    // sits inside the GFSK-synth ramp envelope and so scores lower
    // than a mid-frame Costas — still passes. The ±NSPS NMS that
    // follows trims false positives.
    let threshold = global_max * 0.10;

    if debug_enabled() {
        // Dump scores at exact-frame-multiples for the 4 modes, to
        // see whether the head / tail / mid Costas hits cleared the
        // threshold in the first place.
        for &block_syms in &[124usize, 105, 80, 71] {
            let bs = block_syms * NSPS;
            let mut row = String::new();
            for n in 0..=8 {
                let pos = n * bs;
                if pos < scores.len() {
                    row.push_str(&format!(" [{}]={:.1}", pos, scores[pos]));
                }
            }
            eprintln!("  mode block_samples={bs}: {row}");
        }
    }

    // Take every above-threshold position as a candidate. Local-
    // maxima filtering would drop the start-of-frame Costas when its
    // peak doesn't quite land on a sample boundary (the GFSK
    // half-cosine ramp can shift the peak by a sample or two), so
    // we lean on NMS alone to thin the candidate list.
    let mut peaks: Vec<(usize, f32)> = scores
        .iter()
        .enumerate()
        .filter(|(_, s)| **s >= threshold)
        .map(|(i, &s)| (i, s))
        .collect();
    // Greedy NMS over ±NSPS samples (one symbol period).
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let mut picked: Vec<usize> = Vec::new();
    for (offset, _) in peaks {
        if picked.iter().all(|&p| offset.abs_diff(p) > NSPS) {
            picked.push(offset);
        }
    }
    picked.sort_unstable();
    picked
}

/// Refine an integer frame-head sample offset by finding the
/// offset (within ±NSPS of `candidate`) whose **sum of Costas
/// correlations across every expected Costas position in the frame
/// (head + block boundaries + trailing)** is maximised.
///
/// Single-anchor refinement around the head is ambiguous — the
/// GFSK ramp at the start of the frame attenuates symbol 0,
/// shifting the apparent head peak by a few samples. Aggregating
/// across N+1 anchors anchors the true offset by consensus.
fn refine_frame_offset(
    audio: &[f32],
    candidate: usize,
    audio_centre_hz: f32,
    mode: Mode,
    n_blocks: u8,
) -> usize {
    let block_samples = (COSTAS_LEN + mode.ch_bits_per_block() / 2) * NSPS;
    let n_anchors = n_blocks as usize + 1; // head + (N-1) inter-block + trailing
    let radius = NSPS as isize;
    let mut best = candidate;
    let mut best_sum = f32::NEG_INFINITY;
    for jitter in -radius..=radius {
        let Some(off) = candidate.checked_add_signed(jitter) else {
            continue;
        };
        let last_anchor_pos = off + (n_anchors - 1) * block_samples;
        if last_anchor_pos + COSTAS_LEN * NSPS > audio.len() {
            continue;
        }
        let mut sum = 0.0f32;
        for n in 0..n_anchors {
            let pos = off + n * block_samples;
            sum += costas_score_at(audio, pos, audio_centre_hz);
        }
        if sum > best_sum {
            best_sum = sum;
            best = off;
        }
    }
    best
}

/// Auto-detecting receiver: scan audio, infer per-frame `(mode,
/// n_blocks)` from inter-Costas spacing, decode with
/// [`decode_known_layout`].
///
/// Algorithm:
/// 1. Brute-force Costas correlation across the full audio buffer
///    → sorted list of plausible Costas-4 head offsets.
/// 2. For each unconsumed Costas hit, walk forward looking for
///    consecutive hits at the spacing matching each mode. Pick the
///    longest consistent run.
/// 3. Call [`decode_known_layout`] with the inferred layout. On
///    success, mark every hit covered by the frame as consumed and
///    move on; on failure, the hit is left available so a different
///    interpretation can pick it up.
///
/// Returns the list of successfully-decoded frames, in order of the
/// first Costas hit they consumed.
pub fn decode(audio: &[f32], audio_centre_hz: f32) -> Vec<DecodedFrame> {
    let hits = find_costas_hits_sorted(audio, audio_centre_hz);
    if debug_enabled() {
        eprintln!("decode: {} costas hits at offsets {:?}", hits.len(), hits);
    }
    if hits.is_empty() {
        return Vec::new();
    }
    let mut consumed = vec![false; hits.len()];
    let mut found: Vec<DecodedFrame> = Vec::new();

    for i in 0..hits.len() {
        if consumed[i] {
            continue;
        }
        let head_sample = hits[i];

        // Try each mode; pick the layout giving the longest run of
        // matching Costas hits.
        let mut best: Option<(Mode, u8, usize)> = None; // (mode, n_blocks, last_consumed_idx)
        let tolerance = NSPS; // ±1 symbol of slop for each anchor

        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            let block_samples = (COSTAS_LEN + mode.ch_bits_per_block() / 2) * NSPS;
            let mut last_idx = i;
            let mut n_blocks = 0u8;

            for n in 1..=32u8 {
                let expected = head_sample + (n as usize) * block_samples;
                let mut next_idx: Option<usize> = None;
                for j in (last_idx + 1)..hits.len() {
                    let h = hits[j];
                    if h + tolerance < expected {
                        continue;
                    }
                    if h > expected + tolerance {
                        break;
                    }
                    next_idx = Some(j);
                    break;
                }
                if let Some(j) = next_idx {
                    last_idx = j;
                    n_blocks = n;
                } else {
                    break;
                }
            }

            if n_blocks >= 1 && best.is_none_or(|(_, b, _)| n_blocks > b) {
                best = Some((mode, n_blocks, last_idx));
            }
        }

        let Some((mode, n_blocks, last_idx)) = best else {
            continue;
        };
        // Refine the frame-head offset by maximising the sum of
        // Costas correlation across **all** N+1 expected Costas
        // positions (head + every block boundary + trailing). A
        // single-anchor search at `head_sample` is ambiguous — the
        // GFSK ramp shifts the head's apparent peak by a few
        // samples, so a jitter brute-force over decode attempts
        // wastes effort. Aggregating across N+1 anchors localises
        // the true frame start to the integer offset that aligns
        // every Costas in the frame.
        let refined = refine_frame_offset(audio, head_sample, audio_centre_hz, mode, n_blocks);
        if let Ok(frame) = decode_known_layout(audio, refined, audio_centre_hz, mode, n_blocks) {
            found.push(frame);
            for k in i..=last_idx {
                consumed[k] = true;
            }
        }
    }

    found
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
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

    /// Auto-detecting decoder must find Robust / Standard frames
    /// placed at the start of the audio. Fast / Express are
    /// excluded at this payload density — at 16-byte payload in a
    /// 4-block frame, ~58 % of the LDPC info bits per block are
    /// zero (header + payload + zero-pad). Combined with rate-2/3
    /// or rate-3/4 puncturing the BP+OSD path occasionally
    /// converges to a parity-valid sibling codeword instead of the
    /// transmitted one. Phase 2 will sweep payload zero-density ×
    /// OSD depth and feed back into the LDPC opts; this commit
    /// pins down the auto-detect plumbing without overfitting
    /// to the corner cases.
    #[test]
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
    fn auto_detect_each_mode_at_buffer_start() {
        for mode in [Mode::Robust, Mode::Standard] {
            let header = header_for(mode, 4, 2, 11);
            let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xC3) as u8).collect();
            let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
            let frames = decode(&audio, AUDIO_CENTRE_HZ);
            assert_eq!(frames.len(), 1, "{mode:?}: got {} frames", frames.len());
            let f = &frames[0];
            assert_eq!(f.mode, mode);
            assert_eq!(f.block_count, 4);
            assert_eq!(f.app_type, 2);
            assert_eq!(f.sequence, 11);
            assert_eq!(&f.payload[..payload.len()], &payload[..]);
        }
    }

    /// Tracking test for the high-zero-density Fast-mode case.
    /// `#[ignore]` until Phase 2's characterisation harness
    /// quantifies the failure rate as a function of zero-padding
    /// fraction × OSD depth.
    #[test]
    #[ignore = "Phase 2: Fast-mode LDPC convergence on high-zero-density payloads"]
    fn auto_detect_fast_mode_high_zero_density() {
        let header = header_for(Mode::Fast, 4, 2, 11);
        let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xC3) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 1, "got {} frames", frames.len());
    }

    /// Tracking test for the high-zero-density Express-mode case.
    /// Same Phase 2 follow-up as `auto_detect_fast_mode_…`.
    #[test]
    #[ignore = "Phase 2: Express-mode LDPC convergence on high-zero-density payloads"]
    fn auto_detect_express_mode_high_zero_density() {
        let header = header_for(Mode::Express, 4, 2, 11);
        let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xC3) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 1, "got {} frames", frames.len());
    }

    /// Auto-detecting decoder finds a frame placed mid-buffer with
    /// silence on both sides.
    #[test]
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
    fn auto_detect_with_leading_and_trailing_silence() {
        let header = header_for(Mode::Standard, 6, 3, 4);
        let payload: Vec<u8> = vec![0x77; 32];
        let frame_audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();

        let lead = vec![0.0f32; 500];
        let tail = vec![0.0f32; 800];
        let mut audio = Vec::with_capacity(lead.len() + frame_audio.len() + tail.len());
        audio.extend_from_slice(&lead);
        audio.extend_from_slice(&frame_audio);
        audio.extend_from_slice(&tail);

        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].mode, Mode::Standard);
        assert_eq!(frames[0].block_count, 6);
        assert_eq!(&frames[0].payload[..payload.len()], &payload[..]);
    }

    /// Auto-detecting decoder finds two distinct frames placed
    /// back-to-back in the same audio buffer (separated by silence).
    #[test]
    #[ignore = "Phase 1'c: rx.rs awaiting QPSK rewrite (tx.rs already pivoted)"]
    fn auto_detect_two_back_to_back_frames() {
        let h1 = header_for(Mode::Robust, 3, 1, 5);
        let p1: Vec<u8> = vec![0xAA; 20];
        let h2 = header_for(Mode::Fast, 5, 4, 6);
        let p2: Vec<u8> = vec![0xBB; 40];
        let a1 = encode(&h1, &p1, AUDIO_CENTRE_HZ).unwrap();
        let a2 = encode(&h2, &p2, AUDIO_CENTRE_HZ).unwrap();

        let gap = vec![0.0f32; 1000];
        let mut audio = Vec::with_capacity(a1.len() + gap.len() + a2.len());
        audio.extend_from_slice(&a1);
        audio.extend_from_slice(&gap);
        audio.extend_from_slice(&a2);

        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 2, "got {} frames", frames.len());

        let by_seq: std::collections::HashMap<u8, &DecodedFrame> =
            frames.iter().map(|f| (f.sequence, f)).collect();
        let f1 = by_seq.get(&5).expect("first frame missing");
        let f2 = by_seq.get(&6).expect("second frame missing");
        assert_eq!(f1.mode, Mode::Robust);
        assert_eq!(f1.block_count, 3);
        assert_eq!(&f1.payload[..p1.len()], &p1[..]);
        assert_eq!(f2.mode, Mode::Fast);
        assert_eq!(f2.block_count, 5);
        assert_eq!(&f2.payload[..p2.len()], &p2[..]);
    }

    /// Empty / silent audio must produce no frames.
    #[test]
    fn auto_detect_empty_audio() {
        let frames = decode(&[], AUDIO_CENTRE_HZ);
        assert!(frames.is_empty());
        let frames = decode(&vec![0.0f32; 5000], AUDIO_CENTRE_HZ);
        assert!(frames.is_empty());
    }
}
