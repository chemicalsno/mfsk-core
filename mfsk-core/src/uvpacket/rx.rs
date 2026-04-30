// SPDX-License-Identifier: GPL-3.0-or-later
//! RX path: 12 kHz f32 PCM audio → decoded frames (post 0.4.0
//! redesign).
//!
//! ## Pipeline
//!
//! 1. **Sync + mode identification** — correlate against all four
//!    127-chip preambles in one matched-filter pass per frequency
//!    candidate. The strongest peak's preamble identifies both the
//!    timing offset and the payload [`Mode`]. (No more brute-force
//!    LDPC sweep across `4 modes × 32 n_blocks` to discover layout.)
//! 2. **AFC** — frequency-grid coarse search around `audio_centre_hz`,
//!    parabolic refinement.
//! 3. **Symbol extraction** — sub-sample timing recovery + linear
//!    interpolation of the matched-filter output at NSPS spacing.
//! 4. **Equaliser training** — least-squares fit of a 9-tap T-spaced
//!    linear equaliser on the long preamble (known BPSK chips →
//!    closed-form normal-equations solve). Removes multipath ISI.
//! 5. **Differential demod** — `r_diff[k] = e[k] · conj(e[k-1])` on
//!    equaliser output. π/4-rotated to land on the standard QPSK
//!    constellation axes for [`qpsk_llrs`].
//! 6. **Amplitude / noise estimate** — joint estimator from the
//!    long preamble's known pair products; gives `a_sq_est`,
//!    `residual_rotation`, `sigma_sq_n_diff`.
//! 7. **Header decode** — 1 LDPC decode (Robust, unpunctured) of the
//!    first post-preamble block. Reads `n_payload_blocks` etc. from
//!    the recovered header bytes.
//! 8. **Payload decode** — `n_blocks` LDPC decodes (mode-puncture +
//!    de-interleave). Concatenate info bytes; verify the header's
//!    CRC over `header_word ++ padded_payload`.
//!
//! ## LDPC decode count
//!
//! `1 + n_blocks` per frame (vs `≤ 128` brute-force in the prior
//! design). Worst case (32-block frame) ≈ 33 LDPC decodes; typical
//! 16-byte payload Robust ≈ 1 + 2 = 3 LDPC decodes (~30 ms).

use std::f32::consts::PI;

use num_complex::Complex32;

use crate::core::{FecCodec, FecOpts};
use crate::fec::Ldpc240_101;

use super::framing::{HEADER_BYTES, INFO_BYTES_PER_BLOCK, UnpackError, unpack_header};
use super::interleaver::deinterleave_llr;
use super::puncture::{Mode, de_puncture_llr};
use super::sync_pattern::{NUM_PREAMBLES, PREAMBLE_LEN, PREAMBLES, mode_for_index, preamble_for};
use super::tx::{NSPS, RRC_ALPHA, RRC_SPAN_SYMS, SAMPLE_RATE_HZ, rrc_pulse};

/// LDPC mother-codeword length.
const N_LDPC: usize = 240;
/// LDPC info-bit count.
const K_LDPC: usize = 101;
/// Symbols carrying one full unpunctured LDPC codeword (240 ch bits / 2 bit/sym).
const HEADER_BLOCK_SYMS: usize = N_LDPC / 2;
/// RRC pulse length in samples (`span × NSPS + 1` = 61).
const RRC_LEN: usize = RRC_SPAN_SYMS * NSPS + 1;
/// Symbol-peak position offset within the matched-filter output for
/// a transmitted symbol at TX baseband index 0.
const SYM_PEAK_OFFSET: usize = RRC_LEN - 1;

/// Equaliser tap count. 9 T-spaced (NSPS-spaced in baseband) taps,
/// centred (4 past + 1 centre + 4 future).
const EQ_N_TAPS: usize = 9;
const EQ_TAP_HALF: usize = EQ_N_TAPS / 2;
/// Diagonal loading for the LS solve (fraction of `R`'s trace).
const EQ_LS_DIAG_LOAD: f32 = 1e-2;

/// Errors returned by the decode functions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Audio buffer ended before a full preamble + header block
    /// could fit at `sample_offset`.
    Truncated,
    /// Audio buffer ended before all `n_blocks` payload blocks
    /// could fit (after header decode revealed `n_blocks`).
    PayloadTruncated { needed_samples: usize },
    /// Header LDPC block failed to decode (BP did not converge,
    /// even with OSD).
    HeaderFecFailed,
    /// At least one payload LDPC block failed to decode.
    PayloadFecFailed,
    /// Reassembled frame data failed CRC verification.
    Crc(UnpackError),
}

/// Result of a successful frame decode.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecodedFrame {
    pub app_type: u8,
    pub sequence: u8,
    pub mode: Mode,
    pub block_count: u8,
    pub payload: Vec<u8>,
}

/// AFC configuration.
#[derive(Clone, Copy, Debug)]
pub struct AfcOpts {
    /// One-sided search range in Hz (total window = ±search_hz).
    pub search_hz: f32,
}

impl Default for AfcOpts {
    fn default() -> Self {
        Self { search_hz: 200.0 }
    }
}

/// Default LDPC decode options.
pub fn default_fec_opts() -> FecOpts<'static> {
    FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
    }
}

// ────────────────────────────────────────────────────────────────────
// Public API — decode_known_layout
// ────────────────────────────────────────────────────────────────────

/// Decode a uvpacket frame whose start sample (`sample_offset`),
/// audio centre frequency, and payload [`Mode`] are known. Reads
/// `n_blocks` (and other header fields) from the dedicated header
/// block — no caller hint needed.
pub fn decode_known_layout(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
) -> Result<DecodedFrame, DecodeError> {
    decode_at_inner(audio, sample_offset, audio_centre_hz, mode, fec_opts)
}

/// AFC-wrapped variant of [`decode_known_layout`]. Searches for the
/// best frequency offset over `±afc_opts.search_hz` first, then
/// decodes at the corrected centre.
pub fn decode_known_layout_with_afc(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
    afc_opts: &AfcOpts,
) -> Result<DecodedFrame, DecodeError> {
    // AFC needs at least preamble + header-block worth of samples.
    let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * NSPS + RRC_LEN;
    if sample_offset + needed > audio.len() {
        return Err(DecodeError::Truncated);
    }
    let delta_hz = estimate_freq_offset_for_mode(
        audio,
        sample_offset,
        needed,
        audio_centre_hz,
        mode,
        afc_opts,
    );
    decode_at_inner(
        audio,
        sample_offset,
        audio_centre_hz + delta_hz,
        mode,
        fec_opts,
    )
}

// ────────────────────────────────────────────────────────────────────
// Public API — auto-detect decode (single channel + multi-channel)
// ────────────────────────────────────────────────────────────────────

/// Auto-detect: scan the audio at `audio_centre_hz`, find every
/// candidate sync peak (any of the four preamble variants), and
/// decode each with the [`Mode`] the winning preamble identified.
pub fn decode(audio: &[f32], audio_centre_hz: f32) -> Vec<DecodedFrame> {
    decode_inner(audio, audio_centre_hz, &default_fec_opts())
}

/// Configuration for the multi-channel RX primitive and the
/// slot-occupancy survey helper.
#[derive(Clone, Copy, Debug)]
pub struct MultiChannelOpts {
    pub band_lo_hz: f32,
    pub band_hi_hz: f32,
    pub coarse_step_hz: f32,
    pub nms_radius_hz: f32,
}

impl Default for MultiChannelOpts {
    fn default() -> Self {
        Self {
            band_lo_hz: 300.0,
            band_hi_hz: 2700.0,
            coarse_step_hz: 25.0,
            nms_radius_hz: 600.0,
        }
    }
}

/// Per-slot received-signal-energy report.
#[derive(Clone, Copy, Debug)]
pub struct SlotEnergy {
    pub audio_centre_hz: f32,
    pub mean_mf_magnitude: f32,
}

/// Decode every uvpacket frame whose audio centre lies in
/// `[band_lo_hz, band_hi_hz]`. Returns `(detected_centre_hz, frame)`
/// pairs.
pub fn decode_multichannel(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    fec_opts: &FecOpts,
) -> Vec<(f32, DecodedFrame)> {
    decode_multichannel_inner(audio, mc_opts, fec_opts)
}

/// Measure the per-slot received-signal energy across the configured
/// band. Used by the LBT step before a random-slot TX.
pub fn measure_slot_energies(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    slot_spacing_hz: f32,
) -> Vec<SlotEnergy> {
    let mut out = Vec::new();
    if audio.is_empty() {
        return out;
    }
    let half = slot_spacing_hz / 2.0;
    let mut centre = mc_opts.band_lo_hz + half;
    while centre <= mc_opts.band_hi_hz {
        let mf_out = downconvert_and_matched_filter(audio, centre);
        let mean_mag = if mf_out.is_empty() {
            0.0
        } else {
            mf_out.iter().map(|c| c.norm_sqr()).sum::<f32>() / mf_out.len() as f32
        };
        out.push(SlotEnergy {
            audio_centre_hz: centre,
            mean_mf_magnitude: mean_mag,
        });
        centre += slot_spacing_hz;
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Public diagnostics
// ────────────────────────────────────────────────────────────────────

/// Peak / median / ratio of the differential preamble-correlation
/// score distribution at a given audio centre, evaluated against
/// every preamble variant and reported as `(mode, stats)`.
#[derive(Copy, Clone, Debug)]
pub struct SyncStats {
    pub global_max: f32,
    pub median: f32,
    /// `global_max / median`. Bounded above by ~`PREAMBLE_LEN − 1
    /// = 126` (Cauchy-Schwarz on 126 differential pairs).
    pub ratio: f32,
    pub n_scores: usize,
}

/// Run the differential preamble correlator at the given centre and
/// return both the winning mode and the score stats. Useful for
/// callers that want to inspect sync quality without running a full
/// LDPC decode.
pub fn diag_sync_at(audio: &[f32], audio_centre_hz: f32) -> Option<(Mode, SyncStats)> {
    if audio.len() < PREAMBLE_LEN * NSPS + RRC_LEN {
        return None;
    }
    let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz);
    let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
    if max_corr_offset == 0 {
        return None;
    }
    let mut per_mode_max: [f32; NUM_PREAMBLES] = [0.0; NUM_PREAMBLES];
    let mut all_scores: Vec<f32> = Vec::with_capacity(max_corr_offset * NUM_PREAMBLES);
    for offset in 0..max_corr_offset {
        for (mode_idx, p) in PREAMBLES.iter().enumerate() {
            let s = preamble_differential_score(&mf_out, offset, &p[..]);
            if s > per_mode_max[mode_idx] {
                per_mode_max[mode_idx] = s;
            }
            all_scores.push(s);
        }
    }
    let (best_mode_idx, &best_score) = per_mode_max
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())?;
    let median = robust_median(&all_scores);
    let ratio = if median > 0.0 {
        best_score / median
    } else {
        0.0
    };
    let mode = mode_for_index(best_mode_idx)?;
    Some((
        mode,
        SyncStats {
            global_max: best_score,
            median,
            ratio,
            n_scores: max_corr_offset * NUM_PREAMBLES,
        },
    ))
}

/// AFC frequency-offset estimate for a known mode. Same algorithm
/// as inside [`decode_known_layout_with_afc`]'s AFC step, exposed
/// for diagnostic harnesses.
pub fn diag_estimate_freq_offset(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    afc_opts: &AfcOpts,
) -> Option<f32> {
    let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * NSPS + RRC_LEN;
    if sample_offset + needed > audio.len() {
        return None;
    }
    Some(estimate_freq_offset_for_mode(
        audio,
        sample_offset,
        needed,
        audio_centre_hz,
        mode,
        afc_opts,
    ))
}

// ────────────────────────────────────────────────────────────────────
// Internal — single-frame decode at a known layout/mode
// ────────────────────────────────────────────────────────────────────

fn decode_at_inner(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
) -> Result<DecodedFrame, DecodeError> {
    // We don't yet know n_blocks. Phase 1: decode header. Phase 2:
    // once header reveals n_blocks, decode payload. The equaliser's
    // future-tap window past the last sampled symbol is zero-padded
    // by `sample_symbols`, so we only require the audio span for the
    // symbols themselves (preamble + header block).
    let need_for_header = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * NSPS + RRC_LEN;
    if sample_offset + need_for_header > audio.len() {
        return Err(DecodeError::Truncated);
    }

    // Down-convert + matched-filter just enough audio for sync +
    // header. Payload extraction will re-MF the full span once we
    // know n_blocks. (MF cost is small vs LDPC cost.)
    let header_slice = &audio[sample_offset..sample_offset + need_for_header];
    let mf_out_header = downconvert_and_matched_filter(header_slice, audio_centre_hz);

    // Coherent integer-sample sync inside the ±NSPS jitter window
    // for the supplied mode's preamble.
    let preamble_bits = preamble_for(mode);
    let (best_off, best_mag2) = best_preamble_offset(&mf_out_header, preamble_bits);
    if best_mag2 <= 0.0 {
        return Err(DecodeError::HeaderFecFailed);
    }
    let frac_off = parabolic_subsample_refine(&mf_out_header, best_off, preamble_bits, best_mag2);

    // Train equaliser on preamble + decode header block.
    let header_syms_total = PREAMBLE_LEN + HEADER_BLOCK_SYMS;
    let symbols_header = sample_symbols(
        &mf_out_header,
        best_off,
        frac_off,
        header_syms_total + EQ_TAP_HALF,
    );
    let weights = train_ls_equaliser(&symbols_header, preamble_bits);
    let equalised_header = apply_equaliser(&symbols_header, &weights, header_syms_total);
    let r_diff_header = differential(&equalised_header);
    let stats = estimate_diff_stats(&r_diff_header[..PREAMBLE_LEN - 1], preamble_bits);
    let header_llrs = compute_llrs(
        &r_diff_header[PREAMBLE_LEN - 1..],
        &stats,
        HEADER_BLOCK_SYMS,
    );
    let header_codeword_llrs: Vec<f32> = header_llrs;

    let fec = Ldpc240_101;
    let header_decode = fec
        .decode_soft(&header_codeword_llrs, fec_opts)
        .ok_or(DecodeError::HeaderFecFailed)?;
    let header_info_bytes = bits_to_bytes_msb(&header_decode.info[..96]);

    // Phase 2: header decoded, parse it speculatively to discover
    // n_blocks. We can't verify CRC yet (CRC covers payload too).
    let (proto_n_blocks, _proto_app, _proto_seq) = peek_header_block_count(&header_info_bytes)?;

    // Now extract & decode the payload.
    let block_ch_bits = mode.ch_bits_per_block();
    let payload_syms = (proto_n_blocks as usize) * block_ch_bits / 2;
    let total_syms = PREAMBLE_LEN + HEADER_BLOCK_SYMS + payload_syms;
    let need_full = total_syms * NSPS + RRC_LEN;
    if sample_offset + need_full > audio.len() {
        return Err(DecodeError::PayloadTruncated {
            needed_samples: need_full,
        });
    }
    let full_slice = &audio[sample_offset..sample_offset + need_full];
    let mf_out_full = downconvert_and_matched_filter(full_slice, audio_centre_hz);
    let symbols_full = sample_symbols(&mf_out_full, best_off, frac_off, total_syms + EQ_TAP_HALF);
    let equalised_full = apply_equaliser(&symbols_full, &weights, total_syms);
    let r_diff_full = differential(&equalised_full);

    // Re-estimate amplitude/noise from the full-span preamble (same
    // weights, same preamble — values should match `stats`, but we
    // recompute to handle minor sample-position variation).
    let stats_full = estimate_diff_stats(&r_diff_full[..PREAMBLE_LEN - 1], preamble_bits);

    // Payload data starts at r_diff index `PREAMBLE_LEN + HEADER_BLOCK_SYMS - 1`.
    let payload_start = PREAMBLE_LEN + HEADER_BLOCK_SYMS - 1;
    let payload_llrs = compute_llrs(&r_diff_full[payload_start..], &stats_full, payload_syms);

    // De-interleave + de-puncture + decode each payload block.
    let n_blocks_u = proto_n_blocks as usize;
    let llr_per_block = deinterleave_llr(&payload_llrs, n_blocks_u);
    let mut decoded_info_bytes: Vec<u8> = Vec::with_capacity(n_blocks_u * INFO_BYTES_PER_BLOCK);
    for block_llrs in &llr_per_block {
        let full_llrs = de_puncture_llr(block_llrs, mode);
        let result = fec
            .decode_soft(&full_llrs, fec_opts)
            .ok_or(DecodeError::PayloadFecFailed)?;
        debug_assert_eq!(result.info.len(), K_LDPC);
        decoded_info_bytes.extend_from_slice(&bits_to_bytes_msb(&result.info[..96]));
    }

    // Verify the header CRC over (header_word ++ padded_payload).
    let mut all_bytes: Vec<u8> = Vec::with_capacity(HEADER_BYTES + decoded_info_bytes.len());
    all_bytes.extend_from_slice(&header_info_bytes[..HEADER_BYTES]);
    all_bytes.extend_from_slice(&decoded_info_bytes);
    let (frame_header, payload_slice) =
        unpack_header(&all_bytes, mode).map_err(DecodeError::Crc)?;
    Ok(DecodedFrame {
        app_type: frame_header.app_type,
        sequence: frame_header.sequence,
        mode,
        block_count: frame_header.block_count,
        payload: payload_slice.to_vec(),
    })
}

/// Speculatively read the block_count field from the (unverified)
/// header info bytes. The CRC has not been verified yet — we use
/// this to decide how many payload symbols to extract; the actual
/// validation happens after payload decode via [`unpack_header`].
fn peek_header_block_count(header_info: &[u8]) -> Result<(u8, u8, u8), DecodeError> {
    if header_info.len() < HEADER_BYTES {
        return Err(DecodeError::HeaderFecFailed);
    }
    let header_word = u16::from_be_bytes([header_info[0], header_info[1]]);
    let blocks_code = ((header_word >> 11) & 0x1F) as u8;
    let app_type = ((header_word >> 7) & 0x0F) as u8;
    let sequence = ((header_word >> 2) & 0x1F) as u8;
    let reserved = (header_word & 0x3) as u8;
    if reserved != 0 {
        return Err(DecodeError::HeaderFecFailed);
    }
    Ok((blocks_code + 1, app_type, sequence))
}

// ────────────────────────────────────────────────────────────────────
// Internal — auto-detect decode pipeline (single channel)
// ────────────────────────────────────────────────────────────────────

const MAX_PICKED_PEAKS: usize = 3;
const SYNC_GATE_RATIO: f32 = 18.0;

fn decode_inner(audio: &[f32], audio_centre_hz: f32, fec_opts: &FecOpts) -> Vec<DecodedFrame> {
    if audio.len() < PREAMBLE_LEN * NSPS + RRC_LEN {
        return Vec::new();
    }
    let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz);
    let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
    if max_corr_offset == 0 {
        return Vec::new();
    }

    // Score every offset against every preamble variant. Keep the
    // best (mode, offset, score) entries that pass the sync gate.
    let mut peaks: Vec<(usize, Mode, f32)> = Vec::new();
    let mut all_scores: Vec<f32> = Vec::with_capacity(max_corr_offset * NUM_PREAMBLES);
    for offset in 0..max_corr_offset {
        for (mode_idx, p) in PREAMBLES.iter().enumerate() {
            let score = preamble_differential_score(&mf_out, offset, &p[..]);
            all_scores.push(score);
            let mode = mode_for_index(mode_idx).unwrap();
            peaks.push((offset, mode, score));
        }
    }
    let median = robust_median(&all_scores);
    if median <= 0.0 {
        return Vec::new();
    }

    // Threshold + rank.
    let threshold = median * SYNC_GATE_RATIO;
    peaks.retain(|&(_, _, s)| s >= threshold);
    peaks.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

    // NMS in offset (within ±NSPS) — keep at most MAX_PICKED_PEAKS.
    let mut picked: Vec<(usize, Mode)> = Vec::new();
    for (offset, mode, _score) in peaks {
        if picked.iter().all(|&(po, _)| offset.abs_diff(po) > NSPS) {
            picked.push((offset, mode));
            if picked.len() >= MAX_PICKED_PEAKS {
                break;
            }
        }
    }
    picked.sort_unstable_by_key(|&(o, _)| o);

    // For each kept peak, run the full single-frame decode at the
    // corrected centre frequency.
    let afc_opts = AfcOpts::default();
    let mut frames: Vec<DecodedFrame> = Vec::new();
    let mut consumed_until: usize = 0;
    for (mf_off, mode) in picked {
        let Some(audio_off) = mf_off.checked_sub(SYM_PEAK_OFFSET) else {
            continue;
        };
        if audio_off < consumed_until {
            continue;
        }
        let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * NSPS + RRC_LEN;
        if audio_off + needed > audio.len() {
            continue;
        }
        let delta_hz = estimate_freq_offset_for_mode(
            audio,
            audio_off,
            needed,
            audio_centre_hz,
            mode,
            &afc_opts,
        );
        if let Ok(frame) =
            decode_at_inner(audio, audio_off, audio_centre_hz + delta_hz, mode, fec_opts)
        {
            consumed_until = audio_off
                + (PREAMBLE_LEN
                    + HEADER_BLOCK_SYMS
                    + (frame.block_count as usize) * mode.ch_bits_per_block() / 2)
                    * NSPS;
            frames.push(frame);
        }
    }
    frames
}

fn decode_multichannel_inner(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    fec_opts: &FecOpts,
) -> Vec<(f32, DecodedFrame)> {
    let mut centre = mc_opts.band_lo_hz;
    let mut peaks: Vec<(f32, usize, Mode, f32)> = Vec::new();
    while centre <= mc_opts.band_hi_hz {
        let mf_out = downconvert_and_matched_filter(audio, centre);
        let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
        if max_corr_offset > 0 {
            // Just record the best (offset, mode, score) at this centre.
            let mut best_off = 0usize;
            let mut best_mode = Mode::Robust;
            let mut best_score = 0.0_f32;
            for offset in 0..max_corr_offset {
                for (mode_idx, p) in PREAMBLES.iter().enumerate() {
                    let s = preamble_differential_score(&mf_out, offset, &p[..]);
                    if s > best_score {
                        best_score = s;
                        best_off = offset;
                        best_mode = mode_for_index(mode_idx).unwrap();
                    }
                }
            }
            peaks.push((centre, best_off, best_mode, best_score));
        }
        centre += mc_opts.coarse_step_hz;
    }
    if peaks.is_empty() {
        return Vec::new();
    }

    // NMS in frequency.
    peaks.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());
    let mut kept: Vec<(f32, usize, Mode)> = Vec::new();
    for (f, off, mode, _score) in peaks {
        if kept
            .iter()
            .all(|&(kf, _, _)| (f - kf).abs() > mc_opts.nms_radius_hz)
        {
            kept.push((f, off, mode));
        }
    }

    // Per-peak decode.
    let afc_opts = AfcOpts::default();
    let mut out: Vec<(f32, DecodedFrame)> = Vec::new();
    for (centre, mf_off, mode) in kept {
        let Some(audio_off) = mf_off.checked_sub(SYM_PEAK_OFFSET) else {
            continue;
        };
        let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * NSPS + RRC_LEN;
        if audio_off + needed > audio.len() {
            continue;
        }
        let delta_hz =
            estimate_freq_offset_for_mode(audio, audio_off, needed, centre, mode, &afc_opts);
        if let Ok(frame) = decode_at_inner(audio, audio_off, centre + delta_hz, mode, fec_opts) {
            out.push((centre + delta_hz, frame));
        }
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Internal — DSP helpers (matched filter, sync, equaliser, demod)
// ────────────────────────────────────────────────────────────────────

fn downconvert_and_matched_filter(audio: &[f32], audio_centre_hz: f32) -> Vec<Complex32> {
    let two_pi_fc_dt = 2.0 * PI * audio_centre_hz / SAMPLE_RATE_HZ;
    let mut bb: Vec<Complex32> = Vec::with_capacity(audio.len());
    for (n, &s) in audio.iter().enumerate() {
        let phase = two_pi_fc_dt * n as f32;
        let (sin, cos) = phase.sin_cos();
        bb.push(Complex32::new(2.0 * s * cos, -2.0 * s * sin));
    }
    let rrc = rrc_pulse(RRC_ALPHA, RRC_SPAN_SYMS, NSPS);
    let n_out = bb.len() + rrc.len() - 1;
    let mut out = vec![Complex32::new(0.0, 0.0); n_out];
    for (i, &x) in bb.iter().enumerate() {
        for (j, &h) in rrc.iter().enumerate() {
            out[i + j] += x * h;
        }
    }
    out
}

fn preamble_correlation(mf_out: &[Complex32], offset: usize, bits: &[bool]) -> Complex32 {
    let mut acc = Complex32::new(0.0, 0.0);
    for (i, &b) in bits.iter().enumerate() {
        let pos = offset + i * NSPS;
        let s = if b { -1.0_f32 } else { 1.0 };
        acc += mf_out[pos] * s;
    }
    acc
}

/// Differential preamble correlator: `score = |Σ aᵢ · cᵢ|² / Σ |aᵢ|²`
/// where `aᵢ = mf[k]·conj(mf[k-1])` and `cᵢ = bᵢ·bᵢ₋₁ ∈ ±1`. Phase-
/// rotation invariant (cancels in the differential product), so it
/// survives clarifier offset / LO walk much better than the
/// coherent score.
fn preamble_differential_score(mf_out: &[Complex32], offset: usize, bits: &[bool]) -> f32 {
    let n = bits.len();
    if n < 2 {
        return 0.0;
    }
    let last_pos = offset + (n - 1) * NSPS;
    if last_pos >= mf_out.len() {
        return 0.0;
    }
    let mut acc = Complex32::new(0.0, 0.0);
    let mut energy = 0.0_f32;
    let mut prev_sample = mf_out[offset];
    let mut prev_sign: f32 = if bits[0] { -1.0 } else { 1.0 };
    for (i, &b) in bits.iter().enumerate().skip(1) {
        let pos = offset + i * NSPS;
        let s = mf_out[pos];
        let cur_sign = if b { -1.0_f32 } else { 1.0 };
        let a = s * prev_sample.conj();
        let c = cur_sign * prev_sign;
        acc += a * c;
        energy += a.norm_sqr();
        prev_sample = s;
        prev_sign = cur_sign;
    }
    if energy <= 0.0 {
        0.0
    } else {
        acc.norm_sqr() / energy
    }
}

fn best_preamble_offset(mf_out: &[Complex32], bits: &[bool]) -> (usize, f32) {
    let radius = NSPS as isize;
    let base = SYM_PEAK_OFFSET as isize;
    let n = bits.len();
    let mut best_off = SYM_PEAK_OFFSET;
    let mut best_mag2 = -1.0_f32;
    for jitter in -radius..=radius {
        let off = base + jitter;
        if off < 0 {
            continue;
        }
        let off = off as usize;
        if off + (n - 1) * NSPS >= mf_out.len() {
            continue;
        }
        let mag2 = preamble_correlation(mf_out, off, bits).norm_sqr();
        if mag2 > best_mag2 {
            best_mag2 = mag2;
            best_off = off;
        }
    }
    (best_off, best_mag2)
}

fn parabolic_subsample_refine(
    mf_out: &[Complex32],
    best_off: usize,
    bits: &[bool],
    best_mag2: f32,
) -> f32 {
    let n = bits.len();
    let need_minus = best_off > 0 && (best_off - 1) + (n - 1) * NSPS < mf_out.len();
    let need_plus = (best_off + 1) + (n - 1) * NSPS < mf_out.len();
    if !(need_minus && need_plus) {
        return 0.0;
    }
    let m_minus = preamble_correlation(mf_out, best_off - 1, bits).norm_sqr();
    let m_plus = preamble_correlation(mf_out, best_off + 1, bits).norm_sqr();
    let denom = 2.0 * (m_plus - 2.0 * best_mag2 + m_minus);
    if denom.abs() > 1e-9 {
        ((m_minus - m_plus) / denom).clamp(-0.5, 0.5)
    } else {
        0.0
    }
}

/// Sample `n_syms` symbols at NSPS spacing starting from
/// `best_off + frac_off`. Returns zero-padded values for any
/// out-of-range position (caller sees them as the equaliser's
/// edge contribution).
fn sample_symbols(
    mf_out: &[Complex32],
    best_off: usize,
    frac_off: f32,
    n_syms: usize,
) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(n_syms);
    for i in 0..n_syms {
        let pos = best_off as f32 + frac_off + (i * NSPS) as f32;
        if pos < 0.0 || pos as usize + 1 >= mf_out.len() {
            out.push(Complex32::new(0.0, 0.0));
        } else {
            out.push(sample_mf_lerp(mf_out, pos));
        }
    }
    out
}

fn sample_mf_lerp(mf_out: &[Complex32], pos: f32) -> Complex32 {
    let p_int = pos.floor() as usize;
    let alpha = pos - p_int as f32;
    mf_out[p_int] * (1.0 - alpha) + mf_out[p_int + 1] * alpha
}

/// Train a 9-tap T-spaced linear equaliser via least-squares on the
/// preamble. Returns the tap weights `w` such that
/// `e[k] = Σ_n w[n] · symbols[k + n − TAP_HALF]` is optimised for
/// MMSE against the BPSK reference `±1`.
fn train_ls_equaliser(symbols: &[Complex32], preamble_bits: &[bool]) -> Vec<Complex32> {
    let n_pre = preamble_bits.len();
    let mut r_mat: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.0, 0.0); EQ_N_TAPS]; EQ_N_TAPS];
    let mut p_vec: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); EQ_N_TAPS];
    for k in EQ_TAP_HALF..n_pre.saturating_sub(EQ_TAP_HALF) {
        let d_re: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        let d = Complex32::new(d_re, 0.0);
        let mut x = [Complex32::new(0.0, 0.0); EQ_N_TAPS];
        for n in 0..EQ_N_TAPS {
            x[n] = symbols[k + n - EQ_TAP_HALF];
        }
        for i in 0..EQ_N_TAPS {
            for j in 0..EQ_N_TAPS {
                r_mat[i][j] += x[i].conj() * x[j];
            }
            p_vec[i] += x[i].conj() * d;
        }
    }
    // Diagonal loading (small fraction of trace for stability).
    let trace: f32 = (0..EQ_N_TAPS).map(|i| r_mat[i][i].re).sum();
    let load = EQ_LS_DIAG_LOAD * trace / EQ_N_TAPS as f32;
    for i in 0..EQ_N_TAPS {
        r_mat[i][i] += Complex32::new(load, 0.0);
    }

    // Solve via Gauss-Jordan on the augmented matrix.
    let mut aug: Vec<Vec<Complex32>> = (0..EQ_N_TAPS)
        .map(|i| {
            let mut row = r_mat[i].clone();
            row.push(p_vec[i]);
            row
        })
        .collect();
    for i in 0..EQ_N_TAPS {
        let mut piv = i;
        let mut piv_mag = aug[i][i].norm();
        for r in (i + 1)..EQ_N_TAPS {
            let m = aug[r][i].norm();
            if m > piv_mag {
                piv_mag = m;
                piv = r;
            }
        }
        if piv != i {
            aug.swap(i, piv);
        }
        if aug[i][i].norm() < 1e-12 {
            return identity_equaliser();
        }
        let pivot = aug[i][i];
        for j in i..=EQ_N_TAPS {
            aug[i][j] /= pivot;
        }
        for r in 0..EQ_N_TAPS {
            if r == i {
                continue;
            }
            let factor = aug[r][i];
            for j in i..=EQ_N_TAPS {
                let sub = factor * aug[i][j];
                aug[r][j] -= sub;
            }
        }
    }
    (0..EQ_N_TAPS).map(|i| aug[i][EQ_N_TAPS]).collect()
}

fn identity_equaliser() -> Vec<Complex32> {
    let mut w = vec![Complex32::new(0.0, 0.0); EQ_N_TAPS];
    w[EQ_TAP_HALF] = Complex32::new(1.0, 0.0);
    w
}

fn apply_equaliser(symbols: &[Complex32], weights: &[Complex32], n_out: usize) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(n_out);
    for k in 0..n_out {
        if k < EQ_TAP_HALF || k + EQ_TAP_HALF >= symbols.len() {
            out.push(Complex32::new(0.0, 0.0));
        } else {
            let mut e = Complex32::new(0.0, 0.0);
            for n in 0..EQ_N_TAPS {
                e += weights[n] * symbols[k + n - EQ_TAP_HALF];
            }
            out.push(e);
        }
    }
    out
}

fn differential(equalised: &[Complex32]) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(equalised.len().saturating_sub(1));
    for k in 1..equalised.len() {
        out.push(equalised[k] * equalised[k - 1].conj());
    }
    out
}

struct DiffStats {
    a_sq_est: f32,
    sigma_sq_n_diff: f32,
    residual_rotation: f32,
}

fn estimate_diff_stats(r_diff_preamble: &[Complex32], preamble_bits: &[bool]) -> DiffStats {
    let n = r_diff_preamble.len();
    let mut signed_acc = Complex32::new(0.0, 0.0);
    for k in 0..n {
        let bk: f32 = if preamble_bits[k + 1] { -1.0 } else { 1.0 };
        let bkm1: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        signed_acc += r_diff_preamble[k] * (bk * bkm1);
    }
    let signed_mean = signed_acc / (n.max(1) as f32);
    let a_sq_est = signed_mean.norm().max(1e-6);
    let residual_rotation = signed_mean.arg();

    let derot_resid = Complex32::from_polar(1.0, -residual_rotation);
    let mut noise_im_sq = 0.0_f32;
    for k in 0..n {
        let bk: f32 = if preamble_bits[k + 1] { -1.0 } else { 1.0 };
        let bkm1: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        let aligned = r_diff_preamble[k] * (bk * bkm1) * derot_resid;
        noise_im_sq += aligned.im * aligned.im;
    }
    let sigma_sq_n_diff = (noise_im_sq / n.max(1) as f32).max(1e-6);
    DiffStats {
        a_sq_est,
        sigma_sq_n_diff,
        residual_rotation,
    }
}

fn compute_llrs(r_diff: &[Complex32], stats: &DiffStats, n_data: usize) -> Vec<f32> {
    let llr_scale = stats.a_sq_est / stats.sigma_sq_n_diff;
    let combined_rotate = Complex32::from_polar(1.0, -stats.residual_rotation - PI / 4.0);
    let mut out: Vec<f32> = Vec::with_capacity(n_data * 2);
    for k in 0..n_data {
        let derot = r_diff[k] * combined_rotate;
        let (llr_b1, llr_b0) = qpsk_llrs(derot);
        out.push(llr_b1 * llr_scale);
        out.push(llr_b0 * llr_scale);
    }
    out
}

/// Soft-output QPSK demap to LLR (max-log). Returns
/// `(LLR(b1), LLR(b0))` with `LLR > 0 ⇔ bit = 1`. Matches the TX
/// Gray map [0, 1, 3, 2].
fn qpsk_llrs(r: Complex32) -> (f32, f32) {
    let re = r.re;
    let im = r.im;
    let llr_b1 = -(re + im);
    let llr_b0 = im.max(-re) - re.max(-im);
    (llr_b1, llr_b0)
}

fn bits_to_bytes_msb(bits: &[u8]) -> Vec<u8> {
    let n_bytes = bits.len() / 8;
    let mut out = vec![0u8; n_bytes];
    for byte_idx in 0..n_bytes {
        let mut byte = 0u8;
        for bit_idx in 0..8 {
            if bits[byte_idx * 8 + bit_idx] != 0 {
                byte |= 1 << (7 - bit_idx);
            }
        }
        out[byte_idx] = byte;
    }
    out
}

fn robust_median(scores: &[f32]) -> f32 {
    let mut nz: Vec<f32> = scores.iter().copied().filter(|&s| s > 0.0).collect();
    if nz.is_empty() {
        return 0.0;
    }
    let mid = nz.len() / 2;
    nz.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap());
    nz[mid]
}

/// AFC for a known mode: try a coarse grid of frequency offsets,
/// pick the one whose preamble correlation peaks highest, refine
/// by parabolic fit.
fn estimate_freq_offset_for_mode(
    audio: &[f32],
    sample_offset: usize,
    needed_samples: usize,
    audio_centre_hz: f32,
    mode: Mode,
    afc_opts: &AfcOpts,
) -> f32 {
    let coarse_step_hz: f32 = 25.0;
    let n_coarse = (afc_opts.search_hz / coarse_step_hz).ceil() as i32;
    let slice = &audio[sample_offset..sample_offset + needed_samples];
    let preamble_bits = preamble_for(mode);

    let mut grid_mags: Vec<(i32, f32)> = Vec::with_capacity(2 * n_coarse as usize + 1);
    for k in -n_coarse..=n_coarse {
        let f_test = audio_centre_hz + k as f32 * coarse_step_hz;
        let mf_out = downconvert_and_matched_filter(slice, f_test);
        let (_, mag2) = best_preamble_offset(&mf_out, preamble_bits);
        grid_mags.push((k, mag2));
    }
    let (best_k_idx, _) = grid_mags
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.1.partial_cmp(&b.1.1).unwrap())
        .map(|(idx, &(_, m))| (idx, m))
        .unwrap_or((0, 0.0));
    let best_k = grid_mags[best_k_idx].0;
    let frac = if best_k_idx > 0 && best_k_idx + 1 < grid_mags.len() {
        let m_minus = grid_mags[best_k_idx - 1].1;
        let m_zero = grid_mags[best_k_idx].1;
        let m_plus = grid_mags[best_k_idx + 1].1;
        let denom = 2.0 * (m_plus - 2.0 * m_zero + m_minus);
        if denom.abs() > 1e-9 {
            ((m_minus - m_plus) / denom).clamp(-0.5, 0.5)
        } else {
            0.0
        }
    } else {
        0.0
    };
    (best_k as f32 + frac) * coarse_step_hz
}
