// SPDX-License-Identifier: GPL-3.0-or-later
//! RX path: 12 kHz f32 PCM audio → decoded frames.
//!
//! Phase 1'c: matches the QPSK + RRC + m-sequence-preamble TX in
//! [`crate::uvpacket::tx`]. Pipeline:
//!
//! ```text
//! audio @ 12 kHz f32
//!   ↓ down-convert: rx_baseband[n] = 2 · audio[n] · e^{-j·2π·fc·n/fs}
//!   ↓ matched-filter: convolve with the same RRC pulse used by TX
//!   → mf_out (complex, length audio.len() + RRC_LEN − 1)
//!
//! frame head detected at sample offset s:
//!   ↓ correlate the 31-sym BPSK m-sequence preamble against
//!     mf_out[s + RRC_LEN−1 + i·NSPS] for i ∈ 0..31
//!     → complex correlation peak C₀; magnitude scores frame
//!       presence, arg(C₀) seeds initial carrier phase
//!   ↓ extract data + pilot symbols at NSPS spacing past the preamble
//!   ↓ pilot-aided phase tracking: known QPSK pilots every 32 sym;
//!     phase is linearly interpolated between consecutive pilots
//!     (preamble bookends the leading pilot at sym 0)
//!   ↓ de-rotate each data symbol; QPSK soft-demap to (LLR_b1, LLR_b0)
//!   ↓ block-deinterleave LLRs → de-puncture per mode → LDPC decode
//!   ↓ unpack header + verify CRC; check (mode, block_count) match
//!   → DecodedFrame
//! ```
//!
//! All layers above the QPSK soft-demap (de-interleave, de-puncture,
//! LDPC, framing) are unchanged from the 4-FSK Phase 1f path.
//!
//! ## Auto-detect ([`decode`])
//!
//! Replaces the Phase 1f Costas search with a preamble m-sequence
//! correlator. For each above-threshold preamble peak, attempt
//! [`decode_known_layout`] across the (mode × n_blocks) candidate
//! grid; the first that succeeds wins. n_blocks is inferred from the
//! decoded header for verification.

use num_complex::Complex32;
use std::f32::consts::PI;

use crate::core::{FecCodec, FecOpts};
use crate::fec::Ldpc240_101;

use super::framing::{INFO_BYTES_PER_BLOCK, UnpackError, unpack as unpack_frame};
use super::interleaver::deinterleave_llr;
use super::puncture::{Mode, de_puncture_llr};
use super::sync_pattern::{
    PILOT_QPSK_POINT, PILOT_SYMBOL_INTERVAL, PREAMBLE_LEN, UVPACKET_PREAMBLE_BPSK_BITS,
};

/// LDPC info-bit count.
const K_LDPC: usize = 101;

/// Sample rate the modem operates at.
const SAMPLE_RATE_HZ: f32 = 12_000.0;
/// 1200 baud → 10 samples per symbol at 12 kHz.
const NSPS: usize = 10;
/// RRC pulse: span in symbols (3 each side of centre tap).
const RRC_SPAN_SYMS: usize = 6;
/// RRC roll-off factor.
const RRC_ALPHA: f32 = 0.5;
/// RRC pulse length in samples (`span × NSPS + 1` = 61).
const RRC_LEN: usize = RRC_SPAN_SYMS * NSPS + 1;
/// Symbol-peak position offset within the matched-filter output for a
/// transmitted symbol at TX baseband index 0. Equals `RRC_LEN − 1`
/// because TX writes symbol i at samples `[i·NSPS, i·NSPS+RRC_LEN−1]`
/// and RX matched-filter convolution adds another centre offset of
/// `RRC_LEN/2 − 0.5`. Rounding to integers, the composite g = h*h
/// peak (raised cosine) lands at lag `RRC_LEN − 1` from the transmit-
/// side reference.
const SYM_PEAK_OFFSET: usize = RRC_LEN - 1;

/// Errors returned by [`decode_known_layout`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Audio buffer ended before the layout's `n_blocks` worth of
    /// samples could be extracted.
    Truncated,
    /// At least one LDPC block failed to decode (BP did not
    /// converge within the iteration budget, even with OSD-2).
    FecFailed,
    /// The frame data unpacked but its CRC-16 did not match.
    Crc(UnpackError),
    /// The decoded frame's header's mode / block_count differ from
    /// the layout the caller requested.
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
    pub app_type: u8,
    pub sequence: u8,
    pub mode: Mode,
    pub block_count: u8,
    pub payload: Vec<u8>,
}

/// Default LDPC decode options for uvpacket.
///
/// `osd_depth = 2` gives a good cost / threshold-margin trade-off
/// for the Robust / Standard / Fast modes. Express needs OSD-3 to
/// hit its rated threshold reliably (the rate-3/4 puncturing pushes
/// BP-only into the saturation region), so callers driving Express
/// in fading channels may want to override via
/// [`decode_known_layout_with_opts`].
fn default_fec_opts() -> FecOpts<'static> {
    FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
    }
}

/// Decode a uvpacket frame at a known location with known layout.
///
/// `audio` is 12 kHz f32 PCM. `sample_offset` is the audio sample
/// index of the *first* preamble sample (= start of TX burst).
/// `audio_centre_hz` is the modem audio centre (typically 1500 Hz).
///
/// Uses default LDPC options (BP 50 iter, OSD-2). Use
/// [`decode_known_layout_with_opts`] to override — e.g. for Express
/// in fading channels where OSD-3 helps, or for application-specific
/// `verify_info` hooks.
pub fn decode_known_layout(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    n_blocks: u8,
) -> Result<DecodedFrame, DecodeError> {
    decode_known_layout_with_opts(
        audio,
        sample_offset,
        audio_centre_hz,
        mode,
        n_blocks,
        &default_fec_opts(),
    )
}

/// AFC (automatic frequency control) options for SSB use.
///
/// On NFM the TX/RX share the same audio centre exactly so AFC
/// is unnecessary. On SSB, VFO-dial mismatches shift the audio
/// centre by ±50–100 Hz typically. AFC searches for the offset
/// via FFT-based preamble-magnitude maximisation, then re-runs
/// the demod at the corrected centre frequency.
#[derive(Clone, Copy, Debug)]
pub struct AfcOpts {
    /// One-sided search range in Hz (so the total search window
    /// is `±search_hz`). 200 Hz covers typical SSB VFO mismatch
    /// worst-case without a meaningful CPU cost.
    pub search_hz: f32,
}

impl Default for AfcOpts {
    fn default() -> Self {
        Self { search_hz: 200.0 }
    }
}

/// Decode a uvpacket frame at a known location, with AFC and
/// caller-supplied LDPC options.
///
/// 1. Sweep `Δf_test` in 25 Hz steps across `[−search_hz,
///    +search_hz]`. At each step, down-convert + matched-filter
///    at `audio_centre_hz + Δf_test` and take the best
///    preamble-correlation magnitude over the ±NSPS jitter
///    window.
/// 2. Pick the coarse-grid winner and parabolic-fit the three
///    adjacent magnitudes for sub-grid resolution.
/// 3. Re-run [`decode_known_layout_with_opts`] at
///    `audio_centre_hz + Δf`.
///
/// Returns the decoded frame on success; otherwise the same
/// `DecodeError` variants as `decode_known_layout_with_opts`. If
/// AFC mis-estimates Δf (preamble below threshold), the FEC stage
/// will surface as `FecFailed` / `Crc` / `LayoutMismatch` rather
/// than a wrong-frame.
pub fn decode_known_layout_with_afc(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    n_blocks: u8,
    fec_opts: &FecOpts,
    afc_opts: &AfcOpts,
) -> Result<DecodedFrame, DecodeError> {
    let n_blocks_u = n_blocks as usize;
    let block_ch_bits = mode.ch_bits_per_block();
    debug_assert!(block_ch_bits.is_multiple_of(2));
    let n_data_syms = n_blocks_u * block_ch_bits / 2;
    let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
    let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
    let needed_samples = total_syms * NSPS + RRC_LEN;

    if sample_offset + needed_samples > audio.len() {
        return Err(DecodeError::Truncated);
    }

    // Frequency-grid AFC: find Δf at which the matched-filter
    // preamble correlator peaks. Refine the 25-Hz coarse winner
    // by parabolic fit on its three neighbours.
    let delta_hz = estimate_freq_offset(
        audio,
        sample_offset,
        needed_samples,
        audio_centre_hz,
        afc_opts,
    );

    // Re-run the full decoder at the corrected centre frequency.
    decode_known_layout_with_opts(
        audio,
        sample_offset,
        audio_centre_hz + delta_hz,
        mode,
        n_blocks,
        fec_opts,
    )
}

/// Best-preamble-correlation magnitude squared at any of the
/// integer-sample offsets within the standard ±NSPS jitter window.
/// Used by the AFC frequency-grid search.
fn best_preamble_mag2_around_anchor(mf_out: &[Complex32]) -> f32 {
    let radius = NSPS as isize;
    let base = SYM_PEAK_OFFSET as isize;
    let mut best = -1.0_f32;
    for jitter in -radius..=radius {
        let off = base + jitter;
        if off < 0 {
            continue;
        }
        let off = off as usize;
        if off + (PREAMBLE_LEN - 1) * NSPS >= mf_out.len() {
            continue;
        }
        let m2 = preamble_correlation(mf_out, off).norm_sqr();
        if m2 > best {
            best = m2;
        }
    }
    best
}

/// Carrier-frequency-offset search by **frequency-grid preamble
/// correlation**: try each candidate `audio_centre_hz + Δf_test` in
/// a coarse grid, run the matched filter and integer-sample
/// preamble correlator, pick the Δf that gives the strongest
/// preamble peak. Refines the winner by fitting a parabola to the
/// magnitude over the three adjacent coarse-grid points.
///
/// Why this and not an FFT over the chip-rate samples: at a
/// non-zero frequency offset, the integer-sample preamble
/// correlator already has a sinc roll-off (`sinc(δ · 31 / 1200)`),
/// so `best_off` is found at a noise sample for `|δ| ≳ 20 Hz`
/// (sinc dives below 0.5). An FFT downstream of that wrong
/// `best_off` operates on garbage. The frequency-grid search
/// instead searches for the Δf at which the **preamble correlator
/// magnitude itself peaks** — by construction, this is the Δf
/// that makes the chip phases align.
fn estimate_freq_offset(
    audio: &[f32],
    sample_offset: usize,
    needed_samples: usize,
    audio_centre_hz: f32,
    afc_opts: &AfcOpts,
) -> f32 {
    let coarse_step_hz: f32 = 25.0; // rolls off ~3 dB at half-step worst-case
    let n_coarse = (afc_opts.search_hz / coarse_step_hz).ceil() as i32;
    let slice = &audio[sample_offset..sample_offset + needed_samples];

    let mut grid_mags: Vec<(i32, f32)> = Vec::with_capacity(2 * n_coarse as usize + 1);
    for k in -n_coarse..=n_coarse {
        let f_test = audio_centre_hz + k as f32 * coarse_step_hz;
        let mf_out = downconvert_and_matched_filter(slice, f_test);
        let m2 = best_preamble_mag2_around_anchor(&mf_out);
        grid_mags.push((k, m2));
    }

    // Pick coarse winner.
    let (best_k_idx, _) = grid_mags
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.1.partial_cmp(&b.1.1).unwrap())
        .map(|(idx, &(_, m))| (idx, m))
        .unwrap_or((0, 0.0));
    let best_k = grid_mags[best_k_idx].0;

    // Parabolic refinement on the three points centred on best_k.
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

/// Public diagnostic accessor for the AFC's frequency-offset
/// estimate. Returns the same Δf that
/// [`decode_known_layout_with_afc`] would derive from the same
/// audio + offset + AFC settings, without running the full decode
/// roundtrip. Intended for tests and characterisation harnesses.
pub fn diag_estimate_freq_offset(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    needed_samples: usize,
    afc_opts: &AfcOpts,
) -> Option<f32> {
    if sample_offset + needed_samples > audio.len() {
        return None;
    }
    Some(estimate_freq_offset(
        audio,
        sample_offset,
        needed_samples,
        audio_centre_hz,
        afc_opts,
    ))
}

/// Same as [`decode_known_layout`] but with caller-supplied LDPC
/// options. Use this to opt into deeper OSD (~30× slower per
/// decode but ~10–15 % better PER near threshold for the higher-
/// rate modes), to add `ap_mask` priors, or to plug in a
/// `verify_info` hook.
pub fn decode_known_layout_with_opts(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    n_blocks: u8,
    fec_opts: &FecOpts,
) -> Result<DecodedFrame, DecodeError> {
    let n_blocks_u = n_blocks as usize;
    let block_ch_bits = mode.ch_bits_per_block();
    debug_assert!(block_ch_bits.is_multiple_of(2));
    let n_data_syms = n_blocks_u * block_ch_bits / 2;
    let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
    let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
    // Audio samples needed: total_syms × NSPS + RRC tail (RRC_LEN
    // samples past the last symbol position).
    let needed_samples = total_syms * NSPS + RRC_LEN;

    if sample_offset + needed_samples > audio.len() {
        return Err(DecodeError::Truncated);
    }

    // 1. Down-convert + matched-filter the relevant audio slice.
    let slice = &audio[sample_offset..sample_offset + needed_samples];
    let mf_out = downconvert_and_matched_filter(slice, audio_centre_hz);

    // 2. Refine the integer-sample frame offset by maximising preamble
    //    correlation magnitude over a ±NSPS jitter window. The base
    //    offset is `SYM_PEAK_OFFSET` (where TX symbol 0's matched-
    //    filter peak lands assuming the burst starts at sample 0 of
    //    the slice).
    let radius = NSPS as isize;
    let base = SYM_PEAK_OFFSET as isize;
    let mut best_off = SYM_PEAK_OFFSET;
    let mut best_corr = Complex32::new(0.0, 0.0);
    let mut best_mag2 = -1.0_f32;
    for jitter in -radius..=radius {
        let off = base + jitter;
        if off < 0 {
            continue;
        }
        let off = off as usize;
        let last = off + (PREAMBLE_LEN - 1) * NSPS;
        if last >= mf_out.len() {
            continue;
        }
        let c = preamble_correlation(&mf_out, off);
        let mag2 = c.norm_sqr();
        if mag2 > best_mag2 {
            best_mag2 = mag2;
            best_corr = c;
            best_off = off;
        }
    }

    // 2b. Sub-sample timing refinement via parabolic peak fit on
    //     `|C(off)|²` at the three integer offsets `{best_off−1,
    //     best_off, best_off+1}`. Integer-sample preamble
    //     correlation lands within ±0.5 sample of the true peak;
    //     parabolic refinement narrows that to a few percent of a
    //     sample. The remaining timing offset (≤ ~0.05 sample at
    //     low SNR) shaves a final 0.05–0.1 dB off the worst-case
    //     symbol-amplitude loss from `g(t)` mismatch.
    let frac_off: f32 = {
        let need_minus = best_off > 0 && (best_off - 1) + (PREAMBLE_LEN - 1) * NSPS < mf_out.len();
        let need_plus = (best_off + 1) + (PREAMBLE_LEN - 1) * NSPS < mf_out.len();
        if need_minus && need_plus {
            let m_minus = preamble_correlation(&mf_out, best_off - 1).norm_sqr();
            let m_plus = preamble_correlation(&mf_out, best_off + 1).norm_sqr();
            let m_zero = best_mag2;
            let denom = 2.0 * (m_plus - 2.0 * m_zero + m_minus);
            if denom.abs() > 1e-9 {
                ((m_minus - m_plus) / denom).clamp(-0.5, 0.5)
            } else {
                0.0
            }
        } else {
            0.0
        }
    };

    // Re-evaluate preamble correlation at the refined fractional
    // offset, so `best_corr.arg()` gives the carrier phase at the
    // true symbol-centre rather than at the integer-snapped offset.
    if frac_off.abs() > 1e-3 {
        let mut acc = Complex32::new(0.0, 0.0);
        for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
            let s = if b { -1.0_f32 } else { 1.0 };
            acc += sample_mf_lerp(&mf_out, best_off as f32 + frac_off + (i * NSPS) as f32) * s;
        }
        best_corr = acc;
    }

    // 3. Extract the full transmitted-symbol stream at the refined
    //    sub-sample offset (preamble + pilot/data interleave).
    let mut symbols: Vec<Complex32> = Vec::with_capacity(total_syms);
    for i in 0..total_syms {
        let pos = best_off as f32 + frac_off + (i * NSPS) as f32;
        if pos < 0.0 || pos as usize + 1 >= mf_out.len() {
            return Err(DecodeError::Truncated);
        }
        symbols.push(sample_mf_lerp(&mf_out, pos));
    }

    // 4. Build a per-symbol phase reference. Anchors:
    //    - The whole preamble (BPSK ±1) → one combined phase via
    //      `best_corr` (already computed).
    //    - Each pilot symbol (known QPSK constellation point at
    //      sym index `PREAMBLE_LEN + k·PILOT_SYMBOL_INTERVAL`).
    //    Linearly interpolate phase between consecutive anchors;
    //    extrapolate flat past the last anchor.
    let pilot_ref = qpsk_constellation_point(PILOT_QPSK_POINT);
    let mut anchor_idx: Vec<usize> = Vec::with_capacity(n_pilots + 1);
    let mut anchor_phase: Vec<f32> = Vec::with_capacity(n_pilots + 1);

    // Preamble anchor: phase at the *centre* of the preamble (use
    // the combined preamble correlation).
    let preamble_centre = (PREAMBLE_LEN - 1) / 2;
    anchor_idx.push(preamble_centre);
    anchor_phase.push(best_corr.arg());

    // Pilot anchors.
    for k in 0..n_pilots {
        let sym_pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
        if sym_pos >= total_syms {
            break;
        }
        let r = symbols[sym_pos];
        // Phase = arg(r / pilot_ref). pilot_ref = +1+0j, so simply arg(r).
        let phase = (r * pilot_ref.conj()).arg();
        // Unwrap relative to the previous anchor to avoid 2π jumps.
        let prev = *anchor_phase.last().unwrap();
        let unwrapped = unwrap_phase(prev, phase);
        anchor_idx.push(sym_pos);
        anchor_phase.push(unwrapped);
    }

    // 4a-LMS. Replace the per-segment linear interp with a global
    //        weighted least-squares quadratic fit over all anchors
    //        (preamble centre + each pilot). The motivation is that
    //        linear-interp-between-adjacent-pilots gives no global
    //        averaging — each segment's phase is set by 2 noisy
    //        pilots only. With ~16+ anchors per Robust 4-block burst
    //        and a 3-coefficient quadratic, LSE delivers variance
    //        reduction proportional to the number of anchors after
    //        accounting for the basis dimension.
    //
    //        Fit: φ(t̂) ≈ c₀ + c₁·t̂ + c₂·t̂² where t̂ ∈ [0, 1]
    //        is the symbol-index normalised to total_syms. The
    //        preamble centre anchor is given high weight (31, the
    //        number of chips it averages); each pilot has weight 1.
    //
    //        For pure AWGN the true phase is constant and the fit
    //        absorbs the noise into c₀. For Doppler / Rayleigh the
    //        quadratic captures slow drift; for fast Doppler the
    //        per-block DDPT below adds the residual correction.
    let lms_coeffs: Option<[f32; 3]> = if anchor_idx.len() >= 3 {
        let n_total = total_syms.max(1) as f32;
        let mut a_mat = [[0.0_f32; 3]; 3];
        let mut a_vec = [0.0_f32; 3];
        let weights: Vec<f32> = (0..anchor_idx.len())
            .map(|k| {
                if k == 0 {
                    (PREAMBLE_LEN as f32).sqrt()
                } else {
                    1.0
                }
            })
            .collect();
        for k in 0..anchor_idx.len() {
            let t = anchor_idx[k] as f32 / n_total;
            let w = weights[k];
            let row = [1.0, t, t * t];
            for i in 0..3 {
                for j in 0..3 {
                    a_mat[i][j] += w * row[i] * row[j];
                }
                a_vec[i] += w * row[i] * anchor_phase[k];
            }
        }
        solve_3x3(&a_mat, &a_vec)
    } else {
        None
    };

    // Effective phase function: use LMS fit if available, else fall
    // back to linear interpolation. For numerical stability the LMS
    // fit is evaluated on normalised t = idx / total_syms.
    let total_syms_f = total_syms.max(1) as f32;

    // 4b. Decision-directed phase refinement. The pilot grid (every
    //     32 sym) leaves residual phase-tracking error on data
    //     symbols even after LMS smoothing — DDPT collapses that
    //     residual at the per-LDPC-block level.
    //
    //     One-pass DDPT: hard-decide each data symbol using the
    //     pilot-only phase track, then incorporate that decision as
    //     an additional phase anchor. Wrong decisions add noise to
    //     the anchor set but the gain from 32× density typically
    //     dominates as long as the bit error rate is < ~25 %.
    //
    //     Since the channel is slowly-varying (AWGN: phase is
    //     constant; Rayleigh: ≤ 10 Hz Doppler), a single global
    //     additive phase offset captures most of the per-block
    //     phase noise. We do the refinement **per LDPC block** —
    //     each block gets its own averaged DD phase correction
    //     applied on top of the pilot-interpolated phase. Within a
    //     block (≤ ~120 syms = 100 ms) the phase is stable to a few
    //     mrad even at 10 Hz Doppler.
    let block_data_syms = block_ch_bits / 2;
    let mut block_dd_correction = vec![0.0_f32; n_blocks_u];
    {
        // Walk all symbols in TX order. The data slot for block b
        // covers data-symbol indices `[b·block_data_syms,
        // (b+1)·block_data_syms)`. We need to convert that to
        // `total_syms` index, accounting for the preamble + pilots
        // interspersed.
        let mut data_running = 0_usize;
        // Per-block accumulator of complex residual.
        let mut block_resid = vec![Complex32::new(0.0, 0.0); n_blocks_u];
        for i in PREAMBLE_LEN..total_syms {
            let rel = i - PREAMBLE_LEN;
            if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
                continue;
            }
            let phase = lms_phase(lms_coeffs, total_syms_f, &anchor_idx, &anchor_phase, i);
            let derot = symbols[i] * Complex32::from_polar(1.0, -phase);
            // Hard decide closest constellation point at amplitude 1
            // (we don't have `amplitude` yet; use unit constellation
            // since hard decision is only sensitive to the *direction*
            // of `derot`).
            let candidates = [
                Complex32::new(1.0, 0.0),
                Complex32::new(0.0, 1.0),
                Complex32::new(-1.0, 0.0),
                Complex32::new(0.0, -1.0),
            ];
            let mut best_c = candidates[0];
            let mut best_d2 = f32::INFINITY;
            for &c in &candidates {
                // Maximise Re(derot · c̄) = pick closest direction.
                let d2 = (derot - c).norm_sqr();
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_c = c;
                }
            }
            // Residual of derot w.r.t. its hard decision: any phase
            // offset shows up as a rotation of `derot` away from
            // `best_c`. Accumulate `derot · best_c.conj()` (which
            // should be ≈ A · 1 + small noise if phase is right; any
            // residual phase error rotates the accumulator).
            let r = derot * best_c.conj();
            let block_idx = data_running / block_data_syms;
            if block_idx < n_blocks_u {
                block_resid[block_idx] += r;
            }
            data_running += 1;
            if data_running >= n_data_syms {
                break;
            }
        }
        for b in 0..n_blocks_u {
            // Per-block correction: arg of the accumulated residual.
            // If residual is dominated by signal (correct decisions),
            // arg ≈ residual phase error; if dominated by wrong
            // decisions, arg ≈ random and we damage the track. The
            // |residual| / N gives confidence; if confidence is low
            // we leave the correction at 0.
            let mag = block_resid[b].norm();
            let n_per_block = block_data_syms as f32;
            // Confidence proxy: |sum| / N. With unit-amplitude hard
            // decisions and ε fraction wrong, |sum|/N ≈ (1 − 2ε) · A.
            // For ε = 0.25 (≈ +6 dB Eb/N0_info Robust threshold),
            // |sum|/N ≈ 0.5·A. We require ≥ 0.25·A_proxy = 0.25
            // (using A=1 as the unit-decided constellation), below
            // which we trust the pilot track instead.
            if mag > 0.25 * n_per_block {
                block_dd_correction[b] = block_resid[b].arg();
            }
        }
    }

    // 5a. Estimate signal amplitude `A` and per-axis noise variance
    //     `σ²_n` from the **known** symbols (preamble BPSK chips +
    //     QPSK pilots) so the LLRs delivered to BP have correct
    //     likelihood magnitude (without σ²-scaling, BP receives high-
    //     confidence LLRs even at noisy channels — at low rate where
    //     σ is largest, this fools BP into propagating wrong signs
    //     instead of relying on parity correction).
    //
    //     For preamble chip i: expected = ±1 + 0j (sign per
    //     `UVPACKET_PREAMBLE_BPSK_BITS`). For pilot k: expected =
    //     `pilot_ref` (+1 + 0j). De-rotate by interpolated phase,
    //     then accumulate ⟨r, expected⟩ for the amplitude estimator
    //     and Σ |r − A·expected|² for the noise estimator.
    let mut a_acc = 0.0_f32;
    let mut a_norm = 0.0_f32;
    // Preamble:
    for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
        let expected = Complex32::new(if b { -1.0 } else { 1.0 }, 0.0);
        let phase = lms_phase(lms_coeffs, total_syms_f, &anchor_idx, &anchor_phase, i);
        let derot = symbols[i] * Complex32::from_polar(1.0, -phase);
        a_acc += derot.re * expected.re + derot.im * expected.im;
        a_norm += expected.norm_sqr();
    }
    // Pilots:
    for k in 0..n_pilots {
        let sym_pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
        if sym_pos >= total_syms {
            break;
        }
        let phase = lms_phase(
            lms_coeffs,
            total_syms_f,
            &anchor_idx,
            &anchor_phase,
            sym_pos,
        );
        let derot = symbols[sym_pos] * Complex32::from_polar(1.0, -phase);
        a_acc += derot.re * pilot_ref.re + derot.im * pilot_ref.im;
        a_norm += pilot_ref.norm_sqr();
    }
    let amplitude = if a_norm > 0.0 { a_acc / a_norm } else { 1.0 };
    let amplitude = amplitude.max(1e-6); // guard against pathological cases

    // σ²_n estimation: use the **data-symbol magnitude variance**
    // rather than pilot/preamble residuals. For QPSK with
    // constellation amplitude `A` (`|c|=1`) and per-axis noise
    // variance `σ²_n`,
    //
    //     E[|r|²] = A²·E[|c|²] + 2·A·Re(E[c̄]·E[n]) + E[|n|²]
    //             = A² + 0 + 2·σ²_n
    //
    // so σ²_n = (E[|r|²] − A²) / 2. This estimator captures the
    // **total** noise on data symbols — AWGN plus any inter-pilot
    // phase-tracking residual — which is the noise BP actually has
    // to overcome. Pilot-only residual under-counts by 6–16 % and
    // makes the LLR scale over-confident at low SNR.
    // Per-data-symbol DD correction must be applied before measuring
    // |r|² for σ²_n (the correction takes phase-tracking residual
    // out of the noise budget).
    let mut data_mag_sq = 0.0_f32;
    let mut data_seen = 0_usize;
    {
        let mut data_running = 0_usize;
        for i in PREAMBLE_LEN..total_syms {
            let rel = i - PREAMBLE_LEN;
            if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
                continue;
            }
            let block_idx = data_running / block_data_syms;
            let phase = lms_phase(lms_coeffs, total_syms_f, &anchor_idx, &anchor_phase, i)
                + block_dd_correction.get(block_idx).copied().unwrap_or(0.0);
            let derot = symbols[i] * Complex32::from_polar(1.0, -phase);
            data_mag_sq += derot.norm_sqr();
            data_seen += 1;
            data_running += 1;
            if data_running >= n_data_syms {
                break;
            }
        }
    }
    let sigma_sq_n = if data_seen > 0 {
        let mean_mag_sq = data_mag_sq / data_seen as f32;
        ((mean_mag_sq - amplitude * amplitude) / 2.0).max(1e-6)
    } else {
        1.0
    };

    // 5b. QPSK soft-demap each data symbol after de-rotation, with
    //     proper LLR scaling. The true max-log LLR for QPSK with
    //     constellation amplitude `A` (`c ∈ A·{±1, ±j}`) and per-
    //     axis noise variance `σ²_n` is:
    //
    //         LLR(b1) = −A·(re + im) / σ²_n
    //         LLR(b0) = A·(max(im,−re) − max(re,−im)) / σ²_n
    //
    //     `qpsk_llrs(r)` returns the unit-amplitude form (= the
    //     bracketed expression without the `A`), so the scale that
    //     turns it into a true likelihood-ratio LLR is
    //     `A / σ²_n` — *multiplication* by `A`, not division.
    //
    //     Without `1/σ²_n`, BP receives over-confident LLRs at
    //     noisy channels (low-rate modes at fixed Eb/N0_info) and
    //     propagates wrong-sign decisions instead of relying on the
    //     parity-correction structure. This was the +1 dB extra
    //     implementation-loss penalty observed for Robust pre-fix.
    let llr_scale = amplitude / sigma_sq_n;
    let mut llrs_channel: Vec<f32> = Vec::with_capacity(n_blocks_u * block_ch_bits);
    let mut data_count = 0usize;
    let mut data_running = 0_usize;
    for i in 0..total_syms {
        // Skip preamble.
        if i < PREAMBLE_LEN {
            continue;
        }
        let rel = i - PREAMBLE_LEN;
        // Pilots are at rel == 0, PILOT_SYMBOL_INTERVAL, 2·PILOT_…, …
        if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
            continue;
        }
        let block_idx = data_running / block_data_syms;
        let phase = lms_phase(lms_coeffs, total_syms_f, &anchor_idx, &anchor_phase, i)
            + block_dd_correction.get(block_idx).copied().unwrap_or(0.0);
        let derot = symbols[i] * Complex32::from_polar(1.0, -phase);
        let (llr_b1, llr_b0) = qpsk_llrs(derot);
        llrs_channel.push(llr_b1 * llr_scale);
        llrs_channel.push(llr_b0 * llr_scale);
        data_count += 1;
        data_running += 1;
        if data_count >= n_data_syms {
            break;
        }
    }
    debug_assert_eq!(llrs_channel.len(), n_blocks_u * block_ch_bits);

    // 6. Block-deinterleave channel LLRs back to per-codeword vectors.
    let llrs_per_block = deinterleave_llr(&llrs_channel, n_blocks_u);

    // 7. De-puncture and LDPC-decode each block.
    let fec = Ldpc240_101;
    let mut decoded_info: Vec<Vec<u8>> = Vec::with_capacity(n_blocks_u);
    for block_llrs in &llrs_per_block {
        let full_llrs = de_puncture_llr(block_llrs, mode);
        let result = fec
            .decode_soft(&full_llrs, fec_opts)
            .ok_or(DecodeError::FecFailed)?;
        decoded_info.push(result.info);
    }

    // 8. Pack info bits back to bytes (12 bytes per block).
    let mut frame_data = Vec::with_capacity(n_blocks_u * INFO_BYTES_PER_BLOCK);
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

    // 9. Unpack header + verify CRC, check layout self-consistency.
    let (header, payload) = unpack_frame(&frame_data).map_err(DecodeError::Crc)?;
    if header.mode != mode || header.block_count as usize != n_blocks_u {
        return Err(DecodeError::LayoutMismatch {
            wanted_mode: mode,
            got_mode: header.mode,
            wanted_blocks: n_blocks,
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

/// QPSK constellation: index `0 → +1+0j`, `1 → 0+1j`, `2 → −1+0j`,
/// `3 → 0−1j`. Matches [`crate::uvpacket::tx`]'s map.
fn qpsk_constellation_point(idx: u8) -> Complex32 {
    match idx & 0x3 {
        0 => Complex32::new(1.0, 0.0),
        1 => Complex32::new(0.0, 1.0),
        2 => Complex32::new(-1.0, 0.0),
        3 => Complex32::new(0.0, -1.0),
        _ => unreachable!(),
    }
}

/// Compute LLRs for the two QPSK bits given a (de-rotated) received
/// complex sample.
///
/// TX maps `(b1, b0)` to constellation index via `pair = (b1<<1) |
/// b0`, then through Gray map `[0, 1, 3, 2]`:
///
/// | (b1, b0) | pair | idx | point   |
/// |---------:|-----:|----:|:--------|
/// | (0, 0)   | 0    | 0   | +1 + 0j |
/// | (0, 1)   | 1    | 1   |  0 + 1j |
/// | (1, 0)   | 2    | 3   |  0 − 1j |
/// | (1, 1)   | 3    | 2   | −1 + 0j |
///
/// Max-log LLR with `LLR > 0 ⇔ bit=1` (BP convention):
///
/// - LLR(b1) ≈ −(re + im)         (×2; absolute scale absorbed by BP)
/// - LLR(b0) ≈ max(im, −re) − max(re, −im)
fn qpsk_llrs(r: Complex32) -> (f32, f32) {
    let re = r.re;
    let im = r.im;
    let llr_b1 = -(re + im);
    let llr_b0 = im.max(-re) - re.max(-im);
    (llr_b1, llr_b0)
}

/// Down-convert audio to complex baseband and matched-filter with the
/// transmit RRC pulse. Returns a `Vec<Complex32>` of length
/// `audio.len() + RRC_LEN − 1` (full convolution).
fn downconvert_and_matched_filter(audio: &[f32], audio_centre_hz: f32) -> Vec<Complex32> {
    let two_pi_fc_dt = 2.0 * PI * audio_centre_hz / SAMPLE_RATE_HZ;
    // Down-conversion: 2·audio·e^{-jωn}. The factor 2 sets the
    // matched-filter output to the unit-amplitude QPSK constellation
    // when the audio peak is 1.
    let mut bb: Vec<Complex32> = Vec::with_capacity(audio.len());
    for (n, &s) in audio.iter().enumerate() {
        let phase = two_pi_fc_dt * n as f32;
        let (sin, cos) = phase.sin_cos();
        bb.push(Complex32::new(2.0 * s * cos, -2.0 * s * sin));
    }
    // Matched filter: convolve with RRC. The 2·ωc image lands at the
    // ±2ωc baseband bands and is well-rejected by the RRC LPF
    // (effective bandwidth ≈ ½·R_s = 600 Hz; image at 3 kHz).
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

/// Generate root-raised-cosine pulse coefficients. Returns
/// `span_syms × samples_per_sym + 1` taps, normalised so `Σ h² = 1`.
/// Identical to the TX pulse so the composite TX·RX response is a
/// raised-cosine (zero-ISI at symbol-rate sampling).
fn rrc_pulse(alpha: f32, span_syms: usize, samples_per_sym: usize) -> Vec<f32> {
    let n = span_syms * samples_per_sym;
    let mut h = vec![0.0_f32; n + 1];
    let center = n as f32 / 2.0;
    for (i, h_i) in h.iter_mut().enumerate() {
        let t = (i as f32 - center) / samples_per_sym as f32;
        *h_i = if t.abs() < 1e-6 {
            1.0 - alpha + 4.0 * alpha / PI
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-6 {
            (alpha / 2.0_f32.sqrt())
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let pi_t = PI * t;
            let four_at = 4.0 * alpha * t;
            ((pi_t * (1.0 - alpha)).sin() + four_at * (pi_t * (1.0 + alpha)).cos())
                / (pi_t * (1.0 - four_at * four_at))
        };
    }
    let norm: f32 = h.iter().map(|x| x * x).sum::<f32>().sqrt();
    if norm > 0.0 {
        for x in h.iter_mut() {
            *x /= norm;
        }
    }
    h
}

/// Correlate the 31-bit BPSK preamble against the matched-filter
/// output starting at sample `offset`. Each preamble bit (`true →
/// −1`, `false → +1`) is multiplied by the matched-filter sample at
/// `offset + i·NSPS`. Result is the complex inner product (so its
/// argument carries the carrier-phase reference).
fn preamble_correlation(mf_out: &[Complex32], offset: usize) -> Complex32 {
    let mut acc = Complex32::new(0.0, 0.0);
    for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
        let pos = offset + i * NSPS;
        let s = if b { -1.0_f32 } else { 1.0_f32 };
        acc += mf_out[pos] * s;
    }
    acc
}

/// **Normalised** correlation score, used for sync detection: the
/// ratio of the coherent sum's squared magnitude to the sum of
/// per-sample energies.
///
/// `score = |Σ sᵢ·bᵢ|² / Σ|sᵢ|²`
///
/// By Cauchy-Schwarz, this ratio is bounded above by `PREAMBLE_LEN
/// = 31`. The bound is saturated **only** when `sᵢ ∝ b̄ᵢ` for all i
/// — the defining signature of a coherent BPSK preamble. For pure
/// noise or for a single dominant sample (impulsive interference),
/// the ratio collapses to `~1`, regardless of the absolute magnitude
/// of the dominant sample.
///
/// This is the discriminator that separates a real 31-bit preamble
/// from a microphone click (the latter has |·|² huge but coherence
/// ratio = 1). Replacing `|acc|²` as the sync score eliminates the
/// false-sync class observed in uvpacket-web field reports
/// (`max/median = 139` from a single mic impulse vs theoretical
/// `≤ 17` for noise).
fn preamble_coherence_score(mf_out: &[Complex32], offset: usize) -> f32 {
    let mut acc = Complex32::new(0.0, 0.0);
    let mut energy = 0.0_f32;
    for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
        let pos = offset + i * NSPS;
        let s = mf_out[pos];
        let sign = if b { -1.0_f32 } else { 1.0_f32 };
        acc += s * sign;
        energy += s.norm_sqr();
    }
    if energy <= 0.0 {
        0.0
    } else {
        acc.norm_sqr() / energy
    }
}

/// Unwrap `new` to lie within ±π of `prev`.
fn unwrap_phase(prev: f32, new: f32) -> f32 {
    let mut delta = new - prev;
    while delta > PI {
        delta -= 2.0 * PI;
    }
    while delta < -PI {
        delta += 2.0 * PI;
    }
    prev + delta
}

/// Linear interpolation of the matched-filter output at a fractional
/// sample position. The position must lie within `[0,
/// mf_out.len() − 1)`; callers are responsible for the bounds check.
fn sample_mf_lerp(mf_out: &[Complex32], pos: f32) -> Complex32 {
    let p_int = pos.floor() as usize;
    let alpha = pos - p_int as f32;
    mf_out[p_int] * (1.0 - alpha) + mf_out[p_int + 1] * alpha
}

/// Solve a 3×3 linear system `A·x = b` by Cramer's rule. Returns
/// `None` if the determinant is too close to zero (degenerate or
/// near-degenerate anchor distribution).
fn solve_3x3(a: &[[f32; 3]; 3], b: &[f32; 3]) -> Option<[f32; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-9 {
        return None;
    }
    let det_x = b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]);
    let det_y = a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]);
    let det_z = a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    Some([det_x / det, det_y / det, det_z / det])
}

/// Evaluate the LMS-fitted quadratic phase at symbol index `sym_idx`,
/// or fall back to linear interpolation if the LMS fit was not
/// available (too few anchors).
fn lms_phase(
    coeffs: Option<[f32; 3]>,
    total_syms_f: f32,
    anchor_idx: &[usize],
    anchor_phase: &[f32],
    sym_idx: usize,
) -> f32 {
    if let Some(c) = coeffs {
        let t = sym_idx as f32 / total_syms_f;
        c[0] + c[1] * t + c[2] * t * t
    } else {
        interp_phase(anchor_idx, anchor_phase, sym_idx)
    }
}

/// Linearly interpolate the per-symbol phase between phase anchors.
/// `anchor_idx` is sorted ascending. For `sym_idx` outside the
/// covered range, extrapolates flat (= clamp to nearest endpoint).
fn interp_phase(anchor_idx: &[usize], anchor_phase: &[f32], sym_idx: usize) -> f32 {
    debug_assert_eq!(anchor_idx.len(), anchor_phase.len());
    debug_assert!(!anchor_idx.is_empty());
    if sym_idx <= anchor_idx[0] {
        return anchor_phase[0];
    }
    let last = anchor_idx.len() - 1;
    if sym_idx >= anchor_idx[last] {
        return anchor_phase[last];
    }
    // Find the segment [k, k+1] that brackets sym_idx.
    let mut k = 0;
    while k + 1 < anchor_idx.len() && anchor_idx[k + 1] < sym_idx {
        k += 1;
    }
    let i0 = anchor_idx[k] as f32;
    let i1 = anchor_idx[k + 1] as f32;
    let p0 = anchor_phase[k];
    let p1 = anchor_phase[k + 1];
    let t = (sym_idx as f32 - i0) / (i1 - i0);
    p0 + t * (p1 - p0)
}

// ────────────────────────────────────────────────────────────────────
// Auto-detecting decoder
// ────────────────────────────────────────────────────────────────────

/// Threshold (relative to the global preamble-correlation peak) for
/// considering an offset a candidate frame head. Tuned for clean and
/// moderate-SNR conditions; Phase 2 will revisit.
const PREAMBLE_PEAK_REL_THRESHOLD: f32 = 0.5;

/// Hard sync-rejection threshold: the global preamble-correlation peak
/// must be at least this multiple of the score-distribution median to
/// be considered a real preamble.
///
/// On pure Gaussian noise the score distribution is χ²(2)-like
/// (preamble correlation = sum of 31 ± noise samples) and `max/median`
/// follows extreme-value statistics — `≤ ln(N)/ln(2) ≈ 17` for the
/// `N ≈ 80 k` offsets in a 7 s buffer, with comfortable variance.
///
/// On real signal at +1 dB Eb/N0_info — mfsk-core's lowest 50%-PER
/// decoding threshold (Robust mode, AWGN) — the ratio is ≈ 56. Setting
/// the gate at 20 rejects pure noise reliably without rejecting any
/// signal the FEC could plausibly recover (the signal floor for the
/// gate is `−3.5 dB`, 4.5 dB below the FEC's own decoding threshold).
///
/// The point of this gate is **not** to discriminate signal/noise as
/// the decoder eventually does — it's to prevent the
/// `(picked_peaks × 4 modes × 32 n_blocks)` LDPC BP+OSD-2 sweep from
/// running on noise-only buffers, which can take 30+ s in release for
/// a 7 s window.
const SYNC_PEAK_REL_TO_MEDIAN: f32 = 20.0;

/// Auto-detecting receiver: scan audio for preamble correlations,
/// attempt [`decode_known_layout`] across the (mode × n_blocks) grid
/// for each candidate head, return the successful frames.
pub fn decode(audio: &[f32], audio_centre_hz: f32) -> Vec<DecodedFrame> {
    if audio.len() < PREAMBLE_LEN * NSPS + RRC_LEN {
        return Vec::new();
    }
    let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz);
    // Compute |⟨preamble, mf_out⟩|² across every starting offset that
    // can fit a full preamble correlation.
    let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
    if max_corr_offset == 0 {
        return Vec::new();
    }
    // Coherence score per offset — bounded by `PREAMBLE_LEN = 31`,
    // saturated only by a true BPSK preamble. Replaces the raw `|acc|²`
    // score that was vulnerable to single-impulse mic noise (one
    // dominant sample → ratio = 2200+ false sync).
    let mut scores = vec![0.0f32; max_corr_offset];
    for (offset, slot) in scores.iter_mut().enumerate() {
        *slot = preamble_coherence_score(&mf_out, offset);
    }
    let global_max = scores.iter().cloned().fold(0.0f32, f32::max);
    if global_max <= 0.0 {
        return Vec::new();
    }
    // Sync gate — see SYNC_PEAK_REL_TO_MEDIAN.
    if !global_max_is_sync_outlier(&scores, global_max) {
        return Vec::new();
    }
    let threshold = global_max * PREAMBLE_PEAK_REL_THRESHOLD;

    // Pick local maxima above threshold, ±NSPS-NMS.
    let mut peaks: Vec<(usize, f32)> = scores
        .iter()
        .enumerate()
        .filter(|(_, s)| **s >= threshold)
        .map(|(i, &s)| (i, s))
        .collect();
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let mut picked: Vec<usize> = Vec::new();
    for (offset, _) in peaks {
        if picked.iter().all(|&p| offset.abs_diff(p) > NSPS) {
            picked.push(offset);
        }
    }
    picked.sort_unstable();

    // For each candidate matched-filter offset, the implied audio
    // sample offset (start of the burst) is `mf_off − SYM_PEAK_OFFSET`.
    let mut frames: Vec<DecodedFrame> = Vec::new();
    let mut consumed_until: usize = 0;
    for mf_off in picked {
        let Some(audio_off) = mf_off.checked_sub(SYM_PEAK_OFFSET) else {
            continue;
        };
        if audio_off < consumed_until {
            continue;
        }
        // Try every (mode, n_blocks) — first success wins. To keep
        // cost bounded, iterate n_blocks descending so a successful
        // decode of the largest-fit frame consumes the whole burst.
        let mut decoded: Option<DecodedFrame> = None;
        let mut consumed_end = audio_off;
        'outer: for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            for n_blocks in (1u8..=32).rev() {
                let needed = needed_samples_for(mode, n_blocks);
                if audio_off + needed > audio.len() {
                    continue;
                }
                if let Ok(f) =
                    decode_known_layout(audio, audio_off, audio_centre_hz, mode, n_blocks)
                {
                    decoded = Some(f);
                    consumed_end = audio_off + needed;
                    break 'outer;
                }
            }
        }
        if let Some(f) = decoded {
            frames.push(f);
            consumed_until = consumed_end;
        }
    }

    frames
}

/// Diagnostic: peak / median / ratio of the preamble-correlation
/// score distribution at a given audio centre. Lets callers (e.g. the
/// uvpacket-web PWA) verify whether the sync gate is firing on their
/// real audio without having to instrument the decoder itself.
#[derive(Copy, Clone, Debug)]
pub struct SyncStats {
    pub global_max: f32,
    pub median: f32,
    /// `global_max / median`. Values ≤ ~17 are consistent with pure
    /// χ²(2) noise (extreme value statistics over the offset count);
    /// values ≥ 20 trip the sync gate; real signals at +1 dB Robust
    /// reach ≥ 56.
    pub ratio: f32,
    /// Number of correlation offsets sampled (≈ audio.len() − preamble).
    pub n_scores: usize,
}

/// Compute [`SyncStats`] for a given audio buffer at a known centre.
/// Same matched-filter + preamble correlation + median statistic that
/// [`decode`] uses for its sync gate, but returned to the caller
/// without running the per-peak LDPC sweep.
pub fn diag_sync_stats(audio: &[f32], audio_centre_hz: f32) -> SyncStats {
    if audio.len() < PREAMBLE_LEN * NSPS + RRC_LEN {
        return SyncStats {
            global_max: 0.0,
            median: 0.0,
            ratio: 0.0,
            n_scores: 0,
        };
    }
    let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz);
    let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
    if max_corr_offset == 0 {
        return SyncStats {
            global_max: 0.0,
            median: 0.0,
            ratio: 0.0,
            n_scores: 0,
        };
    }
    let mut scores = vec![0.0f32; max_corr_offset];
    for (offset, slot) in scores.iter_mut().enumerate() {
        *slot = preamble_coherence_score(&mf_out, offset);
    }
    let global_max = scores.iter().cloned().fold(0.0f32, f32::max);
    // Median over non-zero scores only (see `global_max_is_sync_outlier`).
    let mut nz: Vec<f32> = scores.iter().copied().filter(|&s| s > 0.0).collect();
    let median = if nz.is_empty() {
        0.0
    } else {
        let mid = nz.len() / 2;
        nz.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap());
        nz[mid]
    };
    let ratio = if median > 0.0 {
        global_max / median
    } else {
        0.0
    };
    SyncStats {
        global_max,
        median,
        ratio,
        n_scores: max_corr_offset,
    }
}

/// Returns `true` iff `global_max` is plausibly a real preamble peak
/// — i.e. far enough above the score-distribution median that it can't
/// be the natural extreme value of pure noise.
///
/// Uses `select_nth_unstable` for O(N) median (no full sort). Filters
/// exact-zero scores out of the median input; otherwise a partial-fill
/// ring buffer (e.g. the first few seconds after a fresh ▶ Listen)
/// pulls the median to 0 and lets noise through. If every score is
/// zero, no preamble is plausible and we reject.
fn global_max_is_sync_outlier(scores: &[f32], global_max: f32) -> bool {
    let mut nz: Vec<f32> = scores.iter().copied().filter(|&s| s > 0.0).collect();
    if nz.is_empty() {
        return false;
    }
    let mid = nz.len() / 2;
    nz.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap());
    let median = nz[mid];
    if median <= 0.0 {
        return false;
    }
    global_max >= SYNC_PEAK_REL_TO_MEDIAN * median
}

/// Compute the audio sample count required for a given (mode,
/// n_blocks). Mirrors the formula in [`decode_known_layout`].
fn needed_samples_for(mode: Mode, n_blocks: u8) -> usize {
    let block_ch_bits = mode.ch_bits_per_block();
    let n_data_syms = (n_blocks as usize) * block_ch_bits / 2;
    let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
    let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
    total_syms * NSPS + RRC_LEN
}

// ────────────────────────────────────────────────────────────────────
// Multi-channel SSB: decode_multichannel + slot-survey helpers
// ────────────────────────────────────────────────────────────────────

/// Configuration for [`decode_multichannel`] and
/// [`measure_slot_energies`].
///
/// On a shared SSB channel (e.g., 430.090 MHz USB) carrying
/// multiple uvpacket users in the audio passband, this struct
/// describes the search window and the coarse-grid step used to
/// scan for preamble peaks.
#[derive(Clone, Copy, Debug)]
pub struct MultiChannelOpts {
    /// Low edge of the search band (Hz). Typical SSB:
    /// 300 Hz to clear the analog HPF.
    pub band_lo_hz: f32,
    /// High edge of the search band (Hz). Typical SSB:
    /// 2700 Hz for a 2.4 kHz passband.
    pub band_hi_hz: f32,
    /// Coarse-grid step (Hz). 25 Hz matches the AFC search.
    pub coarse_step_hz: f32,
    /// Frequency-axis NMS radius (Hz). Peaks closer than this
    /// in frequency are merged. 600 Hz = half slot spacing for
    /// the typical 1200 Hz slot grid.
    pub nms_radius_hz: f32,
    /// Magnitude threshold for the preamble correlator,
    /// expressed as a fraction of the global maximum across the
    /// scan. 0.5 picks any local peak ≥ 50 % of the strongest.
    pub peak_rel_threshold: f32,
}

impl Default for MultiChannelOpts {
    fn default() -> Self {
        Self {
            band_lo_hz: 300.0,
            band_hi_hz: 2700.0,
            coarse_step_hz: 25.0,
            nms_radius_hz: 600.0,
            peak_rel_threshold: 0.5,
        }
    }
}

/// Per-slot energy report from [`measure_slot_energies`].
#[derive(Clone, Copy, Debug)]
pub struct SlotEnergy {
    /// Slot centre frequency (Hz).
    pub audio_centre_hz: f32,
    /// Mean matched-filter |output|² over the audio body, after
    /// down-conversion at this slot's centre. Higher = more
    /// uvpacket-like signal at this slot. Policy-free: callers
    /// pick a free-vs-busy threshold themselves.
    pub mean_mf_magnitude: f32,
}

/// Decode every uvpacket frame found in `audio` whose audio
/// centre lies in `[mc_opts.band_lo_hz, mc_opts.band_hi_hz]`.
/// Returns `(detected_audio_centre_hz, frame)` pairs.
///
/// The algorithm is a coarse-grid frequency sweep at
/// `coarse_step_hz` (default 25 Hz), running matched-filter +
/// preamble-correlation peak detection at each candidate centre,
/// then frequency-axis NMS to drop adjacent-grid duplicates of
/// the same signal, and finally per-peak `(mode × n_blocks)`
/// decode. The returned `f32` is the picked grid centre — the
/// LMS phase fit inside the per-peak decoder absorbs the
/// ≤ 12.5 Hz residual.
///
/// Cost is dominated by the coarse-grid scan: ~one matched-filter
/// pass over `audio` per grid step (~70 ms for a 1-second buffer
/// at the default 25 Hz step over the default 300–2700 Hz band).
pub fn decode_multichannel(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    fec_opts: &FecOpts,
) -> Vec<(f32, DecodedFrame)> {
    if audio.len() < PREAMBLE_LEN * NSPS + RRC_LEN {
        return Vec::new();
    }
    let mut all_peaks: Vec<(f32, usize, f32)> = Vec::new(); // (f, mf_off, mag2)

    // 1. Coarse-grid scan.
    let mut f = mc_opts.band_lo_hz;
    while f <= mc_opts.band_hi_hz {
        let mf_out = downconvert_and_matched_filter(audio, f);
        let max_off = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * NSPS + 1);
        if max_off > 0 {
            let mut local_max: f32 = 0.0;
            let mut scores: Vec<f32> = Vec::with_capacity(max_off);
            for offset in 0..max_off {
                let m2 = preamble_correlation(&mf_out, offset).norm_sqr();
                scores.push(m2);
                if m2 > local_max {
                    local_max = m2;
                }
            }
            if local_max > 0.0 && global_max_is_sync_outlier(&scores, local_max) {
                let local_thr = local_max * mc_opts.peak_rel_threshold;
                // Time-axis local maxima within this frequency.
                let mut local_peaks: Vec<(usize, f32)> = scores
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| **s >= local_thr)
                    .map(|(i, &s)| (i, s))
                    .collect();
                local_peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
                let mut local_picked: Vec<usize> = Vec::new();
                for (off, _) in &local_peaks {
                    if local_picked.iter().all(|&p| off.abs_diff(p) > NSPS) {
                        local_picked.push(*off);
                    }
                }
                for off in local_picked {
                    all_peaks.push((f, off, scores[off]));
                }
            }
        }
        f += mc_opts.coarse_step_hz;
    }

    if all_peaks.is_empty() {
        return Vec::new();
    }

    // 2. Frequency-axis NMS — drop duplicate detections of the
    //    same signal at adjacent grid points.
    all_peaks.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    let mut kept: Vec<(f32, usize, f32)> = Vec::new();
    for (cf, off, mag) in all_peaks {
        let collides = kept.iter().any(|(kf, koff, _)| {
            (cf - *kf).abs() < mc_opts.nms_radius_hz && off.abs_diff(*koff) < NSPS * 4
        });
        if !collides {
            kept.push((cf, off, mag));
        }
    }

    // 3. Per-peak decode. The coarse grid leaves ≤ 12.5 Hz
    //    residual at the picked centre; the LMS phase fit inside
    //    `decode_known_layout_with_opts` absorbs that, so we use
    //    the non-AFC entry point.
    let mut frames: Vec<(f32, DecodedFrame)> = Vec::new();
    for (cf, mf_off, _) in kept {
        let Some(audio_off) = mf_off.checked_sub(SYM_PEAK_OFFSET) else {
            continue;
        };
        'modes: for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            for n_blocks in (1u8..=32).rev() {
                let needed = needed_samples_for(mode, n_blocks);
                if audio_off + needed > audio.len() {
                    continue;
                }
                if let Ok(frame) =
                    decode_known_layout_with_opts(audio, audio_off, cf, mode, n_blocks, fec_opts)
                {
                    frames.push((cf, frame));
                    break 'modes;
                }
            }
        }
    }

    frames
}

/// Measure the per-slot received-signal energy across the band
/// configured by `mc_opts`, using slot centres at
/// `band_lo_hz + 0.5·slot_spacing`, `band_lo_hz + 1.5·slot_spacing`,
/// … up to `band_hi_hz`. Used by applications as the LBT step
/// before a slotted-ALOHA TX.
///
/// Energy is measured by matched-filtering `audio` at each slot
/// centre and averaging |mf|² over the body. Policy-free: the
/// helper just reports energies. Typical caller pattern:
///
/// ```ignore
/// let slots = measure_slot_energies(&audio, &mc, 1200.0);
/// // band median:
/// let mut mags: Vec<f32> = slots.iter().map(|s| s.mean_mf_magnitude).collect();
/// mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
/// let median = mags[mags.len() / 2];
/// // free if ≤ 3 dB above median:
/// let free: Vec<f32> = slots.iter()
///     .filter(|s| s.mean_mf_magnitude < median * 2.0)
///     .map(|s| s.audio_centre_hz)
///     .collect();
/// ```
///
/// Cost: one matched-filter pass over `audio` per slot. For a
/// 2-slot grid that's effectively free.
pub fn measure_slot_energies(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    slot_spacing_hz: f32,
) -> Vec<SlotEnergy> {
    let mut centres: Vec<f32> = Vec::new();
    let mut f = mc_opts.band_lo_hz + slot_spacing_hz / 2.0;
    while f + slot_spacing_hz / 2.0 <= mc_opts.band_hi_hz {
        centres.push(f);
        f += slot_spacing_hz;
    }
    centres
        .into_iter()
        .map(|f| {
            let mf_out = downconvert_and_matched_filter(audio, f);
            let n = mf_out.len().max(1) as f32;
            let mean_sq: f32 = mf_out.iter().map(|c| c.norm_sqr()).sum::<f32>() / n;
            SlotEnergy {
                audio_centre_hz: f,
                mean_mf_magnitude: mean_sq,
            }
        })
        .collect()
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
            for &b in &decoded.payload[payload.len()..] {
                assert_eq!(b, 0, "{mode:?} non-zero padding byte");
            }
        }
    }

    /// Round-trip a 19-block Standard frame at QSL size (214 byte
    /// payload).
    #[test]
    fn roundtrip_qsl_size_standard() {
        let header = header_for(Mode::Standard, 19, 1, 0);
        let payload: Vec<u8> = (0..214).map(|i| (i ^ 0xAA) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let decoded = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, Mode::Standard, 19).unwrap();
        assert_eq!(&decoded.payload[..214], &payload[..]);
    }

    /// Round-trip a 32-block Robust frame.
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

    /// Audio shorter than the layout demands → `Truncated`.
    #[test]
    fn truncated_audio_is_reported() {
        let header = header_for(Mode::Robust, 4, 1, 0);
        let audio = encode(&header, b"hi", AUDIO_CENTRE_HZ).unwrap();
        let short = &audio[..audio.len() / 2];
        let err = decode_known_layout(short, 0, AUDIO_CENTRE_HZ, Mode::Robust, 4).unwrap_err();
        assert_eq!(err, DecodeError::Truncated);
    }

    /// Decoding as the wrong mode must not silently succeed.
    #[test]
    fn wrong_mode_rejects() {
        let header = header_for(Mode::Robust, 4, 1, 0);
        let audio = encode(&header, b"abc", AUDIO_CENTRE_HZ).unwrap();
        let err = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, Mode::Standard, 4).unwrap_err();
        assert!(
            matches!(
                err,
                DecodeError::FecFailed | DecodeError::Crc(_) | DecodeError::LayoutMismatch { .. }
            ),
            "expected FecFailed / Crc / LayoutMismatch, got {err:?}",
        );
    }

    /// Auto-detecting decoder must find a frame at the start of the
    /// buffer.
    #[test]
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

    /// Tracking test for high-zero-density Fast.
    #[test]
    #[ignore = "Phase 2: Fast-mode LDPC convergence on high-zero-density payloads"]
    fn auto_detect_fast_mode_high_zero_density() {
        let header = header_for(Mode::Fast, 4, 2, 11);
        let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xC3) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 1, "got {} frames", frames.len());
    }

    /// Tracking test for high-zero-density Express.
    #[test]
    #[ignore = "Phase 2: Express-mode LDPC convergence on high-zero-density payloads"]
    fn auto_detect_express_mode_high_zero_density() {
        let header = header_for(Mode::Express, 4, 2, 11);
        let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xC3) as u8).collect();
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let frames = decode(&audio, AUDIO_CENTRE_HZ);
        assert_eq!(frames.len(), 1, "got {} frames", frames.len());
    }

    /// Auto-detect with leading + trailing silence.
    #[test]
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

    /// Two distinct frames back-to-back.
    #[test]
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

    /// Empty / silent audio → no frames.
    #[test]
    fn auto_detect_empty_audio() {
        let frames = decode(&[], AUDIO_CENTRE_HZ);
        assert!(frames.is_empty());
        let frames = decode(&vec![0.0f32; 5000], AUDIO_CENTRE_HZ);
        assert!(frames.is_empty());
    }

    /// QPSK soft-demap sanity: a clean +1+0j sample should give
    /// strongly-negative LLRs (= bit 0) for both bits.
    #[test]
    fn qpsk_llr_clean_constellation_points() {
        let (b1, b0) = qpsk_llrs(Complex32::new(1.0, 0.0));
        assert!(b1 < 0.0 && b0 < 0.0, "+1+0j → ({b1}, {b0})");
        let (b1, b0) = qpsk_llrs(Complex32::new(0.0, 1.0));
        assert!(b1 < 0.0 && b0 > 0.0, "0+1j → ({b1}, {b0})");
        let (b1, b0) = qpsk_llrs(Complex32::new(-1.0, 0.0));
        assert!(b1 > 0.0 && b0 > 0.0, "-1+0j → ({b1}, {b0})");
        let (b1, b0) = qpsk_llrs(Complex32::new(0.0, -1.0));
        assert!(b1 > 0.0 && b0 < 0.0, "0-1j → ({b1}, {b0})");
    }
}
