// SPDX-License-Identifier: GPL-3.0-or-later
//! Modem-layer diagnostic: trace per-mode signal power, per-data-
//! symbol energy, and rx-side amplitude / σ_n estimates against
//! ground-truth σ.
//!
//! Goal: explain why the QPSK end-to-end pipeline gives Robust no
//! advantage over Express despite the LDPC mother code being
//! designed for native rate 0.42. The direct-LDPC sweep
//! (`uvpacket_ldpc_direct.rs`) confirms Robust ≥ Express by ~1 dB
//! at the FEC layer; the QPSK pipeline must be eating that gain.

#![cfg(feature = "uvpacket")]

use std::f32::consts::PI;

use num_complex::Complex32;

use mfsk_core::core::FecCodec;
use mfsk_core::fec::Ldpc240_101;
use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::interleaver::interleave;
use mfsk_core::uvpacket::puncture::puncture;
use mfsk_core::uvpacket::sync_pattern::{
    PILOT_SYMBOL_INTERVAL, PREAMBLE_LEN, UVPACKET_PREAMBLE_BPSK_BITS,
};
use mfsk_core::uvpacket::framing::{HEADER_BYTES, INFO_BYTES_PER_BLOCK, pack_to_size};
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, tx};

const N_LDPC: usize = 240;
const K_LDPC: usize = 101;
const HEADER_BITS: usize = HEADER_BYTES * 8;
const HEADER_CHUNK_BITS: usize = K_LDPC - INFO_BYTES_PER_BLOCK * 8;
const HEADER_SPREAD_PERIOD: usize = 7;
const PAYLOAD_BITS_PER_BLOCK: usize = INFO_BYTES_PER_BLOCK * 8;

/// Reproduce TX-side channel-bit stream (in transmit / interleaved
/// order) for known header + payload. Mirrors `tx::encode` up to the
/// QPSK symbol mapping. Returns the bit vector that the channel
/// transmits.
fn expected_channel_bits(header: &FrameHeader, payload: &[u8]) -> Vec<u8> {
    let mode = header.mode;
    let n_blocks = header.block_count as usize;
    let frame_data_total = n_blocks * INFO_BYTES_PER_BLOCK;
    let frame_data = pack_to_size(header, payload, frame_data_total).unwrap();
    let header_bytes: [u8; HEADER_BYTES] = frame_data[..HEADER_BYTES].try_into().unwrap();
    let mut header_bits = [0u8; HEADER_BITS];
    for (i, bit) in header_bits.iter_mut().enumerate() {
        let byte = header_bytes[i / 8];
        *bit = (byte >> (7 - (i % 8))) & 1;
    }

    let fec = Ldpc240_101;
    let mut info_buf = vec![0u8; K_LDPC];
    let mut codeword_buf = vec![0u8; N_LDPC];
    let mut concat = Vec::with_capacity(n_blocks * N_LDPC);
    for block_idx in 0..n_blocks {
        let chunk = &frame_data[block_idx * INFO_BYTES_PER_BLOCK..(block_idx + 1) * INFO_BYTES_PER_BLOCK];
        for (byte_idx, &byte) in chunk.iter().enumerate() {
            for bit_idx in 0..8 {
                info_buf[byte_idx * 8 + bit_idx] = (byte >> (7 - bit_idx)) & 1;
            }
        }
        let chunk_offset = HEADER_CHUNK_BITS * (block_idx % HEADER_SPREAD_PERIOD);
        for cb in 0..HEADER_CHUNK_BITS {
            let h_idx = chunk_offset + cb;
            info_buf[PAYLOAD_BITS_PER_BLOCK + cb] = if h_idx < HEADER_BITS { header_bits[h_idx] } else { 0 };
        }
        fec.encode(&info_buf, &mut codeword_buf);
        concat.extend_from_slice(&codeword_buf);
    }

    let mut punctured_concat = Vec::with_capacity(n_blocks * mode.ch_bits_per_block());
    for block_idx in 0..n_blocks {
        let cw = &concat[block_idx * N_LDPC..(block_idx + 1) * N_LDPC];
        punctured_concat.extend_from_slice(&puncture(cw, mode));
    }
    interleave(&punctured_concat, n_blocks)
}

mod common;
use common::channel::{AwgnChannel, awgn_sigma_for_eb_n0_info, signal_power};

const ALL_MODES: [Mode; 4] = [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express];

/// Per-mode signal-power audit. Encodes the same payload across all
/// four modes, prints:
///   - total burst sample count
///   - peak / RMS / mean(audio²) of the burst (post-peak-normalisation)
///   - data-symbol count
///   - "energy per data symbol" = mean(audio²) × T_burst / N_data_syms
///   - σ that the harness will inject at +4 dB Eb/N0_info
///
/// If `energy_per_data_symbol` differs across modes by more than ~1 dB,
/// that's a TX-side scaling bug that explains the Robust handicap.
#[test]
fn signal_power_per_data_symbol_audit() {
    let n_blocks = 4u8;
    let payload: Vec<u8> = (0..44).map(|i| ((i ^ 0x5A) & 0xFF) as u8).collect();

    eprintln!(
        "{:>10}  {:>8}  {:>7}  {:>7}  {:>10}  {:>9}  {:>11}  {:>9}",
        "mode", "samples", "peak", "rms", "mean_sq", "n_data", "E/data_sym", "sigma_+4"
    );
    let mut e_per_sym: Vec<f32> = Vec::new();
    for mode in ALL_MODES {
        let header = FrameHeader { mode, block_count: n_blocks, app_type: 0, sequence: 0 };
        let audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
        let mean_sq = signal_power(&audio);
        let rms = mean_sq.sqrt();
        let n_data = (n_blocks as usize) * mode.ch_bits_per_block() / 2;
        // E per symbol (proportional): total energy / n_data symbols.
        let total_e = mean_sq * audio.len() as f32;
        let e_sym = total_e / n_data as f32;
        let sigma = awgn_sigma_for_eb_n0_info(mode, 4.0, mean_sq);
        eprintln!(
            "{:>10?}  {:>8}  {:>7.4}  {:>7.4}  {:>10.5}  {:>9}  {:>11.5}  {:>9.4}",
            mode,
            audio.len(),
            peak,
            rms,
            mean_sq,
            n_data,
            e_sym,
            sigma
        );
        e_per_sym.push(e_sym);
    }
    let max_e = e_per_sym.iter().fold(0.0_f32, |a, &x| a.max(x));
    let min_e = e_per_sym.iter().fold(f32::INFINITY, |a, &x| a.min(x));
    eprintln!(
        "\nE/data_sym spread: {:.2} dB (max/min = {:.3})",
        10.0 * (max_e / min_e).log10(),
        max_e / min_e,
    );
}

const SAMPLE_RATE: f32 = 12_000.0;
const NSPS: usize = 10;
const RRC_SPAN_SYMS: usize = 6;
const RRC_ALPHA: f32 = 0.5;
const RRC_LEN: usize = RRC_SPAN_SYMS * NSPS + 1;
const SYM_PEAK_OFFSET: usize = RRC_LEN - 1;

fn rrc_pulse() -> Vec<f32> {
    let n = RRC_SPAN_SYMS * NSPS;
    let mut h = vec![0.0_f32; n + 1];
    let center = n as f32 / 2.0;
    for (i, h_i) in h.iter_mut().enumerate() {
        let t = (i as f32 - center) / NSPS as f32;
        *h_i = if t.abs() < 1e-6 {
            1.0 - RRC_ALPHA + 4.0 * RRC_ALPHA / PI
        } else if (t.abs() - 1.0 / (4.0 * RRC_ALPHA)).abs() < 1e-6 {
            (RRC_ALPHA / 2.0_f32.sqrt())
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * RRC_ALPHA)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * RRC_ALPHA)).cos())
        } else {
            let pi_t = PI * t;
            let four_at = 4.0 * RRC_ALPHA * t;
            ((pi_t * (1.0 - RRC_ALPHA)).sin() + four_at * (pi_t * (1.0 + RRC_ALPHA)).cos())
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

fn down_mf(audio: &[f32]) -> Vec<Complex32> {
    let two_pi_fc_dt = 2.0 * PI * AUDIO_CENTRE_HZ / SAMPLE_RATE;
    let mut bb: Vec<Complex32> = Vec::with_capacity(audio.len());
    for (n, &s) in audio.iter().enumerate() {
        let phase = two_pi_fc_dt * n as f32;
        let (sin, cos) = phase.sin_cos();
        bb.push(Complex32::new(2.0 * s * cos, -2.0 * s * sin));
    }
    let h = rrc_pulse();
    let n_out = bb.len() + h.len() - 1;
    let mut out = vec![Complex32::new(0.0, 0.0); n_out];
    for (i, &x) in bb.iter().enumerate() {
        for (j, &t) in h.iter().enumerate() {
            out[i + j] += x * t;
        }
    }
    out
}

/// Audit rx-side amplitude / σ_n estimators against the truth.
/// Generates a burst, adds AWGN at known σ_true, runs the rx demod
/// up to the symbol-extraction stage, and prints:
///   - σ_true (the σ injected by the harness, per audio sample)
///   - σ_n_axis_true (= σ_true × matched-filter gain for unit-RRC)
///   - amplitude estimate `A_hat` (from preamble + pilots)
///   - σ_n_hat estimate (from preamble + pilot residuals)
///   - data-symbol-only σ_n (computed by hard-decision residual,
///     to see if data symbols carry more noise than pilots)
///
/// If A_hat differs across modes, we have an amplitude bias. If
/// σ_n_hat is smaller than σ_n_data, the LLR-scale we compute
/// from pilots/preamble is over-confident for data — and that's a
/// systematic bias against the highest-σ (= lowest-rate) mode.
#[test]
fn rx_estimator_audit_at_eb_n0_4db() {
    let n_blocks = 4u8;
    let payload: Vec<u8> = (0..44).map(|i| ((i ^ 0x5A) & 0xFF) as u8).collect();
    let eb_n0 = 4.0_f32;

    eprintln!(
        "{:>10}  {:>9}  {:>9}  {:>9}  {:>9}  {:>9}  {:>9}",
        "mode", "sigma_in", "sigma_mf", "A_hat", "sn_hat", "sn_data", "ratio"
    );
    for mode in ALL_MODES {
        let header = FrameHeader { mode, block_count: n_blocks, app_type: 0, sequence: 0 };
        let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let sp = signal_power(&audio);
        let sigma_in = awgn_sigma_for_eb_n0_info(mode, eb_n0, sp);
        // After matched filtering with unit-norm RRC, AWGN per-axis
        // variance at the matched-filter output is σ² × Σh² × 2 (the
        // ×2 is from the down-convert factor of 2). Σh² = 1 for our
        // RRC, so σ_n_axis_true = sqrt(2) × σ_in × ... actually the
        // exact relationship depends on the down-convert + MF chain;
        // we just print σ_in and let the absolute number speak.
        AwgnChannel::new(sigma_in, 0xCAFE_BABE).apply(&mut audio);

        let mf = down_mf(&audio);

        // Re-run the rx's preamble correlation jitter search.
        let radius = NSPS as isize;
        let base = SYM_PEAK_OFFSET as isize;
        let mut best_off = SYM_PEAK_OFFSET;
        let mut best_corr = Complex32::new(0.0, 0.0);
        let mut best_m2 = -1.0_f32;
        for j in -radius..=radius {
            let off = base + j;
            if off < 0 {
                continue;
            }
            let off = off as usize;
            if off + (PREAMBLE_LEN - 1) * NSPS >= mf.len() {
                continue;
            }
            let mut acc = Complex32::new(0.0, 0.0);
            for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
                let s = if b { -1.0_f32 } else { 1.0 };
                acc += mf[off + i * NSPS] * s;
            }
            if acc.norm_sqr() > best_m2 {
                best_m2 = acc.norm_sqr();
                best_corr = acc;
                best_off = off;
            }
        }

        // Symbol extraction.
        let block_ch_bits = mode.ch_bits_per_block();
        let n_data_syms = (n_blocks as usize) * block_ch_bits / 2;
        let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
        let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
        let mut symbols: Vec<Complex32> = Vec::with_capacity(total_syms);
        for i in 0..total_syms {
            symbols.push(mf[best_off + i * NSPS]);
        }

        // Phase anchors: preamble centre + each pilot.
        let pilot_ref = Complex32::new(1.0, 0.0);
        let preamble_centre = (PREAMBLE_LEN - 1) / 2;
        let mut anchor_idx = vec![preamble_centre];
        let mut anchor_phase = vec![best_corr.arg()];
        for k in 0..n_pilots {
            let pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
            if pos >= total_syms {
                break;
            }
            let r = symbols[pos];
            let phase = (r * pilot_ref.conj()).arg();
            let prev = *anchor_phase.last().unwrap();
            let mut delta = phase - prev;
            while delta > PI {
                delta -= 2.0 * PI;
            }
            while delta < -PI {
                delta += 2.0 * PI;
            }
            anchor_idx.push(pos);
            anchor_phase.push(prev + delta);
        }

        let interp = |idx: usize| -> f32 {
            if idx <= anchor_idx[0] {
                return anchor_phase[0];
            }
            let last = anchor_idx.len() - 1;
            if idx >= anchor_idx[last] {
                return anchor_phase[last];
            }
            let mut k = 0;
            while k + 1 < anchor_idx.len() && anchor_idx[k + 1] < idx {
                k += 1;
            }
            let i0 = anchor_idx[k] as f32;
            let i1 = anchor_idx[k + 1] as f32;
            let p0 = anchor_phase[k];
            let p1 = anchor_phase[k + 1];
            let t = (idx as f32 - i0) / (i1 - i0);
            p0 + t * (p1 - p0)
        };

        // Estimate A from preamble + pilots.
        let mut a_acc = 0.0_f32;
        let mut a_norm = 0.0_f32;
        let mut known: Vec<(Complex32, Complex32)> = Vec::new();
        for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
            let exp = Complex32::new(if b { -1.0 } else { 1.0 }, 0.0);
            let derot = symbols[i] * Complex32::from_polar(1.0, -interp(i));
            a_acc += derot.re * exp.re + derot.im * exp.im;
            a_norm += exp.norm_sqr();
            known.push((derot, exp));
        }
        for k in 0..n_pilots {
            let pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
            if pos >= total_syms {
                break;
            }
            let derot = symbols[pos] * Complex32::from_polar(1.0, -interp(pos));
            a_acc += derot.re * pilot_ref.re + derot.im * pilot_ref.im;
            a_norm += pilot_ref.norm_sqr();
            known.push((derot, pilot_ref));
        }
        let a_hat = a_acc / a_norm;
        let mut sn_sq = 0.0_f32;
        for &(d, e) in &known {
            let r = d - e * a_hat;
            sn_sq += r.norm_sqr();
        }
        let sn_hat = (sn_sq / known.len() as f32 / 2.0).sqrt();

        // Hard-decide each data symbol after de-rotation/scaling, then
        // measure residual variance — this is the "true" σ_n on data.
        let mut data_sn_sq = 0.0_f32;
        let mut data_count = 0usize;
        for i in PREAMBLE_LEN..total_syms {
            let rel = i - PREAMBLE_LEN;
            if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
                continue;
            }
            let derot = symbols[i] * Complex32::from_polar(1.0, -interp(i));
            // Hard decide: closest QPSK constellation point at amplitude a_hat.
            let candidates = [
                Complex32::new(a_hat, 0.0),
                Complex32::new(0.0, a_hat),
                Complex32::new(-a_hat, 0.0),
                Complex32::new(0.0, -a_hat),
            ];
            let (_, best) = candidates
                .iter()
                .map(|c| ((derot - c).norm_sqr(), *c))
                .fold((f32::INFINITY, candidates[0]), |(b, bc), (d, c)| {
                    if d < b {
                        (d, c)
                    } else {
                        (b, bc)
                    }
                });
            let r = derot - best;
            data_sn_sq += r.norm_sqr();
            data_count += 1;
            if data_count >= n_data_syms {
                break;
            }
        }
        let sn_data = (data_sn_sq / data_count as f32 / 2.0).sqrt();

        eprintln!(
            "{:>10?}  {:>9.5}  {:>9.5}  {:>9.5}  {:>9.5}  {:>9.5}  {:>9.4}",
            mode,
            sigma_in,
            sigma_in, // for symmetry; MF gain depends on impl details
            a_hat,
            sn_hat,
            sn_data,
            sn_data / sn_hat
        );
    }
}

/// Demod-only BER measurement: encode a known frame, add AWGN at a
/// known Eb/N0_info, run the demod up to channel-bit hard decisions
/// (LLR sign), compare to ground-truth interleaved channel bits.
///
/// Reports per-mode demod-only BER averaged over many trials.
/// Theoretical QPSK BER at +4 dB Eb/N0_info, rate `r`:
///   Es/N0 = Eb/N0_info + 10·log10(2·r) → BER = Q(sqrt(2·Es/N0))
///   - Robust (r=0.42): Es/N0 = +3.2 dB → BER ≈ 5e-3
///   - Express (r=0.75): Es/N0 = +5.7 dB → BER ≈ 5e-4
///
/// If our measured demod BER is **substantially worse** than these,
/// the modem implementation has a real problem.
#[test]
fn demod_only_ber_per_mode() {
    let n_trials = 60;
    let n_blocks = 4u8;
    let payload_size = 44;
    let eb_n0 = 4.0_f32;

    eprintln!(
        "{:>10}  {:>10}  {:>10}  {:>10}",
        "mode", "ber_meas", "ber_thy", "ratio_dB"
    );
    for mode in ALL_MODES {
        let mut total_bits: usize = 0;
        let mut bit_errors: usize = 0;
        for trial in 0..n_trials {
            let payload: Vec<u8> = (0..payload_size)
                .map(|i| ((i + trial) ^ 0xA5) as u8)
                .collect();
            let header = FrameHeader { mode, block_count: n_blocks, app_type: 0, sequence: 0 };
            let truth_bits = expected_channel_bits(&header, &payload);

            let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
            let sp = signal_power(&audio);
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, sp);
            AwgnChannel::new(sigma, 0xC0FFEE + trial as u64).apply(&mut audio);

            // Replicate rx demod up to channel-bit hard decisions.
            let mf = down_mf(&audio);
            let radius = NSPS as isize;
            let base = SYM_PEAK_OFFSET as isize;
            let mut best_off = SYM_PEAK_OFFSET;
            let mut best_corr = Complex32::new(0.0, 0.0);
            let mut best_m2 = -1.0_f32;
            for j in -radius..=radius {
                let off = base + j;
                if off < 0 {
                    continue;
                }
                let off = off as usize;
                if off + (PREAMBLE_LEN - 1) * NSPS >= mf.len() {
                    continue;
                }
                let mut acc = Complex32::new(0.0, 0.0);
                for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
                    let s = if b { -1.0_f32 } else { 1.0 };
                    acc += mf[off + i * NSPS] * s;
                }
                if acc.norm_sqr() > best_m2 {
                    best_m2 = acc.norm_sqr();
                    best_corr = acc;
                    best_off = off;
                }
            }
            let block_ch_bits = mode.ch_bits_per_block();
            let n_data_syms = (n_blocks as usize) * block_ch_bits / 2;
            let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
            let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
            let mut symbols: Vec<Complex32> = Vec::with_capacity(total_syms);
            for i in 0..total_syms {
                symbols.push(mf[best_off + i * NSPS]);
            }
            // Phase anchors: preamble centre + pilots.
            let pilot_ref = Complex32::new(1.0, 0.0);
            let preamble_centre = (PREAMBLE_LEN - 1) / 2;
            let mut anchor_idx = vec![preamble_centre];
            let mut anchor_phase = vec![best_corr.arg()];
            for k in 0..n_pilots {
                let pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
                if pos >= total_syms {
                    break;
                }
                let r = symbols[pos];
                let phase = (r * pilot_ref.conj()).arg();
                let prev = *anchor_phase.last().unwrap();
                let mut delta = phase - prev;
                while delta > PI {
                    delta -= 2.0 * PI;
                }
                while delta < -PI {
                    delta += 2.0 * PI;
                }
                anchor_idx.push(pos);
                anchor_phase.push(prev + delta);
            }
            let interp = |idx: usize| -> f32 {
                if idx <= anchor_idx[0] {
                    return anchor_phase[0];
                }
                let last = anchor_idx.len() - 1;
                if idx >= anchor_idx[last] {
                    return anchor_phase[last];
                }
                let mut k = 0;
                while k + 1 < anchor_idx.len() && anchor_idx[k + 1] < idx {
                    k += 1;
                }
                let i0 = anchor_idx[k] as f32;
                let i1 = anchor_idx[k + 1] as f32;
                let p0 = anchor_phase[k];
                let p1 = anchor_phase[k + 1];
                let t = (idx as f32 - i0) / (i1 - i0);
                p0 + t * (p1 - p0)
            };

            // Per-block DD phase correction (matches rx.rs Pass 4b).
            let block_data_syms = block_ch_bits / 2;
            let mut block_resid = vec![Complex32::new(0.0, 0.0); n_blocks as usize];
            {
                let mut data_running = 0_usize;
                for i in PREAMBLE_LEN..total_syms {
                    let rel = i - PREAMBLE_LEN;
                    if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
                        continue;
                    }
                    let derot = symbols[i] * Complex32::from_polar(1.0, -interp(i));
                    let candidates = [
                        Complex32::new(1.0, 0.0),
                        Complex32::new(0.0, 1.0),
                        Complex32::new(-1.0, 0.0),
                        Complex32::new(0.0, -1.0),
                    ];
                    let (_, best_c) = candidates
                        .iter()
                        .map(|&c| ((derot - c).norm_sqr(), c))
                        .fold((f32::INFINITY, candidates[0]), |(b, bc), (d, c)| {
                            if d < b {
                                (d, c)
                            } else {
                                (b, bc)
                            }
                        });
                    let block_idx = data_running / block_data_syms;
                    block_resid[block_idx] += derot * best_c.conj();
                    data_running += 1;
                    if data_running >= n_data_syms {
                        break;
                    }
                }
            }
            let block_correction: Vec<f32> = block_resid
                .iter()
                .map(|&r| {
                    let n_per_block = block_data_syms as f32;
                    if r.norm() > 0.25 * n_per_block {
                        r.arg()
                    } else {
                        0.0
                    }
                })
                .collect();

            // Hard-decide each data symbol with DDPT correction.
            let mut data_bits: Vec<u8> = Vec::with_capacity(n_data_syms * 2);
            let mut data_count = 0_usize;
            let mut data_running = 0_usize;
            for i in PREAMBLE_LEN..total_syms {
                let rel = i - PREAMBLE_LEN;
                if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) {
                    continue;
                }
                let block_idx = data_running / block_data_syms;
                let phase = interp(i) + block_correction.get(block_idx).copied().unwrap_or(0.0);
                let derot = symbols[i] * Complex32::from_polar(1.0, -phase);
                let llr_b1 = -(derot.re + derot.im);
                let llr_b0 = derot.im.max(-derot.re) - derot.re.max(-derot.im);
                data_bits.push(if llr_b1 > 0.0 { 1 } else { 0 });
                data_bits.push(if llr_b0 > 0.0 { 1 } else { 0 });
                data_count += 1;
                data_running += 1;
                if data_count >= n_data_syms {
                    break;
                }
            }
            // Compare against ground truth.
            for (a, b) in truth_bits.iter().zip(data_bits.iter()) {
                if a != b {
                    bit_errors += 1;
                }
                total_bits += 1;
            }
        }
        let ber = bit_errors as f32 / total_bits as f32;
        let r_code = K_LDPC as f32 / mode.ch_bits_per_block() as f32;
        // Theoretical channel BER at this Eb/N0_info, rate r_code:
        //   Eb_ch/N0 = r_code · γ_info_lin
        //   BER = Q(sqrt(2·Eb_ch/N0)) = Q(sqrt(2·r_code·γ_info))
        let gamma = 10f32.powf(eb_n0 / 10.0);
        let q_arg = (2.0 * r_code * gamma).sqrt();
        // Q(x) = 0.5·erfc(x/√2)
        let ber_thy = 0.5 * erfc(q_arg / 2.0_f32.sqrt());
        let ratio_db = if ber_thy > 0.0 { 10.0 * (ber / ber_thy).log10() } else { 0.0 };
        eprintln!("{:>10?}  {:>10.4e}  {:>10.4e}  {:>+10.2}", mode, ber, ber_thy, ratio_db);
    }
}

/// Demod-only BER swept across Eb/N0 to characterise where the
/// implementation loss actually sits.
#[test]
#[ignore = "slow: demod-only BER sweep"]
fn demod_only_ber_sweep() {
    let n_trials = 30;
    let n_blocks = 4u8;
    let payload_size = 44;
    let grid: [f32; 8] = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 14.0, 20.0];

    eprintln!("{:>10} {:>6} {:>10} {:>10} {:>9}", "mode", "eb_n0", "ber_meas", "ber_thy", "loss_dB");
    for mode in ALL_MODES {
        for &eb_n0 in &grid {
            let mut total_bits: usize = 0;
            let mut bit_errors: usize = 0;
            for trial in 0..n_trials {
                let payload: Vec<u8> = (0..payload_size).map(|i| ((i + trial) ^ 0xA5) as u8).collect();
                let header = FrameHeader { mode, block_count: n_blocks, app_type: 0, sequence: 0 };
                let truth_bits = expected_channel_bits(&header, &payload);
                let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                let sp = signal_power(&audio);
                let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, sp);
                AwgnChannel::new(sigma, 0xC0FFEE + trial as u64).apply(&mut audio);

                let mf = down_mf(&audio);
                let radius = NSPS as isize;
                let base = SYM_PEAK_OFFSET as isize;
                let mut best_off = SYM_PEAK_OFFSET;
                let mut best_corr = Complex32::new(0.0, 0.0);
                let mut best_m2 = -1.0_f32;
                for j in -radius..=radius {
                    let off = base + j;
                    if off < 0 { continue; }
                    let off = off as usize;
                    if off + (PREAMBLE_LEN - 1) * NSPS >= mf.len() { continue; }
                    let mut acc = Complex32::new(0.0, 0.0);
                    for (i, &b) in UVPACKET_PREAMBLE_BPSK_BITS.iter().enumerate() {
                        let s = if b { -1.0_f32 } else { 1.0 };
                        acc += mf[off + i * NSPS] * s;
                    }
                    if acc.norm_sqr() > best_m2 {
                        best_m2 = acc.norm_sqr();
                        best_corr = acc;
                        best_off = off;
                    }
                }
                let block_ch_bits = mode.ch_bits_per_block();
                let n_data_syms = (n_blocks as usize) * block_ch_bits / 2;
                let n_pilots = n_data_syms.div_ceil(PILOT_SYMBOL_INTERVAL - 1);
                let total_syms = PREAMBLE_LEN + n_pilots + n_data_syms;
                let mut symbols: Vec<Complex32> = Vec::with_capacity(total_syms);
                for i in 0..total_syms { symbols.push(mf[best_off + i * NSPS]); }
                let pilot_ref = Complex32::new(1.0, 0.0);
                let preamble_centre = (PREAMBLE_LEN - 1) / 2;
                let mut anchor_idx = vec![preamble_centre];
                let mut anchor_phase = vec![best_corr.arg()];
                for k in 0..n_pilots {
                    let pos = PREAMBLE_LEN + k * PILOT_SYMBOL_INTERVAL;
                    if pos >= total_syms { break; }
                    let r = symbols[pos];
                    let phase = (r * pilot_ref.conj()).arg();
                    let prev = *anchor_phase.last().unwrap();
                    let mut delta = phase - prev;
                    while delta > PI { delta -= 2.0 * PI; }
                    while delta < -PI { delta += 2.0 * PI; }
                    anchor_idx.push(pos);
                    anchor_phase.push(prev + delta);
                }
                let interp = |idx: usize| -> f32 {
                    if idx <= anchor_idx[0] { return anchor_phase[0]; }
                    let last = anchor_idx.len() - 1;
                    if idx >= anchor_idx[last] { return anchor_phase[last]; }
                    let mut k = 0;
                    while k + 1 < anchor_idx.len() && anchor_idx[k + 1] < idx { k += 1; }
                    let i0 = anchor_idx[k] as f32; let i1 = anchor_idx[k + 1] as f32;
                    let p0 = anchor_phase[k]; let p1 = anchor_phase[k + 1];
                    let t = (idx as f32 - i0) / (i1 - i0);
                    p0 + t * (p1 - p0)
                };
                // Hard-decide bits.
                let mut data_bits: Vec<u8> = Vec::with_capacity(n_data_syms * 2);
                let mut data_count = 0_usize;
                for i in PREAMBLE_LEN..total_syms {
                    let rel = i - PREAMBLE_LEN;
                    if rel.is_multiple_of(PILOT_SYMBOL_INTERVAL) { continue; }
                    let derot = symbols[i] * Complex32::from_polar(1.0, -interp(i));
                    let llr_b1 = -(derot.re + derot.im);
                    let llr_b0 = derot.im.max(-derot.re) - derot.re.max(-derot.im);
                    data_bits.push(if llr_b1 > 0.0 { 1 } else { 0 });
                    data_bits.push(if llr_b0 > 0.0 { 1 } else { 0 });
                    data_count += 1;
                    if data_count >= n_data_syms { break; }
                }
                for (a, b) in truth_bits.iter().zip(data_bits.iter()) {
                    if a != b { bit_errors += 1; }
                    total_bits += 1;
                }
            }
            let ber = bit_errors as f32 / total_bits as f32;
            let r_code = K_LDPC as f32 / mode.ch_bits_per_block() as f32;
            let gamma = 10f32.powf(eb_n0 / 10.0);
            let q_arg = (2.0 * r_code * gamma).sqrt();
            let ber_thy = 0.5 * erfc(q_arg / 2.0_f32.sqrt());
            // Reverse-solve effective Eb/N0 from measured BER.
            let loss_db = if ber > 1e-7 && ber < 0.5 {
                // Q(x) = ber → x = qinv(ber). Approximate Q^-1 numerically.
                let q_x_eff = qinv(ber);
                let gamma_eff_lin = q_x_eff * q_x_eff / (2.0 * r_code);
                let gamma_eff_db = 10.0 * gamma_eff_lin.log10();
                eb_n0 - gamma_eff_db
            } else {
                0.0
            };
            eprintln!("{:>10?} {:+6.1} {:>10.3e} {:>10.3e} {:+9.2}", mode, eb_n0, ber, ber_thy, loss_db);
        }
    }
}

/// Approximate inverse of Q(x) for x in (1e-8, 0.5). Uses
/// Beasley-Springer-Moro-style rational approximation, only good
/// enough for diagnostic display.
fn qinv(p: f32) -> f32 {
    // Q^-1(p) = √2 · erfcinv(2p). Use bisection (slow but simple).
    let mut lo = 0.0_f32;
    let mut hi = 8.0_f32;
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        let q = 0.5 * erfc(mid / 2.0_f32.sqrt());
        if q > p { lo = mid; } else { hi = mid; }
    }
    0.5 * (lo + hi)
}

/// Inline complementary error function (Abramowitz & Stegun 7.1.26).
fn erfc(x: f32) -> f32 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let y = 1.0
        - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
            + 0.254829592)
            * t
            * (-x * x).exp();
    1.0 - sign * y
}
