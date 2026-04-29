// SPDX-License-Identifier: GPL-3.0-or-later
//! Demod-path diagnostic: isolate which layer of the audio modem
//! is responsible for the >10 dB SNR-threshold gap observed in
//! Phase 2a between the audio path and the pure-LLR pipeline. Bugs
//! are explicitly **not** ruled out — these tests print
//! intermediate quantities (audio peak / RMS, per-symbol DFT
//! signal-vs-noise magnitudes, per-bit BER from the demod alone,
//! LLR statistics) so the offender can be fingered before any more
//! demod-design rework happens.

#![cfg(feature = "uvpacket")]

use mfsk_core::core::FecOpts;
use mfsk_core::fec::Ldpc240_101;
use mfsk_core::uvpacket::framing::{FrameHeader, INFO_BYTES_PER_BLOCK};
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

mod common;
use common::channel::{AwgnChannel, awgn_sigma_for_eb_n0_info};

/// Confirm that synthesised audio sits where the σ formula assumes:
/// peak ≤ 1.0, RMS ≈ 1/√2.
#[test]
#[ignore = "Phase 1'c: σ formula assumed unit-envelope 4-FSK; QPSK+RRC has different PAPR/RMS"]
fn audio_peak_and_rms_match_assumed_unit_amplitude() {
    let header = FrameHeader {
        mode: Mode::Robust,
        block_count: 4,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..44).map(|i| (i as u8).wrapping_mul(17)).collect();
    let audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
    let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
    let mean_sq: f32 = audio.iter().map(|s| s * s).sum::<f32>() / audio.len() as f32;
    let rms = mean_sq.sqrt();
    eprintln!("audio diag: peak={peak:.4}, rms={rms:.4}, mean_sq={mean_sq:.4}");
    assert!(peak <= 1.05, "peak {peak} exceeds amplitude=1 by more than 5%");
    assert!(
        (0.55..=0.85).contains(&rms),
        "RMS {rms} far off the expected ≈ 0.707",
    );
}

/// Threshold-finder: extend the `awgn_per_sweep` Eb/N0 grid into
/// the high-SNR regime to pin down where each mode actually starts
/// decoding. Output is the headline data point — if Robust's
/// threshold sits at, say, +15 dB then the audio-path penalty is
/// "the textbook non-coherent-vs-coherent loss + GFSK pulse-shape
/// loss" (≈ +14 dB total expected); much higher than that points
/// at a bug.
#[test]
#[ignore = "slow: high-SNR threshold sweep"]
fn awgn_threshold_finder_per_mode() {
    let n_trials = 30;
    let n_blocks = 4u8;
    let payload_size = 44; // full frame capacity
    let eb_n0_grid: [f32; 11] = [10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0];

    eprintln!("mode,eb_n0_db,decoded,total");
    for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
        for &eb_n0 in &eb_n0_grid {
            let header = FrameHeader {
                mode,
                block_count: n_blocks,
                app_type: 0,
                sequence: 0,
            };
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0);
            let mut decoded = 0;
            for trial in 0..n_trials {
                let payload: Vec<u8> = (0..payload_size)
                    .map(|i| ((i + trial) ^ 0xA5) as u8)
                    .collect();
                let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                let mut chan = AwgnChannel::new(sigma, 0xC0FFEE + trial as u64);
                chan.apply(&mut audio);
                if let Ok(frame) = rx::decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, n_blocks)
                    && frame.payload[..payload_size] == payload[..]
                {
                    decoded += 1;
                }
            }
            eprintln!("{mode:?},{eb_n0:+.0},{decoded},{n_trials}");
        }
    }
}

/// Per-symbol DFT signal-vs-noise magnitudes at clean channel and
/// at +10 dB Eb/N0_info, sampled for the Robust mode's first data
/// block. Should show:
/// - Clean: signal-tone magnitude ≈ A·N/2 ≈ 5; noise-tone ≈ small
/// - Noisy: signal magnitude ≈ 5; noise-tone magnitude ≈ √(N·σ²) ≈ 1.7
///
/// If the **clean** signal magnitude is far from 5, the GFSK
/// pulse-shape leakage is bigger than expected and a longer
/// matched filter is the right fix. If the **noisy** signal
/// magnitude is far below the noise floor, the σ formula is wrong.
#[test]
fn per_tone_magnitude_distribution() {
    use std::f32::consts::PI;

    const NSPS: usize = 10;
    const COSTAS_LEN: usize = 4;
    const SAMPLE_RATE: f32 = 12_000.0;
    const TONE_SPACING: f32 = 600.0;

    let header = FrameHeader {
        mode: Mode::Robust,
        block_count: 4,
        app_type: 0,
        sequence: 0,
    };
    let payload = vec![0u8; 44];
    let clean_audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();

    let sigma_10db = awgn_sigma_for_eb_n0_info(Mode::Robust, 10.0);
    eprintln!("σ for +10 dB Eb/N0_info Robust: {sigma_10db:.4}");
    let mut noisy_audio = clean_audio.clone();
    let mut chan = AwgnChannel::new(sigma_10db, 0xDEAD_BEEF);
    chan.apply(&mut noisy_audio);

    // Per-symbol DFT of the head Costas + the first 6 data symbols.
    let f0 = AUDIO_CENTRE_HZ - 1.5 * TONE_SPACING;
    let tone_freqs = [f0, f0 + TONE_SPACING, f0 + 2.0 * TONE_SPACING, f0 + 3.0 * TONE_SPACING];

    let dft_mag = |audio: &[f32], sample_offset: usize, freq: f32| -> f32 {
        let mut re = 0.0_f32;
        let mut im = 0.0_f32;
        for n in 0..NSPS {
            let phase = 2.0 * PI * freq * n as f32 / SAMPLE_RATE;
            let s = audio[sample_offset + n];
            re += s * phase.cos();
            im -= s * phase.sin();
        }
        (re * re + im * im).sqrt()
    };

    eprintln!("\nPer-symbol DFT magnitudes (clean | +10 dB):");
    eprintln!("sym  expected  | tone0  tone1  tone2  tone3 (clean) | tone0  tone1  tone2  tone3 (noisy)");
    let head = 0; // first sample of head Costas = sample 0
    for sym in 0..(COSTAS_LEN + 6) {
        let off = head + sym * NSPS;
        let mags_c: Vec<f32> = (0..4).map(|t| dft_mag(&clean_audio, off, tone_freqs[t])).collect();
        let mags_n: Vec<f32> = (0..4).map(|t| dft_mag(&noisy_audio, off, tone_freqs[t])).collect();
        eprintln!(
            "{sym:3}            | {:5.2}  {:5.2}  {:5.2}  {:5.2} | {:5.2}  {:5.2}  {:5.2}  {:5.2}",
            mags_c[0], mags_c[1], mags_c[2], mags_c[3],
            mags_n[0], mags_n[1], mags_n[2], mags_n[3],
        );
    }
}

/// Direct demod-only BER measurement at a fixed Eb/N0_info: encode
/// → AWGN → demod first LDPC block's data symbols → compute hard
/// bit decisions from LLR sign → re-encode the original info bits
/// to know expected channel bits → count mismatches.
///
/// This isolates the demod from the LDPC. If demod BER is high
/// (≥ 30 %) at moderate Eb/N0, the LDPC has no chance and the
/// demod is the problem.
#[test]
fn demod_only_ber_at_known_eb_n0() {
    use std::f32::consts::PI;

    const NSPS: usize = 10;
    const COSTAS_LEN: usize = 4;
    const SAMPLE_RATE: f32 = 12_000.0;
    const TONE_SPACING: f32 = 600.0;
    const GRAY_4: [u8; 4] = [0, 1, 3, 2];
    const N_LDPC: usize = 240;
    const K_LDPC: usize = 101;

    let mode = Mode::Robust;
    let header = FrameHeader {
        mode,
        block_count: 4,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..44).map(|i| ((i * 31) ^ 0xA5) as u8).collect();
    let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();

    // Reproduce TX-side info bits for block 0.
    // Block 0's 12-byte chunk = first 12 bytes of (header pack output).
    // We re-encode that 12-byte chunk through Ldpc240_101 to get the
    // expected 240 channel bits — no interleaver in this test (we
    // only look at block 0, where interleaver maps the channel bits
    // to TX positions in a known way).
    //
    // Skip the interleaver subtlety: we fetch what `tx::encode` would
    // produce by grabbing the first block's symbols directly and just
    // measure if our DEMOD recovers the same bits.

    let snr_db = 14.0;
    let sigma = awgn_sigma_for_eb_n0_info(mode, snr_db);
    eprintln!("\nDemod-only BER, Robust @ +{snr_db:.0} dB Eb/N0_info, σ = {sigma:.4}");

    let mut chan = AwgnChannel::new(sigma, 0xBEEF_FACE);
    chan.apply(&mut audio);

    // Demod the first 12 data symbols of block 0 to get 24 LLRs.
    // That's a small spot check — enough to see if the demod is
    // making sensible decisions or producing garbage.
    let f0 = AUDIO_CENTRE_HZ - 1.5 * TONE_SPACING;
    let tone_freqs = [f0, f0 + TONE_SPACING, f0 + 2.0 * TONE_SPACING, f0 + 3.0 * TONE_SPACING];

    let dft_mag = |audio: &[f32], sample_offset: usize, freq: f32| -> f32 {
        let mut re = 0.0_f32;
        let mut im = 0.0_f32;
        for n in 0..NSPS {
            let phase = 2.0 * PI * freq * n as f32 / SAMPLE_RATE;
            let s = audio[sample_offset + n];
            re += s * phase.cos();
            im -= s * phase.sin();
        }
        (re * re + im * im).sqrt()
    };

    // Re-encode block 0's expected channel bits by running TX again
    // *without* noise, demod the clean audio, and use that as the
    // ground truth. (This sidesteps having to reproduce the
    // interleaver's permutation by hand.)
    let clean_audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
    let head = 0;
    let n_data_syms = 120; // Robust block: 240 ch bits / 2 b/sym

    // Hard decisions from clean DFT = "ground truth" channel bits.
    let mut clean_bits = Vec::with_capacity(n_data_syms * 2);
    let mut noisy_bits = Vec::with_capacity(n_data_syms * 2);
    for sym in 0..n_data_syms {
        let off = head + (COSTAS_LEN + sym) * NSPS;
        let m_c: [f32; 4] = std::array::from_fn(|t| dft_mag(&clean_audio, off, tone_freqs[t]));
        let m_n: [f32; 4] = std::array::from_fn(|t| dft_mag(&audio, off, tone_freqs[t]));
        // Pick best tone via max magnitude (hard decision).
        let pick = |m: &[f32; 4]| -> usize {
            let (idx, _) = m.iter().enumerate().fold((0_usize, f32::NEG_INFINITY), |acc, (i, &v)| {
                if v > acc.1 { (i, v) } else { acc }
            });
            idx
        };
        let tone_c = pick(&m_c);
        let tone_n = pick(&m_n);
        // Inverse Gray map: tone → bit pair (b1, b0).
        let inv_gray = |t: usize| -> (u8, u8) {
            // GRAY_4[bin] = tone, so bin = inverse:
            //   tone 0 → bin 0 → (0,0); tone 1 → bin 1 → (0,1);
            //   tone 3 → bin 2 → (1,0); tone 2 → bin 3 → (1,1).
            let bin = GRAY_4.iter().position(|&g| g as usize == t).unwrap_or(0);
            (((bin >> 1) & 1) as u8, (bin & 1) as u8)
        };
        let (b1c, b0c) = inv_gray(tone_c);
        let (b1n, b0n) = inv_gray(tone_n);
        clean_bits.push(b1c);
        clean_bits.push(b0c);
        noisy_bits.push(b1n);
        noisy_bits.push(b0n);
    }

    let bit_errors: usize = clean_bits
        .iter()
        .zip(noisy_bits.iter())
        .filter(|(c, n)| c != n)
        .count();
    let total = clean_bits.len();
    let ber = bit_errors as f32 / total as f32;
    eprintln!(
        "block-0 demod-only BER @ +{snr_db:.0} dB: {bit_errors}/{total} = {ber:.4}",
    );
    eprintln!(
        "(Theoretical non-coherent 4-FSK at the corresponding per-symbol SNR is ~few-percent; \
         BER ≫ that points at a demod bug.)",
    );
    let _ = (Ldpc240_101, FecOpts::default(), N_LDPC, K_LDPC, INFO_BYTES_PER_BLOCK);
}
