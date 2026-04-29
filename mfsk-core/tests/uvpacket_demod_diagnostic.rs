// SPDX-License-Identifier: GPL-3.0-or-later
//! Demod-path diagnostic for the QPSK + RRC modem (Phase 2'a).
//!
//! Phase 1 / Phase 2 had a much larger version of this file with
//! per-tone DFT inspections and 4-FSK-specific BER probes; the
//! modulation pivot to coherent QPSK + RRC made those obsolete and
//! they were dropped at Phase 2'a. What remains are the cross-
//! modulation-applicable diagnostics: signal-envelope sanity check
//! and an Eb/N0 threshold finder.

#![cfg(feature = "uvpacket")]

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

mod common;
use common::channel::{AwgnChannel, awgn_sigma_for_eb_n0_info, signal_power};

/// Confirm the synthesised audio's envelope sits in the expected
/// QPSK + RRC range. The burst is no longer constant-envelope: TX
/// peak-normalises to ≤ 1 and RMS sits well below `1/√2` (~7 dB
/// PAPR for RRC-shaped QPSK at α = 0.5).
#[test]
fn audio_envelope_matches_qpsk_rrc_assumptions() {
    let header = FrameHeader {
        mode: Mode::Robust,
        block_count: 4,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..44).map(|i| (i as u8).wrapping_mul(17)).collect();
    let audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
    let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
    let mean_sq = signal_power(&audio);
    let rms = mean_sq.sqrt();
    eprintln!("audio diag: peak={peak:.4}, rms={rms:.4}, mean_sq={mean_sq:.4}");
    assert!(peak <= 1.05, "peak {peak} > 1.05 (TX peak-normalisation broke)");
    // QPSK + RRC + 1500 Hz upconvert: empirically RMS ≈ 0.2 — 0.5
    // depending on payload-driven peak distribution. Anything outside
    // that range signals a TX wiring change.
    assert!(
        (0.10..=0.55).contains(&rms),
        "RMS {rms} outside the expected QPSK+RRC envelope range",
    );
}

/// Threshold-finder: sweep Eb/N0_info per mode, full 4-block frame.
/// Output is the headline data point — at the corrected σ formula
/// (per-burst signal power) Robust should hit 100 % PER somewhere
/// around the QPSK + LDPC theoretical threshold (single-digit dB).
#[test]
#[ignore = "slow: high-SNR threshold sweep"]
fn awgn_threshold_finder_per_mode() {
    let n_trials = 30;
    let n_blocks = 4u8;
    let payload_size = 44; // full frame capacity
    let eb_n0_grid: [f32; 13] =
        [-2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0];

    eprintln!("mode,eb_n0_db,decoded,total");
    for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
        for &eb_n0 in &eb_n0_grid {
            let header = FrameHeader {
                mode,
                block_count: n_blocks,
                app_type: 0,
                sequence: 0,
            };
            let mut decoded = 0;
            for trial in 0..n_trials {
                let payload: Vec<u8> = (0..payload_size)
                    .map(|i| ((i + trial) ^ 0xA5) as u8)
                    .collect();
                let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&audio));
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
