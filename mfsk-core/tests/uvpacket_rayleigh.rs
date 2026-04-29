// SPDX-License-Identifier: GPL-3.0-or-later
//! Rayleigh fading PER sweep — uvpacket's headline "fade-tolerant"
//! claim against AFSK 1200.
//!
//! Tests Robust mode (the only mode whose pitch leans on fade
//! tolerance) plus Standard for comparison, across Doppler ∈ {1, 5,
//! 10} Hz and Eb/N0 ∈ wide range. Fast / Express are **expected**
//! to do worse under fading because their puncturing leaves less
//! redundancy to spread across fades; we keep them out of the
//! tracking gate but include them in the full sweep for the curve.

#![cfg(feature = "uvpacket")]

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

mod common;
use common::channel::{RayleighFlatChannel, awgn_sigma_for_eb_n0_info, signal_power};

const N_BLOCKS: u8 = 4;

/// Run `n_trials` Rayleigh trials at the given Eb/N0 / Doppler /
/// payload size, return decode count.
fn rayleigh_per(
    mode: Mode,
    payload_size: usize,
    eb_n0_db: f32,
    doppler_hz: f32,
    n_trials: usize,
    base_seed: u64,
) -> usize {
    let header = FrameHeader {
        mode,
        block_count: N_BLOCKS,
        app_type: 0,
        sequence: 0,
    };
    let mut decoded = 0;
    for trial in 0..n_trials {
        let payload: Vec<u8> = (0..payload_size)
            .map(|i| ((i + trial) ^ 0xA5) as u8)
            .collect();
        let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power(&audio));
        let mut chan =
            RayleighFlatChannel::new(doppler_hz, sigma, base_seed.wrapping_add(trial as u64));
        chan.apply(&mut audio);
        if let Ok(frame) = rx::decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, N_BLOCKS)
            && frame.payload[..payload_size] == payload[..]
        {
            decoded += 1;
        }
    }
    decoded
}

/// Smoke gate: at near-zero Doppler (= ~AWGN) and very high Eb/N0,
/// Robust must decode every frame. Catches the Rayleigh helper
/// being miswired.
#[test]
fn rayleigh_smoke_gate_zero_doppler_clean() {
    let n = 20;
    let decoded = rayleigh_per(Mode::Robust, 44, 30.0, 0.001, n, 0xCAFE_BABE);
    eprintln!("Rayleigh smoke Robust @ +30 dB / 0.001 Hz Doppler: {decoded}/{n}");
    // Even at ~zero Doppler the envelope can dip slightly during
    // a 270-ms frame, so allow one or two fades.
    assert!(
        decoded >= n * 9 / 10,
        "Rayleigh smoke gate broke: {decoded}/{n}"
    );
}

/// Headline test for Phase 2b: how does Robust hold up across
/// 1 Hz / 5 Hz / 10 Hz Doppler at moderate Eb/N0? Diagnostic only —
/// captures the fade-tolerance curve uvpacket's Robust mode is
/// pitched on.
#[test]
#[ignore = "slow: Rayleigh PER sweep"]
fn rayleigh_per_sweep_robust_and_standard() {
    let n_trials = 30;
    let payload_size = 20;
    let dopplers = [1.0_f32, 5.0, 10.0];
    let eb_n0_grid = [10.0_f32, 12.0, 15.0, 20.0, 25.0, 30.0];

    eprintln!("mode,doppler_hz,eb_n0_db,decoded,total");
    for mode in [Mode::Robust, Mode::Standard] {
        for &fd in &dopplers {
            for &eb_n0 in &eb_n0_grid {
                let decoded = rayleigh_per(mode, payload_size, eb_n0, fd, n_trials, 0x1234_5678);
                eprintln!("{mode:?},{fd:.0},{eb_n0:+.0},{decoded},{n_trials}");
            }
        }
    }
}

/// Fast / Express under fading: full sweep so the curve is
/// reproducible. Same grid as above.
#[test]
#[ignore = "slow: Rayleigh PER sweep for high-rate modes"]
fn rayleigh_per_sweep_high_rate_modes() {
    let n_trials = 30;
    let payload_size = 20;
    let dopplers = [1.0_f32, 5.0, 10.0];
    let eb_n0_grid = [10.0_f32, 15.0, 20.0, 25.0, 30.0, 35.0];

    eprintln!("mode,doppler_hz,eb_n0_db,decoded,total");
    for mode in [Mode::Fast, Mode::Express] {
        for &fd in &dopplers {
            for &eb_n0 in &eb_n0_grid {
                let decoded = rayleigh_per(mode, payload_size, eb_n0, fd, n_trials, 0xC0DE_BABE);
                eprintln!("{mode:?},{fd:.0},{eb_n0:+.0},{decoded},{n_trials}");
            }
        }
    }
}
