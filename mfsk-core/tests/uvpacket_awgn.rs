// SPDX-License-Identifier: GPL-3.0-or-later
//! AWGN PER sweep across modes × payload-density × Eb/N0.
//!
//! The sweep test is `#[ignore]` because it runs hundreds of
//! decodes; the quick test is the regression gate.
//!
//! ## Reading the output
//!
//! Each sweep emits CSV-like rows to stderr:
//!
//! ```text
//! mode,density,payload_bytes,eb_n0_db,decoded,total
//! Robust,low,4,-2,12,30
//! Robust,low,4,0,28,30
//! Robust,low,4,2,30,30
//! ...
//! ```
//!
//! Pipe through `2>&1 | grep ',[A-Za-z]'` (or just inspect the
//! captured test output) for the table.

#![cfg(feature = "uvpacket")]

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

mod common;
use common::channel::{AwgnChannel, awgn_sigma_for_eb_n0_info, signal_power};

const ALL_MODES: [Mode; 4] = [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express];

/// Run `n_trials` AWGN trials at the given Eb/N0 and payload size,
/// return how many decoded with a payload-bytes match. Frame layout
/// is fixed at 4 LDPC blocks, audio centre 1700 Hz.
fn awgn_per(
    mode: Mode,
    payload_size: usize,
    eb_n0_db: f32,
    n_trials: usize,
    base_seed: u64,
) -> usize {
    let n_blocks = 4u8;
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
        let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power(&audio));
        let mut chan = AwgnChannel::new(sigma, base_seed.wrapping_add(trial as u64));
        chan.apply(&mut audio);
        if let Ok(frame) = rx::decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, n_blocks)
            && frame.payload[..payload_size] == payload[..]
        {
            decoded += 1;
        }
    }
    decoded
}

/// Smoke gate: at a very high Eb/N0 (= +30 dB; effectively a clean
/// channel) Robust + Standard must decode every frame. Fast and
/// Express are characterised by the `#[ignore]` sweeps; they show
/// non-trivial PER even at high SNR, which is one of the things
/// Phase 2 is in business to measure.
///
/// This gate's job is to catch sweep-infrastructure breakage (σ
/// formula bug, AWGN helper miswired) — not to gate any SNR or
/// PER threshold.
#[test]
fn awgn_smoke_gate_clean_channel_low_rate_modes() {
    let n = 20;
    let full_payload = 44; // 4 blocks × 12 byte − 4 byte header
    for mode in [Mode::Robust, Mode::Standard] {
        let decoded = awgn_per(mode, full_payload, 30.0, n, 0xCAFE_BABE);
        eprintln!("AWGN smoke {mode:?} @ +30 dB / {full_payload}-byte: {decoded}/{n}");
        assert_eq!(decoded, n, "{mode:?}: clean-channel smoke gate broke");
    }
}

/// Tracking gate: Fast / Express at +30 dB (essentially clean) +
/// full-capacity payload. Captures the absolute-best-case PER
/// these modes show through the audio path with the current
/// max-log non-coherent demod and OSD-2 fallback. The number is
/// expected to be ≤ 100 % — not 100 %; treat it as the headline
/// data point Phase 2 will improve.
#[test]
#[ignore = "Phase 2 tracking: high-rate-mode floor at clean channel"]
fn awgn_tracking_high_rate_modes_clean_channel() {
    let n = 60;
    let full_payload = 44;
    for mode in [Mode::Fast, Mode::Express] {
        let decoded = awgn_per(mode, full_payload, 30.0, n, 0xCAFE_BABE);
        eprintln!("AWGN tracking {mode:?} @ +30 dB / {full_payload}-byte: {decoded}/{n}");
    }
}

// Reading note: the audio-modem path is empirically several dB
// less efficient than the pure-LLR pipeline used in
// `puncture::tests::modes_awgn_sweep_uniform_vs_kSR`. The Robust
// BP / OSD threshold there is ~+1 dB Eb/N0_info; through the
// audio modem (Costas + 4-FSK non-coherent demod + max-log LLRs)
// the same 50 % PER point sits noticeably higher. The
// `awgn_per_sweep` ignored test below is the way to find out
// where exactly. Phase 2 will tighten the demod (longer matched
// filter, soft de-spreading) once the curves are in.

/// Full PER sweep: 4 modes × 3 payload densities × 7 Eb/N0 values
/// × 30 trials = 2520 decodes. Emit a CSV-like table to stderr;
/// run with `--include-ignored --nocapture` to see it.
#[test]
#[ignore = "slow: AWGN PER sweep across modes × density × Eb/N0"]
fn awgn_per_sweep() {
    let n_trials = 30;
    let densities = [
        ("low", 4usize), // 4-byte payload, ~58 % info zeros (incl. padding)
        ("med", 20),     // half capacity
        ("high", 44),    // full capacity (4 × 12 − 4 = 44 bytes)
    ];
    let eb_n0_grid = [-2.0_f32, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0];

    eprintln!("mode,density,payload_bytes,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for (density_name, payload_size) in densities {
            for &eb_n0 in &eb_n0_grid {
                let decoded = awgn_per(mode, payload_size, eb_n0, n_trials, 0x1234_5678);
                eprintln!(
                    "{mode:?},{density_name},{payload_size},{eb_n0:+.1},{decoded},{n_trials}",
                );
            }
        }
    }
}

/// Targeted Fast / Express high-zero-density study (Phase 2c
/// tracking from rx::tests). Sweeps payload size at fixed clean
/// channel; quantifies how often the BP+OSD path picks a sibling
/// codeword as zero-padding fraction increases.
#[test]
#[ignore = "slow: high-zero-density edge-case characterisation"]
fn awgn_zero_density_sweep_fast_and_express() {
    let n_trials = 60;
    let eb_n0 = 12.0; // effectively clean
    let n_blocks = 4u8;
    let max_payload = (n_blocks as usize) * 12 - 4;

    eprintln!("mode,payload_bytes,zero_pct,eb_n0_db,decoded,total");
    for mode in [Mode::Fast, Mode::Express] {
        for payload_size in [4_usize, 8, 16, 24, 32, 40, max_payload] {
            let decoded = awgn_per(mode, payload_size, eb_n0, n_trials, 0xDEAD_BEEF);
            // Approximate zero-byte fraction in LDPC info: 4-byte
            // header + payload + zeros in the last block's padding.
            let zero_bytes = max_payload - payload_size;
            let zero_pct = 100.0 * zero_bytes as f32 / (n_blocks as f32 * 12.0);
            eprintln!("{mode:?},{payload_size},{zero_pct:.0},{eb_n0:+.1},{decoded},{n_trials}",);
        }
    }
}
