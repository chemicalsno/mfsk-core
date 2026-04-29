// SPDX-License-Identifier: GPL-3.0-or-later
//! AFC (automatic frequency control) tests for SSB-style carrier
//! offsets.
//!
//! Inject a known frequency offset into the audio (multiply by
//! `e^{j·2π·Δf·n/fs}`'s real part — equivalent to a TX/RX VFO
//! mismatch on SSB), and verify the decoder recovers it.

#![cfg(feature = "uvpacket")]

use std::f32::consts::PI;

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::rx::{AfcOpts, decode_known_layout, decode_known_layout_with_afc};
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, tx};

mod common;
use common::channel::{AwgnChannel, awgn_sigma_for_eb_n0_info, signal_power};

const SAMPLE_RATE: f32 = 12_000.0;

fn default_fec_opts() -> mfsk_core::core::FecOpts<'static> {
    mfsk_core::core::FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
    }
}

/// Apply a frequency offset to a real-valued audio buffer in-place.
/// `audio[n] *= cos(2π·Δf·n/fs)` is **not** what you want for an
/// SSB-style offset — that distorts the spectrum. The standard SSB
/// equivalent is to take the analytic signal (Hilbert transform),
/// multiply by `e^{j·2π·Δf·n/fs}`, and take the real part. For the
/// purposes of this test we use the cheaper "real-multiplier"
/// approximation, which preserves the BPSK symmetry but drops a
/// frequency-mirrored copy at `−Δf` that the matched filter rejects.
/// This is sufficient to validate the AFC's frequency-search logic.
///
/// More accurate SSB-equivalent shift used here: bandpass mix.
/// `audio[n] = audio[n] · cos(2π·Δf·n/fs)` distorts. Instead we use
/// the analytic signal approach manually: down-convert, shift,
/// re-upconvert.
///
/// For this test, simulate a TX-side carrier offset by re-doing
/// the upconvert at `audio_centre_hz + Δf`. We need access to the
/// baseband, so we'll synthesise the burst at the offset centre
/// directly.
fn shift_burst_carrier(audio: &mut [f32], delta_hz: f32) {
    // Approximation: complex up-shift the real signal. For a
    // narrow-band signal centred around f_c, multiplying by
    // cos(2π·Δf·t) shifts both the f_c and -f_c images by ±Δf,
    // creating a 2-image spectrum. The matched filter's lowpass
    // rejects the unwanted image, so for our purposes this is OK
    // as long as |Δf| < f_c (the desired image stays far from DC).
    //
    // For a cleaner shift, do `out = Re{(audio_analytic) · e^{jωΔt}}`.
    // Approximate the analytic signal by Hilbert-transforming the
    // real audio. Costly to do exactly; for this test, we cheat
    // and just multiply by cos — the AFC sees the desired peak in
    // its FFT and ignores the mirror.
    let two_pi = 2.0 * PI * delta_hz / SAMPLE_RATE;
    for (n, s) in audio.iter_mut().enumerate() {
        *s *= (two_pi * n as f32).cos();
    }
}

/// Cleaner SSB-equivalent: re-encode the burst at a shifted carrier
/// frequency. This bypasses the test's need for a Hilbert transform
/// and matches what an SSB TX/RX dial mismatch would actually look
/// like (a real TX at f_TX, a real RX at f_RX = f_TX − Δf, both via
/// SSB = ideal frequency-translating channel).
fn encode_with_offset_audio_centre(
    header: &FrameHeader,
    payload: &[u8],
    audio_centre_hz: f32,
) -> Vec<f32> {
    tx::encode(header, payload, audio_centre_hz).unwrap()
}

/// Diagnostic: print AFC's estimated Δf vs the true injected Δf.
#[test]
#[ignore = "diagnostic, prints AFC frequency-offset accuracy"]
fn afc_diag_estimate_accuracy() {
    use mfsk_core::uvpacket::sync_pattern::{PREAMBLE_LEN, UVPACKET_PREAMBLE_BPSK_BITS};
    let mode = Mode::Robust;
    let n_blocks = 4u8;
    let header = FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();

    eprintln!("Δf_true (Hz)  | AFC search uses these bins;");
    eprintln!("              | each bin = 1200 / fft_n Hz wide");
    let _ = (PREAMBLE_LEN, UVPACKET_PREAMBLE_BPSK_BITS);
    for delta in [
        -200.0_f32, -150.0, -100.0, -50.0, -20.0, 0.0, 20.0, 50.0, 100.0, 150.0, 200.0,
    ] {
        let audio = encode_with_offset_audio_centre(&header, &payload, AUDIO_CENTRE_HZ + delta);
        // Probe AFC's estimate by sweeping decode at known offsets
        // and reporting which one wins. Also try the public API.
        let opts = default_fec_opts();
        let afc = AfcOpts::default();
        let res = decode_known_layout_with_afc(
            &audio,
            0,
            AUDIO_CENTRE_HZ,
            mode,
            n_blocks,
            &opts,
            &afc,
        );
        // Find the Δf at which baseline (no AFC) decode succeeds — that
        // is the "true" centre the AFC should converge to.
        let mut baseline_works_at: Option<f32> = None;
        for try_d in -250..=250 {
            let d = try_d as f32;
            if decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ + d, mode, n_blocks).is_ok() {
                baseline_works_at = Some(d);
                break;
            }
        }
        eprintln!(
            "Δf_true {delta:+7.1} Hz: AFC = {:?}  baseline_centre @ {:?}",
            res.as_ref().map(|_| "✓").map_err(|e| format!("{e:?}")),
            baseline_works_at,
        );
    }
}

#[test]
fn afc_zero_offset_matches_baseline() {
    let mode = Mode::Robust;
    let n_blocks = 4u8;
    let header = FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();
    let audio = encode_with_offset_audio_centre(&header, &payload, AUDIO_CENTRE_HZ);

    let opts = default_fec_opts();
    let afc = AfcOpts::default();
    let res = decode_known_layout_with_afc(&audio, 0, AUDIO_CENTRE_HZ, mode, n_blocks, &opts, &afc)
        .unwrap();
    assert_eq!(res.payload[..payload.len()], payload[..]);
}

/// AFC must recover the frame when the TX and RX dial frequencies
/// differ by ±50/100/150 Hz on a clean channel.
#[test]
fn afc_recovers_clean_channel_offset() {
    let mode = Mode::Robust;
    let n_blocks = 4u8;
    let header = FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();

    for delta in [-150.0_f32, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0] {
        // TX at AUDIO_CENTRE_HZ + delta, RX dialed in at AUDIO_CENTRE_HZ.
        // Simulates: TX dialed in correctly, RX is `delta` Hz off.
        let audio =
            encode_with_offset_audio_centre(&header, &payload, AUDIO_CENTRE_HZ + delta);

        let opts = default_fec_opts();
        let afc = AfcOpts::default();
        let res = decode_known_layout_with_afc(
            &audio,
            0,
            AUDIO_CENTRE_HZ,
            mode,
            n_blocks,
            &opts,
            &afc,
        );
        assert!(
            res.is_ok(),
            "AFC failed at Δf = {delta:+.0} Hz: {:?}",
            res.err()
        );
        let frame = res.unwrap();
        assert_eq!(
            frame.payload[..payload.len()],
            payload[..],
            "Δf = {delta:+.0} Hz: payload mismatch"
        );
    }
}

/// Without AFC, the same offset frames should fail or decode wrong
/// at any non-trivial Δf — confirms the test infrastructure
/// actually injects the offset.
#[test]
fn baseline_decoder_fails_at_offset() {
    let mode = Mode::Robust;
    let n_blocks = 4u8;
    let header = FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();

    // Δf = 100 Hz is well inside SSB-mismatch territory and should
    // break the AFC-less decoder.
    let audio =
        encode_with_offset_audio_centre(&header, &payload, AUDIO_CENTRE_HZ + 100.0);
    let res = decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, n_blocks);
    assert!(
        res.is_err(),
        "AFC-less decoder unexpectedly decoded a +100 Hz-offset burst"
    );
}

/// AFC + AWGN at moderate Eb/N0_info: confirm the threshold
/// curve is roughly Δf-invariant within the search window.
#[test]
fn afc_recovers_offset_at_moderate_snr() {
    let mode = Mode::Robust;
    let n_blocks = 4u8;
    let header = FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 0,
        sequence: 0,
    };
    let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();
    let eb_n0_db = 6.0;
    let n_trials = 10;
    let opts = default_fec_opts();
    let afc = AfcOpts::default();

    for &delta in &[-100.0_f32, 0.0, 100.0] {
        let mut decoded = 0;
        for trial in 0..n_trials {
            let mut audio = encode_with_offset_audio_centre(
                &header,
                &payload,
                AUDIO_CENTRE_HZ + delta,
            );
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power(&audio));
            AwgnChannel::new(sigma, 0xC0FFEE + trial as u64).apply(&mut audio);
            if decode_known_layout_with_afc(
                &audio,
                0,
                AUDIO_CENTRE_HZ,
                mode,
                n_blocks,
                &opts,
                &afc,
            )
            .is_ok()
            {
                decoded += 1;
            }
        }
        eprintln!("AFC @ +6 dB, Δf = {delta:+.0} Hz: {decoded}/{n_trials}");
        assert!(
            decoded >= n_trials * 8 / 10,
            "Δf = {delta} Hz: decoded {decoded}/{n_trials} (expected ≥ 80 %)"
        );
    }
}

// Suppress the lint about the unused helper above; it's there for
// reference / future diagnostic use.
#[allow(dead_code)]
fn _suppress_unused() {
    let mut a = vec![0.0f32; 4];
    shift_burst_carrier(&mut a, 100.0);
}
