// SPDX-License-Identifier: GPL-3.0-or-later
//! Empirical check: does `uvpacket::rx::decode` emit zero frames on
//! pure Gaussian noise input within a reasonable wall-clock budget?
//!
//! Tested with a deliberately small (1 s) audio buffer so the runaway
//! pattern can't peg the CI machine for minutes if the false-sync
//! rejection is missing — at 7 s the same logic would take 30–180 s
//! in release.
//!
//! Run with:
//!     cargo test --features uvpacket --release noise_floor -- --nocapture

#![cfg(feature = "uvpacket")]

use std::time::Instant;

use mfsk_core::uvpacket::rx;

struct Awgn {
    state: u64,
}
impl Awgn {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
        }
    }
    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
}

/// Direct probe: feed various audio scenarios to `diag_sync_stats`
/// and print the ratio. Theoretical max/median for `N≈30k`
/// exponentially-distributed scores is `ln(N)/ln(2) ≈ 14.9`.
#[test]
fn noise_ratio_distribution() {
    use mfsk_core::uvpacket::rx::diag_sync_stats;
    let n_samples = 30_000_usize;
    let two_pi = 2.0 * std::f32::consts::PI;

    // 1. Pure white noise — baseline.
    {
        let mut rng = Awgn::new(0xa1);
        let audio: Vec<f32> = (0..n_samples).map(|_| 0.05 * rng.gaussian()).collect();
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[white]   ratio={:.1}", s.ratio);
    }

    // 2. Noise + tone at carrier (1500 Hz) — DC after downconvert.
    {
        let mut rng = Awgn::new(0xa2);
        let audio: Vec<f32> = (0..n_samples)
            .map(|i| {
                let t = i as f32 / 12_000.0;
                0.005 * (two_pi * 1500.0 * t).sin() + 0.001 * rng.gaussian()
            })
            .collect();
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[tone @ 1500Hz] ratio={:.1}", s.ratio);
    }

    // 3. Noise + 1200 Hz offset tone — symbol-rate alignment.
    {
        let mut rng = Awgn::new(0xa3);
        let audio: Vec<f32> = (0..n_samples)
            .map(|i| {
                let t = i as f32 / 12_000.0;
                0.005 * (two_pi * 1200.0 * t).sin() + 0.001 * rng.gaussian()
            })
            .collect();
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[tone @ 1200Hz] ratio={:.1}", s.ratio);
    }

    // 4. Noise + a single impulse.
    {
        let mut rng = Awgn::new(0xa4);
        let mut audio: Vec<f32> = (0..n_samples).map(|_| 0.001 * rng.gaussian()).collect();
        audio[15_000] = 0.5; // single sample click
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[noise+click]  peak={:.3} ratio={:.1}", 0.5, s.ratio);
    }

    // 5. Noise + 1500 Hz tone gated at 1200 Hz (AM modulation by symbol rate).
    //    This is what would happen if the user's mic picks up some uvpacket-
    //    like or modem-like background.
    {
        let mut rng = Awgn::new(0xa5);
        let audio: Vec<f32> = (0..n_samples)
            .map(|i| {
                let t = i as f32 / 12_000.0;
                let envelope = 0.5 + 0.5 * (two_pi * 1200.0 * t).sin();
                0.005 * envelope * (two_pi * 1500.0 * t).sin() + 0.001 * rng.gaussian()
            })
            .collect();
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[1500Hz AM 1200Hz] ratio={:.1}", s.ratio);
    }

    // 6. Pure signal at 1500 Hz, large amplitude — sanity check
    {
        let audio: Vec<f32> = (0..n_samples)
            .map(|i| {
                let t = i as f32 / 12_000.0;
                0.5 * (two_pi * 1500.0 * t).sin()
            })
            .collect();
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[strong tone 1500Hz] ratio={:.1}", s.ratio);
    }

    // 7. Real preamble at moderate SNR — sanity check the gate's
    //    accept-side. Ratio should be near `PREAMBLE_LEN = 31`.
    {
        use mfsk_core::uvpacket::framing::FrameHeader;
        use mfsk_core::uvpacket::puncture::Mode;
        use mfsk_core::uvpacket::tx;
        let header = FrameHeader {
            mode: Mode::Standard,
            block_count: 4,
            app_type: 1,
            sequence: 0,
        };
        let payload = b"hello world";
        let burst = tx::encode(&header, payload, 1500.0).unwrap();
        let mut rng = Awgn::new(0xa7);
        let sigma = 0.03_f32; // ~+10 dB SNR
        let mut audio: Vec<f32> = vec![0.0; n_samples];
        for (i, &b) in burst.iter().enumerate() {
            if i + 5_000 < audio.len() {
                audio[i + 5_000] = b;
            }
        }
        for s in audio.iter_mut() {
            *s += sigma * rng.gaussian();
        }
        let s = diag_sync_stats(&audio, 1500.0);
        eprintln!("[real preamble] ratio={:.1}", s.ratio);
    }
}

#[test]
fn noise_floor_short_buffer() {
    let n_samples = 12_000; // 1 s at 12 kHz
    let mut rng = Awgn::new(0x12345);
    let sigma = 0.05_f32;
    let audio: Vec<f32> = (0..n_samples).map(|_| sigma * rng.gaussian()).collect();

    let t0 = Instant::now();
    let frames = rx::decode(&audio, 1500.0);
    let elapsed = t0.elapsed();
    eprintln!(
        "[white]   1 s σ={}: {} frames, decode took {:?}",
        sigma,
        frames.len(),
        elapsed
    );
    assert_eq!(frames.len(), 0, "white noise must not produce frames");
    assert!(elapsed.as_millis() < 1500, "white noise took {:?}", elapsed);
}

/// Mic-input-like noise: AWGN + low-frequency 1/f tilt + 50 Hz hum +
/// a 60 Hz hum harmonic. Closer to what a real Mac/PC built-in mic
/// returns in a "quiet room" — spectrally non-uniform, which can make
/// the χ²(2) statistical assumption of `decode`'s sync gate too lax.
#[test]
fn noise_floor_mic_like() {
    let sample_rate = 12_000.0_f32;
    let n_samples = 7 * sample_rate as usize; // 7 s = the uvpacket-web window
    let mut rng = Awgn::new(0xdeadbeef);
    let sigma = 0.02_f32;
    // Independent state for the 1/f filter (single-pole low-pass on
    // white noise) — gives a typical mic-preamp pink-ish tail.
    let mut pink = 0.0_f32;
    let pink_alpha = 0.995_f32;
    let two_pi = 2.0 * std::f32::consts::PI;
    let audio: Vec<f32> = (0..n_samples)
        .map(|i| {
            let g = sigma * rng.gaussian();
            pink = pink_alpha * pink + (1.0 - pink_alpha) * g * 30.0;
            let t = i as f32 / sample_rate;
            let hum50 = 0.005 * (two_pi * 50.0 * t).sin();
            let hum60 = 0.003 * (two_pi * 60.0 * t).sin();
            g + pink + hum50 + hum60
        })
        .collect();

    let peak = audio.iter().fold(0.0_f32, |m, &s| m.max(s.abs()));
    let t0 = Instant::now();
    let frames = rx::decode(&audio, 1500.0);
    let elapsed = t0.elapsed();
    eprintln!(
        "[mic-like] 7 s σ={} (peak={:.4}): {} frames, decode took {:?}",
        sigma,
        peak,
        frames.len(),
        elapsed
    );
    assert_eq!(frames.len(), 0, "mic-like noise must not produce frames");
    assert!(
        elapsed.as_millis() < 1500,
        "mic-like noise took {:?} — sync gate is leaky against structured noise",
        elapsed
    );
}

/// Regression test for the bug uvpacket-web hit in production: a
/// partial-fill ring buffer (front half = zeros from initial state,
/// back half = real noise). The exact-zero scores in the front half
/// pull the score median to 0, which previously bypassed the sync
/// gate via the `median <= 0` defensive branch and let the LDPC sweep
/// run on noise.
#[test]
fn noise_floor_half_zero_buffer() {
    let n_samples = 7 * 12_000;
    let half = n_samples / 2;
    let mut rng = Awgn::new(0xfeedface);
    let sigma = 0.003_f32;
    let mut audio: Vec<f32> = vec![0.0; n_samples];
    for s in &mut audio[half..] {
        *s = sigma * rng.gaussian();
    }
    let peak = audio.iter().fold(0.0_f32, |m, &s| m.max(s.abs()));

    let t0 = Instant::now();
    let frames = rx::decode(&audio, 1500.0);
    let elapsed = t0.elapsed();
    eprintln!(
        "[half-zero] 7 s (peak={:.4}, half pre-fill zeros): {} frames, decode took {:?}",
        peak,
        frames.len(),
        elapsed
    );
    assert_eq!(frames.len(), 0);
    assert!(
        elapsed.as_millis() < 1500,
        "half-zero buffer took {:?} — gate's median-of-non-zero rule isn't holding",
        elapsed
    );
}

/// Buffer with an *amplitude* peak that matches what uvpacket-web is
/// reporting in the field (0.012). This is the operating regime where
/// the sync gate must hold.
#[test]
fn noise_floor_field_amplitude() {
    let sample_rate = 12_000.0_f32;
    let n_samples = 7 * sample_rate as usize;
    let mut rng = Awgn::new(0xfeedface);
    // σ tuned so peak ≈ 0.012 (matches user's reported snapshot peak).
    let sigma = 0.003_f32;
    let audio: Vec<f32> = (0..n_samples).map(|_| sigma * rng.gaussian()).collect();
    let peak = audio.iter().fold(0.0_f32, |m, &s| m.max(s.abs()));

    let t0 = Instant::now();
    let frames = rx::decode(&audio, 1500.0);
    let elapsed = t0.elapsed();
    eprintln!(
        "[field]    7 s σ={} (peak={:.4}): {} frames, decode took {:?}",
        sigma,
        peak,
        frames.len(),
        elapsed
    );
    assert_eq!(frames.len(), 0);
    assert!(
        elapsed.as_millis() < 1500,
        "field-amplitude noise took {:?} — gate is leaky",
        elapsed
    );
}
