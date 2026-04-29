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
