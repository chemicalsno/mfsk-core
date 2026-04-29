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
        "noise-only 1 s @ σ={}: {} frames, decode took {:?}",
        sigma,
        frames.len(),
        elapsed
    );
    assert_eq!(frames.len(), 0, "noise must not produce decoded frames");
    assert!(
        elapsed.as_millis() < 1500,
        "noise-only 1 s buffer took {:?} — false-sync rejection is missing or weak",
        elapsed
    );
}
