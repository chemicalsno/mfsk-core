// SPDX-License-Identifier: GPL-3.0-or-later
//! Channel models for the uvpacket characterisation harness.
//!
//! Currently: AWGN with seeded Box-Muller noise generation. Phase 2b
//! extends with a Rayleigh fading channel.
//!
//! Eb/N0 sign convention: **Eb is per information bit** (not per
//! channel bit). This is the cross-mode-fair convention used in the
//! WSJT family — at the same Eb/N0 the noise variance per audio
//! sample varies by mode because higher rates spend less channel
//! energy per info bit.

use std::f32::consts::PI;

use mfsk_core::uvpacket::Mode;

/// `Ldpc240_101` info-bit count — used to compute the per-mode
/// info rate (`K / ch_bits_per_block`).
const K_INFO: f32 = 101.0;

/// uvpacket modulation parameters (for the AWGN-σ derivation).
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const SYMBOL_RATE_HZ: f32 = 1_200.0;
const BITS_PER_SYMBOL: f32 = 2.0;

/// Box-Muller AWGN channel with a deterministic LCG seed. Apply
/// to a buffer of f32 audio samples in-place.
pub struct AwgnChannel {
    sigma: f32,
    state: u64,
}

impl AwgnChannel {
    /// Build a channel that adds N(0, σ²) noise per sample.
    pub fn new(sigma: f32, seed: u64) -> Self {
        Self {
            sigma,
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
        }
    }

    /// Add the channel's AWGN to `audio` in-place.
    pub fn apply(&mut self, audio: &mut [f32]) {
        for sample in audio.iter_mut() {
            *sample += self.sigma * self.gaussian();
        }
    }

    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    fn uniform(&mut self) -> f32 {
        // PCG-style LCG, top 32 bits → uniform on (0, 1).
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
}

/// Per-mode info-rate (information bits per channel bit).
fn info_rate(mode: Mode) -> f32 {
    K_INFO / mode.ch_bits_per_block() as f32
}

/// Compute the per-sample AWGN standard deviation that yields a
/// target `Eb/N0` *per information bit* on uvpacket audio at unit
/// peak amplitude.
///
/// Derivation (signal `A · sin(2πfn/fs)`, A = 1):
///
/// - Average signal power      `P = A²/2 = 0.5`
/// - Energy per channel bit    `Eb_ch = P / (R_s · b/sym)
///   = 0.5 / (1200 · 2) ≈ 2.08 × 10⁻⁴ J`
/// - Energy per info bit       `Eb_info = Eb_ch / r`  where
///   `r = K / N_post-puncture`
/// - Target ratio `linear = 10^(eb_n0_db / 10)`
/// - One-sided AWGN PSD        `N0 = Eb_info / linear`
/// - Per-sample variance       `σ² = N0 · fs / 2`
pub fn awgn_sigma_for_eb_n0_info(mode: Mode, eb_n0_db: f32) -> f32 {
    let amplitude = 1.0f32;
    let signal_power = amplitude * amplitude / 2.0;
    let e_b_ch = signal_power / (SYMBOL_RATE_HZ * BITS_PER_SYMBOL);
    let e_b_info = e_b_ch / info_rate(mode);
    let target_linear = 10f32.powf(eb_n0_db / 10.0);
    let n0 = e_b_info / target_linear;
    let sigma_sq = n0 * SAMPLE_RATE_HZ / 2.0;
    sigma_sq.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// AWGN channel with σ = 0 must be a no-op (modulo the f32
    /// representation of `0.0 × x = 0.0`).
    #[test]
    fn awgn_with_zero_sigma_is_identity() {
        let mut c = AwgnChannel::new(0.0, 42);
        let mut buf = vec![0.5_f32; 100];
        let original = buf.clone();
        c.apply(&mut buf);
        assert_eq!(buf, original);
    }

    /// AWGN with a non-zero σ produces non-zero noise — and seeded
    /// runs are reproducible.
    #[test]
    fn awgn_seeded_is_reproducible() {
        let mut a = AwgnChannel::new(0.5, 123);
        let mut b = AwgnChannel::new(0.5, 123);
        let mut buf_a = vec![0.0_f32; 200];
        let mut buf_b = vec![0.0_f32; 200];
        a.apply(&mut buf_a);
        b.apply(&mut buf_b);
        assert_eq!(buf_a, buf_b);
        // …and at least 90 % of samples are non-zero.
        let nonzero = buf_a.iter().filter(|&&s| s != 0.0).count();
        assert!(nonzero > 180);
    }

    /// At a high Eb/N0 (= 100 dB) the σ for every mode should be
    /// vanishingly small; at 0 dB it should be a meaningful fraction
    /// of the unit-amplitude signal.
    #[test]
    fn sigma_decreases_with_eb_n0() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            let sigma_clean = awgn_sigma_for_eb_n0_info(mode, 100.0);
            let sigma_zero = awgn_sigma_for_eb_n0_info(mode, 0.0);
            assert!(sigma_clean < 1e-3, "{mode:?}: clean σ {sigma_clean}");
            assert!(sigma_zero > 0.5, "{mode:?}: 0-dB σ {sigma_zero}");
            assert!(
                sigma_clean < sigma_zero,
                "{mode:?}: σ should decrease as Eb/N0 grows",
            );
        }
    }

    /// Higher-rate modes spend less channel energy per info bit, so
    /// at a fixed Eb/N0_info the channel-domain noise must be lower
    /// for them — i.e. σ decreases with rate.
    #[test]
    fn sigma_decreases_with_rate() {
        let eb_n0 = 0.0;
        let sigmas: Vec<f32> = [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express]
            .iter()
            .map(|&m| awgn_sigma_for_eb_n0_info(m, eb_n0))
            .collect();
        for w in sigmas.windows(2) {
            assert!(
                w[0] > w[1],
                "expected σ to decrease across rates: {sigmas:?}",
            );
        }
    }
}
