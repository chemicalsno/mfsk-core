// SPDX-License-Identifier: GPL-3.0-or-later
//! Channel models for the uvpacket characterisation harness.
//!
//! - [`AwgnChannel`] — additive white Gaussian noise (Phase 2a).
//! - [`RayleighFlatChannel`] — flat Rayleigh fading (frequency-non-
//!   selective, time-selective at a configurable Doppler rate).
//!   Two independent low-pass-filtered Gaussian processes form the
//!   real and imaginary parts of the fading envelope; the audio
//!   signal is multiplied by the resulting complex magnitude.
//!   Followed by AWGN. Phase 2b.
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

/// Flat (non-frequency-selective) Rayleigh fading channel.
///
/// The fading envelope is a complex Gaussian process whose
/// magnitude follows a Rayleigh distribution. Time selectivity is
/// controlled by a low-pass filter that bandlimits the underlying
/// real / imaginary processes to the configured maximum Doppler
/// frequency `f_d`. Real-valued audio after fading: `out[n] =
/// |envelope[n]| · in[n] + AWGN`. (Phase-rotated complex envelope
/// would matter for coherent demod; we only need the magnitude
/// for the non-coherent receiver.)
///
/// Implementation: each call advances the underlying state by one
/// sample. A 1st-order IIR LPF (single-pole, `α = 1 − exp(−2π f_d
/// / fs)`) shapes white Gaussian innovations into the desired
/// Doppler-bandlimited process — this is the simplest model that
/// produces the expected Rayleigh amplitude statistics and
/// approximately correct autocorrelation. For more elaborate
/// fading (Jakes / sum-of-sinusoids) the harness can be extended.
pub struct RayleighFlatChannel {
    awgn: AwgnChannel,
    re_state: f32,
    im_state: f32,
    alpha: f32,
    /// Innovation σ — picked so that the steady-state envelope has
    /// E[|h|²] = 1 (i.e. the fading neither amplifies nor
    /// attenuates on average).
    inn_sigma: f32,
    state: u64,
}

impl RayleighFlatChannel {
    /// `f_doppler_hz` is the maximum Doppler frequency (Hz).
    /// `awgn_sigma` is the per-sample post-fading AWGN σ.
    /// `seed` makes runs reproducible.
    pub fn new(f_doppler_hz: f32, awgn_sigma: f32, seed: u64) -> Self {
        let fs = SAMPLE_RATE_HZ;
        // Single-pole LPF coefficient α = 1 − exp(−2π f_d / fs).
        let alpha = 1.0 - (-2.0 * PI * f_doppler_hz / fs).exp();
        // Innovation σ chosen for steady-state E[X²] = 0.5 per axis
        // → E[|h|²] = 1. For the IIR `y = (1 − α) y + α u`, steady
        // E[y²] = α / (2 − α) × E[u²]. Solve E[u²] = 0.5 × (2 − α)
        // / α.
        let inn_var = 0.5 * (2.0 - alpha) / alpha;
        let inn_sigma = inn_var.sqrt();
        Self {
            awgn: AwgnChannel::new(awgn_sigma, seed.wrapping_add(1)),
            re_state: 0.0,
            im_state: 0.0,
            alpha,
            inn_sigma,
            state: seed.wrapping_add(0xBF58_476D_1CE4_E5B9),
        }
    }

    /// Apply the channel to `audio` in-place: per-sample fading
    /// magnitude × signal, then AWGN.
    pub fn apply(&mut self, audio: &mut [f32]) {
        // Pre-roll the IIR for ~5 / α samples so the envelope has
        // reached steady state by the time the frame samples
        // arrive. (`α ≪ 1` for slow fading, so this matters most
        // at low Doppler.)
        let pre_roll = (5.0 / self.alpha.max(1e-6)) as usize;
        for _ in 0..pre_roll {
            let _ = self.next_envelope_magnitude();
        }
        for sample in audio.iter_mut() {
            let h = self.next_envelope_magnitude();
            *sample *= h;
        }
        self.awgn.apply(audio);
    }

    fn next_envelope_magnitude(&mut self) -> f32 {
        let u_re = self.gaussian() * self.inn_sigma;
        let u_im = self.gaussian() * self.inn_sigma;
        self.re_state = (1.0 - self.alpha) * self.re_state + self.alpha * u_re;
        self.im_state = (1.0 - self.alpha) * self.im_state + self.alpha * u_im;
        (self.re_state * self.re_state + self.im_state * self.im_state).sqrt()
    }

    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
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

    /// Rayleigh envelope statistics: over a long buffer, the
    /// magnitude has E[|h|²] ≈ 1 by construction.
    #[test]
    fn rayleigh_envelope_has_unit_mean_square() {
        let mut chan = RayleighFlatChannel::new(5.0, 0.0, 0xABC1_2345);
        let mut audio = vec![1.0_f32; 12_000]; // 1 sec at fs=12kHz
        chan.apply(&mut audio);
        let mean_sq: f32 = audio.iter().map(|s| s * s).sum::<f32>() / audio.len() as f32;
        // Allow generous tolerance because 1 sec at 5 Hz Doppler
        // ≈ 5 fading cycles → high finite-sample variance.
        assert!(
            (0.5..2.0).contains(&mean_sq),
            "Rayleigh mean-square {mean_sq} far off the expected 1.0",
        );
    }

    /// Rayleigh + AWGN at huge Doppler approaches AWGN-only stats
    /// in the limit (envelope decorrelates fast, magnitude
    /// distribution converges to Rayleigh independent of past).
    /// Sanity smoke test that nothing panics or NaNs.
    #[test]
    fn rayleigh_apply_does_not_nan() {
        for &fd in &[0.5_f32, 1.0, 5.0, 20.0] {
            let mut chan = RayleighFlatChannel::new(fd, 0.1, 0xFEED + fd as u64);
            let mut audio = vec![0.5_f32; 4096];
            chan.apply(&mut audio);
            assert!(audio.iter().all(|s| s.is_finite()));
        }
    }
}
