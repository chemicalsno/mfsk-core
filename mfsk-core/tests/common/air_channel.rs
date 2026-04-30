// SPDX-License-Identifier: GPL-3.0-or-later
//! Phase A "air channel" sim — SSB / FM compound channel models that
//! capture the impairment stack actually seen on-air, going beyond
//! the AWGN + flat-magnitude-Rayleigh that `channel.rs` covers.
//!
//! Crucially these models include **phase**-domain impairments
//! (LO phase walk, clarifier / discriminator frequency offset,
//! multipath, AR(1) phase jitter, Rician fading with both magnitude
//! and phase). This is the dimension the Phase 2 sim missed and
//! which causes the current coherent-QPSK uvpacket to fail over
//! real SSB / FM relay despite passing the AWGN + amplitude-Rayleigh
//! benches.
//!
//! Background: `~/.claude/plans/dynamic-cooking-mountain.md` §1.3.

use std::f32::consts::PI;

use num_complex::Complex32;
use rustfft::FftPlanner;

const SAMPLE_RATE_HZ: f32 = 12_000.0;

/// Phase-domain impairments shared between SSB and FM compound
/// channels. Set fields to zero / empty / `INFINITY` to disable a
/// sub-impairment.
#[derive(Clone, Debug)]
pub struct PhaseFadingModel {
    /// LO phase Wiener-process σ (rad / √s). Drives slow phase
    /// wander; over a burst of T seconds the accumulated phase
    /// variance is σ² · T. Cheap-HT radios: ~1 rad/√s; clean SSB
    /// transceiver: ~0.05 rad/√s.
    pub lo_phase_walk_rad_per_sqrt_s: f32,

    /// AR(1) phase jitter — fast, white-ish phase noise on top of
    /// the slow Wiener walk. RMS in radians, with a correlation
    /// time. Models e.g. mic-in noise leaking into a soft demod.
    pub phase_jitter_rms_rad: f32,
    pub phase_jitter_corr_ms: f32,

    /// Maximum Doppler frequency (Hz). 0 disables fading. Drives a
    /// filtered-Gaussian time-varying complex gain h(t).
    pub doppler_hz: f32,

    /// Rician K-factor (dB). `INFINITY` → no fading (LOS only),
    /// `−INFINITY` → pure Rayleigh, 0 dB → equal LOS / scattered.
    /// VHF NFM typical: +5 to +15 dB.
    pub rician_k_db: f32,

    /// Multipath tap delay line: `(delay_ms, attenuation_db)` pairs.
    /// Empty = no multipath. Acoustic coupling reverb: 5 ms / -10 dB.
    /// Outdoor obstacle multipath: 30 ms / -15 dB.
    pub multipath_taps: Vec<(f32, f32)>,
}

impl PhaseFadingModel {
    /// All-zeros / empty model = no phase fading at all.
    pub fn off() -> Self {
        Self {
            lo_phase_walk_rad_per_sqrt_s: 0.0,
            phase_jitter_rms_rad: 0.0,
            phase_jitter_corr_ms: 1.0,
            doppler_hz: 0.0,
            rician_k_db: f32::INFINITY,
            multipath_taps: Vec::new(),
        }
    }

    /// Realistic SSB through-air defaults: cheap-HT-grade phase
    /// wander, no multipath, stationary. The headline "this should
    /// reproduce field failure" parameter set when combined with a
    /// modest clarifier offset on `SsbChannel`.
    pub fn ssb_typical() -> Self {
        Self {
            lo_phase_walk_rad_per_sqrt_s: 0.5,
            phase_jitter_rms_rad: 0.05,
            phase_jitter_corr_ms: 2.0,
            doppler_hz: 0.0,
            rician_k_db: f32::INFINITY,
            multipath_taps: Vec::new(),
        }
    }

    /// Realistic FM through-air defaults: heavier wander (FM
    /// discriminator floor) + 5 ms acoustic-style multipath.
    pub fn fm_typical() -> Self {
        Self {
            lo_phase_walk_rad_per_sqrt_s: 0.3,
            phase_jitter_rms_rad: 0.15,
            phase_jitter_corr_ms: 1.0,
            doppler_hz: 0.0,
            rician_k_db: 10.0,
            multipath_taps: vec![(5.0, -10.0)],
        }
    }
}

/// PCG-style LCG state — matches the convention in `channel.rs` so
/// that seeds reproduce bit-for-bit across helpers.
struct Pcg64 {
    state: u64,
}

impl Pcg64 {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
        }
    }

    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }

    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Compute the analytic signal `x_a[n] = x[n] + j · H{x}[n]` via
/// FFT (zero negative frequencies, double positive ones, leave DC
/// and Nyquist unchanged).
fn analytic_signal(x: &[f32]) -> Vec<Complex32> {
    let n = x.len();
    if n == 0 {
        return Vec::new();
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut buf: Vec<Complex32> = x.iter().map(|&v| Complex32::new(v, 0.0)).collect();
    fft.process(&mut buf);

    let half = n / 2;
    for c in buf.iter_mut().skip(1).take(half.saturating_sub(1)) {
        *c *= 2.0;
    }
    for c in buf.iter_mut().take(n).skip(half + 1) {
        *c = Complex32::new(0.0, 0.0);
    }

    ifft.process(&mut buf);
    let scale = 1.0 / n as f32;
    for c in buf.iter_mut() {
        *c *= scale;
    }
    buf
}

/// FFT-domain bandpass filter with raised-cosine edges
/// (`transition_hz` wide on each side). Real-in → real-out.
fn bpf(audio: &[f32], low_hz: f32, high_hz: f32, transition_hz: f32, fs: f32) -> Vec<f32> {
    let n = audio.len();
    if n == 0 {
        return Vec::new();
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut buf: Vec<Complex32> = audio.iter().map(|&v| Complex32::new(v, 0.0)).collect();
    fft.process(&mut buf);

    let bin_hz = fs / n as f32;
    let half = n / 2;
    for (k, c) in buf.iter_mut().enumerate() {
        let f = if k <= half {
            k as f32 * bin_hz
        } else {
            (k as f32 - n as f32) * bin_hz
        };
        let h = bpf_response(f.abs(), low_hz, high_hz, transition_hz);
        *c *= h;
    }

    ifft.process(&mut buf);
    let scale = 1.0 / n as f32;
    buf.iter().map(|c| c.re * scale).collect()
}

fn bpf_response(f: f32, low: f32, high: f32, trans: f32) -> f32 {
    if f < low - trans || f > high + trans {
        return 0.0;
    }
    if f >= low && f <= high {
        return 1.0;
    }
    if f < low {
        let t = (f - (low - trans)) / trans;
        return 0.5 * (1.0 - (PI * (1.0 - t)).cos());
    }
    let t = (f - high) / trans;
    0.5 * (1.0 + (PI * (1.0 - t)).cos())
}

/// Single-pole IIR low-pass — used for FM de-emphasis.
fn deemphasis(audio: &mut [f32], tau_s: f32, fs: f32) {
    if tau_s <= 0.0 {
        return;
    }
    let alpha = 1.0 - (-1.0 / (tau_s * fs)).exp();
    let mut y = 0.0_f32;
    for sample in audio.iter_mut() {
        y += alpha * (*sample - y);
        *sample = y;
    }
}

/// Apply LO phase noise + frequency offset + Rician/Doppler complex
/// fading + multipath to a real audio signal in-place.
fn apply_phase_impairments(
    audio: &mut [f32],
    fs: f32,
    freq_offset_hz: f32,
    model: &PhaseFadingModel,
    rng: &mut Pcg64,
) {
    if audio.is_empty() {
        return;
    }

    // 1. Real → analytic (carries both amplitude and phase).
    let mut anal = analytic_signal(audio);

    // 2. Multipath as a tap delay line on the analytic signal.
    if !model.multipath_taps.is_empty() {
        let direct = anal.clone();
        for (delay_ms, atten_db) in &model.multipath_taps {
            let delay_samp = (delay_ms / 1000.0 * fs).round() as usize;
            let gain = 10f32.powf(atten_db / 20.0);
            for n in delay_samp..anal.len() {
                anal[n] += direct[n - delay_samp] * gain;
            }
        }
    }

    // 3. Per-sample multiplicative impairments: cumulative phase
    // (constant offset + Wiener walk + AR(1) jitter) and Rician
    // complex gain.
    let dt = 1.0 / fs;
    let walk_step_sigma = model.lo_phase_walk_rad_per_sqrt_s * dt.sqrt();

    let jitter_alpha = if model.phase_jitter_corr_ms > 0.0 {
        1.0 - (-1.0 / (model.phase_jitter_corr_ms * 1e-3 * fs)).exp()
    } else {
        1.0
    };
    // AR(1) y[n] = (1-α) y[n-1] + α u[n], steady E[y²] = α/(2-α) E[u²]
    // → σ_u = σ_y · √((2-α)/α).
    let jitter_inn_sigma = if jitter_alpha > 0.0 {
        model.phase_jitter_rms_rad * ((2.0 - jitter_alpha) / jitter_alpha).sqrt()
    } else {
        0.0
    };

    let k_lin = 10f32.powf(model.rician_k_db / 10.0);
    let no_fading = !k_lin.is_finite() || model.doppler_hz <= 0.0;
    let los_amp = if k_lin.is_finite() {
        (k_lin / (k_lin + 1.0)).sqrt()
    } else {
        1.0
    };
    let scat_amp = if k_lin.is_finite() {
        (1.0 / (k_lin + 1.0)).sqrt()
    } else {
        0.0
    };
    let dop_alpha = if model.doppler_hz > 0.0 {
        1.0 - (-2.0 * PI * model.doppler_hz / fs).exp()
    } else {
        1.0
    };
    // Per-axis steady σ should be √0.5 so that E[|h_scat|²] = 1.
    // From AR(1) variance relation: σ_inn = σ_y · √((2-α)/α).
    let dop_inn_sigma = if dop_alpha > 0.0 {
        std::f32::consts::FRAC_1_SQRT_2 * ((2.0 - dop_alpha) / dop_alpha).sqrt()
    } else {
        0.0
    };

    let mut phi_walk = 0.0_f32;
    let mut phi_jitter = 0.0_f32;
    let mut h_re = 0.0_f32;
    let mut h_im = 0.0_f32;

    for (n, samp) in anal.iter_mut().enumerate() {
        phi_walk += rng.gaussian() * walk_step_sigma;
        phi_jitter =
            (1.0 - jitter_alpha) * phi_jitter + jitter_alpha * rng.gaussian() * jitter_inn_sigma;
        let phi_total = 2.0 * PI * freq_offset_hz * n as f32 / fs + phi_walk + phi_jitter;
        let phase_factor = Complex32::new(phi_total.cos(), phi_total.sin());

        let h = if no_fading {
            Complex32::new(1.0, 0.0)
        } else {
            let u_re = rng.gaussian() * dop_inn_sigma;
            let u_im = rng.gaussian() * dop_inn_sigma;
            h_re = (1.0 - dop_alpha) * h_re + dop_alpha * u_re;
            h_im = (1.0 - dop_alpha) * h_im + dop_alpha * u_im;
            Complex32::new(los_amp + scat_amp * h_re, scat_amp * h_im)
        };

        *samp = *samp * phase_factor * h;
    }

    for (i, c) in anal.iter().enumerate() {
        audio[i] = c.re;
    }
}

/// SSB through-air channel: TX-side BPF + clarifier offset + LO
/// phase impairments + RX-side BPF + AWGN.
#[derive(Clone, Debug)]
pub struct SsbChannel {
    pub bpf_hz: (f32, f32),
    pub bpf_transition_hz: f32,
    pub clarifier_offset_hz: f32,
    pub awgn_sigma: f32,
    pub phase_fading: PhaseFadingModel,
    pub seed: u64,
}

impl Default for SsbChannel {
    fn default() -> Self {
        Self {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 30.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel::ssb_typical(),
            seed: 0xDEAD_BEEF_CAFE_F00D,
        }
    }
}

impl SsbChannel {
    pub fn apply(&self, audio: &mut [f32]) {
        let mut rng = Pcg64::new(self.seed);
        let fs = SAMPLE_RATE_HZ;

        let mut buf = bpf(
            audio,
            self.bpf_hz.0,
            self.bpf_hz.1,
            self.bpf_transition_hz,
            fs,
        );
        apply_phase_impairments(
            &mut buf,
            fs,
            self.clarifier_offset_hz,
            &self.phase_fading,
            &mut rng,
        );
        let mut buf = bpf(
            &buf,
            self.bpf_hz.0,
            self.bpf_hz.1,
            self.bpf_transition_hz,
            fs,
        );
        for sample in buf.iter_mut() {
            *sample += self.awgn_sigma * rng.gaussian();
        }
        audio.copy_from_slice(&buf[..audio.len()]);
    }
}

/// FM through-air channel (post-discriminator audio): discriminator
/// DC drift + LO phase impairments + de-emphasis + AWGN. Models the
/// audio that emerges from a typical NFM repeater path.
#[derive(Clone, Debug)]
pub struct FmChannel {
    pub deemphasis_us: f32,
    pub discriminator_dc_drift_hz: f32,
    pub awgn_sigma: f32,
    pub phase_fading: PhaseFadingModel,
    pub seed: u64,
}

impl Default for FmChannel {
    fn default() -> Self {
        Self {
            deemphasis_us: 75.0,
            discriminator_dc_drift_hz: 50.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel::fm_typical(),
            seed: 0xC0FF_EE00_BEEF_F00D,
        }
    }
}

impl FmChannel {
    pub fn apply(&self, audio: &mut [f32]) {
        let mut rng = Pcg64::new(self.seed);
        let fs = SAMPLE_RATE_HZ;

        apply_phase_impairments(
            audio,
            fs,
            self.discriminator_dc_drift_hz,
            &self.phase_fading,
            &mut rng,
        );
        deemphasis(audio, self.deemphasis_us * 1e-6, fs);
        for sample in audio.iter_mut() {
            *sample += self.awgn_sigma * rng.gaussian();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_signal_real_part_matches_input() {
        let n = 512;
        let f = 1500.0 / SAMPLE_RATE_HZ;
        let x: Vec<f32> = (0..n).map(|i| (2.0 * PI * f * i as f32).cos()).collect();
        let anal = analytic_signal(&x);
        // Skip edges (FFT-Hilbert has wrap-around there).
        for (i, (a, xi)) in anal[64..n - 64].iter().zip(&x[64..n - 64]).enumerate() {
            assert!((a.re - xi).abs() < 0.05, "i={i}: re={} vs x={xi}", a.re,);
        }
    }

    #[test]
    fn analytic_signal_quadrature_is_sine_for_cosine_input() {
        let n = 512;
        let f = 1500.0 / SAMPLE_RATE_HZ;
        let x: Vec<f32> = (0..n).map(|i| (2.0 * PI * f * i as f32).cos()).collect();
        let anal = analytic_signal(&x);
        for (offset, a) in anal[64..n - 64].iter().enumerate() {
            let i = offset + 64;
            let want = (2.0 * PI * f * i as f32).sin();
            assert!(
                (a.im - want).abs() < 0.05,
                "i={i}: im={} vs sin={want}",
                a.im,
            );
        }
    }

    #[test]
    fn bpf_passes_in_band_blocks_out_of_band() {
        let n = 4096;
        let fs = SAMPLE_RATE_HZ;
        // 1500 Hz tone (in the SSB passband) and 100 Hz tone (out).
        let in_band: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let out_band: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f32 / fs).cos())
            .collect();
        let in_filtered = bpf(&in_band, 300.0, 2700.0, 100.0, fs);
        let out_filtered = bpf(&out_band, 300.0, 2700.0, 100.0, fs);
        let pwr =
            |buf: &[f32]| buf[200..n - 200].iter().map(|s| s * s).sum::<f32>() / (n - 400) as f32;
        let in_pwr = pwr(&in_filtered);
        let out_pwr = pwr(&out_filtered);
        assert!(in_pwr > 0.4, "in-band power {in_pwr} too low");
        assert!(out_pwr < 0.05, "out-of-band power {out_pwr} too high");
    }

    #[test]
    fn ssb_channel_off_preserves_passband_signal() {
        let n = 2048;
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel::off(),
            seed: 1,
        };
        chan.apply(&mut audio);
        let pwr =
            |buf: &[f32]| buf[300..n - 300].iter().map(|s| s * s).sum::<f32>() / (n - 600) as f32;
        let r = pwr(&audio) / pwr(&original);
        assert!(
            (r - 1.0).abs() < 0.15,
            "SSB-off should preserve power within 15%, got ratio {r}",
        );
    }

    #[test]
    fn ssb_channel_with_phase_walk_introduces_phase_drift() {
        // A 1500 Hz cosine through SSB with walk should accumulate
        // measurable instantaneous phase deviation by burst-end.
        let n = 12_000; // 1 sec
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel {
                lo_phase_walk_rad_per_sqrt_s: 2.0,
                ..PhaseFadingModel::off()
            },
            seed: 7,
        };
        chan.apply(&mut audio);
        // Compare the late-burst correlation against the early-burst
        // correlation. Phase walk should de-correlate them.
        let early: f32 = audio[1000..2000]
            .iter()
            .zip(&original[1000..2000])
            .map(|(a, b)| a * b)
            .sum::<f32>()
            / 1000.0;
        let late: f32 = audio[10_000..11_000]
            .iter()
            .zip(&original[10_000..11_000])
            .map(|(a, b)| a * b)
            .sum::<f32>()
            / 1000.0;
        // Early correlation should be close to 0.5 (cos² mean), late
        // should be reduced (or sign-reversed) by accumulated phase
        // drift.
        assert!(
            (early - late).abs() > 0.1 || late.abs() < 0.3,
            "phase walk produced no measurable drift: early={early} late={late}",
        );
    }

    #[test]
    fn ssb_default_channel_does_not_nan() {
        let mut audio = vec![0.5_f32; 12_000];
        SsbChannel::default().apply(&mut audio);
        assert!(audio.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn fm_default_channel_does_not_nan() {
        let mut audio = vec![0.5_f32; 12_000];
        FmChannel::default().apply(&mut audio);
        assert!(audio.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn rician_k_infinity_gives_unit_envelope() {
        // K = +∞ + Doppler 5 Hz: should still be near-unit envelope
        // (since LOS component dominates).
        let n = 12_000;
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel {
                doppler_hz: 5.0,
                rician_k_db: f32::INFINITY,
                ..PhaseFadingModel::off()
            },
            seed: 11,
        };
        chan.apply(&mut audio);
        let r = audio[400..n - 400].iter().map(|s| s * s).sum::<f32>()
            / original[400..n - 400].iter().map(|s| s * s).sum::<f32>();
        assert!((r - 1.0).abs() < 0.2, "unit-K envelope ratio {r}");
    }
}
