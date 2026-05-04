//! Successive interference cancellation (SIC) for phase-continuous MFSK.
//!
//! Given a tone sequence plus time/frequency coordinates, reconstructs the
//! ideal IQ waveform, estimates its complex amplitude by least-squares
//! projection onto the received signal, and subtracts a scaled copy in place.
//! Protocol-agnostic — the caller supplies tone sequence, sample rate, tone
//! spacing and timing so the same routine serves FT8/FT4/FT2/FST4.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::PI;
#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Fixed DSP parameters for a single subtraction call.
#[derive(Clone, Copy, Debug)]
pub struct SubtractCfg {
    /// PCM sample rate (Hz), e.g. 12 000 for the WSJT pipeline.
    pub sample_rate: f32,
    /// Tone spacing (Hz). FT8 = 6.25, FT4 = 20.833, …
    pub tone_spacing_hz: f32,
    /// Samples per FT symbol at `sample_rate`. FT8 = 1920, FT4 = 576, …
    pub samples_per_symbol: usize,
    /// Frame origin offset within the slot buffer, seconds. WSJT convention
    /// places `t = 0` of the transmitted frame at 0.5 s for FT8, 0.5 s for
    /// FT4 as well — so typically 0.5.
    pub base_offset_s: f32,
    /// GFSK pulse-shaping parameters. `Some` → match real WSJT-style
    /// transmissions (FT8 BT=2.0, FT4 BT=2.0, FST4 BT=1.0). `None` → use
    /// abrupt phase transitions; reference will not match real signals
    /// — only retained for non-GFSK protocols and round-trip tests.
    pub gfsk: Option<GfskParams>,
}

/// Subset of [`crate::core::dsp::gfsk::GfskCfg`] needed to shape the
/// subtract reference. `sample_rate` and `samples_per_symbol` are
/// already on `SubtractCfg` so we only carry the GFSK-specific knobs.
#[derive(Clone, Copy, Debug)]
pub struct GfskParams {
    /// Bandwidth-time product. FT8/FT4 use 2.0.
    pub bt: f32,
    /// Modulation index. 1.0 for FT8.
    pub hmod: f32,
    /// Cosine ramp length at start/end (samples). FT8 uses `nsps / 8`.
    pub ramp_samples: usize,
}

/// Generate phase-continuous cosine/sine references for a symbol stream.
///
/// Returns `(w_cos, w_sin)` each of length `tones.len() * cfg.samples_per_symbol`.
/// `freq_hz` is the carrier of tone 0.
///
/// When `cfg.gfsk` is `Some`, the references are GFSK-shaped (matches
/// the actual transmitted FT8/FT4 waveform — required for any subtract
/// against real WSJT-style signals). Otherwise the legacy abrupt
/// per-symbol frequency switching is used.
fn generate_iq(tones: &[u8], freq_hz: f32, cfg: &SubtractCfg) -> (Vec<f32>, Vec<f32>) {
    let n = tones.len() * cfg.samples_per_symbol;
    if let Some(g) = cfg.gfsk {
        let gfsk_cfg = crate::core::dsp::gfsk::GfskCfg {
            sample_rate: cfg.sample_rate,
            samples_per_symbol: cfg.samples_per_symbol,
            bt: g.bt,
            hmod: g.hmod,
            ramp_samples: g.ramp_samples,
        };
        let mut w_cos = vec![0.0f32; n];
        let mut w_sin = vec![0.0f32; n];
        crate::core::dsp::gfsk::synth_complex_f32_into(
            &mut w_cos, &mut w_sin, tones, freq_hz, 1.0, &gfsk_cfg,
        );
        return (w_cos, w_sin);
    }
    let mut w_cos = vec![0.0f32; n];
    let mut w_sin = vec![0.0f32; n];
    let mut phase = 0.0f32;
    for (sym, &tone) in tones.iter().enumerate() {
        let freq = freq_hz + tone as f32 * cfg.tone_spacing_hz;
        let dphi = 2.0 * PI * freq / cfg.sample_rate;
        let base = sym * cfg.samples_per_symbol;
        for j in 0..cfg.samples_per_symbol {
            w_cos[base + j] = phase.cos();
            w_sin[base + j] = phase.sin();
            phase += dphi;
            if phase > PI {
                phase -= 2.0 * PI;
            }
        }
    }
    (w_cos, w_sin)
}

/// Subtract a tone sequence from `audio` in place, with a fractional gain.
///
/// `gain = 1.0` performs full least-squares subtraction. `gain < 1.0` is
/// useful when the channel is time-varying: over-subtraction would introduce
/// a negative-amplitude residual that poisons subsequent decode passes.
#[inline]
pub fn subtract_tones(
    audio: &mut [i16],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    gain: f32,
    cfg: &SubtractCfg,
) {
    let (w_cos, w_sin) = generate_iq(tones, freq_hz, cfg);

    // Signed start (samples). For `dt_sec < -base_offset_s` the FT8 frame
    // begins **before** the audio buffer — the leading portion of the
    // reference falls outside `audio[..]` and must be clipped.
    //
    // Pre-fix: `as usize` saturated negative values to 0, leaving
    // `start = 0` and the entire reference aligned to `audio[0..]`.
    // For e.g. `dt_sec = -0.78` (FT8 reports up to ±2.5 s) this misaligned
    // the reference by 3360 samples → near-zero LS amplitude → effectively
    // no-op subtract on legitimately-decoded signals at large negative DT.
    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= w_cos.len() {
        return;
    }
    let len = (w_cos.len() - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return;
    }

    // Complex least-squares: rx[t] ≈ a·cos(φ(t)) + b·sin(φ(t))
    // cos / sin are near-orthogonal over the full frame so the closed-form
    // per-component projection matches the joint solve to floating-point
    // precision.
    let (num_a, num_b, den_a, den_b) =
        (0..len).fold((0.0f32, 0.0f32, 0.0f32, 0.0f32), |(na, nb, da, db), i| {
            let rx = audio[audio_off + i] as f32;
            let wc = w_cos[ref_off + i];
            let ws = w_sin[ref_off + i];
            (na + rx * wc, nb + rx * ws, da + wc * wc, db + ws * ws)
        });

    let a = if den_a > f32::EPSILON {
        num_a / den_a
    } else {
        0.0
    };
    let b = if den_b > f32::EPSILON {
        num_b / den_b
    } else {
        0.0
    };

    for i in 0..len {
        let sub = gain * (a * w_cos[ref_off + i] + b * w_sin[ref_off + i]);
        let new_val = audio[audio_off + i] as f32 - sub;
        audio[audio_off + i] = new_val.clamp(-32_768.0, 32_767.0) as i16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Regression test for `subtract_tones` start-clipping with negative DT.
    ///
    /// Pre-fix: `((base_offset_s + dt_sec) * sample_rate) as usize` silently
    /// saturated negative values to 0, leaving the reference misaligned by
    /// thousands of samples — the LS projection then estimated near-zero
    /// amplitude and the subtract was effectively a no-op for any decode
    /// reporting `dt_sec < -base_offset_s`.
    #[test]
    fn subtract_tones_negative_dt_aligns_via_ref_offset() {
        // Fake FT8-shape config: small frame for cheap test.
        // Tests intentionally exercise the abrupt-transition path so audio
        // and reference share identical shape (cleanest > -100 dB drop).
        // The GFSK path is exercised by `tests/ft8_subtract_self_test.rs`.
        let cfg = SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: None,
        };
        let tones: Vec<u8> = (0..79).map(|k| (k % 8) as u8).collect();
        let (w_cos, _w_sin) = generate_iq(&tones, 1500.0, &cfg);

        // Build audio that starts mid-frame (signal began before sample 0):
        // shift the reference left by 3360 samples and take only the
        // overlapping tail. amplitude = 5000.
        let shift = 3360usize;
        let amp = 5000.0f32;
        let mut audio: Vec<i16> = vec![0i16; 180_000];
        let n = w_cos.len() - shift;
        for i in 0..n.min(audio.len()) {
            audio[i] = (amp * w_cos[shift + i]).clamp(-32_768.0, 32_767.0) as i16;
        }
        let pre_energy: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();

        // Reported dt corresponds to signal-start at sample -3360, i.e.
        // dt_sec = -3360 / 12_000 = -0.28 → start = (0.5 - 0.28) * 12k = 2640.
        // With shift=3360 the actual signal-start is -3360 from sample 0, so
        // the equivalent decoded dt_sec is (0 - shift) / sample_rate -
        // base_offset_s = -shift/sr - 0.5 = -0.78 (mirroring qso3 CQ F5RXL).
        let dt_sec = -(shift as f32) / cfg.sample_rate - cfg.base_offset_s;
        subtract_tones(&mut audio, &tones, 1500.0, dt_sec, 1.0, &cfg);

        let post_energy: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();
        let drop_db = 10.0 * (post_energy / pre_energy).log10();

        // With proper alignment: post energy should drop by ≥ 30 dB
        // (clean signal, exact reference match — only quantization noise left).
        assert!(
            drop_db < -30.0,
            "subtract_tones failed to remove signal at dt_sec={dt_sec:.3} \
             (drop only {drop_db:.1} dB; expected < -30 dB). \
             Pre-fix bug: `start as usize` saturated negative to 0."
        );
    }

    /// Sanity: positive DT path also works (regression check).
    #[test]
    fn subtract_tones_positive_dt_works() {
        // Tests intentionally exercise the abrupt-transition path so audio
        // and reference share identical shape (cleanest > -100 dB drop).
        // The GFSK path is exercised by `tests/ft8_subtract_self_test.rs`.
        let cfg = SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: None,
        };
        let tones: Vec<u8> = (0..79).map(|k| (k % 8) as u8).collect();
        let (w_cos, _) = generate_iq(&tones, 1500.0, &cfg);

        let dt_sec: f32 = 0.2;
        let start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as usize;
        let amp = 5000.0f32;
        let mut audio: Vec<i16> = vec![0i16; 180_000];
        for i in 0..w_cos.len() {
            audio[start + i] = (amp * w_cos[i]).clamp(-32_768.0, 32_767.0) as i16;
        }
        let pre: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();

        subtract_tones(&mut audio, &tones, 1500.0, dt_sec, 1.0, &cfg);
        let post: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();
        let drop_db = 10.0 * (post / pre).log10();
        assert!(
            drop_db < -30.0,
            "positive-DT subtract drop only {drop_db:.1} dB"
        );
    }
}
