// SPDX-License-Identifier: GPL-3.0-or-later
//! FT4 signal subtraction (successive interference cancellation).
//!
//! Thin FT4-tuned wrapper around the protocol-agnostic
//! [`crate::core::dsp::subtract`] implementation. Given a decoded message and
//! its time/frequency coordinates, reconstructs the ideal 4-GFSK waveform and
//! subtracts it in place so weaker signals become decodable.
//!
//! Mirrors [`crate::ft8::subtract`] for API symmetry across the two LDPC
//! 77-bit modes. The internal `decode_frame_subtract` already used the same
//! configuration; this module exposes the standalone subtract calls so
//! consumers can run their own multi-pass / SIC pipelines.
//!
//! ## WSJT-X reference
//!
//! Algorithm and constants ported from WSJT-X `lib/ft4_subtract.f90`
//! (and the GFSK shaping it shares with `lib/genft4.f90`):
//!
//! - `bt = 2.0`, `hmod = 1.0` — `genft4.f90` GFSK shaping pulse.
//! - `samples_per_symbol = 576` — `genft4.f90` `nsps = 576` at 12 kHz.
//! - `tone_spacing_hz = 20.833` — `12000 / nsps`.
//! - `base_offset_s = 0.5` — frame origin offset, `genft4.f90` `tt0 = 0.5`.
//! - `lpf_half = 2000` Hz — matches WSJT-X `NFILT = 4000` (half-bandwidth).
//!
//! All values are reused from [`super::decode::FT4_SUBTRACT`] so any
//! retuning lands in one place.

use super::{decode::DecodeResult, encode::message_to_tones};
use crate::core::dsp::subtract::{subtract_tones, subtract_tones_lpf};

// Reuse the configuration `decode_frame_subtract` already uses, so any
// behavioural tuning lands in one place.
use super::decode::FT4_SUBTRACT;

/// Reconstruct the 4-GFSK channel symbols for a decoded FT4 result.
///
/// Returns `None` if `result.message77()` is shorter than 77 bits, which
/// shouldn't happen for an FT4 decode but is handled defensively so the
/// public subtract APIs become no-ops rather than panicking.
fn get_tones(result: &DecodeResult) -> Option<Vec<u8>> {
    let m77 = <[u8; 77]>::try_from(result.message77()).ok()?;
    Some(message_to_tones(&m77))
}

/// Subtract a decoded FT4 signal from `audio` in-place (full amplitude).
#[inline]
pub fn subtract_signal(audio: &mut [i16], result: &DecodeResult) {
    subtract_signal_weighted(audio, result, 1.0);
}

/// Subtract a decoded FT4 signal with a fractional gain. `gain = 1.0` is full
/// subtraction; `gain < 1.0` partial subtraction to hedge against channel
/// variation that would otherwise leave a negative residual.
#[inline]
pub fn subtract_signal_weighted(audio: &mut [i16], result: &DecodeResult, gain: f32) {
    let tones = match get_tones(result) {
        Some(t) => t,
        None => return,
    };
    subtract_tones(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        gain,
        &FT4_SUBTRACT,
    );
}

/// WSJT-X-style channel-aware subtract for FT4. Wraps
/// [`crate::core::dsp::subtract::subtract_tones_lpf`] with the FT4 cfg
/// and `lpf_half = 2000` (matching the FT8 NFILT=4000 convention; FT4
/// frames are narrower in time but the audio LPF half-bandwidth is
/// the same).
///
/// Use this on real-WAV decodes after [`refine_signal_freq`] to get
/// near-clean signal removal. Falls back to a no-op when audio is
/// shorter than the FT4 frame.
pub fn subtract_signal_lpf(audio: &mut [i16], result: &DecodeResult) {
    let tones = match get_tones(result) {
        Some(t) => t,
        None => return,
    };
    subtract_tones_lpf(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        &FT4_SUBTRACT,
        2000,
    );
}

/// Refine `result.freq_hz` by grid-searching ±2.5 Hz at 0.1 Hz resolution
/// for the carrier that maximises the LS amplitude of the GFSK reference
/// against `audio`. Returns the refined frequency.
///
/// Use this before [`subtract_signal`] / [`subtract_signal_weighted`]
/// when the input is a real-WAV decode (not a self-synthesised signal).
/// FT4's coarse sync reports carriers on ~5.2 Hz bins (NFFT1 = 4 × NSPS);
/// real signals routinely sit ±0.5..2 Hz off-bin and the resulting phase
/// drift over the 4.94 s frame defeats the constant-amplitude LS in
/// `subtract_tones`.
///
/// Cost: ~50 GFSK reference builds × ~60 k samples each. On host f32
/// this is ~1 ms per signal — call once per decoded result rather than
/// per pass-2 candidate.
pub fn refine_signal_freq(audio: &[i16], result: &DecodeResult) -> f32 {
    // info shorter than 77 bits — shouldn't happen for FT4. Fall back
    // to the unrefined freq so callers see a stable result.
    let tones = match get_tones(result) {
        Some(t) => t,
        None => return result.freq_hz,
    };
    crate::core::dsp::subtract::refine_freq(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        &FT4_SUBTRACT,
        2.5,
        0.1,
    )
}

#[cfg(test)]
mod tests {
    use super::super::encode::{message_to_tones, tones_to_i16};
    use super::*;

    /// Build a synthetic `DecodeResult` for testing. The `info` field
    /// is a 91-bit `(message_77 + crc14)` block; we fill the first 77
    /// bits with `msg` and zero the trailing 14 CRC bits — the subtract
    /// path only reads the first 77 bits via `message77()`.
    fn synthetic_result(msg: [u8; 77], freq_hz: f32, dt_sec: f32) -> DecodeResult {
        let mut info = vec![0u8; 91].into_boxed_slice();
        info[..77].copy_from_slice(&msg);
        DecodeResult {
            info,
            freq_hz,
            dt_sec,
            hard_errors: 0,
            sync_score: 10.0,
            pass: 0,
            sync_cv: 0.0,
            snr_db: 0.0,
        }
    }

    /// Self-cancellation: synthesize a clean FT4 signal at known
    /// (freq, dt), build a synthetic DecodeResult, subtract, verify
    /// the residual power is far below the original (mirrors
    /// `ft8::subtract::tests::subtract_with_exact_timing_near_zero`).
    #[test]
    fn subtract_with_exact_timing_near_zero() {
        let msg = [1u8; 77];
        let itone = message_to_tones(&msg);
        // FT4 frame: 103 active symbols × 576 samples = 59_328.
        // Target buffer: 7.5 s × 12 kHz = 90_000.
        let samples = tones_to_i16(&itone, 1500.0, 20_000);

        let mut audio = vec![0i16; 90_000];
        let offset = 6_000usize; // 0.5 s start offset
        let len = samples.len().min(90_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();

        let result = synthetic_result(msg, 1500.0, 0.0);
        subtract_signal(&mut audio, &result);

        let power_after: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();
        assert!(
            power_after < power_before * 0.02,
            "power before={power_before:.0} after={power_after:.0}"
        );
    }

    /// Sanity check: a non-zero-power signal subtracts to <10% of original
    /// power even at modest amplitude.
    #[test]
    fn subtract_reduces_power() {
        let msg = [0u8; 77];
        let itone = message_to_tones(&msg);
        let samples = tones_to_i16(&itone, 1500.0, 15_000);

        let mut audio = vec![0i16; 90_000];
        let offset = 6_000usize;
        let len = samples.len().min(90_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        let result = synthetic_result(msg, 1500.0, 0.0);
        subtract_signal(&mut audio, &result);

        let power_after: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        assert!(
            power_after < power_before * 0.10,
            "power before={power_before:.1} after={power_after:.1}"
        );
    }
}
