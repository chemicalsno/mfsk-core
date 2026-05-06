//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use alloc::vec::Vec;

use crate::msg::WsprMessage;

use super::demodulate_aligned;
use super::rx::{extract_tone_magnitudes, sync_score};
use super::search::{SearchParams, coarse_search};

/// One successful WSPR decode.
#[derive(Clone, Debug)]
pub struct WsprDecode {
    /// Recovered message payload.
    pub message: WsprMessage,
    /// Base frequency (tone 0) used for demodulation.
    pub freq_hz: f32,
    /// Sample index at which symbol 0 started, in the *caller's* audio
    /// buffer. **Clamped to 0** when the signal actually started before
    /// the buffer (negative-dt case); use [`Self::dt_sec`] for the
    /// signed offset that matches wsprd's reporting.
    pub start_sample: usize,
    /// wsprd-equivalent `dt`: signal-start offset in seconds, relative
    /// to the WSPR nominal anchor (slot start + 1 s). Positive values
    /// = signal arrived late, negative = arrived early. Range that
    /// `decode_scan` can express: `−NEGATIVE_DT_PAD_SEC .. +∞`.
    pub dt_sec: f32,
    /// 50-bit FEC info payload returned by Fano. Used by
    /// `decode_scan_subtract` to reconstruct the 162-channel-symbol
    /// stream and subtract the signal from the audio for SIC.
    pub info_bits: [u8; 50],
}

/// Decode one WSPR frame at a known (freq, start_sample). Returns `None`
/// if the Fano decoder fails to converge or the message doesn't unpack.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
) -> Option<WsprDecode> {
    use crate::core::{FecCodec, FecOpts, MessageCodec};
    let mut llrs = demodulate_aligned(audio, sample_rate, start_sample, freq_hz);
    deinterleave_llrs(&mut llrs);
    // Inline the FEC + unpack so we can capture the 50-bit info for
    // later SIC reconstruction.
    let codec = crate::fec::ConvFano;
    let fec_res = codec.decode_soft(&llrs, &FecOpts::default())?;
    let mut info_bits = [0u8; 50];
    info_bits.copy_from_slice(&fec_res.info);
    let message =
        crate::msg::Wspr50Message.unpack(&info_bits, &crate::core::DecodeContext::default())?;
    let dt_sec = start_sample as f32 / sample_rate as f32 - 1.0;
    Some(WsprDecode {
        message,
        freq_hz,
        start_sample,
        dt_sec,
        info_bits,
    })
}

/// Scan an audio buffer for any number of WSPR frames, returning all
/// successful decodes. Runs a coarse (freq, time) search with the given
/// [`SearchParams`], then attempts [`decode_at`] on each candidate in
/// score-descending order. Duplicate decodes (same message within ±5 Hz
/// and ±1 symbol) are collapsed to the single earliest-candidate hit,
/// so each transmission appears at most once in the output.
/// Refine a coarse candidate's (carrier, time) alignment by maximising
/// [`sync_score`] over a 2-D grid of (Δf, Δt) offsets. WSPR's coarse
/// search rounds carriers to the 1.4648-Hz FFT bin and start times to
/// quarter-symbol steps (170 ms); both are too loose for low-SNR
/// signals — Fano can't recover from > ±0.5 bin freq error or > ±50 ms
/// time error. Without this refinement, real WSJT-X recordings drop
/// >50 % of the decodes wsprd recovers.
///
/// Search radius defaults: ±2 Hz × ±170 ms (one t_step). 9 × 9 = 81
/// evaluations per candidate; in release that's < 0.3 s/candidate.
fn refine_align(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz_init: f32,
    freq_radius_hz: f32,
    freq_step_hz: f32,
    time_radius_samples: i64,
    time_step_samples: i64,
) -> (usize, f32) {
    let mut best = (start_sample, freq_hz_init);
    let mut best_score =
        match extract_tone_magnitudes(audio, sample_rate, start_sample, freq_hz_init) {
            Some(tm) => sync_score(&tm),
            None => f32::NEG_INFINITY,
        };
    let mut dt = -time_radius_samples;
    while dt <= time_radius_samples {
        let s_signed = start_sample as i64 + dt;
        if s_signed < 0 {
            dt += time_step_samples;
            continue;
        }
        let s = s_signed as usize;
        let mut df = -freq_radius_hz;
        while df <= freq_radius_hz + 1e-3 {
            let f = freq_hz_init + df;
            if let Some(tm) = extract_tone_magnitudes(audio, sample_rate, s, f) {
                let sc = sync_score(&tm);
                if sc > best_score {
                    best_score = sc;
                    best = (s, f);
                }
            }
            df += freq_step_hz;
        }
        dt += time_step_samples;
    }
    best
}

/// Half-window (in seconds) of front-side zero padding added before
/// the search runs. WSPR transmissions can start up to ~2 s **before**
/// the nominal slot anchor (wsprd reports such cases as `dt < -1.0`);
/// the missing pre-roll samples are not in the recording, but with
/// front padding the demodulator still aligns the rest of the frame
/// and Fano can recover from ~1–2 missing leading symbols. Mirrors
/// wsprd's `wspr_decode.f90` which prepends a configurable buffer
/// for the same reason.
const NEGATIVE_DT_PAD_SEC: f32 = 3.0;

pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprDecode> {
    // Prepend zeros so signals that started before audio[0] (negative
    // dt) become reachable. Internal `start_sample`s are shifted by
    // `pad`; we subtract `pad` back out before returning so callers
    // see the original time base.
    let pad = (NEGATIVE_DT_PAD_SEC * sample_rate as f32) as usize;
    let mut padded = alloc::vec![0f32; pad + audio.len()];
    padded[pad..].copy_from_slice(audio);
    let nominal_shifted = nominal_start_sample + pad;
    let cands = coarse_search(&padded, sample_rate, nominal_shifted, params);
    let audio = &padded[..]; // shadow so all downstream reads use padded buffer
    let mut seen: Vec<WsprDecode> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192; // one WSPR symbol at 12 kHz
    // 2-D refinement: WSPR's Fano (K=32 convolutional, no CRC) is
    // sensitive to *both* sub-bin freq and sub-t_step time mis-
    // alignment. Coarse-search rounds to 1.46 Hz / 170 ms; this
    // pass refines:
    //   time : ±170 ms / 43 ms step ⇒ 9 points
    //   freq : ±2 Hz   / 0.5 Hz step ⇒ 9 points
    // ≈ 81 sync_score evals × candidate.
    //
    // Going finer in time (e.g. 10 ms) actually *hurts* recall on
    // weak signals: WSPR has no CRC, so the highest-sync_score
    // alignment is often a noise-pattern Fano ghost rather than the
    // true signal. 43 ms preserves true peaks while the coarser
    // grid keeps us out of the ghost-attractor region. Better
    // long-term fix is a Fano-metric / callsign-sanity gate; until
    // then, 43 ms is the empirical optimum on the WSJT-X golden.
    // Tightened grid (3 × 3 = 9 evals/cand, was 9 × 9 = 81): wsprd's
    // entire 3-pass SIC runs in seconds; our refine cost dominated
    // total wall time. The small grid still recovers the same 5/8
    // goldens against the WSJT-X reference WAV — most of the recall
    // win came from sub-bin demod (slice 1 of issue #17), not from
    // the dense refine grid.
    const REFINE_FREQ_RADIUS_HZ: f32 = 1.0;
    const REFINE_FREQ_STEP_HZ: f32 = 1.0;
    let nsps = (sample_rate as f32 * <super::Wspr as crate::core::ModulationParams>::SYMBOL_DT)
        .round() as i64;
    let refine_time_radius = nsps / 8; // ≈85 ms half-window
    let refine_time_step = nsps / 8; // 1 step at radius → 3 cells in time
    for c in cands {
        let (start_refined, freq_refined) = refine_align(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            REFINE_FREQ_RADIUS_HZ,
            REFINE_FREQ_STEP_HZ,
            refine_time_radius,
            refine_time_step,
        );
        let Some(mut d) = decode_at(audio, sample_rate, start_refined, freq_refined) else {
            continue;
        };
        // Translate alignment back to the caller's time base.
        // `dt_sec` is the source of truth (signed); `start_sample` is
        // clamped to 0 when the alignment lands inside the prepended
        // silence so its `usize` type is preserved.
        d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
        d.start_sample = start_refined.saturating_sub(pad);
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= TIME_DEDUP_SAMPLES
        });
        if !dup {
            seen.push(d);
        }
    }
    seen
}

/// Convenience: scan using [`SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<WsprDecode> {
    decode_scan(audio, sample_rate, 0, &SearchParams::default())
}

/// WSPR subtract configuration (continuous-phase 4-FSK). Mirrors WSJT-X
/// `subtract_signal2` in `wsprd.c`: tone spacing 1.4648 Hz, 8192
/// samples/symbol at 12 kHz, no GFSK shaping (WSPR is plain CPFSK).
const WSPR_SUBTRACT: crate::core::dsp::subtract::SubtractCfg =
    crate::core::dsp::subtract::SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 1.4648,
        samples_per_symbol: 8192,
        // WSPR's nominal symbol-0 start is 1.0 s into the slot; our
        // `start_sample` is already absolute, so the subtract layer
        // sees `dt_sec` as `(start - 1.0*FS) / FS`. `base_offset_s = 1.0`
        // matches the convention used by `WsprDecode::dt_sec`.
        base_offset_s: 1.0,
        gfsk: None,
    };

/// LPF kernel half-width for the channel-aware subtract (currently
/// unused — see comment in [`decode_scan_subtract`] for why we use
/// the constant-amplitude path instead).
#[allow(dead_code)]
const WSPR_SUBTRACT_LPF_HALF: usize = 600;

/// Multi-pass WSPR decode with successive interference cancellation.
///
/// Mirrors WSJT-X `wsprd.c` `npasses=3` SIC loop (`wsprd.c:998-1438`):
/// each pass runs `decode_scan` on the current residual audio, decodes
/// every signal that survives Fano + (eventually) callsign sanity,
/// reconstructs the on-air channel-symbol sequence via
/// [`super::encode_channel_symbols`], and subtracts a channel-aware
/// LPF reference from the audio so subsequent passes can expose
/// previously-masked weak signals.
///
/// Returns deduplicated decodes from all passes.
pub fn decode_scan_subtract(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprDecode> {
    use crate::core::dsp::subtract::subtract_tones;

    // The subtract helper takes `&mut [i16]`; convert once, mutate
    // across passes, work on `f32` for `decode_scan` per pass.
    let mut residual_i16: Vec<i16> = audio
        .iter()
        .map(|&x| (x * 32767.0).clamp(-32768.0, 32767.0) as i16)
        .collect();

    let mut all: Vec<WsprDecode> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192;
    // wsprd uses 3 passes. Our `decode_scan` is expensive (~30 s on
    // a 120-s WSPR slot due to the 2-D refine grid), so we cap at 2
    // — empirically the bulk of the SIC benefit lands on pass 2 once
    // the strong KB0VHA-class signals have been removed.
    const NPASSES: usize = 2;

    for _pass in 0..NPASSES {
        // Re-convert residual back to f32 for decode_scan (it expects
        // unit-scale samples).
        let residual_f32: Vec<f32> = residual_i16.iter().map(|&s| s as f32 / 32_768.0).collect();
        let new_decodes = decode_scan(&residual_f32, sample_rate, nominal_start_sample, params);
        if new_decodes.is_empty() {
            break;
        }
        let mut added = 0usize;
        for d in new_decodes {
            let dup = all.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
            if dup {
                continue;
            }
            // Reconstruct the on-air channel symbols (162 4-FSK tones)
            // from the recovered 50-bit info, and subtract from the
            // residual at the decoded (freq, dt). Mirrors wsprd.c:1432-
            // 1437 `subtract_signal2(idat, qdat, …, channel_symbols)`.
            let symbols = super::encode_channel_symbols(&d.info_bits);
            // Constant-amplitude LS subtract; ignores QSB / drift but
            // is O(N) and avoids the multi-second convolution that
            // `subtract_tones_lpf` (direct conv, kernel ~600 samples)
            // would cost on a 1.4 M-sample WSPR slot. WSJT-X uses the
            // LPF version on a decimated signal — we'll match once we
            // have FFT-conv or a decimate-process-interpolate path.
            subtract_tones(
                &mut residual_i16,
                &symbols,
                d.freq_hz,
                d.dt_sec,
                1.0,
                &WSPR_SUBTRACT,
            );
            all.push(d);
            added += 1;
        }
        if added == 0 {
            break;
        }
    }
    all
}

/// Deinterleave 162 LLRs in place (same permutation as [`deinterleave`]
/// but for `f32` values).
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    let mut tmp = [0f32; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        // Inline the bit-reverse-8 to avoid exposing a pub helper.
        let i64 = i as u64;
        let j = ((((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64))
            >> 32) as u8 as usize;
        if j < 162 {
            tmp[p as usize] = llrs[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *llrs = tmp;
}

#[cfg(test)]
mod tests {
    use super::super::search::SearchParams;
    use super::super::synthesize_type1;
    use super::*;
    use crate::msg::WsprMessage;

    #[test]
    fn synth_decode_roundtrip_k1abc_fn42_37() {
        let freq = 1500.0;
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("valid message");
        let r = decode_at(&audio, 12_000, 0, freq).expect("decode");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn scan_recovers_message_without_freq_hint() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let decodes = decode_scan(
            &audio,
            12_000,
            0,
            &SearchParams {
                freq_min_hz: 1450.0,
                freq_max_hz: 1550.0,
                ..SearchParams::default()
            },
        );
        assert!(!decodes.is_empty(), "at least one decode");
        let d = decodes.into_iter().next().unwrap();
        assert_eq!(
            d.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
        assert!((d.freq_hz - 1500.0).abs() <= 2.0);
    }

    #[test]
    fn survives_moderate_awgn() {
        use std::f32::consts::PI;

        let freq = 1500.0;
        let mut audio =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.5).expect("valid message");

        // Deterministic "noise": superposition of a handful of off-tone
        // sinusoids plus a pseudorandom dither. This is a cheap AWGN
        // stand-in that keeps the test free of rand dependencies.
        let mut seed: u32 = 0x1234_5678;
        for (i, s) in audio.iter_mut().enumerate() {
            // Linear congruential pseudorandom for reproducible noise.
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let rnd = ((seed >> 16) as f32 / 32768.0 - 1.0) * 0.10;
            let off = 0.05 * (2.0 * PI * 2345.7 * i as f32 / 12_000.0).sin();
            *s += rnd + off;
        }

        let r = decode_at(&audio, 12_000, 0, freq).expect("decode under noise");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K9AN".into(),
                grid: "EN50".into(),
                power_dbm: 33,
            }
        );
    }
}
