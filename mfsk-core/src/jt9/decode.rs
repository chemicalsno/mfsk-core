//! JT9 decode pipeline: WSJT-X-faithful softsym (downsam9 → peakdt9 →
//! AFC → twkfreq → symspec2 → fano232).
//!
//! Replaces the earlier box-car path in `baseband.rs` + `demod_bb.rs`.
//! The big audio FFT is computed once and reused across candidates.

use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};
use crate::fec::ConvFano232;
use crate::msg::{Jt72Codec, Jt72Message};

use super::Jt9Decode;
use super::softsym::{AudioFft, FSAMPLE_DOWN, afc_simple, llrs_from_c5, peakdt9, twkfreq_const};

/// Sync-score gate: peakdt9 returns `(sync_avg/data_avg)−1`. For pure
/// noise this is ≈ 0; clean JT9 frames sit in the 0.3–10 range. WSJT-X
/// uses ccfbest > 30 (raw integrated power) on its own scale; the 1.5
/// threshold here is calibrated from the 130418_1742 sample (golden
/// signals score 1.5..15, phantoms < 0.8).
const SYNC_GATE: f32 = 1.5;

/// Try to decode a JT9 signal centred at `freq_hz` using a pre-built
/// audio FFT. Returns `None` when the sync gate is missed, Fano fails
/// to converge, or the message is not `Jt72Message::Standard`.
pub fn decode_at_baseband_with_fft(big_fft: &AudioFft, freq_hz: f32) -> Option<Jt9Decode> {
    if freq_hz <= 0.0 {
        return None;
    }

    let c2 = big_fft.downsam9(freq_hz);
    let (lagpk, sync_score, mut c3) = peakdt9(&c2);
    if !sync_score.is_finite() || sync_score < SYNC_GATE {
        return None;
    }

    // Simple AFC: measure residual sub-tone offset, apply via twkfreq.
    let df = afc_simple(&c3);
    twkfreq_const(&mut c3, df);

    let llrs = llrs_from_c5(&c3);
    let res = ConvFano232.decode_soft(&llrs, &FecOpts::default())?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    let msg = Jt72Codec::default().unpack(&payload, &DecodeContext::default())?;

    // JT9 has no CRC — Fano can converge on plausible-looking junk.
    // Accepting only Standard form drops most hashed-callsign garbage.
    match &msg {
        Jt72Message::Standard { .. } => {}
        _ => return None,
    }

    // Translate (lagpk, df) back to audio-sample / Hz coordinates so
    // downstream consumers see a consistent timing/freq report.
    let start_sample = lag_to_audio_sample(lagpk);
    let freq_corrected = freq_hz - df;
    Some(Jt9Decode {
        message: msg,
        freq_hz: freq_corrected,
        start_sample,
    })
}

/// One-shot variant: builds the big FFT inline. Useful for tests and
/// callers that don't reuse the same audio across many candidates.
#[cfg(test)]
#[allow(dead_code)]
pub fn decode_at_baseband(
    audio: &[f32],
    _sample_rate: u32,
    _start_sample: usize,
    freq_hz: f32,
) -> Option<Jt9Decode> {
    let big = AudioFft::build(audio);
    decode_at_baseband_with_fft(&big, freq_hz)
}

/// Convert peakdt9 lag (in 27.78 Hz baseband samples) back to a
/// 12 kHz audio-sample index. Reverses the offsetting that peakdt9
/// applies when slicing c3 out of c2.
fn lag_to_audio_sample(lagpk: i64) -> usize {
    // peakdt9 places sample 0 of c3 at c2 index `lagpk - i0 - NSPSD + 1`
    // where i0 = 5 * NSPSD. Each 27.78 Hz sample equals NDOWN = 432
    // audio samples at 12 kHz.
    let i0 = 5 * 16i64;
    let nspsd = 16i64;
    let ndown = 432i64;
    let c2_offset = lagpk - i0 - nspsd + 1;
    // Audio-sample index of symbol-0 within the original audio.
    let sample = c2_offset.max(0) * ndown;
    let _ = FSAMPLE_DOWN; // silence unused
    sample as usize
}
