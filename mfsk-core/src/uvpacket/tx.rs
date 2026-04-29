// SPDX-License-Identifier: GPL-3.0-or-later
//! TX path: bytes → 12 kHz f32 PCM audio.
//!
//! **Phase 2 modulation pivot**: this module implements **single-
//! carrier coherent QPSK** with root raised-cosine pulse shaping,
//! a 31-bit BPSK m-sequence preamble at the frame head, and
//! periodic QPSK pilot symbols for receiver-side phase tracking.
//! See `docs/0.3.1_PLAN.md` for the rationale.
//!
//! Pipeline:
//!
//! ```text
//! bytes + (mode, block_count, app_type, sequence) + audio_centre_hz
//!   ↓ framing::pack_to_size                  N×12 byte frame data
//!   ↓ slice into 12-byte LDPC info chunks
//!   ↓ for each block i:
//!       info[ 0..96 ]  = 12-byte payload chunk
//!       info[96..101]  = D-iii spread-header chunk
//!       Ldpc240_101::encode → 240-bit codeword
//!   ↓ puncture per mode (existing kSR-greedy keep set)
//!   ↓ block-interleave across all blocks
//!   ↓ map channel-bit pairs → QPSK constellation indices (Gray map)
//!   ↓ build symbol stream:
//!       [31-sym BPSK m-sequence preamble]
//!       [pilot, 31 data, pilot, 31 data, ..., pilot, ≤31 data]
//!   ↓ RRC pulse shape (α = 0.5, span 6 sym, NSPS = 10) →
//!     complex baseband samples at 12 kHz
//!   ↓ upconvert to audio centre 1500 Hz: Re{baseband · e^{j·2πfc·t}}
//!   → Vec<f32> at 12 kHz
//! ```
//!
//! Everything **above** the QPSK mapping is unchanged from the
//! Phase 1 4-FSK design — framing, LDPC encode, kSR-greedy
//! puncture, block interleaver, D-iii spread header all reuse
//! the existing modules.

use num_complex::Complex32;
use std::f32::consts::PI;

use crate::core::FecCodec;
use crate::fec::Ldpc240_101;

use super::framing::{FrameHeader, HEADER_BYTES, INFO_BYTES_PER_BLOCK, PackError, pack_to_size};
use super::interleaver::interleave;
use super::puncture::puncture;
use super::sync_pattern::{PILOT_QPSK_POINT, PILOT_SYMBOL_INTERVAL, UVPACKET_PREAMBLE_BPSK_BITS};

/// LDPC mother-codeword length.
const N_LDPC: usize = 240;
/// LDPC info-bit count.
const K_LDPC: usize = 101;
/// Payload bits per block (first 96 of the 101 info bits).
const PAYLOAD_BITS_PER_BLOCK: usize = INFO_BYTES_PER_BLOCK * 8; // 96
/// Spread header bits per block.
const HEADER_CHUNK_BITS: usize = K_LDPC - PAYLOAD_BITS_PER_BLOCK; // 5
/// Spread header repeats every 7 blocks.
const HEADER_SPREAD_PERIOD: usize = 7;
/// Total header bits.
const HEADER_BITS: usize = HEADER_BYTES * 8; // 32

/// 4-symbol QPSK Gray map (bit pair → constellation index). Bit
/// pair `(b1, b0)` gives the index `(b1 << 1) | b0`; the Gray map
/// rotates indices so that adjacent constellation points differ
/// by one bit, matching the FT4 / Phase 1 mapping.
const GRAY_4: [u8; 4] = [0, 1, 3, 2];

/// Sample rate of the modem.
const SAMPLE_RATE_HZ: f32 = 12_000.0;
/// 1200 baud → 10 samples / symbol at 12 kHz.
const NSPS: usize = 10;
/// RRC pulse: span in symbols (3 each side of the centre tap).
const RRC_SPAN_SYMS: usize = 6;
/// RRC roll-off factor.
const RRC_ALPHA: f32 = 0.5;

/// Encode a uvpacket frame to 12 kHz f32 PCM audio.
///
/// `header` carries per-frame metadata (mode + block count + app
/// type + sequence). `payload` is the application-layer byte stream
/// (length must be ≤ `header.block_count * 12 - 4`).
/// `audio_centre_hz` is the carrier frequency to upconvert the QPSK
/// baseband to (typically 1500 Hz; clearing both the typical NFM
/// HT high-pass at 300–500 Hz and the audio-passband corner ≥ 2.7
/// kHz).
///
/// Returns owned `Vec<f32>` PCM at 12 kHz with peak amplitude ≤ 1.
/// Length depends on mode / n_blocks / pilot count + RRC tail —
/// see [`expected_total_symbols`] for the symbol-count formula.
pub fn encode(
    header: &FrameHeader,
    payload: &[u8],
    audio_centre_hz: f32,
) -> Result<Vec<f32>, PackError> {
    let mode = header.mode;
    let n_blocks = header.block_count as usize;

    let per_frame_capacity = n_blocks
        .saturating_mul(INFO_BYTES_PER_BLOCK)
        .saturating_sub(HEADER_BYTES);
    if payload.len() > per_frame_capacity {
        return Err(PackError::PayloadTooLarge(payload.len()));
    }

    // 1. Pack header + payload + zero-pad → exactly N×12 bytes; CRC
    //    covers header word + (payload + padding).
    let frame_data_total = n_blocks * INFO_BYTES_PER_BLOCK;
    let frame_data = pack_to_size(header, payload, frame_data_total)?;
    let header_bytes: [u8; HEADER_BYTES] = frame_data[..HEADER_BYTES].try_into().unwrap();

    // 2. Header bits MSB-first, for the per-block 5-bit spread copy.
    let mut header_bits = [0u8; HEADER_BITS];
    for (i, bit) in header_bits.iter_mut().enumerate() {
        let byte = header_bytes[i / 8];
        *bit = (byte >> (7 - (i % 8))) & 1;
    }

    // 3. LDPC-encode every block.
    let fec = Ldpc240_101;
    let mut info_buf = vec![0u8; K_LDPC];
    let mut codeword_buf = vec![0u8; N_LDPC];
    let mut concat_codewords = Vec::with_capacity(n_blocks * N_LDPC);

    for block_idx in 0..n_blocks {
        let payload_chunk =
            &frame_data[block_idx * INFO_BYTES_PER_BLOCK..(block_idx + 1) * INFO_BYTES_PER_BLOCK];
        for (byte_idx, &byte) in payload_chunk.iter().enumerate() {
            for bit_idx in 0..8 {
                info_buf[byte_idx * 8 + bit_idx] = (byte >> (7 - bit_idx)) & 1;
            }
        }
        let chunk_offset = HEADER_CHUNK_BITS * (block_idx % HEADER_SPREAD_PERIOD);
        for chunk_bit in 0..HEADER_CHUNK_BITS {
            let header_bit_idx = chunk_offset + chunk_bit;
            info_buf[PAYLOAD_BITS_PER_BLOCK + chunk_bit] = if header_bit_idx < HEADER_BITS {
                header_bits[header_bit_idx]
            } else {
                0
            };
        }
        fec.encode(&info_buf, &mut codeword_buf);
        concat_codewords.extend_from_slice(&codeword_buf);
    }

    // 4. Puncture per mode.
    let block_ch_bits = mode.ch_bits_per_block();
    let mut punctured_concat = Vec::with_capacity(n_blocks * block_ch_bits);
    for block_idx in 0..n_blocks {
        let cw = &concat_codewords[block_idx * N_LDPC..(block_idx + 1) * N_LDPC];
        punctured_concat.extend_from_slice(&puncture(cw, mode));
    }

    // 5. Block-interleave.
    let interleaved = interleave(&punctured_concat, n_blocks);

    // 6. Map channel-bit pairs to QPSK constellation indices.
    debug_assert!(interleaved.len().is_multiple_of(2));
    let n_data_syms = interleaved.len() / 2;
    let mut qpsk_data: Vec<u8> = Vec::with_capacity(n_data_syms);
    for sym_idx in 0..n_data_syms {
        let pair = (interleaved[sym_idx * 2] << 1) | interleaved[sym_idx * 2 + 1];
        qpsk_data.push(GRAY_4[pair as usize]);
    }

    // 7. Build the symbol stream:
    //    [preamble (31 BPSK)]
    //    [pilot, 31 data, pilot, 31 data, ..., pilot, ≤31 data]
    let mut symbols: Vec<Complex32> = Vec::new();
    for &b in UVPACKET_PREAMBLE_BPSK_BITS.iter() {
        // m-sequence bit `true` → BPSK -1, `false` → +1.
        symbols.push(Complex32::new(if b { -1.0 } else { 1.0 }, 0.0));
    }
    let pilot = qpsk_constellation_point(PILOT_QPSK_POINT);
    let data_per_interval = PILOT_SYMBOL_INTERVAL - 1;
    let mut data_idx = 0;
    while data_idx < qpsk_data.len() {
        symbols.push(pilot);
        let end = (data_idx + data_per_interval).min(qpsk_data.len());
        for i in data_idx..end {
            symbols.push(qpsk_constellation_point(qpsk_data[i]));
        }
        data_idx = end;
    }

    // 8. RRC pulse-shape into a complex baseband.
    let rrc = rrc_pulse(RRC_ALPHA, RRC_SPAN_SYMS, NSPS);
    let total_samples = symbols.len() * NSPS + rrc.len();
    let mut baseband = vec![Complex32::new(0.0, 0.0); total_samples];
    let center_offset = rrc.len() / 2; // align symbol centres at half-pulse
    for (i, &sym) in symbols.iter().enumerate() {
        let start = i * NSPS;
        for (j, &tap) in rrc.iter().enumerate() {
            let pos = start + j;
            if pos < baseband.len() {
                baseband[pos] += sym * tap;
            }
        }
    }
    let _ = center_offset; // (TX uses left-aligned convolution; centring matters only for RX)

    // 9. Upconvert: real audio = Re{baseband · e^{j 2π fc n / fs}}.
    let mut audio = vec![0.0_f32; total_samples];
    let two_pi_fc_dt = 2.0 * PI * audio_centre_hz / SAMPLE_RATE_HZ;
    for n in 0..total_samples {
        let phase = two_pi_fc_dt * n as f32;
        let (s, c) = phase.sin_cos();
        audio[n] = baseband[n].re * c - baseband[n].im * s;
    }

    // 10. Normalise so the peak is ≤ 1.0 (the σ formula assumes
    //     unit peak; RRC + sum-of-symbols can briefly overshoot
    //     ±1 during transitions).
    let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
    if peak > 1.0 {
        let scale = 1.0 / peak;
        for s in audio.iter_mut() {
            *s *= scale;
        }
    }

    Ok(audio)
}

/// QPSK constellation: `index 0 → +1+0j`, `1 → 0+1j`,
/// `2 → −1+0j`, `3 → 0−1j`.
fn qpsk_constellation_point(idx: u8) -> Complex32 {
    match idx & 0x3 {
        0 => Complex32::new(1.0, 0.0),
        1 => Complex32::new(0.0, 1.0),
        2 => Complex32::new(-1.0, 0.0),
        3 => Complex32::new(0.0, -1.0),
        _ => unreachable!(),
    }
}

/// Generate root-raised-cosine pulse coefficients. Returns
/// `span_syms × samples_per_sym + 1` taps, normalised so that
/// `Σ h² = 1`.
fn rrc_pulse(alpha: f32, span_syms: usize, samples_per_sym: usize) -> Vec<f32> {
    let n = span_syms * samples_per_sym;
    let mut h = vec![0.0_f32; n + 1];
    let center = n as f32 / 2.0;
    for (i, h_i) in h.iter_mut().enumerate() {
        let t = (i as f32 - center) / samples_per_sym as f32;
        *h_i = if t.abs() < 1e-6 {
            // L'Hôpital limit at t = 0.
            1.0 - alpha + 4.0 * alpha / PI
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-6 {
            // L'Hôpital limit at t = ±1/(4α).
            (alpha / 2.0_f32.sqrt())
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let pi_t = PI * t;
            let four_at = 4.0 * alpha * t;
            ((pi_t * (1.0 - alpha)).sin() + four_at * (pi_t * (1.0 + alpha)).cos())
                / (pi_t * (1.0 - four_at * four_at))
        };
    }
    let norm: f32 = h.iter().map(|x| x * x).sum::<f32>().sqrt();
    if norm > 0.0 {
        for x in h.iter_mut() {
            *x /= norm;
        }
    }
    h
}

/// Compute the total transmitted symbol count for a given mode +
/// block count: 31-sym preamble + (pilots interleaved with data).
pub fn expected_total_symbols(mode: super::puncture::Mode, n_blocks: u8) -> usize {
    let block_ch_bits = mode.ch_bits_per_block();
    let n_data = (n_blocks as usize) * block_ch_bits / 2; // 2 bits / QPSK sym
    let data_per_interval = PILOT_SYMBOL_INTERVAL - 1;
    let n_pilots = n_data.div_ceil(data_per_interval);
    UVPACKET_PREAMBLE_BPSK_BITS.len() + n_pilots + n_data
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvpacket::AUDIO_CENTRE_HZ;
    use crate::uvpacket::Mode;

    fn header_for(mode: Mode, n_blocks: u8) -> FrameHeader {
        FrameHeader {
            mode,
            block_count: n_blocks,
            app_type: 1,
            sequence: 0,
        }
    }

    /// Smoke: encode succeeds for every mode × representative
    /// frame size.
    #[test]
    fn encode_succeeds_all_modes() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            for n_blocks in [1u8, 4, 18, 32] {
                let header = header_for(mode, n_blocks);
                let cap = (n_blocks as usize) * INFO_BYTES_PER_BLOCK - HEADER_BYTES;
                let payload = vec![0xA5_u8; cap];
                let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                assert!(!audio.is_empty(), "{mode:?} n={n_blocks}: empty audio");
            }
        }
    }

    /// Audio peak must be ≤ 1 (the σ-for-Eb/N0 formula assumes
    /// unit peak; we normalise the burst to match).
    #[test]
    fn encode_peak_amplitude_bounded() {
        let header = header_for(Mode::Robust, 4);
        let audio = encode(&header, b"hello", AUDIO_CENTRE_HZ).unwrap();
        let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
        assert!(peak <= 1.0001, "peak {peak} > 1");
    }

    /// Sample-count must match `expected_total_symbols × NSPS +
    /// RRC tail`.
    #[test]
    fn encode_sample_count_matches_formula() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            let n_blocks = 4u8;
            let header = header_for(mode, n_blocks);
            let cap = (n_blocks as usize) * INFO_BYTES_PER_BLOCK - HEADER_BYTES;
            let payload = vec![0xCC_u8; cap];
            let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();

            let n_syms = expected_total_symbols(mode, n_blocks);
            let rrc_len = RRC_SPAN_SYMS * NSPS + 1;
            let expected = n_syms * NSPS + rrc_len;
            assert_eq!(
                audio.len(),
                expected,
                "{mode:?}: got {} samples, expected {}",
                audio.len(),
                expected,
            );
        }
    }

    /// Distinct payloads must produce substantively different
    /// audio.
    #[test]
    fn distinct_payloads_diverge() {
        let header = header_for(Mode::Robust, 4);
        let a = encode(&header, b"alpha", AUDIO_CENTRE_HZ).unwrap();
        let b = encode(&header, b"bravo", AUDIO_CENTRE_HZ).unwrap();
        assert_eq!(a.len(), b.len());
        let differences = a
            .iter()
            .zip(b.iter())
            .filter(|(x, y)| (**x - **y).abs() > 1e-4)
            .count();
        assert!(
            differences > a.len() / 4,
            "expected substantial divergence, got {differences} / {}",
            a.len(),
        );
    }

    /// Higher-rate modes (more puncturing) → fewer transmitted
    /// symbols → shorter audio.
    #[test]
    fn modes_have_decreasing_audio_length() {
        let n_blocks = 8u8;
        let payload = vec![0_u8; 32];
        let lens: Vec<usize> = [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express]
            .iter()
            .map(|&m| {
                encode(&header_for(m, n_blocks), &payload, AUDIO_CENTRE_HZ)
                    .unwrap()
                    .len()
            })
            .collect();
        for w in lens.windows(2) {
            assert!(
                w[0] >= w[1],
                "expected non-increasing audio lengths: {lens:?}"
            );
        }
        // Robust strictly longer than Express (different ch_bits).
        assert!(lens[0] > lens[3]);
    }

    /// Oversize payload → PackError.
    #[test]
    fn oversize_payload_rejected() {
        let header = header_for(Mode::Robust, 1);
        let too_big = vec![0_u8; INFO_BYTES_PER_BLOCK]; // 12 byte > capacity 8.
        assert!(matches!(
            encode(&header, &too_big, AUDIO_CENTRE_HZ).unwrap_err(),
            PackError::PayloadTooLarge(_),
        ));
    }
}
