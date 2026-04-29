// SPDX-License-Identifier: GPL-3.0-or-later
//! TX path: bytes → 12 kHz f32 PCM audio.
//!
//! Pipeline:
//!
//! ```text
//! bytes + (mode, app_type, sequence) + audio_centre_hz
//!   ↓ framing::pack                            4-byte header + payload bytes
//!   ↓ pad to N × 12 byte (N = block_count)
//!   ↓ slice into 12-byte chunks                one chunk → one LDPC info block
//!   ↓ for each block i ∈ 0..N:
//!       info[ 0..96] = 12-byte payload chunk
//!       info[96..101] = 5-bit "spread header" chunk
//!                       (= header bits [5(i mod 7) .. 5(i mod 7)+5) ;
//!                        zero where the chunk would extend past bit 31)
//!   ↓ Ldpc240_101::encode each block           240-bit codeword
//!   ↓ puncture per mode                        ch_bits_per_block(mode) bits/block
//!   ↓ block-interleaver across all blocks      same total length
//!   ↓ build symbol stream:
//!       for chunk i ∈ 0..N:
//!         [Costas-4]
//!         [chunk i bits, mapped via 4-FSK Gray to ch_bits/2 symbols]
//!       [trailing Costas-4]
//!   ↓ GFSK synth at audio_centre_hz - 1.5 × tone_spacing
//!   → Vec<f32> at 12 kHz
//! ```
//!
//! ## Spread header (D-iii)
//!
//! Each LDPC info block has 101 information bits; the first 96 carry
//! 12 bytes of frame data (so the **whole** 4-byte frame header sits
//! in block 0's payload at bit positions 0..32). The remaining 5
//! bits per block carry a **redundant copy** of the 32-bit frame
//! header, distributed via:
//!
//! ```text
//! info[96..101]  =  header_bits[5(i mod 7) .. 5(i mod 7) + 5)
//! ```
//!
//! For frames with `n_blocks ≥ 7` the full 32-bit header gets at
//! least one complete copy across the 5-bit fields (each 7 blocks
//! is one cycle); 32-block frames carry > 4 redundant cycles. If
//! block 0 fails to decode but several other blocks survive, the
//! receiver can reconstruct the header from the 5-bit fields by
//! brute-forcing any missing chunks against the frame CRC-16.
//!
//! For `n_blocks < 7` the spread is partial — receiver still uses
//! block 0's payload as the primary header source.

use crate::core::dsp::gfsk::{GfskCfg, synth_f32};
use crate::core::{FecCodec, ModulationParams};
use crate::fec::Ldpc240_101;

use super::framing::{FrameHeader, HEADER_BYTES, INFO_BYTES_PER_BLOCK, PackError, pack_to_size};
use super::interleaver::interleave;
use super::puncture::puncture;
use super::sync_pattern::UVPACKET_COSTAS;

/// LDPC mother-codeword length.
const N_LDPC: usize = 240;
/// LDPC info-bit count.
const K_LDPC: usize = 101;
/// Payload bits per block — first 96 of the 101 info bits.
const PAYLOAD_BITS_PER_BLOCK: usize = INFO_BYTES_PER_BLOCK * 8; // 96
/// Spread header bits per block — last 5 of the 101 info bits.
const HEADER_CHUNK_BITS: usize = K_LDPC - PAYLOAD_BITS_PER_BLOCK; // 5
/// Spread header period (every 7 blocks the cycle repeats).
const HEADER_SPREAD_PERIOD: usize = 7;
/// Total header bits.
const HEADER_BITS: usize = HEADER_BYTES * 8; // 32

/// 4-FSK Gray map (bit pair → tone index). Same permutation as FT4
/// (`[0, 1, 3, 2]`).
const GRAY_4: [u8; 4] = [0, 1, 3, 2];

/// Tone-spacing-derived frequency offset of the lowest tone (`tone 0`)
/// from the audio centre.
const TONE_SPACING_HZ: f32 = 600.0;
const LOWEST_TONE_OFFSET_HZ: f32 = -1.5 * TONE_SPACING_HZ;

/// Encode a uvpacket frame to 12 kHz f32 audio.
///
/// `header` carries the per-frame metadata (mode + block count + app
/// type + sequence). `payload` is the application-layer byte stream
/// (length must be ≤ `header.block_count * 12 - 4`). `audio_centre_hz`
/// is the mid-band carrier (recommended: 1700 Hz, exposed as
/// [`crate::uvpacket::AUDIO_CENTRE_HZ`]).
///
/// Returns owned `Vec<f32>` PCM at 12 kHz, ready to feed an audio
/// output device. Length is `(header.block_count × (4 + ch_bits/2)
/// + 4) × NSPS` samples, where `NSPS = 10` (= 12 kHz / 1200 baud) and
/// `ch_bits = mode.ch_bits_per_block()`.
///
/// Returns `Err(PackError)` if the header fields are out of range or
/// the payload exceeds the per-frame capacity.
pub fn encode(
    header: &FrameHeader,
    payload: &[u8],
    audio_centre_hz: f32,
) -> Result<Vec<f32>, PackError> {
    let mode = header.mode;
    let n_blocks = header.block_count as usize;

    // Per-frame payload capacity: n_blocks × 12 byte − 4 byte header.
    // Reject before `pack` so the caller sees a single failure mode
    // rather than a downstream panic.
    let per_frame_capacity = n_blocks
        .saturating_mul(INFO_BYTES_PER_BLOCK)
        .saturating_sub(HEADER_BYTES);
    if payload.len() > per_frame_capacity {
        return Err(PackError::PayloadTooLarge(payload.len()));
    }

    // 1. Pack header + payload + zero-padding into exactly the LDPC
    // info-byte budget for n_blocks. The CRC is computed over header
    // word + (payload + padding), so the receiver can verify integrity
    // without having to know the original payload length up front.
    let frame_data_total = n_blocks * INFO_BYTES_PER_BLOCK;
    let frame_data = pack_to_size(header, payload, frame_data_total)?;
    let header_bytes: [u8; HEADER_BYTES] = frame_data[..HEADER_BYTES].try_into().unwrap();

    // 2. Pre-compute the 32-bit header bits (MSB-first per byte) for
    // the per-block spread copy.
    let mut header_bits = [0u8; HEADER_BITS];
    for (i, bit) in header_bits.iter_mut().enumerate() {
        let byte = header_bytes[i / 8];
        *bit = (byte >> (7 - (i % 8))) & 1;
    }

    // 3. Encode each LDPC block.
    let fec = Ldpc240_101;
    let mut info_buf = vec![0u8; K_LDPC];
    let mut codeword_buf = vec![0u8; N_LDPC];
    let mut concat_codewords = Vec::with_capacity(n_blocks * N_LDPC);

    for block_idx in 0..n_blocks {
        let payload_chunk =
            &frame_data[block_idx * INFO_BYTES_PER_BLOCK..(block_idx + 1) * INFO_BYTES_PER_BLOCK];

        // info[0..96]: payload bits, MSB-first per byte.
        for (byte_idx, &byte) in payload_chunk.iter().enumerate() {
            for bit_idx in 0..8 {
                info_buf[byte_idx * 8 + bit_idx] = (byte >> (7 - bit_idx)) & 1;
            }
        }

        // info[96..101]: spread-header chunk for this block.
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

    // 4. Puncture each codeword per mode.
    let block_ch_bits = mode.ch_bits_per_block();
    let mut punctured_concat = Vec::with_capacity(n_blocks * block_ch_bits);
    for block_idx in 0..n_blocks {
        let cw = &concat_codewords[block_idx * N_LDPC..(block_idx + 1) * N_LDPC];
        let p = puncture(cw, mode);
        punctured_concat.extend_from_slice(&p);
    }

    // 5. Block-interleave across all blocks.
    let interleaved = interleave(&punctured_concat, n_blocks);

    // 6. Build symbol stream: per-block Costas-4 + data symbols, then
    // trailing Costas-4. 4-FSK = 2 bits/symbol, so block_ch_bits is
    // even by construction (240/202/152/134).
    debug_assert!(block_ch_bits.is_multiple_of(2));
    let block_data_syms = block_ch_bits / 2;
    let costas_len = UVPACKET_COSTAS.len();
    let total_syms = n_blocks * (costas_len + block_data_syms) + costas_len;
    let mut syms = Vec::with_capacity(total_syms);

    for block_idx in 0..n_blocks {
        syms.extend_from_slice(&UVPACKET_COSTAS);
        let chunk = &interleaved[block_idx * block_ch_bits..(block_idx + 1) * block_ch_bits];
        for sym_idx in 0..block_data_syms {
            // MSB-first within each 2-bit symbol.
            let pair = (chunk[sym_idx * 2] << 1) | chunk[sym_idx * 2 + 1];
            syms.push(GRAY_4[pair as usize]);
        }
    }
    syms.extend_from_slice(&UVPACKET_COSTAS);

    // 7. GFSK synth.
    let nsps = <super::protocol::UvRobust as ModulationParams>::NSPS as usize;
    let cfg = GfskCfg {
        sample_rate: 12_000.0,
        samples_per_symbol: nsps,
        bt: 0.5,
        hmod: 0.5,
        // Smooth half-symbol cosine ramps at the very start and end of
        // the burst. NSPS / 2 = 5 samples ≈ 0.4 ms — short enough not
        // to interfere with the head Costas, long enough to suppress
        // brick-wall TX-key click.
        ramp_samples: nsps / 2,
    };
    let f0_hz = audio_centre_hz + LOWEST_TONE_OFFSET_HZ;

    Ok(synth_f32(&syms, f0_hz, 1.0, &cfg))
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvpacket::{AUDIO_CENTRE_HZ, Mode};

    fn header_for(mode: Mode, n_blocks: u8) -> FrameHeader {
        FrameHeader {
            mode,
            block_count: n_blocks,
            app_type: 1,
            sequence: 0,
        }
    }

    #[test]
    fn encode_returns_expected_sample_count() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast, Mode::Express] {
            for n_blocks in [1u8, 4, 18, 32] {
                let header = header_for(mode, n_blocks);
                let payload_size = (n_blocks as usize) * INFO_BYTES_PER_BLOCK - HEADER_BYTES;
                let payload = vec![0xA5u8; payload_size];
                let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();

                let block_data_syms = mode.ch_bits_per_block() / 2;
                let costas_len = UVPACKET_COSTAS.len();
                let total_syms = (n_blocks as usize) * (costas_len + block_data_syms) + costas_len;
                let nsps = 10usize;
                assert_eq!(
                    audio.len(),
                    total_syms * nsps,
                    "{mode:?} n={n_blocks}: sample count {} != expected {}",
                    audio.len(),
                    total_syms * nsps,
                );
            }
        }
    }

    #[test]
    fn encode_audio_is_finite_and_bounded() {
        let header = header_for(Mode::Robust, 4);
        let audio = encode(&header, b"hello", AUDIO_CENTRE_HZ).unwrap();
        for &s in &audio {
            assert!(s.is_finite(), "non-finite sample");
            assert!(s.abs() <= 1.001, "sample exceeds amplitude bound: {s}");
        }
    }

    #[test]
    fn encode_rejects_oversize_payload() {
        let header = header_for(Mode::Robust, 1);
        let too_big = vec![0u8; INFO_BYTES_PER_BLOCK]; // 12 bytes; exceeds 12 - 4 = 8 byte capacity.
        let err = encode(&header, &too_big, AUDIO_CENTRE_HZ).unwrap_err();
        assert!(matches!(err, PackError::PayloadTooLarge(_)));
    }

    #[test]
    fn distinct_payloads_produce_distinct_audio() {
        let header = header_for(Mode::Standard, 4);
        let a = encode(&header, b"alpha", AUDIO_CENTRE_HZ).unwrap();
        let b = encode(&header, b"bravo", AUDIO_CENTRE_HZ).unwrap();
        assert_eq!(a.len(), b.len());
        let differences: usize = a
            .iter()
            .zip(b.iter())
            .filter(|(x, y)| (**x - **y).abs() > 1e-6)
            .count();
        assert!(
            differences > a.len() / 4,
            "expected substantial waveform divergence, got {differences} differing samples",
        );
    }

    #[test]
    fn distinct_modes_produce_distinct_audio_lengths() {
        let robust = encode(&header_for(Mode::Robust, 8), b"hi", AUDIO_CENTRE_HZ).unwrap();
        let standard = encode(&header_for(Mode::Standard, 8), b"hi", AUDIO_CENTRE_HZ).unwrap();
        let fast = encode(&header_for(Mode::Fast, 8), b"hi", AUDIO_CENTRE_HZ).unwrap();
        let express = encode(&header_for(Mode::Express, 8), b"hi", AUDIO_CENTRE_HZ).unwrap();

        // ch_bits_per_block strictly decreases Robust → Express, so
        // audio length should as well.
        assert!(robust.len() > standard.len());
        assert!(standard.len() > fast.len());
        assert!(fast.len() > express.len());
    }

    #[test]
    fn full_capacity_qsl_size_payload_encodes() {
        // 214-byte signed-QSL fits in 19 blocks at 12 byte/block + 4
        // byte header; we round up to fit the full capacity.
        let header = header_for(Mode::Standard, 19);
        let payload = vec![0x42u8; 19 * INFO_BYTES_PER_BLOCK - HEADER_BYTES];
        let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        // 19 blocks × (4 Costas + 101 data) + 4 trailer = 19 × 105 + 4
        // = 1999 syms × 10 samples = 19_990 samples.
        assert_eq!(audio.len(), 19 * (4 + 202 / 2) * 10 + 4 * 10);
    }
}
