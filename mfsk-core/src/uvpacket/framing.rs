// SPDX-License-Identifier: GPL-3.0-or-later
//! Frame header + CRC-16 (CCITT-FALSE).
//!
//! ## Header layout (16-bit big-endian word)
//!
//! ```text
//! Bit:   15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
//! Field: ┌mode┐└── blocks ──┘└── app ──┘└── seq ──────┘
//! ```
//!
//! - `mode` (2 bits) — 0=Robust, 1=Standard, 2=Fast, 3=reserved.
//! - `blocks` (5 bits) — LDPC block count, encoded as `count - 1`
//!   so `0b00000` means 1 block and `0b11111` means 32 blocks.
//! - `app` (4 bits) — application-layer dispatch tag, 0..=15.
//!   Value 0 is reserved for "raw / tagless" data.
//! - `seq` (5 bits) — ARQ sequence number 0..=31, wraps mod 32.
//!
//! The 16-bit header word occupies frame bytes 0..2 in big-endian
//! order. Bytes 2..4 carry CRC-16/CCITT-FALSE computed over
//! header bytes [0..2] concatenated with the payload bytes.
//! Bytes 4.. carry the application payload.
//!
//! ## On-the-wire layout
//!
//! ```text
//! offset    field
//! ──────    ─────────────────────────────────────────────
//! 0..2      header word (mode | blocks | app_type | seq)
//! 2..4      CRC-16/CCITT-FALSE over [bytes 0..2 || payload]
//! 4..       application payload (variable, up to MAX_PAYLOAD_BYTES)
//! ```
//!
//! A single CRC over header + payload catches both header and
//! payload corruption with one check — a corrupted header would
//! mis-parse the payload anyway, so distinguishing the two doesn't
//! help the receiver.

use crc::{CRC_16_IBM_3740, Crc};

use super::puncture::Mode;

/// Total header byte count: 16-bit field word + 16-bit CRC.
pub const HEADER_BYTES: usize = 4;

/// Information bytes carried per LDPC block (96 bits of the 101
/// `Ldpc240_101` info bits; the 5 trailing bits are zero-padded).
pub const INFO_BYTES_PER_BLOCK: usize = 12;

/// Maximum LDPC blocks per frame (5-bit field, encoded as
/// `count − 1` so a count of 1 fits in the field).
pub const MAX_BLOCKS_PER_FRAME: usize = 32;

/// Maximum application payload — the info-byte budget across all 32
/// LDPC blocks minus the 4-byte header that consumes the first
/// frame-data bytes.
pub const MAX_PAYLOAD_BYTES: usize = MAX_BLOCKS_PER_FRAME * INFO_BYTES_PER_BLOCK - HEADER_BYTES;

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no
/// XOR-out. This is the classic "CRC-16/IBM-3740" parameter set.
const CRC16_ALGO: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

/// Decoded uvpacket frame header. The associated payload follows in
/// the byte stream returned by [`pack`] / accepted by [`unpack`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FrameHeader {
    pub mode: Mode,
    /// LDPC block count, 1..=32.
    pub block_count: u8,
    /// Application-layer dispatch tag, 0..=15.
    pub app_type: u8,
    /// ARQ sequence number, 0..=31.
    pub sequence: u8,
}

/// Errors returned by [`pack`] when a header field is out of range
/// or the payload exceeds the per-frame capacity.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PackError {
    /// `block_count` was outside `1..=32`.
    InvalidBlockCount(u8),
    /// `app_type` was outside `0..=15`.
    InvalidAppType(u8),
    /// `sequence` was outside `0..=31`.
    InvalidSequence(u8),
    /// `payload.len()` exceeded [`MAX_PAYLOAD_BYTES`].
    PayloadTooLarge(usize),
}

/// Pack a frame header + payload into the on-the-wire byte stream.
///
/// Returns `[header_word_be (2)] [crc16_be (2)] [payload]`. The CRC
/// covers the header word plus the full payload — a single CRC catches
/// either header or payload corruption.
pub fn pack(header: &FrameHeader, payload: &[u8]) -> Result<Vec<u8>, PackError> {
    if !(1..=32).contains(&header.block_count) {
        return Err(PackError::InvalidBlockCount(header.block_count));
    }
    if header.app_type > 15 {
        return Err(PackError::InvalidAppType(header.app_type));
    }
    if header.sequence > 31 {
        return Err(PackError::InvalidSequence(header.sequence));
    }
    if payload.len() > MAX_PAYLOAD_BYTES {
        return Err(PackError::PayloadTooLarge(payload.len()));
    }

    let mode_bits = u16::from(header.mode.header_code()) & 0x3;
    let blocks_bits = u16::from(header.block_count - 1) & 0x1F;
    let app_bits = u16::from(header.app_type) & 0xF;
    let seq_bits = u16::from(header.sequence) & 0x1F;

    let header_word: u16 = (mode_bits << 14) | (blocks_bits << 9) | (app_bits << 5) | seq_bits;
    let header_be = header_word.to_be_bytes();

    // CRC over header bytes + payload.
    let mut crc_input = Vec::with_capacity(2 + payload.len());
    crc_input.extend_from_slice(&header_be);
    crc_input.extend_from_slice(payload);
    let crc = crc16(&crc_input);
    let crc_be = crc.to_be_bytes();

    let mut out = Vec::with_capacity(HEADER_BYTES + payload.len());
    out.extend_from_slice(&header_be);
    out.extend_from_slice(&crc_be);
    out.extend_from_slice(payload);
    Ok(out)
}

/// Errors returned by [`unpack`] when the byte stream is malformed
/// or fails its integrity check.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UnpackError {
    /// Input was shorter than [`HEADER_BYTES`].
    Truncated,
    /// CRC-16 over header + payload did not match the transmitted CRC.
    CrcMismatch { expected: u16, computed: u16 },
    /// `mode` field decoded to an unknown variant. (All four 2-bit
    /// codes are valid by construction, so this should only happen
    /// after a CRC false-positive — kept for forward compatibility
    /// if more modes are ever introduced.)
    UnknownMode(u8),
}

/// Inverse of [`pack`]: parse the 4-byte header off the front of
/// `bytes`, verify CRC over header + payload, return
/// `(header, payload_slice)` on success.
///
/// Returns:
/// - `Err(UnpackError::Truncated)` if `bytes.len() < HEADER_BYTES`.
/// - `Err(UnpackError::CrcMismatch { .. })` if the CRC fails.
/// - `Err(UnpackError::UnknownMode(_))` if `mode` decodes to an
///   unknown variant.
pub fn unpack(bytes: &[u8]) -> Result<(FrameHeader, &[u8]), UnpackError> {
    if bytes.len() < HEADER_BYTES {
        return Err(UnpackError::Truncated);
    }
    let header_word = u16::from_be_bytes([bytes[0], bytes[1]]);
    let crc_recv = u16::from_be_bytes([bytes[2], bytes[3]]);
    let payload = &bytes[HEADER_BYTES..];

    let mut crc_input = Vec::with_capacity(2 + payload.len());
    crc_input.extend_from_slice(&bytes[..2]);
    crc_input.extend_from_slice(payload);
    let crc_calc = crc16(&crc_input);
    if crc_calc != crc_recv {
        return Err(UnpackError::CrcMismatch {
            expected: crc_recv,
            computed: crc_calc,
        });
    }

    let mode_code = (header_word >> 14) as u8;
    let blocks_code = ((header_word >> 9) & 0x1F) as u8;
    let app_type = ((header_word >> 5) & 0x0F) as u8;
    let sequence = (header_word & 0x1F) as u8;

    let mode = Mode::from_header_code(mode_code).ok_or(UnpackError::UnknownMode(mode_code))?;
    let block_count = blocks_code + 1;

    Ok((
        FrameHeader {
            mode,
            block_count,
            app_type,
            sequence,
        },
        payload,
    ))
}

/// CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, no reflection,
/// no XOR-out). Public so callers can verify checksums against
/// alternative byte slicings.
pub fn crc16(bytes: &[u8]) -> u16 {
    CRC16_ALGO.checksum(bytes)
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_header() -> FrameHeader {
        FrameHeader {
            mode: Mode::Robust,
            block_count: 5,
            app_type: 1,
            sequence: 7,
        }
    }

    #[test]
    fn pack_unpack_roundtrip_empty_payload() {
        let h = sample_header();
        let bytes = pack(&h, &[]).unwrap();
        assert_eq!(bytes.len(), HEADER_BYTES);
        let (h2, p2) = unpack(&bytes).unwrap();
        assert_eq!(h, h2);
        assert!(p2.is_empty());
    }

    #[test]
    fn pack_unpack_roundtrip_with_payload() {
        let h = sample_header();
        let payload: Vec<u8> = (0..60).collect();
        let bytes = pack(&h, &payload).unwrap();
        assert_eq!(bytes.len(), HEADER_BYTES + 60);
        let (h2, p2) = unpack(&bytes).unwrap();
        assert_eq!(h, h2);
        assert_eq!(p2, &payload[..]);
    }

    #[test]
    fn pack_unpack_roundtrip_all_modes() {
        for mode in [Mode::Robust, Mode::Standard, Mode::Fast] {
            let h = FrameHeader {
                mode,
                block_count: 1,
                app_type: 0,
                sequence: 0,
            };
            let bytes = pack(&h, b"hi").unwrap();
            let (h2, p2) = unpack(&bytes).unwrap();
            assert_eq!(h, h2);
            assert_eq!(p2, b"hi");
        }
    }

    #[test]
    fn pack_unpack_boundary_field_values() {
        // Max values for every field.
        let h = FrameHeader {
            mode: Mode::Fast,
            block_count: 32,
            app_type: 15,
            sequence: 31,
        };
        let bytes = pack(&h, b"x").unwrap();
        let (h2, _) = unpack(&bytes).unwrap();
        assert_eq!(h, h2);

        // Min values.
        let h = FrameHeader {
            mode: Mode::Robust,
            block_count: 1,
            app_type: 0,
            sequence: 0,
        };
        let bytes = pack(&h, &[]).unwrap();
        let (h2, _) = unpack(&bytes).unwrap();
        assert_eq!(h, h2);
    }

    #[test]
    fn pack_rejects_invalid_field_values() {
        let invalid = [
            (
                FrameHeader {
                    mode: Mode::Robust,
                    block_count: 0,
                    app_type: 0,
                    sequence: 0,
                },
                PackError::InvalidBlockCount(0),
            ),
            (
                FrameHeader {
                    mode: Mode::Robust,
                    block_count: 33,
                    app_type: 0,
                    sequence: 0,
                },
                PackError::InvalidBlockCount(33),
            ),
            (
                FrameHeader {
                    mode: Mode::Robust,
                    block_count: 1,
                    app_type: 16,
                    sequence: 0,
                },
                PackError::InvalidAppType(16),
            ),
            (
                FrameHeader {
                    mode: Mode::Robust,
                    block_count: 1,
                    app_type: 0,
                    sequence: 32,
                },
                PackError::InvalidSequence(32),
            ),
        ];
        for (h, want) in invalid {
            assert_eq!(pack(&h, &[]).unwrap_err(), want);
        }
    }

    #[test]
    fn pack_rejects_oversize_payload() {
        let h = sample_header();
        let big = vec![0u8; MAX_PAYLOAD_BYTES + 1];
        assert_eq!(
            pack(&h, &big).unwrap_err(),
            PackError::PayloadTooLarge(MAX_PAYLOAD_BYTES + 1),
        );
    }

    #[test]
    fn unpack_detects_header_bit_flip() {
        let h = sample_header();
        let mut bytes = pack(&h, b"hello").unwrap();
        // Flip a header bit (in mode/blocks region, byte 0).
        bytes[0] ^= 0x40;
        // CRC is over the original header bytes; after flipping byte
        // 0 the computed CRC will mismatch the transmitted CRC.
        match unpack(&bytes) {
            Err(UnpackError::CrcMismatch { .. }) => {}
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    #[test]
    fn unpack_detects_payload_bit_flip() {
        let h = sample_header();
        let mut bytes = pack(&h, b"hello world").unwrap();
        // Flip a payload bit.
        bytes[HEADER_BYTES + 3] ^= 0x01;
        match unpack(&bytes) {
            Err(UnpackError::CrcMismatch { .. }) => {}
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    #[test]
    fn unpack_detects_crc_bit_flip() {
        let h = sample_header();
        let mut bytes = pack(&h, b"hello").unwrap();
        // Flip a bit inside the CRC field (bytes 2..4).
        bytes[2] ^= 0x10;
        match unpack(&bytes) {
            Err(UnpackError::CrcMismatch { .. }) => {}
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    #[test]
    fn unpack_rejects_truncated_input() {
        for n in 0..HEADER_BYTES {
            let bytes = vec![0u8; n];
            assert_eq!(unpack(&bytes).unwrap_err(), UnpackError::Truncated);
        }
    }

    #[test]
    fn crc16_matches_published_check_value() {
        // CRC-16/CCITT-FALSE check value is 0x29B1 over the ASCII
        // string "123456789" — this is the canonical sanity check.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn header_bit_layout_is_msb_first() {
        // Pack a header with each field set to a distinct pattern
        // and confirm the resulting bytes match the documented bit
        // layout.
        let h = FrameHeader {
            mode: Mode::Fast,  // header_code = 2 = 0b10
            block_count: 1,    // 0b00000 (encoded as count-1)
            app_type: 0b1010,  // 0xA
            sequence: 0b10101, // 0x15
        };
        let bytes = pack(&h, &[]).unwrap();
        // Header word, MSB on the left:
        //   mode    blocks    app     seq
        //   10   |  00000  |  1010  |  10101
        //   bits 15-14 | 13-9 | 8-5 | 4-0
        // Concatenated MSB-first:  1000 0001 0101 0101 = 0x8155
        assert_eq!(bytes[0], 0x81);
        assert_eq!(bytes[1], 0x55);
    }

    #[test]
    fn capacity_constants_are_consistent() {
        assert_eq!(MAX_BLOCKS_PER_FRAME, 32);
        assert_eq!(INFO_BYTES_PER_BLOCK, 12);
        assert_eq!(MAX_PAYLOAD_BYTES, 32 * 12 - 4); // = 380
    }
}
