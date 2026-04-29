// SPDX-License-Identifier: GPL-3.0-or-later
//! Frame header + CRC-16 (CCITT-FALSE).
//!
//! Frame layout:
//!
//! ```text
//! byte 0: bits 7..6  mode (2)        \
//!         bits 5..1  block count (5)  | header proper, 16 bits
//! byte 1: bits 0..3 (of byte 0..1, MSB-first)
//!                      app type (4)    |
//!         bits 4..0  sequence (5)    /
//! byte 2..3: CRC-16/CCITT-FALSE over
//!            (header bytes 0..2) + payload
//! payload: 12 byte/block × block_count
//! ```
//!
//! Bit positions are MSB-first within each byte. The header proper
//! totals 16 bits; the CRC totals 16 bits; total 4 bytes.
//!
//! The CRC covers both the header (mode / blocks / app_type /
//! sequence) and the payload — a single CRC failure rejects either a
//! corrupted header or a corrupted payload, which is what we want
//! since a corrupted header would mis-parse the payload anyway.
//!
//! Phase 1b implements the actual pack / unpack / CRC routines.
//! Phase 1a exposes only the public type signatures so other modules
//! can compile against them.

use super::puncture::Mode;

/// Total header byte count: 16-bit field block + 16-bit CRC.
pub const HEADER_BYTES: usize = 4;

/// Information bytes carried per LDPC block (96 bits of the 101
/// `Ldpc240_101` info bits; the 5 trailing bits are zero-padded).
pub const INFO_BYTES_PER_BLOCK: usize = 12;

/// Maximum LDPC blocks per frame (5-bit field, encoded as `count − 1`
/// so a count of 1 fits in the field).
pub const MAX_BLOCKS_PER_FRAME: usize = 32;

/// Largest payload (= info bytes from blocks 1..N minus the 4-byte
/// header that consumes the first frame-data bytes).
pub const MAX_PAYLOAD_BYTES: usize = MAX_BLOCKS_PER_FRAME * INFO_BYTES_PER_BLOCK - HEADER_BYTES;

/// Decoded uvpacket frame header. Carries no payload — the payload
/// follows the header in the encoded byte stream.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FrameHeader {
    pub mode: Mode,
    /// LDPC block count, 1..=32. Encoded as `count − 1` in the
    /// 5-bit header field.
    pub block_count: u8,
    /// Application-layer dispatch tag, 0..=15. Value 0 is reserved
    /// for "raw / tagless" application data.
    pub app_type: u8,
    /// ARQ sequence number, 0..=31. Wraps mod 32.
    pub sequence: u8,
}

/// Pack a frame header + payload into the on-the-wire bit layout.
///
/// Returns the 4-byte header followed by the payload, in a single
/// owned `Vec<u8>`. The CRC field (header bytes 2..4) is computed
/// over the first two header bytes plus the payload.
///
/// Panics if `payload.len() > MAX_PAYLOAD_BYTES` or if any header
/// field exceeds its bit width.
///
/// **TODO (Phase 1b)**: implement.
pub fn pack(_header: &FrameHeader, _payload: &[u8]) -> Vec<u8> {
    unimplemented!("Phase 1b: framing.rs::pack")
}

/// Inverse of [`pack`]: parse a 4-byte header off the front of
/// `bytes`, verify CRC over header+payload, return `(header, payload)`
/// on success. Returns `None` on:
/// - input shorter than `HEADER_BYTES`
/// - CRC mismatch
/// - block count of zero (encoded as 0b00000 — reserved)
///
/// **TODO (Phase 1b)**: implement.
pub fn unpack(_bytes: &[u8]) -> Option<(FrameHeader, &[u8])> {
    unimplemented!("Phase 1b: framing.rs::unpack")
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no XOR-out, no
/// reflection). Standard 16-bit CRC; the implementation matches the
/// `crc` crate's `CRC_16_IBM_3740` parameters.
///
/// **TODO (Phase 1b)**: implement.
pub fn crc16(_bytes: &[u8]) -> u16 {
    unimplemented!("Phase 1b: framing.rs::crc16")
}
