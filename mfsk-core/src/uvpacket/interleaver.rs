// SPDX-License-Identifier: GPL-3.0-or-later
//! Block-interleaver across multiple LDPC codewords.
//!
//! Spreads consecutive codeword bits across the entire multi-block
//! channel transmission so that a long fade null (≥ one codeword
//! worth of channel bits) results in scattered erasures across every
//! codeword rather than wiping out one whole codeword. Each codeword
//! then sees the burst as well-distributed errors, well within the
//! soft-decision LDPC's correction capacity.
//!
//! The interleaver is a deterministic permutation parameterised on
//! the *number* of blocks and the *channel-bits per block* (which
//! varies with mode via puncturing). De-interleaving on RX inverts
//! the same permutation.
//!
//! Phase 1d implements the permutation. Phase 1a exposes the public
//! function signatures so callers can compile.

/// Build the channel-bit interleaver permutation for `n_blocks`
/// codewords each of `block_len` channel bits.
///
/// Returns a vector `perm` of length `n_blocks * block_len` such that
/// `perm[j]` is the source index in the concatenated codewords.
/// Standard column-write / row-read block-interleaver: input is
/// concatenated codewords (block 0 first), output index `j` reads
/// from codeword `(j % n_blocks)` at bit position `(j / n_blocks)`.
///
/// **TODO (Phase 1d)**: implement and add a unit test for the
/// burst-spreading property.
pub fn build_permutation(_n_blocks: usize, _block_len: usize) -> Vec<usize> {
    unimplemented!("Phase 1d: interleaver.rs::build_permutation")
}

/// Apply the forward interleave to a hard-decision channel-bit
/// stream. Used by TX before modulation.
///
/// **TODO (Phase 1d)**: implement.
pub fn interleave(_codewords: &[u8], _perm: &[usize]) -> Vec<u8> {
    unimplemented!("Phase 1d: interleaver.rs::interleave")
}

/// De-interleave LLRs (soft decisions) back into per-codeword
/// vectors. Used by RX after demodulation, before LDPC decode.
///
/// **TODO (Phase 1d)**: implement.
pub fn deinterleave_llr(_llrs: &[f32], _perm: &[usize], _n_blocks: usize) -> Vec<Vec<f32>> {
    unimplemented!("Phase 1d: interleaver.rs::deinterleave_llr")
}
