// SPDX-License-Identifier: GPL-3.0-or-later
//! Costas synchronisation pattern for uvpacket.
//!
//! 4-FSK Costas array placed at the head of every uvpacket "protocol
//! unit" (one LDPC block at the Protocol-trait level). The reused
//! pattern matches FT4's Costas-A `[0, 1, 3, 2]`, which `coarse_sync`
//! already has tuned scoring for.
//!
//! At the *frame* level (multiple LDPC blocks), uvpacket's bespoke RX
//! path stitches consecutive Costas-prefixed blocks into a single
//! frame. The stitching layer lives in [`crate::uvpacket::rx`].

use crate::core::SyncBlock;

/// 4-symbol Costas pattern used at the head of each uvpacket LDPC
/// block. Same permutation as FT4_COSTAS_A; reusing it lets the
/// existing `coarse_sync` scoring apply unchanged.
pub const UVPACKET_COSTAS: [u8; 4] = [0, 1, 3, 2];

/// Sync-block layout for one uvpacket protocol unit:
/// a single Costas-4 at symbol 0, followed by `N_DATA` data symbols.
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 1] = [SyncBlock {
    start_symbol: 0,
    pattern: &UVPACKET_COSTAS,
}];
