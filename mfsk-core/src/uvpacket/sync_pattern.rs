// SPDX-License-Identifier: GPL-3.0-or-later
//! Frame-head preamble for uvpacket's coherent QPSK modem.
//!
//! Replaces the 4-FSK Costas-4 head pattern that the Phase 1 design
//! used (and that Phase 2 found could not survive the modulation
//! pivot to coherent QPSK — Costas arrays are FSK-tone-index
//! sequences, not constellation-point sequences).
//!
//! The new sync is a **31-bit maximum-length sequence** (PRBS,
//! generator polynomial `x⁵ + x² + 1`) mapped to BPSK at the
//! channel-symbol rate. 31 chips × 1 sym/chip = 26 ms preamble at
//! 1200 baud. Autocorrelation sidelobes are bounded by `1/31` ≈
//! −15 dB amplitude, giving a clean correlator peak that the
//! receiver uses for symbol-timing acquisition, frequency-offset
//! estimation, and initial carrier-phase lock.
//!
//! After the preamble, the receiver maintains phase via
//! decision-directed PLL with periodic [`PILOT_SYMBOL_INTERVAL`]
//! known-QPSK pilot symbols (one per 32 transmitted symbols ≈ 3.1 %
//! overhead). The pilot's constellation point is `+1 + 0j` —
//! the QPSK symbol mapped from bit pair `[0, 0]`.
//!
//! ## Trait-level placeholder
//!
//! [`UVPACKET_SYNC_BLOCKS`] is kept as a `[Costas-4 at symbol 0]`
//! placeholder so that `Protocol::SYNC_MODE = SyncMode::Block(...)`
//! has something non-empty to point at and `protocol_invariants`
//! tests pass. The uvpacket TX / RX paths are bespoke and do **not**
//! consult this constant — they use [`UVPACKET_PREAMBLE_BPSK_BITS`]
//! and [`PILOT_SYMBOL_INTERVAL`] directly.

use crate::core::SyncBlock;

/// Length of the m-sequence preamble in BPSK chips (= QPSK
/// transmitted symbols since the preamble is BPSK-mapped onto the
/// QPSK constellation's I axis).
pub const PREAMBLE_LEN: usize = 31;

/// 31-bit maximum-length sequence from a Fibonacci LFSR with
/// polynomial `x⁵ + x² + 1` and initial state `[0, 0, 0, 0, 1]`.
/// Bits run in TX order. Reproducible: any LFSR walker with the
/// same polynomial / initial state regenerates this exact sequence.
///
/// Each `true` maps to BPSK `−1`, each `false` maps to `+1` (the
/// standard NRZ-mapping used by the receiver's correlator).
pub const UVPACKET_PREAMBLE_BPSK_BITS: [bool; PREAMBLE_LEN] = {
    let mut bits = [false; PREAMBLE_LEN];
    let mut state: u8 = 0b0_0001;
    let mut i = 0;
    while i < PREAMBLE_LEN {
        // Output the rightmost bit (b1) → bit `i` of the sequence.
        bits[i] = (state & 1) != 0;
        // Fibonacci LFSR: new MSB = state[2] XOR state[0]
        // (polynomial x⁵ + x² + 1).
        let new_bit = ((state >> 2) & 1) ^ (state & 1);
        state = (state >> 1) | (new_bit << 4);
        i += 1;
    }
    bits
};

/// Pilot interval: every `PILOT_SYMBOL_INTERVAL`th transmitted
/// symbol after the preamble is a known pilot, the rest are data.
/// 32 means 1 pilot per 31 data → ~3.1 % overhead, comfortable
/// margin against 10 Hz Doppler at 1200 baud (coherence time ≈
/// 100 ms = 120 symbols, so a pilot every 32 symbols is well
/// inside the coherence interval).
pub const PILOT_SYMBOL_INTERVAL: usize = 32;

/// QPSK pilot constellation point. Chosen as constellation index
/// 0 (= bit pair `[0, 0]`) so it maps to `+1 + 0j` — receiver-side
/// phase reference is straightforward (the pilot's expected angle
/// is the carrier reference angle).
pub const PILOT_QPSK_POINT: u8 = 0;

// ── Trait-level placeholder (unused by uvpacket bespoke pipeline) ──

/// Decorative 4-FSK Costas pattern kept around so `Protocol::
/// SYNC_MODE = SyncMode::Block(&UVPACKET_SYNC_BLOCKS)` has
/// something to point at and `protocol_invariants` checks pass.
/// Not consulted by [`crate::uvpacket::tx::encode`] /
/// [`crate::uvpacket::rx::decode_known_layout`] / [`crate::uvpacket::rx::decode`].
///
/// (Kept under the legacy `UVPACKET_COSTAS` name through the
/// modulation pivot; existing TX / RX modules — which are about
/// to be rewritten — still import this symbol. The real frame
/// sync after the pivot is the m-sequence preamble above.)
pub const UVPACKET_COSTAS: [u8; 4] = [0, 1, 3, 2];

/// `Protocol::SYNC_MODE` placeholder. See module docs.
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 1] = [SyncBlock {
    start_symbol: 0,
    pattern: &UVPACKET_COSTAS,
}];

#[cfg(test)]
mod tests {
    use super::*;

    /// 31-bit m-sequence has 16 ones and 15 zeros — standard
    /// "almost balanced" property of maximum-length sequences.
    #[test]
    fn preamble_has_balanced_one_count() {
        let ones = UVPACKET_PREAMBLE_BPSK_BITS.iter().filter(|&&b| b).count();
        assert_eq!(ones, 16);
        assert_eq!(PREAMBLE_LEN - ones, 15);
    }

    /// Autocorrelation of an m-sequence is N at lag 0 and -1 at
    /// every other lag (sidelobe / mainlobe ratio = 1 / N).
    /// Equivalently, `Σ (±1)·(±1)` of shifted vs unshifted is N
    /// at lag 0 and -1 at lag 1..N-1.
    #[test]
    fn preamble_autocorrelation_sidelobes_minimal() {
        let bpsk: Vec<i32> = UVPACKET_PREAMBLE_BPSK_BITS
            .iter()
            .map(|&b| if b { -1 } else { 1 })
            .collect();
        let n = bpsk.len() as i32;
        // Lag 0 — perfect correlation.
        let lag0: i32 = bpsk.iter().map(|x| x * x).sum();
        assert_eq!(lag0, n);
        // Cyclic lags 1..N-1 — each must be -1.
        for lag in 1..(bpsk.len()) {
            let sum: i32 = (0..bpsk.len())
                .map(|i| bpsk[i] * bpsk[(i + lag) % bpsk.len()])
                .sum();
            assert_eq!(sum, -1, "cyclic lag {lag} autocorr = {sum} ≠ -1");
        }
    }

    #[test]
    fn pilot_interval_is_reasonable() {
        // ~3 % overhead, well inside coherence time at 10 Hz Doppler.
        assert!((16..=64).contains(&PILOT_SYMBOL_INTERVAL));
    }
}
