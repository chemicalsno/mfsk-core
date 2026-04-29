// SPDX-License-Identifier: GPL-3.0-or-later
//! Puncturing patterns for the four uvpacket modes.
//!
//! All four modes share `Ldpc240_101` as their FEC mother code:
//! 101 systematic info bits + 139 parity bits = 240 channel bits at
//! native rate 0.421. Higher-rate modes drop a uniform-spread subset
//! of the **parity** bits before transmission. The decoder recovers
//! by inserting an erasure LLR (value 0.0) at each punctured position
//! before BP / OSD.
//!
//! Per-mode keep counts (info kept always; parity kept after
//! puncture):
//!
//! | Mode      | parity kept | total ch bits | rate              |
//! |-----------|------------:|--------------:|-------------------|
//! | Robust    | 139         | 240           | 101/240 = 0.421   |
//! | Standard  | 101         | 202           | 101/202 = 0.500   |
//! | Fast      |  51         | 152           | 101/152 = 0.665   |
//!
//! `Fast` requires empirical decoder-convergence validation — at
//! 63 % parity puncturing of a hand-tuned irregular LDPC the BP
//! threshold can shift unpredictably (the WSJT-X authors did not
//! design `Ldpc240_101` for puncturing). See the `puncture::tests`
//! module for the high-SNR convergence sweep that gates it.
//!
//! A rate-3/4 mode existed in earlier design drafts but was dropped
//! before implementation: 76 % parity puncturing of a
//! non-rate-compatible code is essentially guaranteed to break BP
//! convergence; reaching rate 3/4 reliably needs either a
//! purpose-designed RC-LDPC or a different mother code, both out of
//! scope for 0.3.1.
//!
//! Selection of which parity positions to keep is done with a
//! Bresenham-style integer recurrence over the 139 parity slots —
//! equivalent to "keep position `i` iff
//! `floor((i + 1) · keep / 139) > floor(i · keep / 139)`". This
//! yields a maximally uniform spread without any tables, and is
//! deterministic (the same `Mode` always produces the same keep
//! set). The info positions (0..101) are always kept.
//!
//! Uniform-spread is the simplest puncture-set construction; it
//! does not exploit the H-matrix's girth or row-weight structure.
//! A more sophisticated selection (greedy minimum-distance, or
//! density-evolution-guided) is left for future work — the
//! empirical sweep tells us whether the simple approach is good
//! enough for the chosen rates.

/// `Ldpc240_101` codeword length.
const N: usize = 240;
/// `Ldpc240_101` info-bit count.
const K_INFO: usize = 101;
/// Parity-bit count = N − K_INFO.
const N_PARITY: usize = N - K_INFO;

/// Per-mode rate / FEC posture.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    /// Unpunctured `Ldpc240_101`. Native rate ≈ 0.42, 1008 net bps.
    /// For mountain / weak-signal / deep-fading channels.
    Robust,
    /// Punctured to rate 1/2. 1200 net bps. AFSK 1200 throughput
    /// parity plus FEC for typical NFM channels.
    Standard,
    /// Punctured to rate 2/3. 1600 net bps. Strong-signal mode;
    /// requires the high-SNR empirical convergence test to pass
    /// before being shipped (see this module's tests).
    Fast,
}

impl Mode {
    /// 2-bit encoding for the frame-header `mode` field. Code `3`
    /// is reserved for forward-compatibility (a future rate-3/4 or
    /// RC-LDPC variant); `from_header_code` returns `None` for it.
    pub const fn header_code(self) -> u8 {
        match self {
            Mode::Robust => 0,
            Mode::Standard => 1,
            Mode::Fast => 2,
        }
    }

    /// Inverse of [`Self::header_code`]. Returns `None` for code
    /// `3` (reserved) or any value `> 3`. The 2-bit header field
    /// encodes the three live modes plus one reserved slot.
    pub const fn from_header_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Mode::Robust),
            1 => Some(Mode::Standard),
            2 => Some(Mode::Fast),
            _ => None,
        }
    }

    /// Channel bits transmitted per LDPC block at this mode. The
    /// difference between this and the mother codeword length (240)
    /// is the count of punctured parity bits.
    pub const fn ch_bits_per_block(self) -> usize {
        match self {
            Mode::Robust => 240,
            Mode::Standard => 202,
            Mode::Fast => 152,
        }
    }

    /// Parity bits kept (transmitted) at this mode.
    const fn parity_kept(self) -> usize {
        // total channel bits − info bits = parity bits transmitted.
        self.ch_bits_per_block() - K_INFO
    }
}

/// Build the keep-index list for a given mode. Returned vector has
/// length `mode.ch_bits_per_block()`; positions are codeword indices
/// (in `0..240`) given in natural order. Info bits `0..101` always
/// appear first, followed by the surviving parity positions in
/// ascending order.
///
/// The exact layout is well-defined and reproducible: parity
/// position `p` (where `p ∈ 0..139`) is kept iff
/// `((p + 1) · keep) / 139 > (p · keep) / 139` (integer division),
/// where `keep = mode.parity_kept()`. This is the standard
/// Bresenham-style uniform selection.
pub fn keep_indices(mode: Mode) -> Vec<usize> {
    let mut keep = Vec::with_capacity(mode.ch_bits_per_block());
    keep.extend(0..K_INFO);
    let n_keep = mode.parity_kept();
    if n_keep == N_PARITY {
        keep.extend(K_INFO..N);
    } else if n_keep > 0 {
        for p in 0..N_PARITY {
            let cur = ((p + 1) * n_keep) / N_PARITY;
            let prev = (p * n_keep) / N_PARITY;
            if cur > prev {
                keep.push(K_INFO + p);
            }
        }
    }
    debug_assert_eq!(keep.len(), mode.ch_bits_per_block());
    keep
}

/// Apply puncturing to a full `Ldpc240_101` codeword: drop the
/// punctured-parity positions and return only the channel bits
/// to transmit, in their original codeword order (the puncture
/// step does not reorder bits, only drops them).
///
/// Panics if `codeword.len() != 240`.
pub fn puncture(codeword: &[u8], mode: Mode) -> Vec<u8> {
    assert_eq!(codeword.len(), N, "codeword must be 240 bits");
    let keep = keep_indices(mode);
    keep.iter().map(|&i| codeword[i]).collect()
}

/// Inverse of [`puncture`]: given LLRs of the transmitted channel
/// bits (length = `mode.ch_bits_per_block()`), expand back to a
/// length-240 LLR vector by inserting an erasure LLR (value 0.0)
/// at every punctured position. The result is fed to
/// `Ldpc240_101::decode_soft`.
///
/// Panics if `channel_llrs.len() != mode.ch_bits_per_block()`.
pub fn de_puncture_llr(channel_llrs: &[f32], mode: Mode) -> Vec<f32> {
    assert_eq!(
        channel_llrs.len(),
        mode.ch_bits_per_block(),
        "channel LLR vector must equal mode's ch_bits_per_block",
    );
    let keep = keep_indices(mode);
    let mut out = vec![0.0f32; N];
    for (k, &j) in keep.iter().enumerate() {
        out[j] = channel_llrs[k];
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_MODES: [Mode; 3] = [Mode::Robust, Mode::Standard, Mode::Fast];

    #[test]
    fn ch_bits_match_keep_indices_len() {
        for mode in ALL_MODES {
            assert_eq!(keep_indices(mode).len(), mode.ch_bits_per_block());
        }
    }

    #[test]
    fn info_bits_always_kept_first() {
        for mode in ALL_MODES {
            let keep = keep_indices(mode);
            for (i, &k) in keep.iter().take(K_INFO).enumerate() {
                assert_eq!(k, i, "{mode:?}: info position {i} not kept first");
            }
        }
    }

    #[test]
    fn keep_indices_sorted_no_dup() {
        for mode in ALL_MODES {
            let keep = keep_indices(mode);
            for w in keep.windows(2) {
                assert!(
                    w[0] < w[1],
                    "{mode:?}: indices not strictly ascending: {} >= {}",
                    w[0],
                    w[1]
                );
            }
            assert!(keep.iter().all(|&i| i < N));
        }
    }

    #[test]
    fn puncture_de_puncture_roundtrip() {
        // Build a codeword whose bit value equals (i & 1) so each
        // surviving channel bit can be checked individually.
        let codeword: Vec<u8> = (0..N).map(|i| (i & 1) as u8).collect();
        for mode in ALL_MODES {
            let punctured = puncture(&codeword, mode);
            assert_eq!(punctured.len(), mode.ch_bits_per_block());

            // Convert hard bits to LLRs (large magnitude, sign by
            // bit value) so de_puncture_llr's interface can be
            // tested. +∞ for 0, −∞ for 1 follows the WSJT
            // convention; here use ±10.0 to keep numbers finite.
            let llrs: Vec<f32> = punctured
                .iter()
                .map(|&b| if b == 0 { 10.0 } else { -10.0 })
                .collect();
            let expanded = de_puncture_llr(&llrs, mode);

            // Punctured positions get LLR 0.0; kept positions match
            // the bit-encoded LLR.
            let keep = keep_indices(mode);
            let kept_set: std::collections::HashSet<_> = keep.iter().copied().collect();
            for i in 0..N {
                if kept_set.contains(&i) {
                    let expected = if codeword[i] == 0 { 10.0 } else { -10.0 };
                    assert!(
                        (expanded[i] - expected).abs() < 1e-6,
                        "{mode:?}: position {i} llr {} ≠ {expected}",
                        expanded[i]
                    );
                } else {
                    assert_eq!(expanded[i], 0.0, "{mode:?}: punctured position {i} not 0");
                }
            }
        }
    }

    #[test]
    fn robust_keeps_everything() {
        let keep = keep_indices(Mode::Robust);
        assert_eq!(keep, (0..N).collect::<Vec<_>>());
    }

    #[test]
    fn rates_match_published_design() {
        // Sanity check against the per-mode rates documented in the
        // module-level table.
        let cases = [
            (Mode::Robust, 240, 0.421),
            (Mode::Standard, 202, 0.500),
            (Mode::Fast, 152, 0.665),
        ];
        for (mode, expected_ch, expected_rate) in cases {
            assert_eq!(mode.ch_bits_per_block(), expected_ch);
            let rate = K_INFO as f32 / expected_ch as f32;
            assert!(
                (rate - expected_rate).abs() < 0.005,
                "{mode:?}: rate {rate:.3} ≠ {expected_rate}",
            );
        }
    }

    /// Empirical decoder-convergence sweep: for each mode, encode
    /// a random codeword, puncture per mode, expand back via
    /// `de_puncture_llr` with high-magnitude clean LLRs, and confirm
    /// the BP / OSD chain recovers the info bits.
    ///
    /// "High SNR" means the channel introduced no errors at all —
    /// the only "noise" the decoder sees is the erasure (0.0 LLR)
    /// at every punctured position. If the decoder cannot recover
    /// in this trivial channel, the puncture set has destroyed the
    /// code structure to the point where BP / OSD do not converge,
    /// and that mode cannot be shipped without a redesigned puncture
    /// pattern (or a different mother code).
    ///
    /// Failure here gates the mode from shipping; a flaky pass
    /// (< 90 %) suggests the puncture set is borderline and would
    /// need either a different selection rule or more decoder
    /// effort (deeper OSD).
    #[test]
    fn modes_decode_at_high_snr() {
        use crate::core::{FecCodec, FecOpts};
        use crate::fec::Ldpc240_101;

        let fec = Ldpc240_101;

        for mode in ALL_MODES {
            let n_trials = 30;
            let mut bp_successes = 0;
            let mut bp_or_osd_successes = 0;

            for trial in 0..n_trials {
                // Deterministic pseudo-random info bits per trial.
                let mut state = (trial as u32)
                    .wrapping_mul(0x9E37_79B1)
                    .wrapping_add(0x1234_5678);
                let info: Vec<u8> = (0..K_INFO)
                    .map(|_| {
                        state = state.wrapping_mul(0x6C07_8965).wrapping_add(1);
                        ((state >> 16) & 1) as u8
                    })
                    .collect();

                // Encode through Ldpc240_101.
                let mut codeword = vec![0u8; N];
                fec.encode(&info, &mut codeword);

                // Puncture per mode.
                let punctured_bits = puncture(&codeword, mode);

                // Clean LLRs: ±10.0 (large magnitude, no channel noise).
                // WSJT-X sign convention (matches `bp_decode_generic`):
                // LLR > 0 → bit 1 likely; LLR < 0 → bit 0.
                let llrs_tx: Vec<f32> = punctured_bits
                    .iter()
                    .map(|&b| if b == 1 { 10.0 } else { -10.0 })
                    .collect();

                // De-puncture: insert 0.0 erasure LLRs.
                let llrs_full = de_puncture_llr(&llrs_tx, mode);

                // Try BP-only first (the production-fast decode path).
                let bp_opts = FecOpts {
                    bp_max_iter: 50,
                    osd_depth: 0,
                    ap_mask: None,
                    verify_info: None,
                };
                if let Some(r) = fec.decode_soft(&llrs_full, &bp_opts)
                    && r.info == info
                {
                    bp_successes += 1;
                    bp_or_osd_successes += 1;
                    continue;
                }

                // Fall back to BP + OSD-1.
                let osd_opts = FecOpts {
                    bp_max_iter: 50,
                    osd_depth: 1,
                    ap_mask: None,
                    verify_info: None,
                };
                if let Some(r) = fec.decode_soft(&llrs_full, &osd_opts)
                    && r.info == info
                {
                    bp_or_osd_successes += 1;
                }
            }

            eprintln!(
                "{mode:?}: BP {bp_successes}/{n_trials}, BP+OSD-1 {bp_or_osd_successes}/{n_trials}"
            );

            // Bar: at trivial channel SNR, BP+OSD-1 must recover
            // ≥ 90 % of trials. Anything weaker means the puncture
            // set has broken decoder convergence.
            assert!(
                bp_or_osd_successes >= n_trials * 9 / 10,
                "{mode:?}: only {bp_or_osd_successes}/{n_trials} decoded at high SNR — \
                 puncture set has likely broken BP/OSD convergence",
            );
        }
    }

    #[test]
    fn header_code_roundtrip() {
        for mode in ALL_MODES {
            let code = mode.header_code();
            assert!(code < 3);
            assert_eq!(Mode::from_header_code(code), Some(mode));
        }
        // Code 3 is the reserved-for-future-use slot; 4..=255 are
        // also invalid. None of them must decode.
        assert_eq!(Mode::from_header_code(3), None);
        for code in 4u8..=255 {
            assert_eq!(
                Mode::from_header_code(code),
                None,
                "invalid code {code} decoded successfully",
            );
        }
    }
}
