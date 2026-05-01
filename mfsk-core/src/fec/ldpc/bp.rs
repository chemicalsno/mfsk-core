//! Belief-Propagation (log-domain) decoder, generic over [`LdpcParams`].
//!
//! Originally ported from WSJT-X `bpdecode174_91.f90`. Phase 0c-B
//! generalised the algorithm so [`Ldpc174_91`](super::Ldpc174_91) and
//! [`Ldpc240_101`](crate::fec::Ldpc240_101) share a single
//! implementation; the matrix shape comes from `P` at compile time.
//!
//! For backward compatibility (FT8's bespoke decode path goes through
//! [`bp_decode`] directly), this module also exposes a non-generic
//! [`bp_decode`] that pins `P = Ldpc174_91Params` — same behaviour as
//! before, just routed through the generic body.

use alloc::vec;
use alloc::vec::Vec;

// Float methods (.atanh / .signum) are inherent on f32 under std but
// require this trait under no_std (where libm provides them).
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::params::{Ldpc174_91Params, LdpcParams};
use super::{LDPC_K, LDPC_N};
pub use crate::core::BpKind;

/// Column weight (variable-node degree). Both LDPC codes in this
/// crate are uniform with `NCW = 3`.
const NCW: usize = 3;

/// Clamped atanh to avoid ±∞ near the boundaries.
/// Equivalent to WSJT-X `platanh`.
#[inline]
fn platanh(x: f32) -> f32 {
    if x.abs() > 0.999_999_9 {
        x.signum() * 4.6
    } else {
        x.atanh()
    }
}

/// CRC-14 (polynomial 0x2757) over `data` bytes, processed MSB-first.
/// Matches boost::augmented_crc<14, 0x2757> used in WSJT-X crc14.cpp.
pub fn crc14(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 13) & 1;
            crc = ((crc << 1) | bit as u16) & 0x3FFF;
            if msb != 0 {
                crc ^= 0x2757;
            }
        }
    }
    crc
}

/// Verify CRC-14 for a 91-bit decoded word (77 msg + 14 CRC).
/// Packs bits into 12 bytes (big-endian, MSB first), zeros the CRC field,
/// computes CRC-14, then compares with the stored CRC bits.
///
/// Accepts any `&[u8]` slice; lengths other than 91 are rejected so the
/// function is suitable as a `MessageCodec::verify_info` implementation
/// passed through `FecOpts::verify_info`.
pub fn check_crc14(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut bytes = [0u8; 12];
    for (i, &bit) in decoded[..77].iter().enumerate() {
        let byte_idx = i / 8;
        let bit_pos = 7 - (i % 8);
        bytes[byte_idx] |= (bit & 1) << bit_pos;
    }

    let computed = crc14(&bytes);

    let mut received: u16 = 0;
    for &bit in &decoded[77..91] {
        received = (received << 1) | (bit as u16 & 1);
    }

    computed == received
}

/// Output of a successful BP decode.
///
/// `info` is the systematic prefix (length `P::K`); `codeword` is the
/// full decoded codeword (length `P::N`). Both are heap-allocated so
/// the struct can serve any [`LdpcParams`] without const-generic
/// gymnastics. `message77` exposes the leading 77 bits as a fixed-size
/// array for the Wsjt77-family ergonomics that pre-existing FT8 code
/// relies on; non-Wsjt77 callers ignore it and read `info`.
pub struct BpResult {
    /// Leading 77 info bits (Wsjt77 message field). Same content as
    /// `info[..77]` — duplicated here for callers that take fixed-size
    /// references.
    pub message77: [u8; 77],
    /// Full systematic info (length `P::K`).
    pub info: Vec<u8>,
    /// Full codeword bits (length `P::N`).
    pub codeword: Vec<u8>,
    /// Number of hard errors (bits where hard decision disagrees with LLR sign).
    pub hard_errors: u32,
    /// Number of BP iterations executed.
    pub iterations: u32,
}

/// Generic log-domain Belief-Propagation decode.
///
/// `llr.len()` and (if present) `ap_mask.len()` must equal `P::N`.
///
/// `verify` is an optional integrity check applied to each parity-
/// converged candidate. When `Some`, BP keeps iterating past a
/// parity-only convergence whose verification fails (mirroring how
/// CRC-aware codecs behave under noise that leaves multiple valid
/// codewords near the LLR estimate). When `None`, BP returns on first
/// parity convergence — appropriate for codecs whose message codec
/// carries no internal integrity field.
pub fn bp_decode_generic<P: LdpcParams>(
    llr: &[f32],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    bp_decode_generic_kind::<P>(llr, ap_mask, max_iter, verify, BpKind::SumProduct)
}

/// Generic log-domain Belief-Propagation decode with selectable
/// check-node kernel. The default-kind wrapper [`bp_decode_generic`]
/// pins `kind = SumProduct` for backward compatibility; embedded
/// callers pick `NormalizedMinSum { alpha: 0.75 }` (or
/// `OffsetMinSum { beta: 0.5 }`) to skip the per-iteration `tanh` /
/// `atanh` cache and use a min-sum approximation instead.
///
/// **min1 / min2 + XOR-sign trick**: for both min-sum kernels the
/// check-node update precomputes `(min1, min2, sign_xor)` per check
/// once per iteration; the per-edge output then picks `min2` if the
/// edge's own |L| matches `min1`, else `min1`. This brings the
/// inner-loop cost down to O(check_degree) instead of the
/// sum-product's O(check_degree²) tanh-cache lookups, before any
/// floating-point savings are counted.
///
/// On WSJT LDPC(174,91) and LDPC(240,101), with `α = 0.75`:
/// threshold loss vs `SumProduct` is sub-0.2 dB on AWGN sweeps —
/// usually invisible at the operating point.
pub fn bp_decode_generic_kind<P: LdpcParams>(
    llr: &[f32],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    kind: BpKind,
) -> Option<BpResult> {
    debug_assert_eq!(llr.len(), P::N, "llr length must equal P::N");
    if let Some(m) = ap_mask {
        debug_assert_eq!(m.len(), P::N, "ap_mask length must equal P::N");
    }

    let n = P::N;
    let m_checks = P::M;
    let k = P::K;
    let max_row = P::MAX_ROW;

    // Heap-allocated working buffers. Sizes:
    //   tov     : N * NCW   (≤ 720 bytes for ldpc240_101)
    //   toc     : M * MAX_ROW
    //   tanhtoc : M * MAX_ROW   (sum-product only)
    //   per-check (min1, min2, idx_min1, sign_xor): 4 × M words
    //                            (min-sum only)
    //   zn      : N
    //   cw      : N
    // For both codes the total stays under 8 KB — negligible vs the
    // 30+ BP iterations of inner-loop arithmetic.
    let mut tov = vec![0f32; n * NCW];
    let mut toc = vec![0f32; m_checks * max_row];
    // Allocate tanhtoc only on the SumProduct path; min-sum doesn't
    // need it. Saving the alloc + the per-iteration loop is one of
    // the speedups; the rest comes from skipping `tanh` / `atanh`.
    let mut tanhtoc: Vec<f32> = match kind {
        BpKind::SumProduct => vec![0f32; m_checks * max_row],
        BpKind::NormalizedMinSum { .. } | BpKind::OffsetMinSum { .. } => Vec::new(),
    };
    // Min-sum scratch: per check node, the two smallest |L|, the
    // edge index that holds min1, and the XOR'd sign of all incoming
    // edges (true = negative). Allocated on min-sum paths only.
    let mut min1 = vec![0f32; m_checks];
    let mut min2 = vec![0f32; m_checks];
    let mut idx_min1 = vec![0u32; m_checks];
    let mut sign_xor = vec![false; m_checks];
    let mut zn = vec![0f32; n];
    let mut cw = vec![0u8; n];

    // Initial messages: each check node receives the raw LLR for the
    // bits it tests.
    for j in 0..m_checks {
        let nrw_j = P::nrw(j) as usize;
        for i in 0..nrw_j {
            let bit = P::nm(j, i) as usize;
            toc[j * max_row + i] = llr[bit];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = 0u32;

    for iter in 0..=max_iter {
        // Variable-node update: zn = llr + Σ tov, except AP-locked
        // bits hold their LLR fixed.
        for i in 0..n {
            let ap = ap_mask.is_some_and(|mm| mm[i]);
            if !ap {
                let mut sum = 0.0f32;
                for k_ in 0..NCW {
                    sum += tov[i * NCW + k_];
                }
                zn[i] = llr[i] + sum;
            } else {
                zn[i] = llr[i];
            }
        }

        // Hard decisions.
        for i in 0..n {
            cw[i] = if zn[i] > 0.0 { 1 } else { 0 };
        }

        // Count parity-violating checks.
        let mut ncheck = 0u32;
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            let mut parity = 0u8;
            for s in 0..nrw_i {
                parity ^= cw[P::nm(i, s) as usize];
            }
            if parity != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            let mut decoded = vec![0u8; k];
            decoded.copy_from_slice(&cw[..k]);
            // No verifier → accept any parity-converged candidate.
            // With a verifier (e.g. CRC-14/24 length-dispatched in
            // Wsjt77Message::verify_info) → accept only on true.
            let accept = match verify {
                Some(f) => f(&decoded),
                None => true,
            };
            if accept {
                let mut hard_errors = 0u32;
                for i in 0..n {
                    if (cw[i] == 1) != (llr[i] > 0.0) {
                        hard_errors += 1;
                    }
                }
                let mut message77 = [0u8; 77];
                message77.copy_from_slice(&decoded[..77]);
                return Some(BpResult {
                    message77,
                    info: decoded,
                    codeword: cw,
                    hard_errors,
                    iterations: iter,
                });
            }
        }

        // Stall detector: same heuristic as the WSJT-X reference.
        if iter > 0 {
            if ncheck < nclast {
                ncnt = 0;
            } else {
                ncnt += 1;
            }
            if ncnt >= 5 && iter >= 10 && ncheck > 15 {
                return None;
            }
        }
        nclast = ncheck;

        // Check-to-variable message update (extrinsic info).
        for j in 0..m_checks {
            let nrw_j = P::nrw(j) as usize;
            for i in 0..nrw_j {
                let ibj = P::nm(j, i) as usize;
                let mut msg = zn[ibj];
                let mn_ibj = P::mn(ibj);
                for kk in 0..NCW {
                    if mn_ibj[kk] as usize == j {
                        msg -= tov[ibj * NCW + kk];
                    }
                }
                toc[j * max_row + i] = msg;
            }
        }

        match kind {
            BpKind::SumProduct => {
                // tanh half-message cache.
                for i in 0..m_checks {
                    let nrw_i = P::nrw(i) as usize;
                    for k_ in 0..nrw_i {
                        tanhtoc[i * max_row + k_] = (-toc[i * max_row + k_] / 2.0).tanh();
                    }
                }

                // Variable-to-check message update via 2·atanh(∏ tanh(L/2)).
                for j in 0..n {
                    let mn_j = P::mn(j);
                    for k_ in 0..NCW {
                        let ichk = mn_j[k_] as usize;
                        let nrw_ichk = P::nrw(ichk) as usize;
                        let mut tmn = 1.0f32;
                        for s in 0..nrw_ichk {
                            let bit = P::nm(ichk, s) as usize;
                            if bit != j {
                                tmn *= tanhtoc[ichk * max_row + s];
                            }
                        }
                        tov[j * NCW + k_] = 2.0 * platanh(-tmn);
                    }
                }
            }
            BpKind::NormalizedMinSum { .. } | BpKind::OffsetMinSum { .. } => {
                // Min-sum kernel — α / β are pulled below.
                //
                // Pass 1: per check node, compute (min1, min2, idx_min1,
                // sign_xor) over all edges. O(check_degree).
                for i in 0..m_checks {
                    let nrw_i = P::nrw(i) as usize;
                    let mut m1 = f32::INFINITY;
                    let mut m2 = f32::INFINITY;
                    let mut imin = 0_usize;
                    let mut sx = false;
                    for s in 0..nrw_i {
                        let v = toc[i * max_row + s];
                        if v < 0.0 {
                            sx = !sx;
                        }
                        let av = v.abs();
                        if av < m1 {
                            m2 = m1;
                            m1 = av;
                            imin = s;
                        } else if av < m2 {
                            m2 = av;
                        }
                    }
                    min1[i] = m1;
                    min2[i] = m2;
                    idx_min1[i] = imin as u32;
                    sign_xor[i] = sx;
                }

                // Pass 2: per outgoing edge (variable j → check ichk),
                // emit α·sign·min1 (or min2 if this edge owns min1).
                // Sign of this edge's own input is XOR'd out so we get
                // the extrinsic sign-product.
                let alpha_eff = match kind {
                    BpKind::NormalizedMinSum { alpha } => alpha,
                    _ => 1.0,
                };
                let beta = match kind {
                    BpKind::OffsetMinSum { beta } => beta,
                    _ => 0.0,
                };
                let is_offset = matches!(kind, BpKind::OffsetMinSum { .. });

                for j in 0..n {
                    let mn_j = P::mn(j);
                    for k_ in 0..NCW {
                        let ichk = mn_j[k_] as usize;
                        // Locate this edge's slot in the check's row to
                        // pull its own sign + magnitude out.
                        let nrw_ichk = P::nrw(ichk) as usize;
                        let mut my_slot = nrw_ichk; // sentinel
                        for s in 0..nrw_ichk {
                            if P::nm(ichk, s) as usize == j {
                                my_slot = s;
                                break;
                            }
                        }
                        // If for some reason the variable isn't found in
                        // the check (shouldn't happen with well-formed
                        // tables), fall back to min1 + full sign.
                        let my_v = if my_slot < nrw_ichk {
                            toc[ichk * max_row + my_slot]
                        } else {
                            0.0
                        };
                        let my_neg = my_v < 0.0;
                        // Match the SumProduct path's sign convention. WSJT-X
                        // computes `tmn = ∏ tanh(−toc/2)` then `tov = 2 ·
                        // atanh(−tmn)`; algebra gives output sign =
                        // `(−1)^nrw · sign(∏ toc[s≠j])`. The textbook NMS
                        // formula `α · sign(∏) · min` lacks the `(−1)^nrw`
                        // factor, so on odd-row-weight checks (nrw=7 in
                        // LDPC174_91, mixed in LDPC240_101) the unflipped
                        // NMS output disagrees with SP and BP diverges.
                        // XOR'ing in `nrw_ichk & 1` gives the correct sign.
                        let nrw_odd = (nrw_ichk & 1) != 0;
                        let extrinsic_sign_neg = sign_xor[ichk] ^ my_neg ^ nrw_odd;

                        let mag = if my_slot < nrw_ichk && my_slot as u32 == idx_min1[ichk] {
                            min2[ichk]
                        } else {
                            min1[ichk]
                        };

                        let scaled = if is_offset {
                            (mag - beta).max(0.0)
                        } else {
                            alpha_eff * mag
                        };
                        tov[j * NCW + k_] = if extrinsic_sign_neg { -scaled } else { scaled };
                    }
                }
            }
        }
    }

    None
}

/// Backward-compatible LDPC(174,91) BP decode — pins
/// [`bp_decode_generic`] to [`Ldpc174_91Params`]. Used by FT8's
/// bespoke decode loop (which still consumes the shared LDPC
/// implementation through `super::ft8::ldpc`'s re-export façade).
///
/// `llr[i]` follows the convention: positive = bit likely 1, negative
/// = bit likely 0. `ap_mask[i] = true` means the bit's LLR is
/// AP-locked and not updated by BP.
pub fn bp_decode(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic::<Ldpc174_91Params>(llr.as_slice(), ap_slice, max_iter, verify)
}

/// LDPC(174,91) BP with selectable check-node kernel — same as
/// [`bp_decode`] but accepts a [`BpKind`] for the embedded /
/// FPU-poor min-sum paths.
pub fn bp_decode_kind(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    kind: BpKind,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic_kind::<Ldpc174_91Params>(llr.as_slice(), ap_slice, max_iter, verify, kind)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_perfect_llr_all_zeros() {
        let llr = [10.0f32; 174];
        let _result = bp_decode(&llr, None, 30, None);
    }

    #[test]
    fn crc14_known_vector() {
        assert_eq!(crc14(&[0u8; 12]), 0);
    }
}
