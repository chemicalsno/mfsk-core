// SPDX-License-Identifier: GPL-3.0-or-later
//! Q15 dot product helper for the embedded fixed-point decode path.
//!
//! `decode_block`'s per-symbol DFT (under the `fixed-point` feature)
//! is structured as 16 dot products per symbol — 8 tones × {cos, sin}
//! basis vectors against the i16 audio buffer. The default
//! [`dot_q15_i32`] is a Rust scalar loop with an i32 accumulator;
//! embedded targets that have a chip-native asm-optimised dot
//! product (e.g. esp-dsp `dsps_dotprod_s16_ae32` on Xtensa LX6,
//! CMSIS-DSP `arm_dot_prod_q15` on Cortex-M) bridge in their own
//! implementation via the `mfsk_core_dot_q15_i32` extern symbol.
//!
//! The override is gated on the same `fft-extern` feature flag as
//! the FFT bridge — both are part of the "embedded backend" picture.
//! When `fft-rustfft` is on (host build), the extern is bypassed
//! and the Rust loop runs.
//!
//! ## Numerical contract
//!
//! Inputs `a`, `b` are i16. Per-sample product fits i32. Sum over
//! up to NSPS=1920 samples: max ≈ 1920 × 32767² ≈ 2 × 10¹² —
//! overflows i32. Implementations MUST accumulate in i64 (or
//! shift-then-i32) to preserve full precision. The default Rust
//! impl right-shifts each product by 15 (Q15 normalisation) so
//! the per-sample contribution fits ~i17 and the sum stays in i32.
//!
//! Embedded overrides should match this contract: shift each MAC
//! by 15 before accumulating, OR accumulate in i64 then shift the
//! result by 15. The host stub uses the former (cheap on FPU-less
//! MCUs); esp-dsp `dsps_dotprod_s16` uses the latter (extra
//! precision, harmless).

/// Q15 dot product: `Σᵢ (aᵢ × bᵢ) >> 15`. Both inputs i16; result i32.
///
/// Default impl is a Rust scalar loop. Embedded targets can override
/// by defining the extern symbol `mfsk_core_dot_q15_i32` in their
/// binary (gated on `fft-extern` feature, mirrors the FFT bridge
/// pattern).
#[inline]
pub fn dot_q15_i32(a: &[i16], b: &[i16]) -> i32 {
    debug_assert_eq!(a.len(), b.len(), "dot_q15_i32: length mismatch");

    #[cfg(all(feature = "fft-extern", not(feature = "fft-rustfft")))]
    {
        unsafe extern "Rust" {
            fn mfsk_core_dot_q15_i32(a: *const i16, b: *const i16, n: usize) -> i32;
        }
        // SAFETY: the linker enforces that exactly one binary in the
        // dependency closure defines this symbol; if the symbol is
        // missing the link fails. The override's contract is to
        // return the Q15 dot product of `a` and `b`, both of length
        // `n`; passing `a.len()` (which equals `b.len()`) is safe.
        return unsafe { mfsk_core_dot_q15_i32(a.as_ptr(), b.as_ptr(), a.len()) };
    }

    #[cfg(not(all(feature = "fft-extern", not(feature = "fft-rustfft"))))]
    {
        let mut acc: i32 = 0;
        for (&x, &y) in a.iter().zip(b.iter()) {
            acc += ((x as i32) * (y as i32)) >> 15;
        }
        acc
    }
}

// ── tests ────────────────────────────────────────────────────────────

#[cfg(all(test, feature = "fft-rustfft"))]
mod tests {
    use super::*;

    #[test]
    fn dot_q15_zero() {
        let a = [0i16; 16];
        let b = [12345i16; 16];
        assert_eq!(dot_q15_i32(&a, &b), 0);
    }

    #[test]
    fn dot_q15_unit_basis() {
        // Σ aᵢ × 32767 / 2^15 ≈ Σ aᵢ × (1 - 2⁻¹⁵)
        let a = [100i16; 100];
        let b = [32767i16; 100];
        // Per term: 100 × 32767 / 32768 = 99.997 → 99 (integer trunc)
        // Sum 100 of those ≈ 9970..9999.
        let result = dot_q15_i32(&a, &b);
        assert!((9900..=9999).contains(&result), "got {result}");
    }

    #[test]
    fn dot_q15_anti_phase() {
        // a · (-a) Q15-normalised. The per-term `>>15` is an
        // arithmetic shift right (floor toward −∞); for non-divisible
        // products this rounds positive products toward 0 and negative
        // products away from 0, so `pos + neg` can differ by up to
        // 1 per term (≈ a.len()) without indicating a real bug.
        let a: alloc::vec::Vec<i16> = (0..1000).map(|k| (k * 30).clamp(0, 30000) as i16).collect();
        let neg_a: alloc::vec::Vec<i16> = a.iter().map(|&x| -x).collect();
        let pos = dot_q15_i32(&a, &a);
        let neg = dot_q15_i32(&a, &neg_a);
        let n = a.len() as i32;
        let drift = (pos + neg).abs();
        assert!(drift <= n, "pos={pos} neg={neg} drift={drift} (max {n})");
        assert!(pos > 0);
    }
}
