//! Scalar abstraction for the LLR / BP arithmetic in the fixed-point
//! embedded path. Both `f32` (host / RasPi / FPU-equipped MCUs) and
//! [`Q11i16`] (FPU-less / consistency-focused embedded targets)
//! implement [`LlrScalar`], so [`crate::core::llr::compute_llr`] and
//! [`crate::fec::ldpc::bp::bp_decode_generic_nms`] can be written
//! once and instantiated for either scalar.
//!
//! Q-format conventions live in `~/.claude/plans/embedded-i16-scalar-design.md`:
//! - `Q11i16` is a Q11.5 fixed-point i16 (range ±16, 1/2048 LSB).
//!   Sized to comfortably hold post-`LLR_SCALE` (≈2.83) LLR values
//!   in ±10 with headroom.
//! - α (NMS scaling) is multiplied as Q15 (`alpha * 32768 → i32`)
//!   and the product right-shifted 15 places.
//! - Wide accumulator (sums during BP variable-node update) is
//!   `f32` for the f32 path and `i32` for the Q11i16 path — chosen
//!   so `llr + 3·tov` never overflows.
//!
//! Subset of operations covered: enough to express the **NMS BP
//! kernel** and the LLR computation. SumProduct BP needs `tanh` /
//! `atanh` and stays f32-only by design.

use core::cmp::Ordering;

/// Scalar trait the LLR/BP NMS implementation uses. `f32` and
/// [`Q11i16`] both implement it.
pub trait LlrScalar: Copy + Default + core::fmt::Debug {
    /// Sum accumulator type. `f32` for `f32`, `i32` for [`Q11i16`].
    type Wide: Copy + Default;

    /// Additive identity.
    const ZERO: Self;
    /// Largest representable value (used as the `min1` / `min2`
    /// initial sentinel in the min-sum check-node update).
    const POS_INF_LIKE: Self;

    /// Convert from f32 with saturation. Used at the LLR pipeline
    /// boundary (final scale-and-round) and at debug paths.
    fn from_f32(x: f32) -> Self;
    /// Convert to f32 (lossless for `f32`, `× 2^-11` for `Q11i16`).
    fn to_f32(self) -> f32;

    /// Saturating negation (`i16::MIN.neg()` clamps to `i16::MAX`).
    fn neg_sat(self) -> Self;
    /// Saturating absolute value.
    fn abs_sat(self) -> Self;

    /// Sign predicate for hard-decision parity check.
    fn is_negative(self) -> bool;

    /// Total-order comparator. NaN-safe for f32 (treats NaN as
    /// equal to itself, equal to all). Used by min-sum's `<` test.
    fn cmp_total(self, other: Self) -> Ordering;
    #[inline]
    fn lt_total(self, other: Self) -> bool {
        matches!(self.cmp_total(other), Ordering::Less)
    }

    /// Multiply by a normalised α (0..1) constant, with saturating
    /// rounding. Bench paths only ever pass `NMS_ALPHA = 0.75`.
    fn mul_alpha(self, alpha: f32) -> Self;

    /// Promote to wide accumulator.
    fn to_wide(self) -> Self::Wide;
    /// Wide identity.
    fn wide_zero() -> Self::Wide;
    /// Wide a + b.
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide;
    /// Wide a − b.
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide;
    /// Demote wide → narrow with saturation.
    fn from_wide_sat(w: Self::Wide) -> Self;
    /// Wide sign predicate (avoids round-trip through `Self`).
    fn wide_is_positive(w: Self::Wide) -> bool;
}

impl LlrScalar for f32 {
    type Wide = f32;
    const ZERO: f32 = 0.0;
    const POS_INF_LIKE: f32 = f32::INFINITY;

    #[inline]
    fn from_f32(x: f32) -> Self {
        x
    }
    #[inline]
    fn to_f32(self) -> f32 {
        self
    }
    #[inline]
    fn neg_sat(self) -> Self {
        -self
    }
    #[inline]
    fn abs_sat(self) -> Self {
        // `f32::abs` is no_std-safe via `num_traits::Float` already
        // imported elsewhere; here it's just `self.abs()` (inherent
        // method under std, libm under no_std).
        #[cfg(feature = "std")]
        {
            self.abs()
        }
        #[cfg(not(feature = "std"))]
        {
            use num_traits::Float;
            Float::abs(self)
        }
    }
    #[inline]
    fn is_negative(self) -> bool {
        self < 0.0
    }
    #[inline]
    fn cmp_total(self, other: Self) -> Ordering {
        // `partial_cmp` returns None on NaN; treat NaN as equal so
        // the min-sum loop never panics on noisy LLRs.
        self.partial_cmp(&other).unwrap_or(Ordering::Equal)
    }
    #[inline]
    fn mul_alpha(self, alpha: f32) -> Self {
        self * alpha
    }

    #[inline]
    fn to_wide(self) -> Self::Wide {
        self
    }
    #[inline]
    fn wide_zero() -> Self::Wide {
        0.0
    }
    #[inline]
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a + b
    }
    #[inline]
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a - b
    }
    #[inline]
    fn from_wide_sat(w: Self::Wide) -> Self {
        w
    }
    #[inline]
    fn wide_is_positive(w: Self::Wide) -> bool {
        w > 0.0
    }
}

/// LLR Q11 fixed-point: inner i16 = `value × 2^11`. Range ±16,
/// resolution 1/2048.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Q11i16(pub i16);

const Q11_FRAC: u32 = 11;
const Q11_ONE: i32 = 1 << Q11_FRAC; // 2048

impl LlrScalar for Q11i16 {
    type Wide = i32;
    const ZERO: Q11i16 = Q11i16(0);
    /// Min-sum sentinel for "never beat me" — `i16::MAX` represents
    /// the largest finite Q11 magnitude.
    const POS_INF_LIKE: Q11i16 = Q11i16(i16::MAX);

    #[inline]
    fn from_f32(x: f32) -> Self {
        let v = (x * Q11_ONE as f32) as i32;
        Q11i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn to_f32(self) -> f32 {
        (self.0 as f32) / (Q11_ONE as f32)
    }
    #[inline]
    fn neg_sat(self) -> Self {
        // i16::MIN.wrapping_neg() == i16::MIN; saturate to i16::MAX
        // so the sign flip is symmetric.
        Q11i16(self.0.checked_neg().unwrap_or(i16::MAX))
    }
    #[inline]
    fn abs_sat(self) -> Self {
        Q11i16(self.0.saturating_abs())
    }
    #[inline]
    fn is_negative(self) -> bool {
        self.0 < 0
    }
    #[inline]
    fn cmp_total(self, other: Self) -> Ordering {
        self.0.cmp(&other.0)
    }
    #[inline]
    fn mul_alpha(self, alpha: f32) -> Self {
        let aq15 = (alpha * 32768.0) as i32;
        let prod = (self.0 as i32) * aq15;
        // Arithmetic shift — preserves sign of the input.
        let v = prod >> 15;
        Q11i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    fn to_wide(self) -> Self::Wide {
        self.0 as i32
    }
    #[inline]
    fn wide_zero() -> Self::Wide {
        0
    }
    #[inline]
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_add(b)
    }
    #[inline]
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_sub(b)
    }
    #[inline]
    fn from_wide_sat(w: Self::Wide) -> Self {
        Q11i16(w.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn wide_is_positive(w: Self::Wide) -> bool {
        w > 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q11_round_trip() {
        for f in [-10.0, -1.5, -0.001, 0.0, 0.001, 1.5, 10.0] {
            let q = Q11i16::from_f32(f);
            let back = q.to_f32();
            assert!(
                (f - back).abs() < 1.0 / Q11_ONE as f32 + 1e-6,
                "f={f} back={back}"
            );
        }
    }

    #[test]
    fn q11_saturation() {
        // Way above range → i16::MAX
        assert_eq!(Q11i16::from_f32(1e6).0, i16::MAX);
        assert_eq!(Q11i16::from_f32(-1e6).0, i16::MIN);
    }

    #[test]
    fn q11_mul_alpha() {
        let q = Q11i16::from_f32(8.0);
        let scaled = q.mul_alpha(0.75);
        // 8.0 × 0.75 = 6.0 ± 1 LSB
        let f = scaled.to_f32();
        assert!((f - 6.0).abs() < 0.01, "f={f}");
    }

    #[test]
    fn q11_neg_handles_min() {
        // i16::MIN cannot be negated in two's complement; saturate.
        let q = Q11i16(i16::MIN);
        assert_eq!(q.neg_sat().0, i16::MAX);
    }

    #[test]
    fn f32_mul_alpha_unchanged() {
        assert!((8.0_f32.mul_alpha(0.75) - 6.0).abs() < 1e-6);
    }
}
