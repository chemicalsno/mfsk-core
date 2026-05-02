//! FT8 LLR — thin wrapper over [`crate::core::llr`].
//!
//! Preserves the pre-refactor `[[Complex;8];79]` input type for
//! compatibility with `decode`, `equalizer`, and external callers.
//! Internally flattens to the row-major layout used by the generic
//! implementation, then re-inflates the output.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use super::Ft8;
use num_complex::Complex;

use super::params::{LDPC_N, LLR_SCALE};
use crate::core::scalar::LlrScalar;

pub use crate::core::llr::LlrSet as GenericLlrSet;

/// FT8 LLR bundle: four fixed-length (174-bit) variants. Generic over
/// the [`LlrScalar`] storage; defaults to `f32` for backward
/// compatibility (`LlrSet` ≡ `LlrSet<f32>`). The Q11i16 instantiation
/// (`LlrSet<Q11i16>`) feeds the integer-only NMS BP under the
/// `fixed-point-llr` feature.
pub struct LlrSet<T: LlrScalar = f32> {
    pub llra: [T; LDPC_N],
    pub llrb: [T; LDPC_N],
    pub llrc: [T; LDPC_N],
    pub llrd: [T; LDPC_N],
}

#[inline]
fn flatten_cs(cs: &[[Complex<f32>; 8]; 79]) -> Vec<Complex<f32>> {
    let mut out = Vec::with_capacity(79 * 8);
    for sym in cs.iter() {
        out.extend_from_slice(sym);
    }
    out
}

#[inline]
fn inflate_llr<T: LlrScalar>(v: Vec<T>) -> [T; LDPC_N] {
    let mut out = [T::ZERO; LDPC_N];
    let n = v.len().min(LDPC_N);
    out[..n].copy_from_slice(&v[..n]);
    out
}

/// Compute 8-tone complex spectra for all 79 FT8 symbols.
pub fn symbol_spectra(cd0: &[Complex<f32>], i_start: usize) -> Box<[[Complex<f32>; 8]; 79]> {
    let flat = crate::core::llr::symbol_spectra::<Ft8>(cd0, i_start);
    let mut out: Box<[[Complex<f32>; 8]; 79]> =
        vec![[Complex::new(0.0, 0.0); 8]; 79].try_into().unwrap();
    for (k, row) in out.iter_mut().enumerate() {
        for t in 0..8 {
            row[t] = flat[k * 8 + t];
        }
    }
    out
}

/// Compute soft LLRs from complex symbol spectra. Generic wrapper —
/// `compute_llr<f32>` for the host path, `compute_llr<Q11i16>` for the
/// integer NMS BP path under `fixed-point-llr`.
pub fn compute_llr<T: LlrScalar>(cs: &[[Complex<f32>; 8]; 79]) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::core::llr::compute_llr::<Ft8, T>(&flat);
    // Sanity check scale consistency at build time.
    debug_assert!((crate::core::llr::LLR_SCALE - LLR_SCALE).abs() < 1e-6);
    LlrSet {
        llra: inflate_llr(g.llra),
        llrb: inflate_llr(g.llrb),
        llrc: inflate_llr(g.llrc),
        llrd: inflate_llr(g.llrd),
    }
}

/// LLRs for the BP-only path: skips nsym=2 and nsym=3 (~5× faster
/// than [`compute_llr`]). `llrb` / `llrc` come back zero — only
/// `llra` and `llrd` are valid.
pub fn compute_llr_fast<T: LlrScalar>(cs: &[[Complex<f32>; 8]; 79]) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::core::llr::compute_llr_fast::<Ft8, T>(&flat);
    LlrSet {
        llra: inflate_llr(g.llra),
        llrb: inflate_llr(g.llrb),
        llrc: inflate_llr(g.llrc),
        llrd: inflate_llr(g.llrd),
    }
}

/// WSJT-X compatible SNR from 8-tone spectra + decoded 79-tone sequence.
pub fn compute_snr_db(cs: &[[Complex<f32>; 8]; 79], itone: &[u8; 79]) -> f32 {
    let flat = flatten_cs(cs);
    crate::core::llr::compute_snr_db::<Ft8>(&flat, itone)
}

/// Hard-decision sync quality (0..21). FT8 threshold ≤ 6 → bail out.
pub fn sync_quality(cs: &[[Complex<f32>; 8]; 79]) -> u32 {
    let flat = flatten_cs(cs);
    crate::core::llr::sync_quality::<Ft8>(&flat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_spectra_zero_llr() {
        let cs: Box<[[Complex<f32>; 8]; 79]> =
            vec![[Complex::new(0.0f32, 0.0); 8]; 79].try_into().unwrap();
        let llr_set: LlrSet = compute_llr(&cs);
        let any_large = llr_set.llra.iter().any(|&x| x.abs() > 1.0);
        assert!(!any_large, "zero input should not produce large LLRs");
    }

    #[test]
    fn llr_length_is_174() {
        let cs: Box<[[Complex<f32>; 8]; 79]> =
            vec![[Complex::new(0.0f32, 0.0); 8]; 79].try_into().unwrap();
        let llr_set: LlrSet = compute_llr(&cs);
        assert_eq!(llr_set.llra.len(), 174);
        assert_eq!(llr_set.llrd.len(), 174);
    }

    #[test]
    fn sync_quality_costas_perfect() {
        use super::super::params::COSTAS;
        let mut cs = vec![[Complex::new(0.0f32, 0.0); 8]; 79];
        for &sym_offset in &[0usize, 36, 72] {
            for t in 0..7 {
                let sym = sym_offset + t;
                cs[sym][COSTAS[t]] = Complex::new(1.0, 0.0);
            }
        }
        let cs_box: Box<[[Complex<f32>; 8]; 79]> = cs.try_into().unwrap();
        assert_eq!(sync_quality(&cs_box), 21);
    }
}
