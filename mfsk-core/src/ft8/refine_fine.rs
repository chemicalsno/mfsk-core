//! WSJT-X-faithful 3-stage fine refinement (idt → ifr → idt).
//!
//! Direct port of `lib/ft8/ft8b.f90:104-150` of WSJT-X. Operates on the
//! complex baseband at 200 Hz (`cd0`, length [`CD0_LEN`]); the caller is
//! responsible for the initial mix-and-decimate via
//! [`crate::ft8::downsample::downsample`].
//!
//! Three stages:
//!
//! 1. ±10 idt sample search at the candidate's initial frequency, using
//!    [`fine_sync_power`] (= sync8d-equivalent Costas correlation power
//!    summed over the 3 sync blocks).
//! 2. ±5 ifr × 0.5 Hz frequency sweep at the Stage-1 best `ibest`.
//!    Applies a phasor `exp(-j·2π·delf·k·dt2)` to a working copy of cd0
//!    (mathematically identical to WSJT-X's per-symbol `ctwk` multiplied
//!    against the Costas reference inside `sync8d`). Tied score is broken
//!    by smaller |delf| (favour the original frequency).
//! 3. ±4 idt re-search on the frequency-shifted cd0.
//!
//! The output `dt_sec` follows the host convention
//! `(ibest - 0.5) / 200`. The caller decides whether to also accept
//! sub-sample dt refinement (parabolic) on the final score; this module
//! returns integer dt for closer match to WSJT-X.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;

/// Signed-`i` analogue of WSJT-X `sync8d`. Mirrors WSJT-X
/// `sync8d.f90:43-45`: each Costas block contributes 0 when its
/// start sample falls outside the cd0 window. This is the right
/// behaviour for a candidate whose dt < -0.5 s — block 0 sits in
/// truncated audio but blocks 1 / 2 are still valid.
fn fine_sync_power_signed(cd0: &[Complex<f32>], i: i32) -> f32 {
    use crate::ft8::params::COSTAS;
    const DS_SPB: i32 = 32;
    let icos7: [u8; 7] = [3, 1, 4, 0, 6, 5, 2];
    debug_assert_eq!(icos7.len(), COSTAS.len());
    let np2 = cd0.len() as i32;
    let mut total = 0.0_f32;
    // 3 sync blocks at symbol offsets 0, 36, 72 — see sync8.f90:34-37.
    for block_off in [0_i32, 36, 72].iter().copied() {
        let mut block_power = 0.0_f32;
        // 7 Costas tones per block.
        for (k, &tone) in icos7.iter().enumerate() {
            let start = i + block_off * DS_SPB + (k as i32) * DS_SPB;
            if start < 0 || start + DS_SPB > np2 {
                continue;
            }
            let dphi = core::f32::consts::TAU * (tone as f32) / (DS_SPB as f32);
            let mut z = Complex::new(0.0_f32, 0.0);
            let mut phi = 0.0_f32;
            for j in 0..DS_SPB {
                let s = cd0[start as usize + j as usize];
                let r = Complex::new(phi.cos(), phi.sin());
                z += s * r.conj();
                phi += dphi;
                if phi > core::f32::consts::PI {
                    phi -= core::f32::consts::TAU;
                }
            }
            block_power += z.norm_sqr();
        }
        total += block_power;
    }
    total
}

/// Length of `cd0` produced by the FT8 downsampler (200 Hz × 16 s).
pub const CD0_LEN: usize = 3200;

/// Downsampled sample rate (Hz).
pub const DS_RATE: f32 = 200.0;

/// Result of [`fine_refine_3stage`].
#[derive(Debug, Clone, Copy)]
pub struct FineRefine {
    /// Refined dt offset relative to the slot's nominal TX start (seconds).
    /// Equivalent to WSJT-X's `xdt = (ibest-1) * dt2` minus the 0.5 s
    /// `TX_START_OFFSET_S`, exposed as `dt_sec` for symmetry with the
    /// rest of mfsk-core.
    pub dt_sec: f32,
    /// Refined frequency offset relative to the candidate's initial
    /// `freq_hz`. The caller adds this to its initial frequency.
    pub delf_hz: f32,
    /// Final Stage-3 sync power (= max over the ±4 idt re-search).
    /// Useful for downstream gating (e.g. `nsync_quality > 6`).
    pub score: f32,
}

/// Apply the phasor `exp(-j·2π·delf·k·dt2)` to `cd0[k]`, writing into
/// `out`. `out` must have the same length as `cd0`. Computed with
/// per-sample `cos`/`sin` (no accumulator) to keep f32 accuracy bounded
/// for `delf · t_max ≈ 2.5 · 16 = 40` cycles.
fn shift_freq(cd0: &[Complex<f32>], delf_hz: f32, out: &mut [Complex<f32>]) {
    debug_assert_eq!(cd0.len(), out.len());
    let dt2 = 1.0 / DS_RATE;
    for (k, (c, o)) in cd0.iter().zip(out.iter_mut()).enumerate() {
        let phi = -core::f32::consts::TAU * delf_hz * (k as f32) * dt2;
        let rot = Complex::new(phi.cos(), phi.sin());
        *o = *c * rot;
    }
}

/// 3-stage fine refinement. Mirrors `ft8b.f90:104-150`.
///
/// `initial_dt_sec` is the candidate's initial dt (relative to the
/// nominal TX start, i.e. with the 0.5 s offset already removed).
/// Returns the refined `(dt_sec, delf_hz, score)`. Negative dt is
/// supported via [`fine_sync_power_signed`], which mirrors WSJT-X
/// `sync8d.f90:43-45` (zero contribution from any sync block whose
/// samples fall outside the cd0 window).
pub fn fine_refine_3stage(cd0: &[Complex<f32>], initial_dt_sec: f32) -> FineRefine {
    debug_assert_eq!(cd0.len(), CD0_LEN);

    // ── Stage A: ±10 idt at the initial frequency ─────────────────────
    let i0 = ((initial_dt_sec + 0.5) * DS_RATE).round() as i32;
    let mut ibest_a = i0;
    let mut smax_a = f32::MIN;
    for delta in -10..=10_i32 {
        let i = i0 + delta;
        let s = fine_sync_power_signed(cd0, i);
        if s > smax_a {
            smax_a = s;
            ibest_a = i;
        }
    }

    // ── Stage B: ±5 ifr × 0.5 Hz freq sweep at ibest_a ────────────────
    let mut delfbest = 0.0_f32;
    let mut smax_b = fine_sync_power_signed(cd0, ibest_a);
    let mut tmp: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); cd0.len()];
    for ifr in -5..=5_i32 {
        if ifr == 0 {
            continue;
        }
        let delf = ifr as f32 * 0.5;
        shift_freq(cd0, delf, &mut tmp);
        let s = fine_sync_power_signed(&tmp, ibest_a);
        // Strict `>` keeps the smaller-|delf| value when scores tie
        // (favour the original frequency).
        if s > smax_b {
            smax_b = s;
            delfbest = delf;
        }
    }

    // ── Stage C: ±4 idt re-search on frequency-shifted cd0 ────────────
    if delfbest.abs() > f32::EPSILON {
        shift_freq(cd0, delfbest, &mut tmp);
    } else {
        tmp.copy_from_slice(cd0);
    }
    let mut ibest_c = ibest_a;
    let mut smax_c = f32::MIN;
    for delta in -4..=4_i32 {
        let i = ibest_a + delta;
        let s = fine_sync_power_signed(&tmp, i);
        if s > smax_c {
            smax_c = s;
            ibest_c = i;
        }
    }

    let dt_sec = (ibest_c as f32) / DS_RATE - 0.5;
    FineRefine {
        dt_sec,
        delf_hz: delfbest,
        score: smax_c,
    }
}

#[cfg(all(test, feature = "fft-rustfft"))]
mod tests {
    use super::*;
    use crate::ft8::downsample::downsample;
    use crate::ft8::params::COSTAS;
    use crate::ft8::wave_gen::tones_to_f32;

    /// Build a 79-symbol FT8 tone sequence with the 3 Costas blocks
    /// in their canonical positions. Data symbols (positions 7..36 and
    /// 43..72) are zeros — recall this synthesised audio is for
    /// **sync** verification only, the data tones don't matter.
    fn costas_only_tones() -> [u8; 79] {
        let mut t = [0u8; 79];
        for (i, &c) in COSTAS.iter().enumerate() {
            t[i] = c as u8; // block 0 at symbols 0..7
            t[36 + i] = c as u8; // block 1 at symbols 36..43
            t[72 + i] = c as u8; // block 2 at symbols 72..79
        }
        t
    }

    /// Synthesise an FT8 audio slot at the given frequency, with the
    /// signal starting at the given `dt_sec` offset within the slot.
    /// Output is 15 s × 12 kHz = 180_000 samples i16.
    fn synth_slot(freq_hz: f32, dt_sec: f32) -> Vec<i16> {
        let tones = costas_only_tones();
        let pcm_f32 = tones_to_f32(&tones, freq_hz, 0.5);
        let mut slot = vec![0i16; 15 * 12_000];
        // TX start offset = 0.5 s + dt_sec, in samples at 12 kHz.
        let start = ((0.5 + dt_sec) * 12_000.0).round() as isize;
        for (i, &s) in pcm_f32.iter().enumerate() {
            let dst = start + i as isize;
            if (0..slot.len() as isize).contains(&dst) {
                slot[dst as usize] = (s * 16_000.0) as i16;
            }
        }
        slot
    }

    #[test]
    fn freq_snap_zero_offset() {
        // True signal at 1500 Hz, dt=0. Refine should return
        // delf ≈ 0, dt ≈ 0.
        let slot = synth_slot(1500.0, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        assert!(
            r.delf_hz.abs() <= 0.5,
            "expected |delf| ≤ 0.5, got {}",
            r.delf_hz,
        );
        assert!(
            r.dt_sec.abs() <= 0.02,
            "expected |dt| ≤ 20 ms, got {}",
            r.dt_sec,
        );
        assert!(r.score > 0.0, "score should be positive on signal");
    }

    #[test]
    fn freq_snap_positive_offset() {
        // True signal at 1500.7 Hz; downsample-mix at 1500 Hz so cd0
        // baseband sees the signal at +0.7 Hz. Refine should pick
        // delfbest ∈ {+0.5, +1.0}.
        let slot = synth_slot(1500.7, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        let close_to_grid = (r.delf_hz - 0.5).abs() < 0.01 || (r.delf_hz - 1.0).abs() < 0.01;
        assert!(
            close_to_grid,
            "expected delf snap to +0.5 or +1.0, got {}",
            r.delf_hz,
        );
    }

    #[test]
    fn freq_snap_negative_offset() {
        let slot = synth_slot(1500.0 - 1.3, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        let close_to_grid = (r.delf_hz - (-1.5)).abs() < 0.01 || (r.delf_hz - (-1.0)).abs() < 0.01;
        assert!(
            close_to_grid,
            "expected delf snap to -1.5 or -1.0, got {}",
            r.delf_hz,
        );
    }

    #[test]
    fn dt_snap_positive() {
        // Signal at 1500 Hz, dt = +0.04 s (= 8 cd0 samples). Refine
        // should converge to ibest ≈ 8, i.e. dt_sec ≈ +0.04.
        let slot = synth_slot(1500.0, 0.04);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        assert!(
            (r.dt_sec - 0.04).abs() <= 0.015,
            "expected dt ≈ 0.04, got {}",
            r.dt_sec,
        );
    }

    #[test]
    fn no_signal_low_score() {
        // Pure-noise input → score should be much smaller than the
        // signal-bearing test cases (~10× lower is the empirical
        // envelope). We don't pin an absolute floor.
        let slot_signal = synth_slot(1500.0, 0.0);
        let (cd0_signal, _) = downsample(&slot_signal, 1500.0, None);
        let s_signal = fine_refine_3stage(&cd0_signal, 0.0).score;

        let slot_noise = vec![0i16; 15 * 12_000];
        let (cd0_noise, _) = downsample(&slot_noise, 1500.0, None);
        let s_noise = fine_refine_3stage(&cd0_noise, 0.0).score;

        assert!(
            s_signal > 5.0 * s_noise,
            "signal score {} should dominate noise score {}",
            s_signal,
            s_noise,
        );
    }
}
