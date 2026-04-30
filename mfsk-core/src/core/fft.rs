// SPDX-License-Identifier: GPL-3.0-or-later
//! Pluggable FFT backend for the decode pipeline.
//!
//! `mfsk-core`'s decode-side modules (sync correlation, LLR symbol
//! spectra, downsample / subtract) all need a single-precision
//! complex FFT. The host build links `rustfft` (the de-facto Rust
//! FFT crate; SIMD-optimised on x86 / aarch64). On embedded targets
//! we cannot use rustfft (it is std-only and its `FftPlanner`
//! depends on thread-local state), so this module defines a small
//! [`Fft`] / [`FftPlanner`] trait pair that callers fulfil with a
//! backend appropriate for their target.
//!
//! ## Built-in backends
//!
//! - [`fft-rustfft`](crate#features) (default for std builds) —
//!   forwards to `rustfft::FftPlanner`. Supports any size, SIMD
//!   accelerated where the host CPU has it. See [`RustFftPlanner`].
//! - [`fft-extern`](crate#features) (caller-provided) — the calling
//!   binary defines a Rust `extern` factory function and the rest of
//!   `mfsk-core`'s decode pipeline picks it up automatically. Use
//!   this on ESP32-S3 to bridge to `esp-dsp` via `esp-idf-sys`, on
//!   RP2350 to bridge to CMSIS-DSP, or for any custom backend (FPGA
//!   accelerator, etc.). See [`default_planner`] for the contract.
//!
//! ## Trait shape
//!
//! The trait deliberately mirrors `rustfft`'s `Fft` / `FftPlanner`
//! API so the existing call sites need only a thin adaptation:
//!
//! - [`FftPlanner::plan_forward`] / [`FftPlanner::plan_inverse`]
//!   return a boxed [`Fft`] for the requested size.
//! - [`Fft::process`] runs the transform in-place on a complex slice
//!   (length must equal [`Fft::len`]).
//!
//! Backends are free to cache plans internally; callers should keep
//! a single planner per decode session and reuse it across sizes.

#[cfg(feature = "alloc")]
use alloc::boxed::Box;

use num_complex::Complex32;

/// In-place complex single-precision FFT for one fixed length.
pub trait Fft {
    /// Run the FFT in-place. `buf.len()` must equal [`Fft::len`];
    /// shorter or longer slices are a programming error.
    fn process(&self, buf: &mut [Complex32]);

    /// Length of the transform this instance was planned for.
    fn len(&self) -> usize;

    /// Convenience — returns `true` when the backend cannot transform
    /// any data (e.g. caller passed `len == 0` to the planner).
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Plans (and typically caches) [`Fft`] instances on demand.
///
/// Callers should construct one planner per decode session and reuse
/// it across calls — backends like `rustfft` cache their twiddle
/// tables between `plan_*` invocations of the same size.
#[cfg(feature = "alloc")]
pub trait FftPlanner {
    /// Plan a forward FFT of length `len`. Returns a boxed instance
    /// the caller drives via [`Fft::process`].
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft>;

    /// Plan an inverse FFT of length `len`.
    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft>;
}

// ── rustfft backend ──────────────────────────────────────────────────

#[cfg(feature = "fft-rustfft")]
mod rustfft_backend {
    use super::*;
    use alloc::sync::Arc;

    /// rustfft-backed [`FftPlanner`]. Single-precision, any size,
    /// SIMD-accelerated where the host CPU supports it.
    pub struct RustFftPlanner {
        inner: rustfft::FftPlanner<f32>,
    }

    impl RustFftPlanner {
        /// Construct a planner. Cheap; reuse the same instance across
        /// all decodes in a session so rustfft's twiddle cache hits.
        pub fn new() -> Self {
            Self {
                inner: rustfft::FftPlanner::new(),
            }
        }
    }

    impl Default for RustFftPlanner {
        fn default() -> Self {
            Self::new()
        }
    }

    struct RustFftAdapter {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft for RustFftAdapter {
        fn process(&self, buf: &mut [Complex32]) {
            self.inner.process(buf);
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    impl FftPlanner for RustFftPlanner {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(RustFftAdapter {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(RustFftAdapter {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }
}

#[cfg(feature = "fft-rustfft")]
pub use rustfft_backend::RustFftPlanner;

// ── Default planner constructor ──────────────────────────────────────

/// Construct the default [`FftPlanner`] for the active FFT backend
/// feature.
///
/// **Resolution order**:
/// 1. `fft-rustfft` (host default) — returns a fresh
///    [`RustFftPlanner`].
/// 2. `fft-extern` — calls the binary-provided factory function
///    declared as:
///    ```ignore
///    #[unsafe(no_mangle)]
///    pub extern "Rust" fn mfsk_core_make_default_fft_planner()
///        -> Box<dyn mfsk_core::core::fft::FftPlanner>;
///    ```
///    The binary must define this symbol; missing it is a link-time
///    error. Typical ESP32-S3 implementation wraps an
///    `EspDspPlanner` (esp-dsp ASM); RP2350 builds wrap a
///    CMSIS-DSP-backed planner; tests / unusual targets can return
///    any `Box<dyn FftPlanner>` impl.
///
/// At least one of these features must be enabled whenever decode-
/// side code that calls `default_planner()` is compiled.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[inline]
pub fn default_planner() -> Box<dyn FftPlanner> {
    #[cfg(feature = "fft-rustfft")]
    {
        Box::new(RustFftPlanner::new())
    }
    #[cfg(all(not(feature = "fft-rustfft"), feature = "fft-extern"))]
    {
        unsafe extern "Rust" {
            fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner>;
        }
        // SAFETY: the linker enforces that exactly one binary in the
        // dependency closure defines this symbol; if the symbol is
        // missing the link fails. The factory's safety contract is
        // simply that it returns a valid `Box<dyn FftPlanner>`.
        unsafe { mfsk_core_make_default_fft_planner() }
    }
}

// ── tests ────────────────────────────────────────────────────────────

#[cfg(all(test, feature = "fft-rustfft"))]
mod tests_rustfft {
    use super::*;
    use core::f32::consts::TAU;

    #[test]
    fn rustfft_roundtrip_64() {
        let mut planner = RustFftPlanner::new();
        let n = 64;
        let mut buf: alloc::vec::Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * 3.0 * k as f32 / n as f32))
            .collect();
        let original = buf.clone();
        let fwd = planner.plan_forward(n);
        let inv = planner.plan_inverse(n);
        fwd.process(&mut buf);
        inv.process(&mut buf);
        let scale = 1.0 / n as f32;
        for c in buf.iter_mut() {
            *c *= scale;
        }
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!((a - b).norm() < 1e-4);
        }
    }

    #[test]
    fn rustfft_forward_picks_correct_bin() {
        let mut planner = RustFftPlanner::new();
        let n = 128;
        let bin = 7;
        let mut buf: alloc::vec::Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * bin as f32 * k as f32 / n as f32))
            .collect();
        let fwd = planner.plan_forward(n);
        fwd.process(&mut buf);
        let peak = buf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm().partial_cmp(&b.1.norm()).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak, bin);
    }
}
