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
//! - [`fft-microfft`](crate#features) (embedded narrow-band) —
//!   forwards to the `microfft` crate. **Power-of-2 only,
//!   ≤ 8192 points** (the WSPR aligned-decode size). Adequate for
//!   sniper-mode FT8 / FT4 decoders and WSPR aligned-decode;
//!   insufficient for the wide-band fft1 cache whose size is
//!   192 000 (FT8) / 92 160 (FT4), which needs a `fft-extern`
//!   backend. See [`MicroFftPlanner`].
//! - [`fft-extern`](crate#features) (caller-provided) — declares
//!   that the calling binary will plug in its own [`FftPlanner`]
//!   implementation. Use this on ESP32-S3 to bridge to `esp-dsp`
//!   via `esp-idf-sys`, on RP2350 to bridge to CMSIS-DSP, or for
//!   any custom backend (FPGA accelerator, etc.).
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

// ── microfft backend (embedded, ≤ 4096-point) ────────────────────────

#[cfg(feature = "fft-microfft")]
mod microfft_backend {
    use super::*;
    use alloc::vec::Vec;

    /// microfft-backed [`FftPlanner`]. Power-of-2 sizes only, capped
    /// at 4096. Pure Rust no_std, scalar single-precision throughout
    /// — call sites that need bigger transforms (e.g. FT8's 192 000-
    /// point fft1_size) need a `fft-extern` backend instead.
    pub struct MicroFftPlanner;

    impl MicroFftPlanner {
        pub fn new() -> Self {
            Self
        }
    }

    impl Default for MicroFftPlanner {
        fn default() -> Self {
            Self::new()
        }
    }

    /// In-memory bridge — microfft's API takes a fixed-length array
    /// and returns an in-place transform. We store the requested
    /// size and dispatch to the matching radix-2 routine at runtime.
    struct MicroFftAdapter {
        len: usize,
        forward: bool,
    }

    impl Fft for MicroFftAdapter {
        fn process(&self, buf: &mut [Complex32]) {
            assert_eq!(buf.len(), self.len, "FFT input length mismatch");
            if self.forward {
                dispatch_forward(buf);
            } else {
                dispatch_inverse(buf);
            }
        }
        fn len(&self) -> usize {
            self.len
        }
    }

    fn dispatch_forward(buf: &mut [Complex32]) {
        // microfft 0.6 uses a typed-size API: each Complex32_N function
        // wants a `&mut [Complex32; N]`. Since N is a constant, dispatch
        // on length at runtime through a match.
        macro_rules! dispatch {
            ($($n:expr => $f:ident),* $(,)?) => {
                match buf.len() {
                    $(
                        $n => {
                            let arr: &mut [Complex32; $n] = buf.try_into().unwrap();
                            // microfft returns the buffer (chainable
                            // API); we operate in place, so discard.
                            let _ = microfft::complex::$f(arr);
                        }
                    )*
                    other => panic!("microfft does not support FFT size {other}"),
                }
            };
        }
        dispatch! {
            2 => cfft_2,
            4 => cfft_4,
            8 => cfft_8,
            16 => cfft_16,
            32 => cfft_32,
            64 => cfft_64,
            128 => cfft_128,
            256 => cfft_256,
            512 => cfft_512,
            1024 => cfft_1024,
            2048 => cfft_2048,
            4096 => cfft_4096,
            8192 => cfft_8192,
        }
    }

    fn dispatch_inverse(buf: &mut [Complex32]) {
        // microfft 0.6 has no inverse; emulate via conjugate +
        // forward + conjugate + 1/N scale (standard identity).
        for c in buf.iter_mut() {
            c.im = -c.im;
        }
        dispatch_forward(buf);
        let scale = 1.0 / buf.len() as f32;
        for c in buf.iter_mut() {
            c.im = -c.im * scale;
            c.re *= scale;
        }
    }

    impl FftPlanner for MicroFftPlanner {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
            assert!(
                len.is_power_of_two() && (2..=8192).contains(&len),
                "microfft requires a power-of-2 size in [2, 8192], got {len}"
            );
            Box::new(MicroFftAdapter { len, forward: true })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
            assert!(
                len.is_power_of_two() && (2..=8192).contains(&len),
                "microfft requires a power-of-2 size in [2, 8192], got {len}"
            );
            Box::new(MicroFftAdapter {
                len,
                forward: false,
            })
        }
    }

    // Suppress "unused" warning when only forward-mode planners are
    // exercised from tests in this module.
    #[allow(dead_code)]
    fn _force_vec_dep() -> Vec<u8> {
        Vec::new()
    }
}

#[cfg(feature = "fft-microfft")]
pub use microfft_backend::MicroFftPlanner;

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

#[cfg(all(test, feature = "fft-microfft"))]
mod tests_microfft {
    use super::*;
    use core::f32::consts::TAU;

    #[test]
    fn microfft_forward_picks_correct_bin_64() {
        let mut planner = MicroFftPlanner::new();
        let n = 64;
        let bin = 5;
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

    #[test]
    fn microfft_roundtrip_128() {
        let mut planner = MicroFftPlanner::new();
        let n = 128;
        let mut buf: alloc::vec::Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * 9.0 * k as f32 / n as f32))
            .collect();
        let original = buf.clone();
        let fwd = planner.plan_forward(n);
        let inv = planner.plan_inverse(n);
        fwd.process(&mut buf);
        inv.process(&mut buf);
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!((a - b).norm() < 1e-3);
        }
    }
}
