//! `mfsk_core::core::fft` backend bridged to Espressif `esp-dsp`.
//!
//! `esp-dsp` ships hand-written Xtensa assembly for the FFT (1.8-3×
//! the C reference on ESP32-S3). We expose it as an
//! [`mfsk_core::core::fft::FftPlanner`] so `mfsk-core`'s decode
//! pipeline can use it without knowing it's there.
//!
//! ## Sizes
//!
//! `dsps_fft2r_fc32_ae32` is a radix-2 FFT and supports any
//! power-of-2 length up to 4096 in the default build, or 32768 if
//! `CONFIG_DSP_TABLE_SIZE_4096_TO_32768` is enabled (we set it via
//! `sdkconfig.defaults`). Plans for non-power-of-2 sizes panic —
//! `mfsk-core`'s wide-band FFT cache (192 000 for FT8 / 92 160 for
//! FT4) is unsupported, but the narrow-band sniper / WSPR aligned
//! paths fit comfortably under the 8192-point cap.
//!
//! ## Memory
//!
//! `dsps_fft2r_init_fc32` allocates a twiddle table the first time;
//! subsequent plans reuse it. We initialise on the first
//! `plan_forward` / `plan_inverse` call.

use alloc::boxed::Box;

use mfsk_core::core::fft::{Fft, FftPlanner};
use num_complex::Complex32;

/// Factory called by `mfsk_core::core::fft::default_planner()` when
/// the crate is built with `fft-extern` (and no built-in backend like
/// `fft-rustfft`). Symbol name + signature are the link-time contract;
/// see `mfsk_core::core::fft::default_planner` for the spec.
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner> {
    Box::new(EspDspPlanner::new())
}

// Manual FFI declarations against `esp-dsp` (the IDF managed component
// pulled by `idf_component.yml`). esp-idf-sys's auto-bindgen doesn't
// cover esp-dsp's headers by default, so we declare just the four
// symbols we need. Signatures match
// `components/dsp/modules/fft/float/dsps_fft2r_fc32_*.{h,c}` in the
// upstream esp-dsp 1.4 source.
const ESP_OK: i32 = 0;

unsafe extern "C" {
    /// Pre-compute the twiddle table for radix-2 FFTs up to `table_size`
    /// points. Pass `table_size = 0` to skip allocation if you've
    /// already initialised at a sufficient size; or pass a NULL buffer
    /// to let the lib `malloc` its own table.
    fn dsps_fft2r_init_fc32(fft_table_buff: *mut f32, table_size: i32) -> i32;

    /// Forward radix-2 FFT, in place. `data` is interleaved {re, im, ...}
    /// with `2 * N` floats; `N` must be a power of 2 ≤ the
    /// initialised table size. Note the trailing `_` — esp-dsp's
    /// header `dsps_fft2r.h` exposes the asm routine via a same-named
    /// macro that redirects to this underscore-suffixed actual
    /// symbol; declaring the underscore form here lets us link
    /// without going through bindgen.
    fn dsps_fft2r_fc32_ae32_(data: *mut f32, N: i32) -> i32;

    /// Bit-reverse the radix-2 output into natural order. Apply after
    /// `dsps_fft2r_fc32_ae32`. ANSI C version (works on every chip);
    /// the Xtensa-asm variant is slightly faster but has the same
    /// signature.
    fn dsps_bit_rev_fc32_ansi(data: *mut f32, N: i32) -> i32;
}

/// `esp-dsp` FFT planner. Construct once per session, share across
/// all decode invocations so the twiddle table inits exactly once.
pub struct EspDspPlanner {
    /// Largest table size already initialised. `0` = uninitialised.
    /// The init API is one-shot per max size; we re-call when a
    /// larger plan is requested (the lib handles it gracefully).
    initialised_max: usize,
}

impl EspDspPlanner {
    pub fn new() -> Self {
        Self {
            initialised_max: 0,
        }
    }

    fn ensure_table(&mut self, len: usize) {
        if len <= self.initialised_max {
            return;
        }
        // SAFETY: NULL buffer asks the lib to alloc its own table.
        // `len` must be a power of 2 ≤ `CONFIG_DSP_TABLE_SIZE_*`
        // (we set 4096-32768 in sdkconfig.defaults).
        unsafe {
            let r = dsps_fft2r_init_fc32(core::ptr::null_mut(), len as i32);
            assert_eq!(r, ESP_OK, "dsps_fft2r_init_fc32({len}) returned {r}");
        }
        self.initialised_max = len;
    }
}

impl Default for EspDspPlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl FftPlanner for EspDspPlanner {
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft {
            len,
            forward: true,
        })
    }

    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft {
            len,
            forward: false,
        })
    }
}

struct EspDspFft {
    len: usize,
    forward: bool,
}

impl Fft for EspDspFft {
    fn process(&self, buf: &mut [Complex32]) {
        assert_eq!(buf.len(), self.len, "FFT input length mismatch");
        // esp-dsp expects an interleaved {re, im, re, im, ...} f32
        // array of length 2*N. Complex32 is repr(C) with this exact
        // layout, so we can cast in place.
        let ptr = buf.as_mut_ptr() as *mut f32;
        if !self.forward {
            // Emulate inverse via conjugate-flip (esp-dsp has no
            // inverse-mode FFT for the radix-2 routine).
            for c in buf.iter_mut() {
                c.im = -c.im;
            }
        }
        // SAFETY: ptr points to 2*N contiguous f32 (Complex32 layout).
        unsafe {
            dsps_fft2r_fc32_ae32_(ptr, self.len as i32);
            // Bit-reverse the in-place output to get natural order.
            dsps_bit_rev_fc32_ansi(ptr, self.len as i32);
        }
        if !self.forward {
            let scale = 1.0 / self.len as f32;
            for c in buf.iter_mut() {
                c.re *= scale;
                c.im = -c.im * scale;
            }
        }
    }

    fn len(&self) -> usize {
        self.len
    }
}

