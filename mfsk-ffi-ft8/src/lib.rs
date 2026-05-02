//! Embedded-friendly C ABI for the FT8 block decoder slice of
//! `mfsk-core`.
//!
//! This crate is `no_std + alloc` under the `embedded-fixed-point`
//! feature so the resulting `libmfsk_ft8.a` can drop into a non-Rust
//! ESP-IDF / RP2040 / Cortex-M C project without dragging Rust's
//! `std` runtime. The linking C project supplies:
//!
//! 1. A `#[panic_handler]` and `#[global_allocator]` (typically a
//!    ~30-line Rust shim — see `embedded-poc/idf-component/shim/`).
//! 2. The `mfsk_core_make_default_fft_planner` extern Rust symbol
//!    (FFT backend factory — esp-dsp on Xtensa, CMSIS-DSP on
//!    Cortex-M).
//! 3. The `mfsk_core_dot_q15_i32` extern Rust symbol (i16 × Q15
//!    dot product — esp-dsp's `dsps_dotprod_s16_ae32` on LX6/LX7).
//!
//! Under the default `host` feature the crate builds with `std` and
//! rustfft for desktop testing; the resulting `cdylib` /
//! `staticlib` works exactly like a slimmed-down `mfsk-ffi`.
//!
//! # API shape
//!
//! Two decode entry points:
//!
//! - [`mfsk_ft8_decode_i16`] — heap-allocates an FFT cache + basis
//!   scratch internally. Convenient one-shot.
//! - [`mfsk_ft8_decode_i16_into`] — takes caller-provided basis
//!   scratch (`int16_t basis_re[BASIS_LEN]`, `int16_t basis_im[…]`).
//!   Required on PSRAM-equipped targets where the basis must live in
//!   internal RAM for the dot-product kernel to hit ASM throughput.
//!
//! Both populate a [`MfskFt8ResultList`] the caller frees with
//! [`mfsk_ft8_result_list_free`].

#![cfg_attr(not(feature = "host"), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ffi::{c_char, c_int};

use mfsk_core::ft8::decode_block::decode_block;

#[cfg(feature = "embedded-fixed-point")]
use mfsk_core::ft8::decode_block::{BASIS_SCRATCH_LEN, decode_block_into};

use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::msg::wsjt77::unpack77;

// ── Status codes ────────────────────────────────────────────────────────────

/// Outcome of a decode call.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskFt8Status {
    /// Decode completed (zero or more results in `out`).
    Ok = 0,
    /// One of the input pointers was null.
    NullPointer = -1,
    /// `n_samples` is too short for a 14-second slot at 12 kHz.
    AudioTooShort = -2,
    /// `depth` is outside `0..=2`.
    BadDepth = -3,
    /// Caller-provided basis scratch too small (only meaningful for
    /// the `_into` variant). The required minimum is
    /// [`mfsk_ft8_basis_scratch_len`].
    ScratchTooSmall = -4,
}

// ── Decode-depth selector ───────────────────────────────────────────────────

/// Mirrors `mfsk_core::ft8::decode::DecodeDepth`. The deeper levels
/// trade decode time for recall on busy bands.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskFt8Depth {
    /// `Bp(llra)` only — fast nsym=1 LLR + one BP run per candidate.
    Bp = 0,
    /// Above + four full LLR variants on candidates that fail Bp.
    BpAll = 1,
    /// Above + OSD-1 / OSD-3 fallback (gated on sync-quality ≥ 12).
    BpAllOsd = 2,
}

#[inline]
fn map_depth(d: MfskFt8Depth) -> DecodeDepth {
    match d {
        MfskFt8Depth::Bp => DecodeDepth::Bp,
        MfskFt8Depth::BpAll => DecodeDepth::BpAll,
        MfskFt8Depth::BpAllOsd => DecodeDepth::BpAllOsd,
    }
}

// ── Result records ──────────────────────────────────────────────────────────

/// One decoded FT8 message.
///
/// `text` is a C string of at most 39 visible UTF-8 characters plus
/// NUL terminator (FT8's 77-bit message decompresses to ≤ ~36 chars
/// in practice — the buffer is sized at 40 for safety).
#[repr(C)]
pub struct MfskFt8Result {
    /// NUL-terminated UTF-8 unpacked message. ASCII in practice.
    pub text: [c_char; 40],
    /// Carrier frequency of the decoded slot, Hz.
    pub freq_hz: f32,
    /// Time offset relative to the slot start, seconds.
    pub dt_sec: f32,
    /// SNR estimate (dB, WSJT-X 2500 Hz reference). On the embedded
    /// path this reads ~4–12 dB low on strong signals — see
    /// `docs/EMBEDDED.md` "Known limitations".
    pub snr_db: f32,
    /// Number of bits the LDPC decoder corrected before CRC pass.
    pub hard_errors: u32,
    /// Stage that produced the decode (0 = fast Bp, 1 = full Bp, …).
    pub pass: u8,
    /// Padding to keep the struct C-friendly across compilers.
    pub _pad: [u8; 3],
}

/// Result list — owned by the FFI side. Free with
/// [`mfsk_ft8_result_list_free`].
#[repr(C)]
pub struct MfskFt8ResultList {
    /// Pointer to the first result, or null if `len == 0`.
    pub items: *mut MfskFt8Result,
    /// Number of valid entries.
    pub len: usize,
    /// Capacity (private — only the free function reads this).
    pub _capacity: usize,
}

#[inline]
fn empty_result_list() -> MfskFt8ResultList {
    MfskFt8ResultList {
        items: core::ptr::null_mut(),
        len: 0,
        _capacity: 0,
    }
}

#[inline]
fn finalise_results(results: Vec<DecodeResult>, out: *mut MfskFt8ResultList) {
    let mut converted: Vec<MfskFt8Result> = Vec::with_capacity(results.len());
    for r in results {
        let mut rec = MfskFt8Result {
            text: [0; 40],
            freq_hz: r.freq_hz,
            dt_sec: r.dt_sec,
            snr_db: r.snr_db,
            hard_errors: r.hard_errors,
            pass: r.pass,
            _pad: [0; 3],
        };
        if let Some(text) = unpack77(&r.message77) {
            let bytes = text.as_bytes();
            let n = bytes.len().min(rec.text.len() - 1);
            // Reinterpret c_char vs u8 portably (c_char is i8 on
            // most targets, u8 on a few — narrowing copy below
            // handles both).
            for (dst, &src) in rec.text.iter_mut().zip(bytes.iter()).take(n) {
                *dst = src as c_char;
            }
        }
        converted.push(rec);
    }
    let mut boxed = converted.into_boxed_slice();
    let items = boxed.as_mut_ptr();
    let len = boxed.len();
    core::mem::forget(boxed);
    unsafe {
        (*out).items = items;
        (*out).len = len;
        (*out)._capacity = len;
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

/// Required scratch length (in i16 elements) for the `_into` variant.
/// Caller must allocate two arrays of at least this length, both in
/// internal RAM (not PSRAM) for the dot-product kernel to perform.
#[unsafe(no_mangle)]
#[cfg(feature = "embedded-fixed-point")]
pub extern "C" fn mfsk_ft8_basis_scratch_len() -> usize {
    BASIS_SCRATCH_LEN
}

/// **Primary FT8 decode** — caller provides BASIS scratch (two i16
/// arrays of [`mfsk_ft8_basis_scratch_len`] elements each, in
/// internal RAM, NOT PSRAM). Avoids per-decode allocation of the
/// 60 KB basis. Required for peak throughput on PSRAM-equipped
/// targets like ESP32 Core2 — the dot-product kernel needs the
/// basis in fast internal RAM. **Use this in production embedded
/// code.**
///
/// `freq_min_hz` / `freq_max_hz` bound the carrier search range
/// (typical: 200, 3000). `sync_min` is the stage-2 candidate
/// threshold (typical 1.0). `max_cand` caps the survivors after
/// Pass 2 (typical 30 for embedded busy-band). `depth` picks the
/// decoder staircase (`MFSK_FT8_DEPTH_BP_ALL_OSD` = 2 is the most
/// thorough).
///
/// Only available under the `embedded-fixed-point` feature (the
/// scratch-bearing path lives in mfsk-core's `cfg(fixed-point)`
/// block). Host code should use [`mfsk_ft8_decode_i16_alloc`].
///
/// # Safety
/// `audio` must point to `n_samples` valid `i16` values; at least
/// 168 000 (14 s × 12 kHz). `basis_re` and `basis_im` must each
/// point to at least `mfsk_ft8_basis_scratch_len()` valid i16
/// elements writable for the duration of the call. `out` must point
/// to a writable [`MfskFt8ResultList`]. Single-threaded — one decode
/// at a time per process.
#[unsafe(no_mangle)]
#[cfg(feature = "embedded-fixed-point")]
pub unsafe extern "C" fn mfsk_ft8_decode_i16(
    audio: *const i16,
    n_samples: usize,
    freq_min_hz: f32,
    freq_max_hz: f32,
    sync_min: f32,
    max_cand: c_int,
    depth: MfskFt8Depth,
    basis_re: *mut i16,
    basis_im: *mut i16,
    out: *mut MfskFt8ResultList,
) -> MfskFt8Status {
    if audio.is_null() || basis_re.is_null() || basis_im.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if n_samples < 168_000 {
        unsafe { *out = empty_result_list() };
        return MfskFt8Status::AudioTooShort;
    }
    let slot = unsafe { core::slice::from_raw_parts(audio, n_samples) };
    let basis_re_s = unsafe { core::slice::from_raw_parts_mut(basis_re, BASIS_SCRATCH_LEN) };
    let basis_im_s = unsafe { core::slice::from_raw_parts_mut(basis_im, BASIS_SCRATCH_LEN) };
    unsafe { *out = empty_result_list() };
    let results = decode_block_into(
        slot,
        freq_min_hz,
        freq_max_hz,
        sync_min,
        map_depth(depth),
        max_cand.max(0) as usize,
        basis_re_s,
        basis_im_s,
    );
    finalise_results(results, out);
    MfskFt8Status::Ok
}

/// **Host-only heap-alloc convenience** — same shape as
/// [`mfsk_ft8_decode_i16`] but allocates the BASIS scratch
/// internally on every call (~60 KB × 2 = ~120 KB heap traffic per
/// decode).
///
/// Only available on host (`feature = "host"`). Deliberately
/// excluded from embedded builds: surprise per-call heap-allocation
/// of a 60 KB scratch is a bad default for an MCU, and on Core2 with
/// PSRAM the scratch lands in slow PSRAM and tanks the dot-product
/// inner kernel. Embedded callers must use [`mfsk_ft8_decode_i16`]
/// with caller-managed scratch in internal RAM.
///
/// # Safety
/// Same as [`mfsk_ft8_decode_i16`] minus the scratch arguments.
#[unsafe(no_mangle)]
#[cfg(feature = "host")]
pub unsafe extern "C" fn mfsk_ft8_decode_i16_alloc(
    audio: *const i16,
    n_samples: usize,
    freq_min_hz: f32,
    freq_max_hz: f32,
    sync_min: f32,
    max_cand: c_int,
    depth: MfskFt8Depth,
    out: *mut MfskFt8ResultList,
) -> MfskFt8Status {
    if audio.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if n_samples < 168_000 {
        unsafe { *out = empty_result_list() };
        return MfskFt8Status::AudioTooShort;
    }
    let slot = unsafe { core::slice::from_raw_parts(audio, n_samples) };
    unsafe { *out = empty_result_list() };
    let results = decode_block(
        slot,
        freq_min_hz,
        freq_max_hz,
        sync_min,
        map_depth(depth),
        max_cand.max(0) as usize,
    );
    finalise_results(results, out);
    MfskFt8Status::Ok
}

/// Free a result list previously populated by
/// `mfsk_ft8_decode_i16` / `mfsk_ft8_decode_i16_into`. Safe to call
/// with a zero-length list. After this returns, the list's `items`
/// pointer is invalid; the struct itself remains valid for reuse.
///
/// # Safety
/// `list` must be either null (no-op) or point to a `MfskFt8ResultList`
/// previously populated by this crate.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_result_list_free(list: *mut MfskFt8ResultList) {
    if list.is_null() {
        return;
    }
    let l = unsafe { &mut *list };
    if !l.items.is_null() && l._capacity > 0 {
        let slice = unsafe { core::slice::from_raw_parts_mut(l.items, l._capacity) };
        let _ = unsafe { Box::from_raw(slice) };
    }
    l.items = core::ptr::null_mut();
    l.len = 0;
    l._capacity = 0;
}

// Suppress dead-code warning on the host build for the embedded-only
// helper imports.
#[cfg(all(feature = "host", not(feature = "embedded-fixed-point")))]
#[allow(dead_code)]
fn _silence_unused() {
    let _ = empty_result_list;
}

// ── Embedded runtime (no_std + staticlib needs these) ──────────────────────
//
// `cdylib`/`staticlib` Rust artifacts under `no_std` require a
// `#[panic_handler]` and a `#[global_allocator]`. The linking C
// project is expected to provide standard `malloc`/`free`/`abort`
// (esp-idf, newlib, picolibc all do). These hooks turn those into
// the Rust runtime symbols Cargo expects.
//
// **Override:** if the same final image already supplies a Rust
// panic handler / global allocator (e.g. a separate Rust shim crate
// that depends on mfsk-ffi-ft8), build mfsk-ffi-ft8 without the
// `embedded-runtime` feature to disable these defaults.
#[cfg(all(not(feature = "host"), feature = "embedded-runtime"))]
mod embedded_runtime {
    use core::alloc::{GlobalAlloc, Layout};
    use core::panic::PanicInfo;

    struct LibcAllocator;
    unsafe impl GlobalAlloc for LibcAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            unsafe extern "C" {
                fn malloc(size: usize) -> *mut u8;
            }
            unsafe { malloc(layout.size()) }
        }
        unsafe fn dealloc(&self, ptr: *mut u8, _: Layout) {
            unsafe extern "C" {
                fn free(ptr: *mut u8);
            }
            unsafe { free(ptr) }
        }
    }

    #[global_allocator]
    static ALLOC: LibcAllocator = LibcAllocator;

    #[panic_handler]
    fn panic(_: &PanicInfo) -> ! {
        unsafe extern "C" {
            fn abort() -> !;
        }
        unsafe { abort() }
    }
}
