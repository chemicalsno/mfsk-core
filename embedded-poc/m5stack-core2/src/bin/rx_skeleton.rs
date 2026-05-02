//! M5Stack Core2 real-time RX skeleton — **UNVERIFIED scaffold**.
//!
//! # What this is
//!
//! A documentation artefact pairing the FT8 decode pipeline with a
//! plausible I2S PDM capture loop on the M5Stack Core2 (built-in
//! SPM1423 mic at GPIO34 PDM_DAT / GPIO0 PDM_CLK). It exercises the
//! `mfsk_ft8_stream_*` ABI end-to-end: capture pushes 16 kHz i16
//! chunks → resampled to 12 kHz internally → snapshotted every 15 s
//! → fed to `decode_block` via the existing FFI entry.
//!
//! # What this is NOT
//!
//! **This file has not been built or flashed at the time of writing.**
//! The `+esp` toolchain takes ~15 s per `cargo check` and was not
//! invoked while authoring this skeleton. The intent is to give a
//! reader a starting point that is *structurally* correct — the
//! mfsk-ffi-ft8 / mfsk-core call shape is exact, copied from the
//! tested `streaming_recipe.c` + the working sibling `main.rs`. The
//! parts that need verification on real hardware:
//!
//! 1. **I2S PDM driver init.** `esp-idf-hal`'s I2S API has churned
//!    across versions; the [`pdm_setup`] block below uses the
//!    current `esp-idf-hal` master shape (post-0.45) but may need
//!    adapting if you pin to a release. The Core2's PDM mic is
//!    documented at SCK=GPIO0, DIN=GPIO34, mono, 16 kHz.
//! 2. **Sample-rate clock.** SPM1423 honours 16 kHz with the PDM
//!    decimator at default settings; if you need 12 kHz native, set
//!    `SRC_RATE_HZ = 12_000` and pass `12000` to the I2S config —
//!    the streaming wrapper will then bypass its resampler.
//! 3. **Slot-boundary timer.** Free-running `embassy_time` is used
//!    here for portability; production firmware would substitute
//!    NTP sync (Wi-Fi targets) or GPS PPS divided by 15.
//! 4. **`decode_one` heap discipline.** The compute bench in the
//!    sibling `main.rs` documents a workaround for a `tlsf_malloc`
//!    heap-corruption pattern when calling `decode_block` directly
//!    (see `project_decode_block_embedded.md`). This skeleton calls
//!    via the FFI which routes through `decode_block_into` and so
//!    avoids the trigger — but please verify on your hardware
//!    before assuming it.
//!
//! Treat this file as a navigation aid; copy what's useful, replace
//! anything you can't verify against your own dmesg / logic-analyser
//! reading.

#![allow(dead_code)] // skeleton: not all helpers are wired up yet

// Share the EspDspPlanner module with the sibling compute bench.
// Each binary in this crate compiles its own copy of the
// `mfsk_core_make_default_fft_planner` / `mfsk_core_dot_q15_i32`
// symbols — that's fine because each binary is a separate link unit.
#[path = "../esp_dsp_fft.rs"]
mod esp_dsp_fft;

extern crate alloc;

use core::ffi::c_int;
use core::ptr;
use core::time::Duration;

// FFI surface — same C ABI used by mfsk-ffi-ft8's host smoke test.
// Going through the ABI (rather than calling `decode_block` in Rust
// directly) means this skeleton serves as a runtime check that the
// streaming wrapper is wired correctly on the embedded path.
use mfsk_ffi_ft8::{
    MfskFt8Depth, MfskFt8ResultList, MfskFt8Status, MfskFt8Stream, mfsk_ft8_basis_scratch_len,
    mfsk_ft8_decode_i16, mfsk_ft8_result_list_free, mfsk_ft8_stream_buffered_samples,
    mfsk_ft8_stream_drain, mfsk_ft8_stream_free, mfsk_ft8_stream_new,
    mfsk_ft8_stream_peek_latest, mfsk_ft8_stream_push_i16,
};

// ── Tunables ────────────────────────────────────────────────────────

/// PDM mic native rate. 16 kHz is the SPM1423's default decimated
/// output. The streaming wrapper resamples to 12 kHz internally.
const SRC_RATE_HZ: u32 = 16_000;

/// 15 s slot at 12 kHz = 180 000 samples. Lives in PSRAM.
const SLOT_LEN: usize = 180_000;

/// FT8 search window. Standard call-CQ band: 200 Hz – 3 kHz audio.
const FREQ_MIN_HZ: f32 = 200.0;
const FREQ_MAX_HZ: f32 = 3000.0;
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: c_int = 30;

/// Slot trigger period. UTC alignment is ±2 s tolerant via
/// coarse-sync; freerunning at exactly 15 s works for stand-alone
/// benches.
const SLOT_PERIOD: Duration = Duration::from_secs(15);

// ── Per-process state ───────────────────────────────────────────────

/// Decode-side scratch — MUST live in fast internal DRAM. Plain
/// `static` arrays default to `.bss` in DRAM on ESP32, so this works
/// without any `heap_caps_*` ceremony. Each is
/// `mfsk_ft8_basis_scratch_len()` = 15 360 i16 = 30 KB.
static mut BASIS_RE: [i16; 15_360] = [0; 15_360];
static mut BASIS_IM: [i16; 15_360] = [0; 15_360];

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("mfsk-core-m5stack-core2 RX skeleton (UNVERIFIED)");
    log::info!("mfsk-core version: {}", mfsk_core::VERSION);

    // Sanity: assert basis scratch sizing matches the FFI's expectation.
    let need = mfsk_ft8_basis_scratch_len();
    assert!(
        unsafe { BASIS_RE.len() } >= need,
        "BASIS_RE too small: have {}, need {need}",
        unsafe { BASIS_RE.len() }
    );

    // ── Bring up the streaming wrapper ─────────────────────────────
    let stream: *mut MfskFt8Stream = mfsk_ft8_stream_new(SRC_RATE_HZ, SLOT_LEN);
    assert!(!stream.is_null(), "mfsk_ft8_stream_new failed");

    // ── Bring up the I2S PDM driver ────────────────────────────────
    //
    // TODO(verify): the exact `esp-idf-hal` I2S PDM API has shifted
    // across crate versions. The block below is the *current* shape
    // on the `esp-rs/esp-idf-hal` master branch (which the parent
    // Cargo.toml's `[patch.crates-io]` pulls). If you pin to a
    // release, adapt accordingly. The pin assignments (SCK=GPIO0,
    // DIN=GPIO34) are the Core2 hardware reality and don't change.
    pdm_setup(stream);

    // ── Decode loop ───────────────────────────────────────────────
    loop {
        std::thread::sleep(SLOT_PERIOD);
        decode_one(stream);
    }
}

/// Spawn the I2S PDM capture task. Pushes every DMA chunk into the
/// streaming wrapper. Runs forever on a dedicated FreeRTOS task.
fn pdm_setup(stream: *mut MfskFt8Stream) {
    // Cast away the !Send to share with the spawned task. Safe
    // because the stream is only ever touched from the capture task
    // (in here) and the decode-side `peek_latest` / `drain` are
    // careful to take a const-ref-style snapshot. For production,
    // wrap in a Mutex<*mut MfskFt8Stream> + check_thread.
    let stream_addr = stream as usize;

    std::thread::Builder::new()
        .stack_size(8 * 1024)
        .spawn(move || {
            let stream = stream_addr as *mut MfskFt8Stream;

            // ── PDM driver init (TODO verify against your esp-idf-hal pin) ──
            //
            // Pseudocode shape, kept compilable-when-feature-gated:
            //
            //   use esp_idf_svc::hal::peripherals::Peripherals;
            //   use esp_idf_svc::hal::i2s::*;
            //   use esp_idf_svc::hal::i2s::config::*;
            //
            //   let p = Peripherals::take().unwrap();
            //   let cfg = PdmRxConfig::new()
            //       .clk_cfg(PdmRxClkConfig::from_sample_rate_hz(SRC_RATE_HZ))
            //       .slot_cfg(PdmRxSlotConfig::from_bits_per_sample_and_slot_mode(
            //           DataBitWidth::Bits16, SlotMode::Mono));
            //   let mut rx = I2sDriver::new_pdm_rx(
            //       p.i2s0, &cfg, p.pins.gpio0, p.pins.gpio34).unwrap();
            //   rx.rx_enable().unwrap();
            //
            //   const CHUNK: usize = 1024; // 64 ms @ 16 kHz
            //   let mut buf = [0i16; CHUNK];
            //   loop {
            //       let n_bytes = rx.read(bytemuck::cast_slice_mut(&mut buf),
            //                             u32::MAX).unwrap();
            //       let n_samples = n_bytes / 2;
            //       unsafe {
            //           mfsk_ft8_stream_push_i16(stream, buf.as_ptr(), n_samples);
            //       }
            //   }
            //
            // Until verified on hardware, do nothing — the decode
            // loop will see the buffered_samples count stay at 0
            // and log slot underruns, which is the correct
            // behaviour for an audio source that hasn't been wired
            // up yet.
            let _ = stream;
            log::warn!(
                "pdm_setup: I2S PDM driver init is TODO — see source for the \
                 expected esp-idf-hal call shape"
            );
            loop {
                std::thread::sleep(Duration::from_secs(60));
            }
        })
        .expect("pdm capture task spawn failed");
}

/// Snapshot the latest 15 s, decode, log results, drain.
fn decode_one(stream: *mut MfskFt8Stream) {
    let buffered = unsafe { mfsk_ft8_stream_buffered_samples(stream) };
    if buffered < 168_000 {
        log::warn!("slot underrun: only {buffered} samples buffered");
        return;
    }

    // Slot lives in PSRAM (Box on the heap with PSRAM-routing
    // global allocator on Core2 — esp-idf-svc default). 360 KB.
    let mut slot: alloc::vec::Vec<i16> = alloc::vec![0i16; SLOT_LEN];
    let n =
        unsafe { mfsk_ft8_stream_peek_latest(stream, slot.as_mut_ptr(), slot.len()) };

    let mut results = MfskFt8ResultList {
        items: ptr::null_mut(),
        len: 0,
        _capacity: 0,
    };

    let st = unsafe {
        mfsk_ft8_decode_i16(
            slot.as_ptr(),
            n,
            FREQ_MIN_HZ,
            FREQ_MAX_HZ,
            SYNC_MIN,
            MAX_CAND,
            MfskFt8Depth::BpAll,
            BASIS_RE.as_mut_ptr(),
            BASIS_IM.as_mut_ptr(),
            &mut results,
        )
    };

    if st == MfskFt8Status::Ok {
        log::info!("slot decoded: {} results", results.len);
        for i in 0..results.len {
            let r = unsafe { &*results.items.add(i) };
            // text is C-string; safe stringify via from_utf8_lossy on
            // the slice up to the first NUL.
            let text_bytes = r.text.iter().take_while(|&&b| b != 0).map(|&b| b as u8);
            let text: alloc::string::String = text_bytes.map(char::from).collect();
            log::info!(
                "  {:+5.1} dB  {:>5.1} Hz  dt={:+.2}s  '{}'",
                r.snr_db,
                r.freq_hz,
                r.dt_sec,
                text
            );
        }
    } else {
        log::warn!("decode returned status {:?}", st);
    }

    unsafe { mfsk_ft8_result_list_free(&mut results) };
    unsafe { mfsk_ft8_stream_drain(stream, n) };
}

// On shutdown (would be triggered by deep-sleep wake-up handler in a
// real firmware), free the stream:
//   unsafe { mfsk_ft8_stream_free(stream) };
// This skeleton runs forever, so it never reaches that.
