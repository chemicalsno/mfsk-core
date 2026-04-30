//! ESP32-S3 PoC for `mfsk-core` embedded port.
//!
//! Demonstrates two things on a real ESP32-S3 dev board:
//!
//! 1. **TX synthesis** — pack a 77-bit FT8 message, generate the
//!    179 200-sample 12 kHz PCM waveform via `mfsk_core::ft8::wave_gen`
//!    using the new caller-buffer API (`tones_to_f32_into`).
//! 2. **FFT round-trip** — exercise the `mfsk_core::core::fft::FftPlanner`
//!    trait via the [`esp_dsp_fft::EspDspPlanner`] adapter (esp-dsp ASM).
//!
//! ## Backend layering
//!
//! `mfsk-core` is built with both `fft-microfft` (the trait's
//! built-in `default_planner()` returns microfft so the in-tree decode
//! pipeline keeps working out of the box for narrow-band sizes) and
//! `fft-extern` (lets us construct an `EspDspPlanner` and use it from
//! application code). A future mfsk-core API revision will let
//! consumers inject the planner so the pipeline picks up esp-dsp
//! transparently — for now the PoC just demonstrates the trait works.
//!
//! No hardware peripherals required — everything runs in main task and
//! reports timing + sanity checks via `log::info!`. Once this works,
//! the next layer (Slice 5+) wires I2S audio out / mic in for real
//! over-the-air RX/TX.

// `esp_dsp_fft` exports `mfsk_core_make_default_fft_planner` —
// the extern "Rust" factory `mfsk-core::core::fft::default_planner()`
// looks up under `fft-extern`. Keep the module behind `pub use` so
// the linker doesn't strip the factory as dead code.
pub mod esp_dsp_fft;

use mfsk_core::core::fft::default_planner;
use mfsk_core::ft4::encode::{TONES_OUTPUT_LEN, message_to_tones, tones_to_f32_into};
use mfsk_core::msg::wsjt77::pack77;
use num_complex::Complex32;

/// Boxed buffer for one full FT4 burst (59 328 × 4 byte = 237 KB).
/// Lives in heap (PSRAM if configured); the caller-buffer API lets us
/// reuse this across bursts without re-allocating.
fn alloc_ft4_audio_buffer() -> alloc::boxed::Box<[f32]> {
    alloc::vec![0.0f32; TONES_OUTPUT_LEN].into_boxed_slice()
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("mfsk-core-esp32s3 PoC starting");
    log::info!("mfsk-core version: {}", mfsk_core::VERSION);

    // ── Step 1: FT4 TX synthesis ────────────────────────────────────────
    // (FT8 wave_gen also works structurally but ft8::decode trips an
    // upstream Xtensa-Rust LLVM codegen bug on this build, so we exercise
    // the FT4 path instead — same shared GFSK / message / FEC stack.)
    let msg = pack77("CQ", "JA1NIE", "PM95").expect("pack77");
    log::info!("Packed CQ JA1NIE PM95 → 77-bit message");

    let tones = message_to_tones(&msg);
    log::info!("Generated FT4 tone sequence ({} symbols)", tones.len());

    let mut audio = alloc_ft4_audio_buffer();
    let t0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    tones_to_f32_into(&mut audio, &tones, /* f0 */ 1500.0, /* amp */ 0.7);
    let t1 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let synth_us = (t1 - t0) as u32;
    log::info!(
        "FT4 wave_gen ({} samples) in {} us = {:.1} ms",
        audio.len(),
        synth_us,
        synth_us as f32 / 1000.0,
    );

    let peak = audio.iter().copied().fold(0.0f32, f32::max);
    let abs_peak = audio.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
    log::info!("audio peak: max={peak:+.3}, |max|={abs_peak:.3}");
    assert!(
        abs_peak > 0.6 && abs_peak <= 0.71,
        "FT4 audio peak outside expected range"
    );

    // ── Step 2: FFT trait round-trip via the esp-dsp ASM adapter ────────
    // Pulls dsps_fft2r_fc32_ae32 from the espressif/esp-dsp managed
    // component (configured via [package.metadata.esp-idf-sys.extra_components]
    // in Cargo.toml). Demonstrates that a caller-supplied FftPlanner
    // satisfies mfsk_core::core::fft::FftPlanner without touching
    // mfsk-core itself.
    let mut planner = default_planner();
    let n = 1024;
    let bin = 73;
    let mut buf: alloc::vec::Vec<Complex32> = (0..n)
        .map(|k| {
            let phase = core::f32::consts::TAU * bin as f32 * k as f32 / n as f32;
            Complex32::new(phase.cos(), phase.sin())
        })
        .collect();

    let fwd = planner.plan_forward(n);
    let t0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    fwd.process(&mut buf);
    let t1 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let fft_us = (t1 - t0) as u32;
    log::info!("esp-dsp 1024-pt FFT in {fft_us} us");

    let peak_idx = buf
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.norm().partial_cmp(&b.1.norm()).unwrap())
        .unwrap()
        .0;
    log::info!("FFT peak bin: {peak_idx} (expected {bin})");
    assert_eq!(peak_idx, bin, "FFT peak landed on wrong bin");

    // ── Larger FFT — narrow-band decode primitive ─────────────────────
    let mut planner2 = default_planner();
    let n2 = 4096;
    let bin2 = 311;
    let mut buf2: alloc::vec::Vec<Complex32> = (0..n2)
        .map(|k| {
            let phase = core::f32::consts::TAU * bin2 as f32 * k as f32 / n2 as f32;
            Complex32::new(phase.cos(), phase.sin())
        })
        .collect();
    let fwd2 = planner2.plan_forward(n2);
    let t0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    fwd2.process(&mut buf2);
    let t1 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    log::info!("esp-dsp 4096-pt FFT in {} us", t1 - t0);
    let peak2 = buf2
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.norm().partial_cmp(&b.1.norm()).unwrap())
        .unwrap()
        .0;
    assert_eq!(peak2, bin2);

    log::info!("All checks passed. mfsk-core works on ESP32-S3.");

    // Idle forever.
    loop {
        std::thread::sleep(std::time::Duration::from_secs(60));
    }
}

extern crate alloc;
