//! WAV-fed audio source — real-time-paced push of baked PCM WAVs into
//! the `mfsk_ft8_stream_*` ring buffer.
//!
//! Substitute for the I2S PDM mic when you don't have one wired up
//! (the M5Stack Core2's built-in SPM1423 PDM is the production source;
//! this module lets the rest of the RX pipeline be developed and
//! validated against deterministic baked audio first).
//!
//! ## Behaviour
//!
//! `spawn` consumes a slice of WAV byte-slices (each 12 kHz / mono /
//! 16-bit, 44-byte RIFF header) and pushes their samples into the
//! provided stream at real-time pace (chunk-based `vTaskDelay`).
//! After all WAVs are exhausted the task loops from the start, so
//! the decode side sees a continuously-running 15 s slot every WAV.
//!
//! ## Pacing
//!
//! Push `CHUNK_SAMPLES = 1200` samples (= 100 ms at 12 kHz) per
//! iteration, then sleep 100 ms. Total throughput is exactly 1× real
//! time. The chunk size is small enough that the decode side's
//! `peek_latest` always sees a fresh 15 s window when the slot timer
//! fires; large enough to keep `vTaskDelay` overhead negligible.
//!
//! ## Threading
//!
//! Spawned as a FreeRTOS task pinned to APP_CPU (core 1) so it doesn't
//! contend with the `dual_core` worker on... wait, that's also core 1.
//! Pinned to PRO_CPU (core 0) instead — the decode main task runs
//! there but spends most of a slot blocked in `vTaskDelay`, which
//! yields immediately. The 100 ms push tick is well-separated from
//! the ~3 s decode burst.

use core::ffi::c_void;
use core::ptr;

use esp_idf_svc::sys::{
    configTICK_RATE_HZ, eNotifyAction_eIncrement, vTaskDelay, xTaskCreatePinnedToCore,
    xTaskGenericNotify, TaskHandle_t,
};

const PD_PASS: i32 = 1;
/// Convert milliseconds to FreeRTOS ticks at compile time. Replicates
/// the `pdMS_TO_TICKS()` macro (not exposed by esp-idf-sys bindgen).
const fn ms_to_ticks(ms: u32) -> u32 {
    ms / (1000 / configTICK_RATE_HZ)
}

/// Decode task handle to notify when a full WAV has been pumped.
/// Set via `set_consumer()` before calling `spawn()`. If null, the
/// sim task runs without notifying anyone.
static mut CONSUMER_TASK: TaskHandle_t = core::ptr::null_mut();

/// Per-chunk hook invoked on each `push_i16` call in the sim loop.
/// Used by the Phase-E stress test to mirror the audio into a parallel
/// incremental stage-1 buffer (see `stage1_inc::push_chunk`). `None`
/// → no-op.
static mut CHUNK_HOOK: Option<fn(&[i16])> = None;

/// Register the task that should be notified after each complete WAV
/// is pushed into the stream. Use as a slot-boundary trigger so the
/// decode loop is synchronised to the simulated audio stream rather
/// than the wall clock.
pub fn set_consumer(handle: TaskHandle_t) {
    unsafe {
        CONSUMER_TASK = handle;
    }
}

/// Register a callback invoked with every chunk before vTaskDelay.
/// Used to fan out simulated DMA-done events to other workers (e.g.
/// `stage1_inc::push_chunk`).
pub fn set_chunk_hook(hook: fn(&[i16])) {
    unsafe {
        CHUNK_HOOK = Some(hook);
    }
}

/// 100 ms at 12 kHz mono.
const CHUNK_SAMPLES: usize = 1_200;
/// PCM WAV header size (RIFF 12 + fmt 24 + data 8). All baked WAVs in
/// this crate share this exact format.
const WAV_HEADER_BYTES: usize = 44;
/// One FT8 slot at 12 kHz. Any baked WAV with >180 000 samples gets
/// trimmed to this length so the ring (cap=180 000 ish) returns
/// exactly slot[0..180 000] from peek_latest. Otherwise the trailing
/// samples evict the leading samples and produce a phase shift that
/// can drop weak-signal recall on busy bands.
const SLOT_SAMPLES: usize = 180_000;

/// Static list of WAV byte-slices the sim task cycles through. Set
/// once at startup via `spawn(...)` then read-only.
static mut SIM_WAVS: &'static [&'static [u8]] = &[];
/// Stream pointer (cast to usize for Send across the task boundary).
static mut SIM_STREAM_ADDR: usize = 0;

/// Spawn the WAV-feed task. Call once after the `mfsk_ft8_stream_*`
/// is created and before the slot decode timer starts.
///
/// `wavs` must outlive the program (typically `&'static [u8]` from
/// `include_bytes!`).
pub fn spawn(stream: *mut mfsk_ft8::MfskFt8Stream, wavs: &'static [&'static [u8]]) {
    assert!(!wavs.is_empty(), "wav_sim::spawn: no WAVs provided");
    unsafe {
        SIM_STREAM_ADDR = stream as usize;
        SIM_WAVS = wavs;
    }
    let mut handle: TaskHandle_t = ptr::null_mut();
    unsafe {
        let r = xTaskCreatePinnedToCore(
            Some(sim_task_main),
            c"wav_sim".as_ptr(),
            4096, // stack — task only loops over byte slices and pushes
            ptr::null_mut(),
            // Priority below the main task (default 5) — when we
            // signal `xTaskGenericNotify` to wake the consumer, FreeRTOS
            // preempts us immediately so decode peeks the ring while
            // it's still clean (WAV[N]); otherwise we'd continue running
            // and push ~1 s of WAV[N+1] before yielding, polluting the
            // peek window with the next WAV's prefix.
            4,
            &raw mut handle,
            0, // PRO_CPU; decode main task is here too but mostly blocked
        );
        assert_eq!(r, PD_PASS, "xTaskCreatePinnedToCore(wav_sim) failed: {r}");
    }
    log::info!("wav_sim: spawned, {} WAV(s) in playlist", wavs.len());
}

extern "C" fn sim_task_main(_arg: *mut c_void) {
    let wavs: &'static [&'static [u8]] = unsafe { SIM_WAVS };
    let stream = unsafe { SIM_STREAM_ADDR } as *mut mfsk_ft8::MfskFt8Stream;

    // Inter-chunk delay in FreeRTOS ticks (100 ms at 100 Hz tick = 10).
    let delay_ticks: u32 = ms_to_ticks(100);

    loop {
        for (idx, wav) in wavs.iter().enumerate() {
            log::info!("wav_sim: starting WAV[{idx}] ({} bytes)", wav.len());
            let payload = &wav[WAV_HEADER_BYTES..];
            let n_samples = (payload.len() / 2).min(SLOT_SAMPLES);

            // Decode samples in 100 ms chunks and push.
            let mut i = 0;
            while i < n_samples {
                let end = (i + CHUNK_SAMPLES).min(n_samples);
                // Source bytes are little-endian i16 pairs.
                let mut chunk = [0i16; CHUNK_SAMPLES];
                let n_this = end - i;
                for k in 0..n_this {
                    let off = (i + k) * 2;
                    chunk[k] = i16::from_le_bytes([payload[off], payload[off + 1]]);
                }
                let st = unsafe {
                    mfsk_ft8::mfsk_ft8_stream_push_i16(stream, chunk.as_ptr(), n_this)
                };
                if st != mfsk_ft8::MfskFt8Status::Ok {
                    log::warn!("wav_sim: push_i16 returned status {:?}", st);
                }
                // Fan out to per-chunk hook (Phase-E stress test).
                unsafe {
                    if let Some(hook) = CHUNK_HOOK {
                        hook(&chunk[..n_this]);
                    }
                }
                i = end;
                unsafe { vTaskDelay(delay_ticks) };
            }

            // WAV fully pushed — notify the consumer (decode task)
            // that a slot is ready to be peeked. This synchronises
            // the decode loop with the simulated audio stream so
            // each decode sees exactly one WAV, not a sliding mix.
            unsafe {
                let consumer = CONSUMER_TASK;
                if !consumer.is_null() {
                    xTaskGenericNotify(
                        consumer,
                        0,
                        0,
                        eNotifyAction_eIncrement,
                        core::ptr::null_mut(),
                    );
                }
            }
        }
    }
}
