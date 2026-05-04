//! WAV-fed streaming RX bench.
//!
//! Stand-in for the real I2S PDM mic. Spawns the `wav_sim` task which
//! pumps baked QSO WAVs into the `mfsk_ft8_stream_*` ring buffer at
//! real-time pace, then runs the decode pipeline once per 15 s slot
//! using the same dual-core split as the compute bench (`main.rs`).
//!
//! Goal: validate the streaming path end-to-end (push → resampler-or-
//! pass-through → ring → peek_latest → dual-core decode) without
//! needing real mic hardware. Once verified, swap `wav_sim::spawn`
//! for `pdm_setup` (see `bin/rx_skeleton.rs`) and the rest of the
//! pipeline keeps working unchanged.
//!
//! Build: `cargo build --release --bin rx-wavsim`.

#![allow(dead_code)]

use embedded_shared::{dual_core, esp_dsp_fft, stage1_inc, wav_sim};

extern crate alloc;

use core::ffi::c_int;
use core::ptr;

use mfsk_ft8::{
    mfsk_ft8_basis_scratch_len, mfsk_ft8_stream_buffered_samples, mfsk_ft8_stream_drain,
    mfsk_ft8_stream_new, mfsk_ft8_stream_peek_latest, MfskFt8Stream,
};

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{compute_spectrogram, BASIS_SCRATCH_LEN};
use mfsk_core::msg::wsjt77::unpack77;

// Same audio source the compute bench uses: 12 kHz / mono / i16 PCM,
// each ≈ 360 KB so the wav_sim task pushes at 1× real time over 15 s.
const QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../assets/qso1.wav"),
    include_bytes!("../../assets/qso2.wav"),
    include_bytes!("../../assets/qso3_busy.wav"),
];

// Ring sized slightly above one slot so the FIFO eviction doesn't
// crop the very first samples of each WAV (qso3.wav is 180 101
// samples — at exactly 180 000 cap the leading 101 samples get
// evicted by the time wav_sim finishes pushing, producing an
// 8.4 ms phase shift that drops weak-signal recall on busy bands).
const STREAM_CAP: usize = 200_000;
const SRC_RATE_HZ: u32 = 12_000; // pass-through (no resampling)
const SLOT_LEN_SAMPLES: usize = 180_000;

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;
const Q_THRESH: u32 = 12;

/// Same per-core BASIS scratch as `main.rs`. Worker side lives in
/// `dual_core.rs`.
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("rx-wavsim starting (mfsk-core {})", mfsk_core::VERSION);

    // Sanity check on basis size.
    let need = mfsk_ft8_basis_scratch_len();
    assert!(BASIS_SCRATCH_LEN >= need, "BASIS_SCRATCH_LEN too small");

    // Bump main task priority above wav_sim's so the notify path
    // immediately preempts the sim task — otherwise wav_sim (higher
    // priority by default vs. ESP-IDF main task at priority 1)
    // continues pumping the next WAV and pollutes the ring before
    // decode peeks it. Default main priority on esp-idf is 1; the
    // sim runs at 4 (see `wav_sim::spawn`), and decode at 6 wins.
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    // Pre-warm FFT planner before any worker uses it.
    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);
    dual_core::init();
    stage1_inc::init();
    wav_sim::set_chunk_hook(stage1_inc::push_chunk);

    // Register this task as the slot-boundary consumer; wav_sim will
    // notify us each time a WAV finishes pushing.
    let main_handle = unsafe { esp_idf_svc::sys::xTaskGetCurrentTaskHandle() };
    wav_sim::set_consumer(main_handle);

    // Bring up the streaming wrapper. SRC_RATE_HZ == 12_000 means the
    // resampler is a no-op pass-through — wav_sim's i16 samples land
    // straight in the ring.
    let stream: *mut MfskFt8Stream = mfsk_ft8_stream_new(SRC_RATE_HZ, STREAM_CAP);
    assert!(!stream.is_null(), "mfsk_ft8_stream_new failed");

    // Start the simulated audio source.
    wav_sim::spawn(stream, QSO_WAVS);

    // Slot decode loop. wav_sim notifies us after each WAV completes
    // pushing → the ring contains exactly one WAV's worth of samples
    // (180000) and peek_latest returns a clean slot.
    log::info!("rx-wavsim: decode loop ready; awaiting wav_sim slot notifications");
    loop {
        unsafe {
            // Block until wav_sim signals "WAV[i] complete".
            let _ = esp_idf_svc::sys::ulTaskGenericNotifyTake(0, 1, u32::MAX);
        }
        decode_one_slot(stream);
    }
}

fn decode_one_slot(stream: *mut MfskFt8Stream) {
    let buffered = unsafe { mfsk_ft8_stream_buffered_samples(stream) };
    if buffered < SLOT_LEN_SAMPLES * 90 / 100 {
        log::warn!("slot underrun: {buffered} / {SLOT_LEN_SAMPLES}");
        return;
    }

    // Snapshot the latest 15 s into a PSRAM-backed Vec.
    let mut slot: alloc::vec::Vec<i16> = alloc::vec![0i16; SLOT_LEN_SAMPLES];
    let n = unsafe { mfsk_ft8_stream_peek_latest(stream, slot.as_mut_ptr(), slot.len()) };
    if n != SLOT_LEN_SAMPLES {
        log::warn!("peek_latest returned {n} (expected {SLOT_LEN_SAMPLES})");
    }

    // Same pipeline as `main.rs::decode_one`, but driven from a live
    // ring snapshot. Phase E shortcut: if `stage1_inc` finished all
    // 92 pairs during slot capture, take its prebuilt Spectrogram and
    // skip stage 1 entirely — the FFT cost was hidden under capture.
    let t0 = now_us();
    // Phase-E2: prefer incremental spec + per-half allsums prebuilt
    // by stage1_inc during the 15 s capture window (saves the
    // 280-300 ms allsum precompute from stage 2). Falls back to
    // legacy spec-only + internal precompute if pairs not all ready.
    let (spec, allsum_pair_opt, stage1_path) = match stage1_inc::take_spec_and_allsum() {
        Some((s, head, tail)) => (s, Some((head, tail)), "incremental"),
        None => (compute_spectrogram(&slot[..], 3_000.0), None, "fallback"),
    };
    let t1 = now_us();
    log::info!(
        "  stage 1 ({:>11}): {:>7} us  ({}× FFT, {} freq bins)",
        stage1_path,
        t1 - t0,
        spec.n_time,
        spec.n_freq,
    );

    let t2 = now_us();
    // **Sequential per-half on main**, not dual_core dispatch — the
    // dispatch path runs the dual_core worker on core 1 (priority 5),
    // which preempts stage1_inc (also core 1, priority 3). Starved
    // stage1_inc misses push_chunk advance_pairs cycles, the audio
    // buffer wraps with stale data from the previous slot, and late
    // pairs (m≈174-175) compute spec/allsum from corrupted audio →
    // qso3 0/7 on the incremental path. Sequential keeps stage 2
    // entirely on core 0 so core 1 is free for stage1_inc.
    let pass1 = match &allsum_pair_opt {
        Some((head, tail)) => {
            // Phase E2 dispatch (`coarse_sync_split_with_allsum`)
            // exists in `dual_core.rs` and is timing-correct (slot 1
            // separation from wav_sim) but produces 0-result decodes
            // on slots 2+ of the rx_wavsim loop — slot 1 works, then
            // recall collapses. Suspect FPU / heap state accumulation
            // on APP_CPU between worker calls. Investigation deferred;
            // sequential per-half on main is the recall-correct path.
            let mut p = mfsk_core::ft8::decode_block::coarse_sync_with_allsum(
                &spec, 100.0, 1550.0, 1.0, PASS1_LIMIT, head,
            );
            let t = mfsk_core::ft8::decode_block::coarse_sync_with_allsum(
                &spec, 1550.0, 3_000.0, 1.0, PASS1_LIMIT, tail,
            );
            p.extend(t);
            p.sort_by(|a, b| {
                b.score
                    .partial_cmp(&a.score)
                    .unwrap_or(core::cmp::Ordering::Equal)
            });
            p.truncate(PASS1_LIMIT);
            p
        }
        None => dual_core::coarse_sync_split(&spec, 100.0, 3_000.0, 1.0, PASS1_LIMIT),
    };
    let t3 = now_us();
    log::info!(
        "  stage 2 (sync): {:>7} us  ({} cand)",
        t3 - t2,
        pass1.len()
    );
    drop(spec);

    let t_p2 = now_us();
    #[allow(static_mut_refs)]
    let pass2 = unsafe {
        dual_core::pass2_split(&slot[..], pass1, MAX_CAND, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t_p2_end = now_us();
    log::info!(
        "  pass 2:         {:>7} us  → top {}",
        t_p2_end - t_p2,
        pass2.len()
    );

    let depth = DecodeDepth::BpAll;
    let t4 = now_us();
    #[allow(static_mut_refs)]
    let results = unsafe {
        dual_core::stage3_split(&slot[..], pass2, depth, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t5 = now_us();
    log::info!(
        "  stage 3:        {:>7} us  ({} result(s))",
        t5 - t4,
        results.len()
    );
    log::info!(
        "  ─── slot total: {:>7} us = {:.3} s",
        t5 - t0,
        (t5 - t0) as f64 / 1e6
    );
    // Phase-E feasibility readout.
    let pairs_done = stage1_inc::PAIR_DONE.load(core::sync::atomic::Ordering::Acquire);
    let inc_us = stage1_inc::last_slot_inc_us();
    log::info!(
        "  Phase-E PoC: stage1_inc {} / 92 pairs done, {} us total in advance ({}% of capture)",
        pairs_done,
        inc_us,
        (inc_us * 100) / 15_000_000
    );
    stage1_inc::mark_slot_boundary();
    for (i, r) in results.iter().enumerate() {
        if let Some(text) = unpack77(&r.message77) {
            log::info!(
                "    [{i}]  {:>4.0} Hz  SNR={:>5.1} dB  e={}  '{}'",
                r.freq_hz,
                r.snr_db,
                r.hard_errors,
                text
            );
        }
    }

    // Don't drain: the ring is FIFO-bounded at STREAM_CAP=180000 so
    // new pushes from wav_sim auto-evict the oldest samples. With
    // notification-driven dispatch (one notify per WAV completion),
    // peek_latest at notify-time always returns exactly one WAV's
    // worth of samples (the most recent 180 000 = entire WAV[N]).
    // Calling drain(180000) here would also throw away wav_sim's
    // already-pushed prefix of WAV[N+1] and cause underrun on the
    // next slot.
    let _ = mfsk_ft8_stream_drain; // keep import; unused
    let _ = c_int::from(0);
    let _: usize = ptr::null::<u8>() as usize;
}
