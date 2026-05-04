//! WAV-fed streaming RX bench, target-agnostic.
//!
//! Runs the full pipeline:
//! ```text
//! wav_sim → ChunkMsg → stage1_inc → SpecBundle/Slot → main → dual_core
//! ```
//!
//! Stage 2 (`coarse_sync_split_with_allsum`) starts on `SpecBundle`
//! arrival (≈ 200 ms before SlotEnd), so it overlaps the tail of audio
//! capture. Post-SlotEnd decode = pass 2 + stage 3 only.
//!
//! Per-target binaries (m5stack-core2 / m5stack-s3) just supply the
//! WAV byte slices and call `run()`. No target-specific code lives
//! here — chip / PSRAM / priority knobs are uniform across LX6 and LX7.

extern crate alloc;

use alloc::vec::Vec;

use mfsk_ft8::mfsk_ft8_basis_scratch_len;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
use mfsk_core::msg::wsjt77::unpack77;

use crate::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// Per-core BASIS scratch (main side). 60 KB × 2, internal DRAM.
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Run the rx_wavsim bench.
///
/// `wavs` is the playlist (≥ 1 baked WAV; cycled indefinitely).
pub fn run(wavs: &'static [&'static [u8]]) -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("rx-wavsim starting (mfsk-core {})", mfsk_core::VERSION);

    let need = mfsk_ft8_basis_scratch_len();
    assert!(BASIS_SCRATCH_LEN >= need, "BASIS_SCRATCH_LEN too small");

    // Bump main task priority above wav_sim (4) and stage1_inc (3).
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);

    dual_core::init();
    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    stage1_inc::spawn(chunk_q, slot_q, spec_q);
    wav_sim::spawn(wavs, chunk_q);

    log::info!("rx-wavsim: decode loop ready; awaiting spec/slot from stage1_inc");
    loop {
        // SpecBundle arrives ≈ 200 ms before SlotEnd, so stage 2 runs
        // in parallel with the tail of capture.
        let spec = pipeline::recv_box::<pipeline::SpecBundle>(spec_q);
        let t_s2_start = now_us();
        let pass1 = dual_core::coarse_sync_split_with_allsum(
            &spec.spec,
            100.0,
            3_000.0,
            1.0,
            PASS1_LIMIT,
            &spec.allsum_head,
            &spec.allsum_tail,
        );
        let t_s2_end = now_us();
        drop(spec);

        let slot = pipeline::recv_box::<pipeline::Slot>(slot_q);
        decode_one_slot(*slot, pass1, t_s2_end - t_s2_start);
    }
}

fn decode_one_slot(slot: pipeline::Slot, pass1: Vec<SyncCandidate>, stage2_us: i64) {
    let wav_idx = slot.wav_idx;
    let inc_us = slot.inc_total_us;
    let pass1_n = pass1.len();
    let post_slot_t0 = now_us();

    log::info!(
        "rx-wavsim: WAV[{wav_idx}] slot received (audio={} samples, pass1={pass1_n})",
        slot.audio.len(),
    );
    log::info!(
        "  stage 2 (during cap): {:>7} us  ({pass1_n} cand)",
        stage2_us
    );

    let t2 = now_us();
    #[allow(static_mut_refs)]
    let pass2 = unsafe {
        dual_core::pass2_split(&slot.audio, pass1, MAX_CAND, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t3 = now_us();
    log::info!(
        "  pass 2:               {:>7} us  → top {}",
        t3 - t2,
        pass2.len()
    );

    let depth = DecodeDepth::BpAll;
    let t4 = now_us();
    #[allow(static_mut_refs)]
    let results = unsafe {
        dual_core::stage3_split(&slot.audio, pass2, depth, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t5 = now_us();
    log::info!(
        "  stage 3:              {:>7} us  ({} result(s))",
        t5 - t4,
        results.len()
    );
    log::info!(
        "  ─── post-SlotEnd:     {:>7} us = {:.3} s",
        t5 - post_slot_t0,
        (t5 - post_slot_t0) as f64 / 1e6
    );
    log::info!(
        "  Phase-E: stage1_inc {} us in advance ({}% of capture)",
        inc_us,
        (inc_us * 100) / 15_000_000
    );

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
}
