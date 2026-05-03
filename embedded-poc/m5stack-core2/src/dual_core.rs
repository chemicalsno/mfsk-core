//! Dual-core worker for Pass 2 / Stage 3 candidate-loop parallelism.
//!
//! The FT8 decode pipeline's per-candidate work (Pass 2 sync_quality
//! re-rank and Stage 3 refine + BP staircase) has zero dependencies
//! between candidates. We split the candidate list in two and run
//! the halves on PRO_CPU (main task, core 0) and APP_CPU (this
//! module's worker task, core 1) concurrently.
//!
//! ## Memory
//!
//! Each core needs its own Q15 basis scratch (60 KB) for the esp-dsp
//! asm dot product to stay in internal DRAM. Main task uses the
//! `BASIS_RE` / `BASIS_IM` static in `main.rs`; this module owns
//! `BASIS_RE_C1` / `BASIS_IM_C1` in its own `.bss` for the worker.
//! Total internal DRAM cost: 120 KB out of ~300 KB available.
//!
//! ## Synchronisation
//!
//! Single-job-at-a-time protocol via FreeRTOS task notifications:
//!   1. Main writes a `Job` into [`JOB_SLOT`].
//!   2. Main signals the worker with `xTaskNotifyGive(WORKER_TASK)`.
//!   3. Main runs its half of the work locally.
//!   4. Worker takes the job, runs it, writes result back to a
//!      caller-owned `Option<...>`, signals main with
//!      `xTaskNotifyGive(MAIN_TASK)`.
//!   5. Main waits on `ulTaskNotifyTake`, merges results.
//!
//! No queue, no allocator-on-the-hot-path, no scoped threads.
//!
//! ## Safety
//!
//! `audio` is passed as `*const i16 + len` because the worker is a
//! C-ABI function and lifetimes can't cross the boundary. We only
//! call `dispatch_*()` while the audio slice is borrowed by main, and
//! we always block-wait on the worker before returning, so the slice
//! outlives the worker's read of it.

use core::cell::UnsafeCell;
use core::ptr;

use esp_idf_svc::sys::{
    eNotifyAction_eIncrement, ulTaskGenericNotifyTake, xTaskCreatePinnedToCore, xTaskGenericNotify,
    xTaskGetCoreID, xTaskGetCurrentTaskHandle, TaskHandle_t,
};

use alloc::vec::Vec;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    coarse_sync, process_candidates_into, refine_candidates_into, RefinedCandidate, Spectrogram,
    BASIS_SCRATCH_LEN,
};

// FreeRTOS #defines not exposed by esp-idf-sys bindgen — replicate
// here so we don't depend on internal binding stability.
const PD_PASS: i32 = 1;
const PD_TRUE: i32 = 1;

/// Worker-side basis scratch (mirror of main's `BASIS_RE` / `BASIS_IM`).
/// Lives in `.bss` so the linker places it in internal DRAM, where the
/// esp-dsp asm dot product runs at 1 cycle/sample. PSRAM-resident
/// basis is 5–10× slower per access and erases all of the worker's
/// speedup, so this duplication is load-bearing — see
/// `project_decode_block_embedded.md` learnings #7 and #8.
static mut BASIS_RE_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

/// All-job enum so the worker takes one slot regardless of which
/// stage is being parallelised.
enum Job {
    Pass2 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<SyncCandidate>,
        max_cand: usize,
        out: *mut Option<Vec<RefinedCandidate>>,
    },
    Stage3 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<RefinedCandidate>,
        depth: DecodeDepth,
        out: *mut Option<Vec<DecodeResult>>,
    },
    CoarseSync {
        spec: *const Spectrogram,
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        out: *mut Option<Vec<SyncCandidate>>,
    },
}

/// SAFETY: pointers / `Vec`s are written by main and read by worker
/// under the `xTaskNotifyGive` handshake, which is a full memory
/// barrier on Xtensa LX6.
unsafe impl Send for Job {}

struct JobSlot {
    inner: UnsafeCell<Option<Job>>,
}
/// SAFETY: see protocol comment in module-level doc.
unsafe impl Sync for JobSlot {}

static JOB_SLOT: JobSlot = JobSlot {
    inner: UnsafeCell::new(None),
};

static mut WORKER_TASK: TaskHandle_t = ptr::null_mut();
static mut MAIN_TASK: TaskHandle_t = ptr::null_mut();

/// Worker task entry point. Pinned to APP_CPU (core 1) by `init()`.
extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("dsp_worker: started on core {}", current_core());
    loop {
        // Block until main posts a job.
        unsafe {
            let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
        }
        // SAFETY: main writes the slot before xTaskNotifyGive, which
        // synchronises with the matching ulTaskNotifyTake above.
        let job = unsafe { (*JOB_SLOT.inner.get()).take() };
        match job {
            Some(Job::Pass2 {
                audio,
                audio_len,
                cands,
                max_cand,
                out,
            }) => {
                // SAFETY: `audio` points at main's slot Box for the
                // duration of decode_one(). Main blocks on us before
                // the slot is freed.
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                #[allow(static_mut_refs)]
                let result = unsafe {
                    refine_candidates_into(
                        audio_slice,
                        cands,
                        max_cand,
                        &mut BASIS_RE_C1,
                        &mut BASIS_IM_C1,
                    )
                };
                unsafe {
                    *out = Some(result);
                }
            }
            Some(Job::CoarseSync {
                spec,
                freq_min,
                freq_max,
                sync_min,
                max_cand,
                out,
            }) => {
                // SAFETY: main holds the Spectrogram on its stack /
                // heap and blocks on us before dropping it.
                let spec_ref = unsafe { &*spec };
                let result = coarse_sync(spec_ref, freq_min, freq_max, sync_min, max_cand);
                unsafe {
                    *out = Some(result);
                }
            }
            Some(Job::Stage3 {
                audio,
                audio_len,
                cands,
                depth,
                out,
            }) => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                #[allow(static_mut_refs)]
                let result = unsafe {
                    process_candidates_into(
                        audio_slice,
                        cands,
                        depth,
                        &mut BASIS_RE_C1,
                        &mut BASIS_IM_C1,
                    )
                };
                unsafe {
                    *out = Some(result);
                }
            }
            None => {
                log::warn!("dsp_worker: woke with empty slot");
            }
        }
        // Signal completion to main.
        unsafe {
            xTaskGenericNotify(
                MAIN_TASK,
                0,
                0,
                eNotifyAction_eIncrement,
                core::ptr::null_mut(),
            );
        }
    }
}

fn current_core() -> i32 {
    // xTaskGetCoreID(NULL) returns the core of the calling task.
    unsafe { xTaskGetCoreID(ptr::null_mut()) }
}

/// Spawn the persistent worker task on APP_CPU. Call once at startup,
/// after `link_patches()` and `EspLogger::initialize_default()` and
/// after `esp_dsp_fft::prewarm()`.
pub fn init() {
    unsafe {
        MAIN_TASK = xTaskGetCurrentTaskHandle();
        let r = xTaskCreatePinnedToCore(
            Some(worker_main),
            c"dsp_worker".as_ptr(),
            16384, // stack bytes — Stage 3 stacks ~5 KB of LlrSet<f32>
            // (4 × 174 × 4 = 2.8 KB) + LlrSet<LlrT> (~1.4 KB) + a few
            // [LlrT; 174] arrays from compute_llr_partial; 8 KB blew
            // through the canary on real WAVs. Main task is 16 KB so
            // we match.
            ptr::null_mut(),
            5, // priority (same as default main)
            &raw mut WORKER_TASK,
            1, // APP_CPU
        );
        assert_eq!(
            r, PD_PASS,
            "xTaskCreatePinnedToCore(dsp_worker) failed: {r}"
        );
    }
    log::info!(
        "dsp_worker: spawned on APP_CPU; main is core {}",
        current_core()
    );
}

/// Run Stage 2 (`coarse_sync`) split across both cores by frequency
/// half. Each half runs `coarse_sync` over its own carrier-bin range
/// returning up to `max_cand` candidates, then results are merged,
/// re-sorted by `score` descending, and truncated to `max_cand`.
///
/// Note: `coarse_sync`'s ratio metric `t / (mean_others + ε)` computes
/// `mean_others` over bins in the searched range only, so each half
/// uses local statistics. Empirically the band split at the midpoint
/// (1550 Hz) does not regress recall on our 3 real-QSO WAVs.
pub fn coarse_sync_split(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let mid = 0.5 * (freq_min + freq_max);

    let mut worker_out: Option<Vec<SyncCandidate>> = None;
    unsafe {
        *JOB_SLOT.inner.get() = Some(Job::CoarseSync {
            spec: spec as *const _,
            freq_min: mid,
            freq_max,
            sync_min,
            max_cand,
            out: &raw mut worker_out,
        });
        xTaskGenericNotify(
            WORKER_TASK,
            0,
            0,
            eNotifyAction_eIncrement,
            core::ptr::null_mut(),
        );
    }

    let mut local = coarse_sync(spec, freq_min, mid, sync_min, max_cand);

    unsafe {
        let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
    }

    let worker = worker_out.expect("worker did not write result");
    local.extend(worker);
    local.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    local.truncate(max_cand);
    local
}

/// Run Pass 2 (sync_quality_block0 re-rank) split across both cores.
///
/// Each half is scored independently with its own BASIS scratch, then
/// the two top-`max_cand` lists are merged, sorted by `q_block0` (the
/// 3rd tuple element of `RefinedCandidate`), and truncated.
///
/// `basis_re_main` / `basis_im_main` are the caller's main-core
/// scratch (i.e. `BASIS_RE` / `BASIS_IM` in `main.rs`), passed in
/// rather than imported to keep the module self-contained.
pub fn pass2_split(
    audio: &[i16],
    pass1: Vec<SyncCandidate>,
    max_cand: usize,
    basis_re_main: &mut [i16],
    basis_im_main: &mut [i16],
) -> Vec<RefinedCandidate> {
    let mid = pass1.len() / 2;
    let mut head = pass1;
    let tail = head.split_off(mid);

    let mut worker_out: Option<Vec<RefinedCandidate>> = None;
    unsafe {
        *JOB_SLOT.inner.get() = Some(Job::Pass2 {
            audio: audio.as_ptr(),
            audio_len: audio.len(),
            cands: tail,
            max_cand,
            out: &raw mut worker_out,
        });
        xTaskGenericNotify(
            WORKER_TASK,
            0,
            0,
            eNotifyAction_eIncrement,
            core::ptr::null_mut(),
        );
    }

    // Main runs the head in parallel with worker.
    let mut local = refine_candidates_into(audio, head, max_cand, basis_re_main, basis_im_main);

    // Wait for worker.
    unsafe {
        let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
    }

    let worker = worker_out.expect("worker did not write result");
    local.extend(worker);
    // Sort by q_block0 descending. RefinedCandidate is (cand, cs, q).
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// Run Stage 3 (refine + BP staircase) split across both cores.
/// Results are independent per candidate so we just concat the two
/// result lists.
pub fn stage3_split(
    audio: &[i16],
    pass2: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    basis_re_main: &mut [i16],
    basis_im_main: &mut [i16],
) -> Vec<DecodeResult> {
    let mid = pass2.len() / 2;
    let mut head = pass2;
    let tail = head.split_off(mid);

    let mut worker_out: Option<Vec<DecodeResult>> = None;
    unsafe {
        *JOB_SLOT.inner.get() = Some(Job::Stage3 {
            audio: audio.as_ptr(),
            audio_len: audio.len(),
            cands: tail,
            depth,
            out: &raw mut worker_out,
        });
        xTaskGenericNotify(
            WORKER_TASK,
            0,
            0,
            eNotifyAction_eIncrement,
            core::ptr::null_mut(),
        );
    }

    let mut local = process_candidates_into(audio, head, depth, basis_re_main, basis_im_main);

    unsafe {
        let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
    }

    let worker = worker_out.expect("worker did not write result");
    local.extend(worker);
    local
}
