//! Dual-core worker for Pass 2 / Stage 3 candidate-loop parallelism.
//!
//! The FT8 decode pipeline's per-candidate work (Pass 2 sync_quality
//! re-rank and Stage 3 refine + BP staircase) has zero dependencies
//! between candidates. We split the candidate list in two and run the
//! halves on PRO_CPU (main task, core 0) and APP_CPU (this module's
//! worker task, core 1) concurrently.
//!
//! Pass 2 / Stage 3 happen *after* the slot's audio capture completes
//! so there's no contention with `stage1_inc` (which only runs while
//! audio is arriving). The worker has been the load-bearing source of
//! the Core2 LX6 sub-2 s achievement and is preserved here.
//!
//! Stage 2 (`coarse_sync_split`) was historically dispatched the same
//! way, but on Phase E2 its CPU contention with `stage1_inc` corrupted
//! the audio ring (`project_decode_block_embedded.md`); rx_wavsim's
//! hot path uses `mfsk_core::ft8::decode_block::coarse_sync_with_allsum`
//! directly on main, and the non-Phase-E fallback here just runs the
//! halves sequentially — the cost is negligible compared to Stage 3.
//!
//! ## Memory
//!
//! Each core needs its own Q15 basis scratch (60 KB) for the esp-dsp
//! asm dot product to stay in internal DRAM. Main task uses the
//! `BASIS_RE` / `BASIS_IM` static in `main.rs`; this module owns
//! `BASIS_RE_C1` / `BASIS_IM_C1` in its own `.bss` for the worker.
//! Total internal DRAM cost: 120 KB.
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
//! ## Safety
//!
//! `audio` is passed as `*const i16 + len` because the worker is a
//! C-ABI function and lifetimes can't cross the boundary. We only
//! call dispatch while the audio slice is borrowed by main, and we
//! always block-wait on the worker before returning, so the slice
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
    coarse_sync, process_candidates_into_with_cs_scratch, refine_candidates_into,
    RefinedCandidate, Spectrogram, BASIS_SCRATCH_LEN,
};

use crate::internal_pool::{CS_SCRATCH_MAIN, CS_SCRATCH_WORKER};

const PD_PASS: i32 = 1;
const PD_TRUE: i32 = 1;

/// Worker-side basis scratch. `.bss` placement keeps it in internal
/// DRAM where esp-dsp's asm dot product runs at 1 cycle/sample (PSRAM
/// is 5–10× slower per access and would erase the worker's speedup —
/// see `project_decode_block_embedded.md` learnings #7 and #8).
static mut BASIS_RE_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

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
    /// Phase E2: coarse_sync with caller-supplied per-half allsum
    /// slice. Spawned only AFTER the slot's audio capture has
    /// completed and `stage1_inc` has finished all 92 pairs (the
    /// original race was contention between this worker and an
    /// in-flight `stage1_inc` on APP_CPU during capture; rx_wavsim
    /// runs stage 2 post-capture so contention does not occur).
    CoarseSyncWithAllsum {
        spec: *const Spectrogram,
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        allsum_ptr: *const f32,
        allsum_len: usize,
        out: *mut Option<Vec<SyncCandidate>>,
    },
}

/// SAFETY: pointers / `Vec`s are written by main and read by worker
/// under the `xTaskNotifyGive` handshake, which is a full memory
/// barrier on Xtensa.
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

extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("dsp_worker: started on core {}", current_core());
    loop {
        unsafe {
            let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
        }
        let job = unsafe { (*JOB_SLOT.inner.get()).take() };
        match job {
            Some(Job::Pass2 {
                audio,
                audio_len,
                cands,
                max_cand,
                out,
            }) => {
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
            Some(Job::CoarseSyncWithAllsum {
                spec,
                freq_min,
                freq_max,
                sync_min,
                max_cand,
                allsum_ptr,
                allsum_len,
                out,
            }) => {
                let spec_ref = unsafe { &*spec };
                let allsum =
                    unsafe { core::slice::from_raw_parts(allsum_ptr, allsum_len) };
                let result =
                    mfsk_core::ft8::decode_block::coarse_sync_with_allsum(
                        spec_ref, freq_min, freq_max, sync_min, max_cand, allsum,
                    );
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
                    process_candidates_into_with_cs_scratch(
                        audio_slice,
                        cands,
                        depth,
                        &mut BASIS_RE_C1,
                        &mut BASIS_IM_C1,
                        &mut CS_SCRATCH_WORKER,
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
            16384,
            ptr::null_mut(),
            5,
            &raw mut WORKER_TASK,
            1,
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

/// Run Stage 2 (`coarse_sync`) sequentially per frequency half on main.
///
/// Historically dispatched to the worker, but Phase E2 exposed a race
/// with `stage1_inc` (worker preempts the live-audio incremental task
/// on core 1, audio ring wraps mid-pair). Sequential per-half on main
/// is the safe fallback; rx_wavsim's hot path uses
/// `coarse_sync_with_allsum` directly so this entry is rarely hit.
///
/// `coarse_sync`'s ratio metric `t / (mean_others + ε)` computes
/// `mean_others` over bins in the searched range only, so each half
/// uses local statistics — splitting at the midpoint is recall-neutral
/// on the 3-real-QSO baseline.
pub fn coarse_sync_split(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let mid = 0.5 * (freq_min + freq_max);
    let mut head = coarse_sync(spec, freq_min, mid, sync_min, max_cand);
    let tail = coarse_sync(spec, mid, freq_max, sync_min, max_cand);
    head.extend(tail);
    head.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head.truncate(max_cand);
    head
}

/// Phase-E2: like [`coarse_sync_split`] but consumes per-half pre-built
/// allsums (built incrementally by `stage1_inc` during capture).
/// Dispatches the tail half to the APP_CPU worker and runs the head
/// half on main concurrently. Safe in rx_wavsim because Phase-E
/// guarantees `stage1_inc` finishes all 92 pairs before stage 2 starts
/// (caller asserts `stage1_inc::PAIR_DONE >= N_PAIRS`).
pub fn coarse_sync_split_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum_head: &[f32],
    allsum_tail: &[f32],
) -> Vec<SyncCandidate> {
    use mfsk_core::ft8::decode_block::coarse_sync_with_allsum;
    let mid = 0.5 * (freq_min + freq_max);

    let mut worker_out: Option<Vec<SyncCandidate>> = None;
    unsafe {
        *JOB_SLOT.inner.get() = Some(Job::CoarseSyncWithAllsum {
            spec: spec as *const _,
            freq_min: mid,
            freq_max,
            sync_min,
            max_cand,
            allsum_ptr: allsum_tail.as_ptr(),
            allsum_len: allsum_tail.len(),
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

    // Main runs head in parallel with worker on tail.
    let mut local =
        coarse_sync_with_allsum(spec, freq_min, mid, sync_min, max_cand, allsum_head);

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
/// Each half is scored independently with its own BASIS scratch, then
/// the two top-`max_cand` lists are merged, sorted by `q_block0`, and
/// truncated.
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

    let mut local = refine_candidates_into(audio, head, max_cand, basis_re_main, basis_im_main);

    unsafe {
        let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
    }

    let worker = worker_out.expect("worker did not write result");
    local.extend(worker);
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// Run Stage 3 (refine + BP staircase) split across both cores. Per-cand
/// results are independent so the two lists are concatenated.
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

    #[allow(static_mut_refs)]
    let mut local = unsafe {
        process_candidates_into_with_cs_scratch(
            audio,
            head,
            depth,
            basis_re_main,
            basis_im_main,
            &mut CS_SCRATCH_MAIN,
        )
    };

    unsafe {
        let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
    }

    let worker = worker_out.expect("worker did not write result");
    local.extend(worker);
    local
}
