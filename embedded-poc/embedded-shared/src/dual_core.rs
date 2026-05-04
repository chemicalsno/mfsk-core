//! Dual-core worker for Pass 2 / Stage 3 / Stage 2 (Phase E2)
//! candidate-loop parallelism.
//!
//! ## Protocol — host mpsc に対応する単一値転送チャネル
//!
//! 旧実装は `*mut Option<Vec<T>>` (out-pointer) と `xTaskNotify` slot を
//! 別チャネルとして併用していた。slot 跨ぎでデータ書き込みと完了通知
//! が desync し、Phase E2 で slot 2+ が 0 results になる症状が出た。
//!
//! 本実装は **FreeRTOS Queue による値転送1チャネル**に統一する。
//! 入力は `Box<Job>` の生ポインタを `JOB_Q` に send、結果は variant
//! 別の result queue (`*mut Vec<T>`) で送り返す。Queue が data 転送と
//! 完了通知を atomic に提供するため、host の `mpsc::sync_channel` と
//! 構造的に等価になる（depth=1 → `sync_channel(1)` 相当）。
//!
//! ## Memory
//!
//! Worker has its own Q15 basis scratch (`BASIS_RE_C1` / `BASIS_IM_C1`,
//! 60 KB each, internal DRAM `.bss`) so the esp-dsp asm dot product
//! stays at 1 cycle/sample. Total internal DRAM: 120 KB, unchanged.
//!
//! ## Safety
//!
//! Job 内の生ポインタ (`audio`, `spec`, `allsum_ptr`) は dispatch 関数
//! の call frame に紐付いた借用を消したもの。dispatch 関数は `xQueueSend`
//! → 自分の half を計算 → `xQueueReceive` をブロック実行するため、
//! worker が触る間 main 側のスライスは必ず生存している。

use core::cell::UnsafeCell;
use core::ptr;

use esp_idf_svc::sys::{
    xQueueGenericCreate, xQueueGenericSend, xQueueReceive, xTaskCreatePinnedToCore,
    xTaskGetCoreID, QueueHandle_t,
};

use alloc::boxed::Box;
use alloc::vec::Vec;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    coarse_sync, process_candidates_into_with_cs_scratch, refine_candidates_into,
    RefinedCandidate, Spectrogram, BASIS_SCRATCH_LEN,
};

use crate::internal_pool::{CS_SCRATCH_MAIN, CS_SCRATCH_WORKER};

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;

/// Worker-side basis scratch (60 KB each, internal DRAM).
static mut BASIS_RE_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

enum Job {
    Pass2 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<SyncCandidate>,
        max_cand: usize,
    },
    Stage3 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<RefinedCandidate>,
        depth: DecodeDepth,
    },
    CoarseSyncWithAllsum {
        spec: *const Spectrogram,
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        allsum_ptr: *const f32,
        allsum_len: usize,
    },
}

/// SAFETY: Job's raw pointers are produced by dispatch fns that
/// block-wait on the result before returning, so the referenced data
/// outlives the worker's access. Vec ownership transfers via Box.
unsafe impl Send for Job {}

/// Queue handle holder — initialised once in `init()`, then read-only.
struct QueueCell(UnsafeCell<QueueHandle_t>);
unsafe impl Sync for QueueCell {}
impl QueueCell {
    const fn new() -> Self {
        Self(UnsafeCell::new(ptr::null_mut()))
    }
    fn get(&self) -> QueueHandle_t {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only call from `init()` before any other access.
    unsafe fn set(&self, q: QueueHandle_t) {
        unsafe { *self.0.get() = q };
    }
}

static JOB_Q: QueueCell = QueueCell::new();
static PASS2_RESULT_Q: QueueCell = QueueCell::new();
static STAGE3_RESULT_Q: QueueCell = QueueCell::new();
static COARSE_RESULT_Q: QueueCell = QueueCell::new();

#[inline]
unsafe fn queue_create(item_size: usize) -> QueueHandle_t {
    let q = unsafe {
        xQueueGenericCreate(
            1, // depth
            item_size as u32,
            QUEUE_TYPE_BASE,
        )
    };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// Send a single boxed pointer through a depth-1 queue.
/// SAFETY: caller transfers ownership of `ptr` to the receiver.
#[inline]
unsafe fn queue_send_ptr<T>(q: QueueHandle_t, ptr: *mut T) {
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&ptr as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

/// Receive a single pointer from a depth-1 queue.
/// SAFETY: receiver takes ownership of returned pointer.
#[inline]
unsafe fn queue_recv_ptr<T>(q: QueueHandle_t) -> *mut T {
    let mut out: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut out as *mut *mut T) as *mut core::ffi::c_void,
            PORT_MAX_DELAY,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueReceive failed: {r}");
    out
}

extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("dsp_worker: started on core {}", current_core());
    loop {
        let job_ptr = unsafe { queue_recv_ptr::<Job>(JOB_Q.get()) };
        let job = unsafe { *Box::from_raw(job_ptr) };
        match job {
            Job::Pass2 {
                audio,
                audio_len,
                cands,
                max_cand,
            } => {
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
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(PASS2_RESULT_Q.get(), raw) };
            }
            Job::Stage3 {
                audio,
                audio_len,
                cands,
                depth,
            } => {
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
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(STAGE3_RESULT_Q.get(), raw) };
            }
            Job::CoarseSyncWithAllsum {
                spec,
                freq_min,
                freq_max,
                sync_min,
                max_cand,
                allsum_ptr,
                allsum_len,
            } => {
                let spec_ref = unsafe { &*spec };
                let allsum = unsafe { core::slice::from_raw_parts(allsum_ptr, allsum_len) };
                let result = mfsk_core::ft8::decode_block::coarse_sync_with_allsum(
                    spec_ref, freq_min, freq_max, sync_min, max_cand, allsum,
                );
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(COARSE_RESULT_Q.get(), raw) };
            }
        }
    }
}

fn current_core() -> i32 {
    unsafe { xTaskGetCoreID(ptr::null_mut()) }
}

/// Spawn the persistent worker task on APP_CPU and create dispatch
/// queues. Call once at startup, after `link_patches()`,
/// `EspLogger::initialize_default()`, and `esp_dsp_fft::prewarm()`.
pub fn init() {
    unsafe {
        JOB_Q.set(queue_create(core::mem::size_of::<*mut Job>()));
        PASS2_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<RefinedCandidate>>()));
        STAGE3_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<DecodeResult>>()));
        COARSE_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<SyncCandidate>>()));

        let r = xTaskCreatePinnedToCore(
            Some(worker_main),
            c"dsp_worker".as_ptr(),
            16384,
            ptr::null_mut(),
            5,
            ptr::null_mut(),
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

/// Run Stage 2 sequentially per frequency half on main. No worker
/// dispatch (Phase E2 race fallback path; rarely hit).
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

/// Phase E2: parallel coarse_sync with pre-built per-half allsums.
/// Worker computes the tail half on APP_CPU; main computes the head
/// half locally in parallel.
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

    let job = Box::new(Job::CoarseSyncWithAllsum {
        spec: spec as *const _,
        freq_min: mid,
        freq_max,
        sync_min,
        max_cand,
        allsum_ptr: allsum_tail.as_ptr(),
        allsum_len: allsum_tail.len(),
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local =
        coarse_sync_with_allsum(spec, freq_min, mid, sync_min, max_cand, allsum_head);

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<SyncCandidate>>(COARSE_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    local.truncate(max_cand);
    local
}

/// Pass 2 split across main + worker. Each half scored with its own
/// BASIS scratch; merged top-`max_cand` returned.
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

    let job = Box::new(Job::Pass2 {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        cands: tail,
        max_cand,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local = refine_candidates_into(audio, head, max_cand, basis_re_main, basis_im_main);

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<RefinedCandidate>>(PASS2_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// Stage 3 split across main + worker. Per-cand results are
/// independent, so concatenated.
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

    let job = Box::new(Job::Stage3 {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        cands: tail,
        depth,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

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

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<DecodeResult>>(STAGE3_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local
}
