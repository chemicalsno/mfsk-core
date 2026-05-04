//! Phase E stress test — incremental stage-1 spectrogram from chunked
//! audio. **Bin-side PoC, not wired into decode.** The goal is to
//! reproduce the production firmware contention pattern and surface
//! "ハマりポイント" before real I2S DMA brings them out:
//!
//! 1. APP_CPU multi-task contention (this worker + `dual_core` worker
//!    + main task — three hot tasks vying for two cores)
//! 2. Concurrent ring-style buffer read/write (wav_sim pushes new
//!    samples while this worker reads earlier ones for FFT input)
//! 3. Time-slice / chunk-boundary alignment (each FFT pair needs
//!    `(2j+3)·NSTEP` samples; chunks are 1 200-sample-aligned, FFT
//!    windows aren't)
//! 4. PSRAM bandwidth contention (audio writes + spec writes from this
//!    worker + dual_core worker spectrogram reads during decode)
//! 5. Completion barrier — by the time `wav_sim` sends "WAV complete"
//!    notify, has this worker finished all 92 pairs?
//!
//! ## What it doesn't do
//!
//! `Spectrogram::data` is private, so the per-pair FFT output here
//! lives in a throwaway PSRAM buffer that decode doesn't read. Decode
//! still calls `compute_spectrogram` as before. If this stress test
//! passes (no crashes, all 92 pairs computed before slot-end notify),
//! it tells us a real Phase E with `compute_spectrogram_partial` would
//! be safe to wire in.
//!
//! ## Constants replicated from `mfsk_core::ft8::decode_block`
//!
//! Hardcoded so the bin doesn't depend on private mfsk-core symbols.
//! NSPS / NMAX / NTONES are public; NSTEP / FP_SPEC_SHIFT / Hann are
//! re-derived. Verified against `compute_spectrogram`'s loop in
//! `mfsk-core/src/ft8/decode_block.rs:383` (post-Hann + 2-for-1 FFT).

use core::cell::UnsafeCell;
use core::ffi::c_void;
use core::ptr;
use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use esp_idf_svc::sys::{
    eNotifyAction_eIncrement, esp_timer_get_time, ulTaskGenericNotifyTake,
    xTaskCreatePinnedToCore, xTaskGenericNotify, TaskHandle_t,
};

use mfsk_core::core::fft::{Fft16, FftPlanner16};
use num_complex::Complex;

const PD_PASS: i32 = 1;
const PD_TRUE: i32 = 1;

// Replicated FT8 constants ----------------------------------------------------
const NSPS: usize = 1_920;
const NSTEP: usize = NSPS / 2; // 960
const NMAX: usize = 180_000;
const NTONES: usize = 8;
const NFFT_SPEC: usize = 4_096;
const FP_SPEC_SHIFT: u32 = 12;
const TONE_SPACING_HZ: f32 = 6.25;
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const N_TIME: usize = NMAX / NSTEP - 3; // 184
const N_PAIRS: usize = N_TIME / 2; // 92
const TARGET_PEAK: i32 = (NFFT_SPEC * 2) as i32; // 8 192

// Spec dims at the call's `max_freq_hz = 3000` ---------------------------------
fn n_freq_for(max_freq_hz: f32) -> usize {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    (((band_top_hz / df).ceil() as usize) + 1).min(NFFT_SPEC / 2)
}

// ── Phase-E2: incremental allsum ─────────────────────────────────────────────
// Mirrors the 16-bin sliding-window allsum that mfsk-core's
// `coarse_sync_inner` builds at the top of stage 2. By accumulating it
// here as new spec rows arrive (≈ 2 columns per FFT pair), we hide the
// 280-300 ms allsum precompute under the 15 s capture window.

/// coarse_sync band for the rx_wavsim pipeline. Matches the
/// `dual_core::coarse_sync_split(spec, 100.0, 3_000.0, ...)` call.
/// **Per-half precompute** to avoid f32 sliding-window drift across
/// the full band (~1k slide steps would drift by signal-scale at
/// busy bands; per-half from-scratch fi=0 init keeps drift bounded).
const ALLSUM_FREQ_MIN: f32 = 100.0;
const ALLSUM_FREQ_MAX: f32 = 3_000.0;
const ALLSUM_FREQ_MID: f32 = 0.5 * (ALLSUM_FREQ_MIN + ALLSUM_FREQ_MAX);
const ALLSUM_WIN: usize = 2 * NTONES; // 16 contiguous bins (FT8 NFFT_SPEC=4096)

/// Per-half carrier-bin range. Mirrors the formulas in mfsk-core's
/// `coarse_allsum_len` for `(freq_min..freq_max)`.
fn band_for(freq_min: f32, freq_max: f32, spec_n_freq: usize) -> (usize, usize, usize) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(spec_n_freq.saturating_sub(max_tone_off));
    (ia, ib, ib - ia + 1)
}

// State -----------------------------------------------------------------------

struct Inner {
    audio: Vec<i16>,           // 180 000 i16, PSRAM. Index = sample idx.
    spec: Vec<u16>,            // n_freq × n_time, PSRAM. Throwaway.
    /// Per-half allsums for the head [100, 1550] Hz and tail
    /// [1550, 3000] Hz sub-bands, built incrementally during capture.
    /// Each layout: `allsum[fi * N_TIME + m]`. Phase-E2 hands these
    /// to mfsk-core's `coarse_sync_with_allsum` at slot end so
    /// stage 2 skips its own precompute. **Per-half** (not full
    /// band sliced) so f32 sliding-window drift stays bounded — see
    /// `mfsk_core::ft8::decode_block::coarse_sync_split_with_allsum_busy_band`
    /// for the documented full-band-slice failure mode.
    allsum_head: Vec<f32>,
    allsum_tail: Vec<f32>,
    allsum_head_ia: usize,
    allsum_head_n_freq: usize,
    allsum_tail_ia: usize,
    allsum_tail_n_freq: usize,
    hann: [i16; NSPS],         // 3.7 KB stack-init then moved here
    fft_planner: Box<dyn FftPlanner16>,
    fft: Box<dyn Fft16>,
    fft_buf: Vec<Complex<i16>>, // NFFT_SPEC complex, PSRAM
    n_freq: usize,
    next_pair: usize,
    shift: u32,
    shift_locked: bool,
    peak_abs: i32,
    /// Total per-chunk wall-time spent in advance (debug). µs.
    inc_total_us: i64,
}

/// Cross-task shared state. Initialised once in `init()` from main,
/// then mutated by `push_chunk` (wav_sim task), `advance_pairs`
/// (APP_CPU worker), and read by `take_spec*` / `mark_slot_boundary` /
/// `last_slot_inc_us` (main task). Synchronisation comes from the
/// `xTaskNotify` handshake (memory barrier on Xtensa) plus the audio
/// fill / pair_done atomics — there's no overlapping mutation between
/// init and any consumer. Uses the same `UnsafeCell + unsafe Sync`
/// pattern as `dual_core::JOB_SLOT`.
struct StateCell {
    inner: UnsafeCell<Option<Inner>>,
}
/// SAFETY: see `STATE` doc comment.
unsafe impl Sync for StateCell {}
static STATE: StateCell = StateCell {
    inner: UnsafeCell::new(None),
};
static WORKER_TASK: AtomicPtr<c_void> = AtomicPtr::new(ptr::null_mut());

/// Number of pairs completed in the current slot. Reset to 0 by
/// `mark_slot_boundary()`. Used by main to verify Phase E feasibility:
/// if value == N_PAIRS at slot-end notify time, all 92 FFT pairs were
/// hidden under capture and a real Phase E implementation could skip
/// stage 1 entirely.
pub static PAIR_DONE: AtomicUsize = AtomicUsize::new(0);
/// Total samples written by `push_chunk`. Reset on slot boundary.
pub static AUDIO_FILL: AtomicUsize = AtomicUsize::new(0);

/// Spawn the incremental-stage-1 stress-test task on APP_CPU at low
/// priority (3) so it gets preempted by both the `dual_core` worker
/// (priority 5) during decode and `wav_sim` (priority 4) during push.
/// Time it actually gets is the slack between those two — exactly
/// the "free CPU during capture" budget Phase E would consume.
pub fn init() {
    let n_freq = n_freq_for(3_000.0);

    // PSRAM-resident buffers. esp-idf-svc's global allocator routes
    // 360 KB / 380 KB allocations to PSRAM under
    // CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 (we lowered it for the
    // dual_core fix; see project_decode_block_embedded.md).
    let audio: Vec<i16> = vec![0i16; NMAX];
    let spec: Vec<u16> = vec![0u16; n_freq * N_TIME];
    let fft_buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); NFFT_SPEC];
    let (allsum_head_ia, _, allsum_head_n_freq) =
        band_for(ALLSUM_FREQ_MIN, ALLSUM_FREQ_MID, n_freq);
    let (allsum_tail_ia, _, allsum_tail_n_freq) =
        band_for(ALLSUM_FREQ_MID, ALLSUM_FREQ_MAX, n_freq);
    let allsum_head: Vec<f32> = vec![0f32; allsum_head_n_freq * N_TIME];
    let allsum_tail: Vec<f32> = vec![0f32; allsum_tail_n_freq * N_TIME];

    // Hann window (q15). Replicates `hann_window_q15` in mfsk-core
    // (raised-cosine, 0…NSPS-1). Uses `f32::cos` from std (esp-idf-svc
    // pulls std in for Core2).
    let mut hann = [0i16; NSPS];
    for n in 0..NSPS {
        let phase = 2.0 * core::f32::consts::PI * (n as f32) / (NSPS as f32);
        let w = 0.5 - 0.5 * phase.cos();
        hann[n] = (w * 32767.0) as i16;
    }

    let mut fft_planner = mfsk_core::core::fft::default_planner_16();
    let fft = fft_planner.plan_forward(NFFT_SPEC);

    let inner = Inner {
        audio,
        spec,
        allsum_head,
        allsum_tail,
        allsum_head_ia,
        allsum_head_n_freq,
        allsum_tail_ia,
        allsum_tail_n_freq,
        hann,
        fft_planner,
        fft,
        fft_buf,
        n_freq,
        next_pair: 0,
        shift: 0,
        shift_locked: false,
        peak_abs: 1,
        inc_total_us: 0,
    };
    // SAFETY: init runs once before any task notification can fire.
    unsafe {
        *STATE.inner.get() = Some(inner);
    }

    let mut handle: TaskHandle_t = ptr::null_mut();
    unsafe {
        let r = xTaskCreatePinnedToCore(
            Some(worker_main),
            c"stage1_inc".as_ptr(),
            16384,
            ptr::null_mut(),
            3, // below dual_core worker (5) and wav_sim (4)
            &raw mut handle,
            1, // APP_CPU
        );
        assert_eq!(r, PD_PASS, "xTaskCreatePinnedToCore(stage1_inc) failed: {r}");
    }
    WORKER_TASK.store(handle as *mut c_void, Ordering::Release);
    log::info!(
        "stage1_inc: spawned (APP_CPU, prio 3); n_time={} n_pairs={} n_freq={}",
        N_TIME,
        N_PAIRS,
        n_freq
    );
}

/// Append a chunk to the bin's mirror audio buffer and wake the
/// stage1_inc worker. Called from `wav_sim` after each `push_i16` so
/// the incremental stage-1 sees the same chunk granularity an I2S
/// DMA-done IRQ would deliver in production.
pub fn push_chunk(samples: &[i16]) {
    // Update audio buffer + peak estimate (lock-free single producer).
    // SAFETY: only the wav_sim task calls push_chunk; the APP_CPU
    // worker mutates `next_pair` / `shift*` / `peak_abs` (via
    // `advance_pairs`) but never touches `audio[]`. The shift / peak
    // fields are mutated only after `audio_len >= 12_000` so init is
    // observed via the AUDIO_FILL Release store above.
    unsafe {
        let inner = (*STATE.inner.get())
            .as_mut()
            .expect("stage1_inc not init'd");
        let off = AUDIO_FILL.load(Ordering::Relaxed);
        if off + samples.len() <= NMAX {
            inner.audio[off..off + samples.len()].copy_from_slice(samples);
            // Track peak for the auto-gain shift.
            for &s in samples {
                let a = (s as i32).unsigned_abs() as i32;
                if a > inner.peak_abs {
                    inner.peak_abs = a;
                }
            }
            AUDIO_FILL.store(off + samples.len(), Ordering::Release);
        }
    }
    // Notify the worker.
    let task = WORKER_TASK.load(Ordering::Acquire);
    if !task.is_null() {
        unsafe {
            xTaskGenericNotify(
                task as TaskHandle_t,
                0,
                0,
                eNotifyAction_eIncrement,
                ptr::null_mut(),
            );
        }
    }
}

/// Reset incremental state for the next slot. Caller is the decode
/// task on slot-boundary notify after observing `PAIR_DONE`.
pub fn mark_slot_boundary() {
    PAIR_DONE.store(0, Ordering::Release);
    AUDIO_FILL.store(0, Ordering::Release);
    unsafe {
        if let Some(inner) = (*STATE.inner.get()).as_mut() {
            inner.next_pair = 0;
            inner.shift = 0;
            inner.shift_locked = false;
            inner.peak_abs = 1;
            inner.inc_total_us = 0;
        }
    }
}

/// Total wall-clock spent in `advance_pairs` over the current slot
/// (µs). Read by main task at slot end to log Phase E feasibility.
pub fn last_slot_inc_us() -> i64 {
    unsafe {
        (*STATE.inner.get())
            .as_ref()
            .map(|s| s.inc_total_us)
            .unwrap_or(0)
    }
}

/// Take the incrementally-built spec out of the worker for this slot.
/// Returns `Some(Spectrogram)` only if all `N_PAIRS` (= 92) pairs are
/// done — otherwise the caller should fall back to a full
/// `compute_spectrogram`. Internal buffer is replaced with a fresh
/// `vec![0u16; n_freq * n_time]` so the next slot starts clean.
pub fn take_spec() -> Option<mfsk_core::ft8::decode_block::Spectrogram> {
    let pairs = PAIR_DONE.load(Ordering::Acquire);
    if pairs < N_PAIRS {
        return None;
    }
    unsafe {
        let inner = (*STATE.inner.get())
            .as_mut()
            .expect("stage1_inc not init'd");
        let n_freq = inner.n_freq;
        // Swap out the prebuilt buffer.
        let mut fresh = vec![0u16; n_freq * N_TIME];
        core::mem::swap(&mut inner.spec, &mut fresh);
        Some(mfsk_core::ft8::decode_block::Spectrogram::from_parts(
            n_freq, N_TIME, fresh,
        ))
    }
}

/// Phase-E2: take both the incremental spec **and** the per-half
/// pre-built allsums for [100, 1550] (head) and [1550, 3000] (tail).
/// Returns `None` if not all pairs are done; otherwise returns
/// `(spec, allsum_head, allsum_tail)`. Each allsum has layout
/// `data[fi * N_TIME + m]` matching what
/// `mfsk_core::ft8::decode_block::precompute_coarse_allsum` produces
/// for its half's freq range — pass directly to
/// `dual_core::coarse_sync_split_with_allsum`.
pub fn take_spec_and_allsum() -> Option<(
    mfsk_core::ft8::decode_block::Spectrogram,
    Vec<f32>, // head
    Vec<f32>, // tail
)> {
    let pairs = PAIR_DONE.load(Ordering::Acquire);
    if pairs < N_PAIRS {
        return None;
    }
    unsafe {
        let inner = (*STATE.inner.get())
            .as_mut()
            .expect("stage1_inc not init'd");
        let n_freq = inner.n_freq;
        let head_n = inner.allsum_head_n_freq;
        let tail_n = inner.allsum_tail_n_freq;
        // Fresh buffers for the *next* slot. Zero-init for safety —
        // tried `Vec::with_capacity` + `set_len` (uninit) to skip the
        // ~100 ms PSRAM zero write but observed sporadic 0-result
        // failures on a re-loop slot. Keeping zero-init costs ~100
        // ms but is reliable (~50 ms net Phase-E2 saving over
        // baseline; the larger win is in row-major Spectrogram which
        // is independent).
        let mut fresh_spec: Vec<u16> = vec![0u16; n_freq * N_TIME];
        let mut fresh_head: Vec<f32> = vec![0f32; head_n * N_TIME];
        let mut fresh_tail: Vec<f32> = vec![0f32; tail_n * N_TIME];
        core::mem::swap(&mut inner.spec, &mut fresh_spec);
        core::mem::swap(&mut inner.allsum_head, &mut fresh_head);
        core::mem::swap(&mut inner.allsum_tail, &mut fresh_tail);
        Some((
            mfsk_core::ft8::decode_block::Spectrogram::from_parts(n_freq, N_TIME, fresh_spec),
            fresh_head,
            fresh_tail,
        ))
    }
}

extern "C" fn worker_main(_arg: *mut c_void) {
    log::info!("stage1_inc: worker entered");
    loop {
        unsafe {
            let _ = ulTaskGenericNotifyTake(0, PD_TRUE, u32::MAX);
        }
        advance_pairs();
    }
}

/// Compute every pair whose audio window is fully present in the
/// mirror buffer. Pair `j` needs samples `[2j·NSTEP, (2j+1)·NSTEP+NSPS)`
/// = `[1920j, 1920j + 2880)`.
fn advance_pairs() {
    let t0 = unsafe { esp_timer_get_time() };
    let audio_len = AUDIO_FILL.load(Ordering::Acquire);
    unsafe {
        let inner = (*STATE.inner.get())
            .as_mut()
            .expect("stage1_inc not init'd");

        // Lock auto-gain shift once we've seen ~1 s of audio.
        if !inner.shift_locked && audio_len >= 12_000 {
            let mut shift: u32 = 0;
            while inner.peak_abs << shift < TARGET_PEAK && shift < 8 {
                shift += 1;
            }
            inner.shift = (shift + 1).min(8);
            inner.shift_locked = true;
        }
        if !inner.shift_locked {
            return; // wait for more audio before any FFT
        }

        while inner.next_pair < N_PAIRS {
            let j = inner.next_pair;
            let j_a = 2 * j;
            let j_b = j_a + 1;
            let need = j_b * NSTEP + NSPS;
            if need > audio_len {
                break;
            }
            compute_pair_into(inner, j_a, j_b);
            inner.next_pair += 1;
            PAIR_DONE.store(inner.next_pair, Ordering::Release);
        }

        let t1 = esp_timer_get_time();
        inner.inc_total_us += t1 - t0;
    }
}

fn compute_pair_into(inner: &mut Inner, j_a: usize, j_b: usize) {
    let shift = inner.shift;
    // Pack audio[j_a*NSTEP..+NSPS] as re, audio[j_b*NSTEP..+NSPS] as im.
    let ia_a = j_a * NSTEP;
    let ia_b = j_b * NSTEP;
    for k in 0..NFFT_SPEC {
        let re = if k < NSPS && ia_a + k < inner.audio.len() {
            let raw = inner.audio[ia_a + k] as i32;
            let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
            ((scaled * inner.hann[k] as i32) >> 15) as i16
        } else {
            0
        };
        let im = if k < NSPS && ia_b + k < inner.audio.len() {
            let raw = inner.audio[ia_b + k] as i32;
            let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
            ((scaled * inner.hann[k] as i32) >> 15) as i16
        } else {
            0
        };
        inner.fft_buf[k] = Complex::new(re, im);
    }
    inner.fft.process(&mut inner.fft_buf);

    // Demux pair → spec rows. Mirrors decode_block.rs:482-497.
    let n_freq = inner.n_freq;
    let row_a = j_a * n_freq;
    let row_b = j_b * n_freq;
    let mask = NFFT_SPEC - 1;
    for k in 0..n_freq {
        let kn = (NFFT_SPEC - k) & mask;
        let yk_re = inner.fft_buf[k].re as i32;
        let yk_im = inner.fft_buf[k].im as i32;
        let yn_re = inner.fft_buf[kn].re as i32;
        let yn_im = inner.fft_buf[kn].im as i32;
        let a_re = (yk_re + yn_re) >> 1;
        let a_im = (yk_im - yn_im) >> 1;
        let b_re = (yk_im + yn_im) >> 1;
        let b_im = (yn_re - yk_re) >> 1;
        let mag2_a = ((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT;
        let mag2_b = ((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT;
        inner.spec[row_a + k] = mag2_a as u16;
        inner.spec[row_b + k] = mag2_b as u16;
    }

    // Phase-E2: now that spec rows m=j_a and m=j_b are filled, update
    // the corresponding allsum columns for both halves. Sliding
    // window across fi at fixed m within each half (each half does
    // its own from-scratch fi=0 init so f32 drift across the band
    // doesn't accumulate).
    update_allsum_columns_for_m(inner, j_a);
    update_allsum_columns_for_m(inner, j_b);
}

/// Update allsum cells at column `m` for both halves (head + tail).
#[inline]
fn update_allsum_columns_for_m(inner: &mut Inner, m: usize) {
    if m >= N_TIME {
        return;
    }
    update_one_half(
        m,
        inner.allsum_head_ia,
        inner.allsum_head_n_freq,
        inner.n_freq,
        &inner.spec,
        &mut inner.allsum_head,
    );
    update_one_half(
        m,
        inner.allsum_tail_ia,
        inner.allsum_tail_n_freq,
        inner.n_freq,
        &inner.spec,
        &mut inner.allsum_tail,
    );
}

/// 16-bin sliding-window sum at fixed `m`, fi=0..n_freq, into
/// `dst[fi * N_TIME + m]`. Mirrors mfsk-core `fill_coarse_allsum`
/// for one column.
#[inline]
fn update_one_half(
    m: usize,
    ia: usize,
    n_freq: usize,
    spec_n_freq: usize,
    spec: &[u16],
    dst: &mut [f32],
) {
    let win = ALLSUM_WIN;
    let mut s: f32 = 0.0;
    let row_base = m * spec_n_freq;
    for j in 0..win {
        let bin = (ia + j).min(spec_n_freq - 1);
        s += spec[row_base + bin] as f32;
    }
    dst[m] = s;
    for fi in 1..n_freq {
        let drop_bin = ia + fi - 1;
        let add_bin = (ia + fi + win - 1).min(spec_n_freq - 1);
        let drop_v = spec[row_base + drop_bin] as f32;
        let add_v = spec[row_base + add_bin] as f32;
        s = s - drop_v + add_v;
        dst[fi * N_TIME + m] = s;
    }
}
