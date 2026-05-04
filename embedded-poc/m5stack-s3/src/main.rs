//! M5Stack S3 (ESP32-S3, Xtensa LX7 dual-core @ 240 MHz, 8 MB Octal
//! PSRAM 想定) FT8 test bench. m5stack-core2 (LX6) クレートからの
//! 複製 — issue #15 Phase 2 baseline。タイミング数値は LX6 の実測値
//! のままなので S3 ベンチ後に更新する。
//!
//! Goal: prove `mfsk_core::ft8::decode_block` (the embedded
//! pow-of-2-FFT-only path) decodes a synthetic FT8 slot inside a
//! reasonable wall-clock on real silicon, paving the way for a live
//! transceiver app.
//!
//! What this binary does
//! ─────────────────────
//!  1. Synthesise a CQ message ("CQ JL1NIE PM95") at 1500 Hz into the
//!     standard 12 kHz / 180 000-sample slot, in PSRAM.
//!  2. Run `decode_block(...)` against that audio, with the
//!     extern-Rust `EspDspPlanner` (see `src/esp_dsp_fft.rs`) supplying
//!     all power-of-2 FFTs to the trait.
//!  3. Log: each per-stage timing (spectrogram, coarse sync,
//!     per-candidate refine + BP) and every recovered message.
//!
//! Decode budget on LX6 @ 240 MHz, NFFT_SPEC=8192:
//!   spectrogram  ~3.0 s   (372 × 8192-pt FFT, esp-dsp ASM)
//!   coarse_sync  ~0.2 s
//!   refinement   ~0.5 s   (5 dt offsets × 5 candidates × 20 ms DFT)
//!   BP (NMS)     ~0.25 s
//!   ─────────────────
//!   total        ~4.0 s
//!
//! ≈ 2 s over the 1.86 s in-slot decode window. Decode therefore
//! spills into the next slot's RX window — see plan doc for the
//! per-period scheduling tradeoff.

// `esp_dsp_fft` exports `mfsk_core_make_default_fft_planner` — the
// `extern "Rust"` factory `mfsk_core::core::fft::default_planner()`
// links against under `fft-extern`. `pub use` keeps the linker from
// stripping the factory as dead code. Re-exported from the shared
// crate so the linker still sees the symbol in this bin.
pub use embedded_shared::esp_dsp_fft;
use embedded_shared::dual_core;

use mfsk_core::fec::ldpc::bp::{bp_decode_kind, check_crc14, BpKind};
use mfsk_core::fec::ldpc::osd::{osd_decode, osd_decode_deep};
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::ft8::decode_block::{
    coarse_sync, compute_spectrogram, fill_symbol_spectra_into, symbol_spectra_direct_into,
    sync_quality_block0, SymMask, BASIS_SCRATCH_LEN,
};
use mfsk_core::ft8::llr::{compute_llr, compute_llr_fast, compute_snr_db, sync_quality};
use mfsk_core::ft8::params::LDPC_N;
use mfsk_core::ft8::wave_gen::message_to_tones;
use mfsk_core::msg::wsjt77::unpack77;

/// Q15 basis scratch for `fill_symbol_spectra_into`. Two flat arrays
/// (cos / sin × 8 tones × 1920 samples = 30 KB each, 60 KB total).
/// In `.bss` so they land in **internal DRAM** — the dot-product
/// inner loop reads basis hundreds of times, and PSRAM at 40 MHz
/// QUAD is 5–10× slower per access. Default-heap allocation routes
/// 60 KB blocks to PSRAM under `CONFIG_SPIRAM_USE_MALLOC` and
/// completely cancels the asm dot product's speed advantage.
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

/// Skip the OSD fallback (Bp staircase still runs all 4 LLR variants).
/// OSD recovers a few weak signals at the cost of producing phantom
/// CRC-passing garbage decodes (~2 per qso3 slot in our tests).
const OSD_ENABLED: bool = false;

/// Pass 1 (coarse_sync) candidate cap, BEFORE the manual Pass 2
/// `sync_quality_block0` re-rank. With the regularised
/// `t / (mean_others + ε)` ratio in mfsk-core, host real-QSO sweep
/// showed:
/// - PASS1=15 (Pass 2 effectively eliminated): qso3 2/13 truth ⚠️ —
///   strong-but-rank-16-30 signals (W1FC, WM3PEN, K1BZM, W1DIG)
///   never reach Pass 2 for the sync_quality re-rank that promotes
///   them. Pass 2 is **necessary**, not redundant.
/// - PASS1=30: qso3 6/13 (full ceiling), 14/22 total (loses qso1 -17 dB
///   OH3NIV).
/// - PASS1=75: 15/22 total (recovers OH3NIV at extra Pass 2 cost).
///
/// 30 ships as Core2 default — accepts the marginal -17 dB loss for
/// ~0.6 s saved on Pass 2 (linear in PASS1, per-cand block-0 DFT).
const PASS1_LIMIT: usize = 30;

/// Real on-air FT8 slots — 12 kHz / mono / 16-bit PCM. Each ≈ 360 KB.
/// Two consecutive slots from `jl1nie/rs-ft8n`'s benchmark data plus
/// one busy-band recording from WSJT-X.
const QSO_WAVS: &[(&str, &[u8])] = &[
    ("qso1 (191111_110130)", include_bytes!("../assets/qso1.wav")),
    ("qso2 (191111_110200)", include_bytes!("../assets/qso2.wav")),
    (
        "qso3 busy band (210703)",
        include_bytes!("../assets/qso3_busy.wav"),
    ),
];

extern crate alloc;

const SLOT_LEN: usize = 180_000; // full 15 s × 12 kHz

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Decode a baked WAV's i16 PCM samples into a fresh PSRAM slot.
/// Skips the standard 44-byte RIFF/WAVE header; bails if the WAV
/// is not 12 kHz mono i16 (asserted at compile time of the
/// recording, not at runtime).
fn load_wav_slot(wav: &[u8]) -> alloc::boxed::Box<[i16]> {
    // Standard PCM WAV: header is 44 bytes (RIFF 12 + fmt 24 + data 8).
    // The recordings we ship are all uniform format, so we trust the
    // offset.
    const HEADER: usize = 44;
    let payload = &wav[HEADER..];
    let n = payload.len() / 2;
    let mut slot: alloc::vec::Vec<i16> = alloc::vec![0i16; SLOT_LEN];
    let copy_n = n.min(SLOT_LEN);
    for i in 0..copy_n {
        slot[i] = i16::from_le_bytes([payload[i * 2], payload[i * 2 + 1]]);
    }
    slot.into_boxed_slice()
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("mfsk-core-m5stack-s3 PoC starting");
    log::info!("mfsk-core version: {}", mfsk_core::VERSION);

    // ── Free heap (DRAM vs PSRAM) so we know the budget ──────────────
    const MALLOC_CAP_INTERNAL: u32 = 1 << 11;
    const MALLOC_CAP_8BIT: u32 = 1 << 2;
    const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
    unsafe {
        log::info!(
            "free heap: internal = {} KB (largest contig {} KB), PSRAM = {} KB",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            ) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
        );
    }

    log::info!(
        "decode_block: NFFT_SPEC={}, NMS α=0.75, Bp, parabolic-dt, Costas-first",
        mfsk_core::ft8::decode_block::NFFT_SPEC,
    );
    log::info!("baked WAVs: {}", QSO_WAVS.len());

    // Pre-warm esp-dsp FFT twiddle tables (f32 + i16) at NFFT_SPEC up
    // front. mfsk-core builds a fresh `EspDspPlanner` per decode_block
    // call via `default_planner()`; without pre-warm each call would
    // re-call `dsps_fft2r_init_*` (idempotent at runtime, but a race
    // hazard once we move to dual-core workers). After this point the
    // global twiddle tables are read-only.
    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);
    log::info!("FFT twiddle tables pre-warmed");

    // Spawn the persistent dual-core worker (pinned to APP_CPU).
    // Pass 2 / Stage 3 candidate halves are dispatched via FreeRTOS
    // task notifications; see `dual_core.rs`.
    dual_core::init();

    // max_cand sweep — host PASS1×max_cand sweep at PASS1=75 showed
    // identical 15/22 truth recall for max_cand ∈ {15, 20, 30}; only
    // the time changes (stage 3 work is per-cand). 15 is the floor
    // for full host recall; 20 leaves slack for the i16 path's slight
    // SNR loss vs host f32; 30 is the previous default (kept for
    // direct A/B). On Core2 the difference shows up as stage 3 cost:
    // ~0.14 s/cand × Δmax_cand. Parabolic dt only (dt/df grids
    // empirically hurt recall on busy bands).
    const MAX_CAND_SWEEP: &[usize] = &[15, 20, 30];
    const DT_GRID_SWEEP: &[u8] = &[0];
    const DF_GRID_SWEEP: &[u8] = &[0];
    // q>12 (instead of q>6): skips compute_llr (full) for borderline
    // cands. Combined with OSD off (see `OSD_ENABLED` at module scope)
    // this trades weak-signal recall (~-1 truth typical) for ~30-40%
    // faster stage 3 and zero phantom CRC-passing decodes.
    const Q_THRESH_SWEEP: &[u32] = &[12];

    // Pre-load slots once into PSRAM. Loading is ~170 ms each, no
    // need to repeat per sweep config.
    let slots: alloc::vec::Vec<(&str, alloc::boxed::Box<[i16]>)> = QSO_WAVS
        .iter()
        .map(|(label, wav)| (*label, load_wav_slot(wav)))
        .collect();

    for &q_thresh in Q_THRESH_SWEEP {
        for &max_cand in MAX_CAND_SWEEP {
            for &dt_grid in DT_GRID_SWEEP {
                for &df_grid in DF_GRID_SWEEP {
                    log::info!("\n════════════════════════════════════════════");
                    log::info!(
                        "RUN: max_cand={max_cand}  dt_grid={dt_grid}  df_grid={df_grid}  q>{q_thresh}"
                    );
                    log::info!("════════════════════════════════════════════");
                    for (label, slot) in &slots {
                        log::info!("\nWAV: {label}");
                        decode_one(slot, max_cand, dt_grid, df_grid, q_thresh);
                    }
                }
            }
        }
    }

    // ── FFI smoke check ──────────────────────────────────────────────
    // Verify that the `mfsk-ffi-ft8` C ABI works on Xtensa: same
    // decoder, called once per WAV through `mfsk_ft8_decode_i16`
    // instead of the manually-staged Rust API. The same
    // `mfsk_core_make_default_fft_planner_*` and `mfsk_core_dot_q15_i32`
    // extern Rust symbols (provided by `esp_dsp_fft.rs`) resolve
    // both code paths from one .a — so success here means a pure-C
    // ESP-IDF project linking `libmfsk_ft8.a` would see the same
    // decode results.
    log::info!("\n════════════════════════════════════════════");
    log::info!("FFI smoke: mfsk_ft8_decode_i16 (C ABI) on each WAV");
    log::info!("════════════════════════════════════════════");
    for (label, slot) in &slots {
        log::info!("\nWAV: {label} (via FFI)");
        ffi_smoke_one(slot);
    }

    log::info!("\n=== Sweep complete. Idling. ===");
    // `std::thread::sleep` on esp-idf goes through pthread/condvar
    // shims that pushed main task past the 16 KB stack canary right
    // after a full sweep (`A stack overflow in task main` →
    // SW_CPU_RESET → infinite re-flash-the-bench loop). Direct
    // `vTaskDelay(portMAX_DELAY)` is a single syscall with no Rust
    // stack growth.
    loop {
        unsafe {
            esp_idf_svc::sys::vTaskDelay(u32::MAX);
        }
    }
}

/// FFI smoke: call `mfsk_ft8_decode_i16` (C ABI) on a slot and
/// print results via `log::info!`. Verifies the `libmfsk_ft8.a`
/// surface end-to-end on Xtensa — the same decoder a pure-C
/// ESP-IDF project linking the .a would see.
fn ffi_smoke_one(slot: &[i16]) {
    use mfsk_ft8::{
        MfskFt8Depth, MfskFt8ResultList, mfsk_ft8_decode_i16, mfsk_ft8_result_list_free,
    };
    let mut results = MfskFt8ResultList {
        items: core::ptr::null_mut(),
        len: 0,
        _capacity: 0,
    };
    let t0 = now_us();
    // SAFETY: slot is a valid &[i16] for its lifetime; we own
    // `results` for the duration of this call and free it before
    // return. BASIS_RE / BASIS_IM are .bss-resident static i16
    // arrays of `BASIS_SCRATCH_LEN` elements — the same scratch the
    // direct-Rust path uses, so the FFI call lands in fast internal
    // RAM exactly like `process_candidates_into`.
    let st = unsafe {
        mfsk_ft8_decode_i16(
            slot.as_ptr(),
            slot.len(),
            100.0,
            3_000.0,
            1.0,
            30,
            MfskFt8Depth::BpAll,
            BASIS_RE.as_mut_ptr(),
            BASIS_IM.as_mut_ptr(),
            &mut results,
        )
    };
    let t1 = now_us();
    log::info!(
        "  ffi status={:?}  {:>3} result(s)  {:>8} us",
        st,
        results.len,
        t1 - t0,
    );
    // SAFETY: results was just populated by the FFI call.
    if !results.items.is_null() {
        let items = unsafe { core::slice::from_raw_parts(results.items, results.len) };
        for (i, r) in items.iter().enumerate() {
            // r.text is NUL-terminated UTF-8; find the NUL.
            let bytes: &[u8] = unsafe {
                core::slice::from_raw_parts(r.text.as_ptr() as *const u8, r.text.len())
            };
            let n = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
            let text = core::str::from_utf8(&bytes[..n]).unwrap_or("<bad utf8>");
            log::info!(
                "    [{i}]  {:>4.0} Hz  SNR={:>5.1} dB  e={}  '{}'",
                r.freq_hz,
                r.snr_db,
                r.hard_errors,
                text,
            );
        }
    }
    // SAFETY: results was populated by the FFI call; freeing returns
    // the underlying Box to the heap.
    unsafe { mfsk_ft8_result_list_free(&mut results) };
}

/// Run the staged `decode_block` on one slot, log per-stage timings
/// and per-message SNR.
///
/// Calls into the manually-staged `mfsk-core` API
/// (`refine_candidates_into` / `process_candidates_into`) so the
/// `BASIS_RE` / `BASIS_IM` static internal-RAM scratch is reused
/// across all DFTs — without that, esp-dsp's asm dot product
/// fall back on PSRAM-resident basis (5-10× slower per access).
fn decode_one(slot: &[i16], max_cand: usize, _dt_grid: u8, _df_grid: u8, _q_thresh: u32) {
    // Stage 1: spectrogram.
    let t0 = now_us();
    let spec = compute_spectrogram(slot, 3_000.0);
    let t1 = now_us();
    log::info!(
        "  stage 1 (spec):       {:>8} us  ({}× FFT, {} freq bins)",
        t1 - t0,
        spec.n_time,
        spec.n_freq,
    );

    // Stage 2: Costas correlation, split across both cores by freq
    // half (worker handles upper half ≥ 1550 Hz, main handles lower).
    // See `dual_core::coarse_sync_split`.
    let t2 = now_us();
    let pass1 = dual_core::coarse_sync_split(&spec, 100.0, 3_000.0, 1.0, PASS1_LIMIT);
    let t3 = now_us();
    log::info!(
        "  stage 2 (sync):       {:>8} us  ({} cand)",
        t3 - t2,
        pass1.len(),
    );
    drop(spec);

    // Pass 2 — re-rank by sync_quality_block0, split across cores.
    // Main runs the first half locally with BASIS_RE/IM (PRO_CPU),
    // worker runs the second half with its own BASIS scratch on
    // APP_CPU. Results are merged + globally sorted by q_block0 and
    // truncated to `max_cand`.
    let t_pass2 = now_us();
    // SAFETY: BASIS_RE / BASIS_IM are only accessed by the main task
    // (PRO_CPU); the worker uses its own scratch.
    #[allow(static_mut_refs)]
    let pass2 = unsafe {
        dual_core::pass2_split(slot, pass1, max_cand, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t_pass2_end = now_us();
    log::info!(
        "  pass 2 (re-rank):     {:>8} us  → top {} by sync_quality_block0",
        t_pass2_end - t_pass2,
        pass2.len(),
    );

    // Stage 3 — fill data symbols + BP staircase. `DecodeDepth::BpAll`
    // matches the OSD_ENABLED=false production setting; the
    // `MFSK_Q_THRESH` env var (default 12) controls the
    // sync_quality early-reject inside process_candidates_into.
    let depth = if OSD_ENABLED {
        mfsk_core::ft8::decode::DecodeDepth::BpAllOsd
    } else {
        mfsk_core::ft8::decode::DecodeDepth::BpAll
    };
    let t4 = now_us();
    #[allow(static_mut_refs)]
    let results = unsafe {
        dual_core::stage3_split(slot, pass2, depth, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t5 = now_us();
    log::info!(
        "  stage 3 (refine+BP):  {:>8} us  ({} result(s))",
        t5 - t4,
        results.len(),
    );
    log::info!(
        "  ─── total decode:    {:>8} us = {:.3} s",
        t5 - t0,
        (t5 - t0) as f64 / 1e6,
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

