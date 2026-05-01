//! M5Stack Core2 (ESP32-D0WD-V3, LX6 dual-core, 8 MB QUAD PSRAM,
//! 16 MB Flash) FT8 test bench.
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
// stripping the factory as dead code.
pub mod esp_dsp_fft;

use mfsk_core::fec::ldpc::bp::{bp_decode_kind, check_crc14, BpKind};
use mfsk_core::fec::ldpc::osd::{osd_decode, osd_decode_deep};
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::ft8::decode_block::{
    coarse_sync, compute_spectrogram, fill_symbol_spectra_into, symbol_spectra_direct_into,
    SymMask, BASIS_SCRATCH_LEN,
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

    log::info!("mfsk-core-m5stack-core2 PoC starting");
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

    // Runtime parameter sweep. v4: focus on the high-recall band
    // (max_cand ∈ {12, 20}) and vary the sync_quality gate
    // (`q > q_thresh`). i16 FFT precision loss may suppress the
    // q value of a real signal below 6 (the f32-calibrated
    // default); a looser gate at q > 3 should recover those.
    // 2 × 2 × 2 × 2 = 16 configs × 3 WAVs = 48 decodes.
    // Single config — sweet spot from earlier sweeps. max_cand=30
    // balances real-QSO recall (~67 % vs frame on host fp) against
    // Core2 time budget. Parabolic dt only (dt/df grids hurt recall
    // empirically). q_thresh=6 (q>3 same recall, just slower).
    const MAX_CAND_SWEEP: &[usize] = &[30];
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

    log::info!("\n=== Sweep complete. Idling. ===");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(60));
    }
}

/// Run the staged `decode_block` on one slot, log per-stage timings
/// and per-message SNR.
fn decode_one(slot: &[i16], max_cand: usize, dt_grid: u8, df_grid: u8, q_thresh: u32) {
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

    // Stage 2: Costas correlation.
    let t2 = now_us();
    let cands = coarse_sync(&spec, 100.0, 3_000.0, 1.0, max_cand);
    let t3 = now_us();
    log::info!(
        "  stage 2 (sync):       {:>8} us  ({} candidate(s))",
        t3 - t2,
        cands.len(),
    );
    drop(spec);

    // Stage 3: per-cand (dt, df) refinement + DFT + LLR + BP.
    let dt_offsets: alloc::vec::Vec<f32> = match dt_grid {
        3 => alloc::vec![-0.040, 0.0, 0.040],
        _ => alloc::vec![0.0],
    };
    let df_step = 12_000.0 / mfsk_core::ft8::decode_block::NFFT_SPEC as f32; // ≈ 1.46 Hz at NFFT=4096
    let f_offsets: alloc::vec::Vec<f32> = match df_grid {
        3 => alloc::vec![-0.25 * df_step, 0.0, 0.25 * df_step],
        _ => alloc::vec![0.0],
    };

    let t4 = now_us();
    let bp_kind = BpKind::NormalizedMinSum { alpha: 0.75 };
    let mut results: alloc::vec::Vec<DecodeResult> = alloc::vec::Vec::new();
    for cand in &cands {
        // Pick the best (df, dt) by sync_quality on Costas-only DFT.
        let mut best: Option<(
            alloc::boxed::Box<[[num_complex::Complex<f32>; 8]; 79]>,
            f32,
            f32,
            u32,
        )> = None;
        for &dt_off in &dt_offsets {
            for &f_off in &f_offsets {
                let f = cand.freq_hz + f_off;
                let dt = cand.dt_sec + dt_off;
                // SAFETY: single-threaded main task, scratch arrays
                // are only accessed here (no overlapping borrow).
                let cs = unsafe {
                    symbol_spectra_direct_into(
                        slot,
                        f,
                        dt,
                        SymMask::SyncOnly,
                        &mut BASIS_RE,
                        &mut BASIS_IM,
                    )
                };
                let q = sync_quality(&cs);
                if q <= q_thresh {
                    continue;
                }
                match &best {
                    Some((_, _, _, q_best)) if q <= *q_best => {}
                    _ => best = Some((cs, f, dt, q)),
                }
            }
        }
        let Some((mut cs, refined_f, refined_dt, q_best)) = best else {
            continue;
        };
        // SAFETY: single-threaded; scratch arrays only used here.
        unsafe {
            fill_symbol_spectra_into(
                &mut cs,
                slot,
                refined_f,
                refined_dt,
                SymMask::DataOnly,
                &mut BASIS_RE,
                &mut BASIS_IM,
            );
        }

        // BpAllOsd staircase:
        //  1) Bp(llra-fast)
        //  2) Bp on full LLR variants a/b/c/d (nsym=1+2+3)
        //  3) OSD-2 (sync_q≥12) / OSD-3 (sync_q≥18)
        let mut accepted = None;
        let mut accepted_pass: u8 = 0;
        let llr_a_fast = compute_llr_fast(&cs);
        if let Some(bp) = bp_decode_kind(&llr_a_fast.llra, None, 30, Some(check_crc14), bp_kind) {
            accepted = Some(bp.message77);
            accepted_pass = 0;
        }
        let mut hard_errors_acc: u32 = 0;
        if accepted.is_none() {
            let llr_full = compute_llr(&cs);
            let variants = [
                (&llr_full.llra, 0u8),
                (&llr_full.llrb, 1),
                (&llr_full.llrc, 2),
                (&llr_full.llrd, 3),
            ];
            for (llr, pid) in variants {
                if let Some(bp) = bp_decode_kind(llr, None, 30, Some(check_crc14), bp_kind) {
                    accepted = Some(bp.message77);
                    accepted_pass = pid;
                    hard_errors_acc = bp.hard_errors;
                    break;
                }
            }
            if OSD_ENABLED && accepted.is_none() && q_best >= 12 {
                let osd_variants = [
                    (&llr_full.llra, 4u8),
                    (&llr_full.llrb, 5),
                    (&llr_full.llrc, 6),
                    (&llr_full.llrd, 7),
                ];
                for (llr, pid) in osd_variants {
                    let osd = if q_best >= 18 {
                        osd_decode_deep(llr, 3, Some(check_crc14))
                    } else {
                        osd_decode(llr)
                    };
                    if let Some(osd) = osd {
                        accepted = Some(osd.message77);
                        accepted_pass = pid;
                        hard_errors_acc = osd.hard_errors;
                        break;
                    }
                }
            }
        }
        let Some(message77) = accepted else { continue };
        let Some(text) = unpack77(&message77) else {
            continue;
        };
        if !mfsk_core::msg::wsjt77::is_plausible_message(&text) {
            continue;
        }
        if results.iter().any(|r| r.message77 == message77) {
            continue;
        }
        let itone = message_to_tones(&message77);
        let snr_db = compute_snr_db(&cs, &itone);
        results.push(DecodeResult {
            message77,
            freq_hz: refined_f,
            dt_sec: refined_dt,
            hard_errors: hard_errors_acc,
            sync_score: cand.score,
            pass: accepted_pass,
            sync_cv: 0.0,
            snr_db,
        });
    }
    let _ = LDPC_N;
    let t5 = now_us();
    log::info!(
        "  stage 3 (refine+BP):  {:>8} us  ({} result(s))",
        t5 - t4,
        results.len(),
    );
    log::info!(
        "  ─── total decode:    {:>8} us = {:.3} s",
        t5 - t0,
        (t5 - t0) as f32 / 1_000_000.0,
    );

    // Per-message report (caller compares with host SNRs).
    for (i, r) in results.iter().enumerate() {
        let text = unpack77(&r.message77).unwrap_or_else(|| "<?>".into());
        log::info!(
            "    [{}] {:>5.0} Hz  SNR={:>+5.1} dB  e={}  '{}'",
            i,
            r.freq_hz,
            r.snr_db,
            r.hard_errors,
            text,
        );
    }
}
