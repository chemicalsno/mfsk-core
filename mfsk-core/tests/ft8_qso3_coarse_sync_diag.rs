// SPDX-License-Identifier: GPL-3.0-or-later
//! Diagnostic harness for issue #40 (host wide-band coarse-sync candidate
//! gap on `qso3_busy.wav`).
//!
//! Runs three pipelines on the same WAV and prints a side-by-side
//! comparison so the divergence between host and embedded paths is
//! visible:
//!
//! 1. **Host `coarse_sync`** — `mfsk_core::ft8::sync::coarse_sync`,
//!    invoked with the same `(sync_min=1.3, max_cand=50)` parameters
//!    `decode_frame_with_ap` uses in `ft8_qso3_apon_recall`. Then a
//!    second pass with the looser `(sync_min=0.5, max_cand=200)` the
//!    block diagnostic uses, so we can see whether the host even has
//!    the candidates at any threshold.
//! 2. **Embedded `decode_block::coarse_sync`** — same
//!    `(sync_min=0.5, max_cand=200)` as `ft8_decode_block_coarse_diag`.
//! 3. **Per-stage gate trace** — for each candidate the host emits
//!    that lives near one of the JTDX-confirmed extras at
//!    1196 / 244 / 472 / 2039 Hz, walk the same stages
//!    `decode::process_candidate` does (refine_fine_3stage, then
//!    `nsync_quality <= 6`) and report which one would have rejected
//!    it. If the host doesn't even emit a candidate near a target
//!    freq, that's reported as MISSING from coarse_sync.
//!
//! This test does **not** assert anything about coverage — it's a
//! diagnostic the human-in-the-loop reads while bisecting #40.
//! The `JTDX_EXTRAS_HARD_FLOOR` regression seam still lives in
//! `ft8_qso3_apon_recall.rs`.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_coarse_sync_diag -- --nocapture --include-ignored
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::ft8::decode::{ApHint, DecodeDepth, decode_frame_with_ap};
use mfsk_core::ft8::decode_block::{coarse_sync as block_coarse_sync, compute_spectrogram};
use mfsk_core::ft8::downsample::{build_fft_cache, downsample};
use mfsk_core::ft8::llr::{symbol_spectra, sync_quality};
use mfsk_core::ft8::refine_fine::fine_refine_3stage;
use mfsk_core::ft8::sync::coarse_sync as host_coarse_sync;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Operator context — kept identical to `ft8_qso3_apon_recall.rs` so
/// the AP-on column reflects the same pipeline state #31's regression
/// test runs in.
const MYCALL: &str = "K1JT";
const HISCALL: &str = "HA0DU";

/// JTDX-confirmed extras whose coarse-sync candidate the host
/// pipeline currently misses. Source: ROADMAP.md A0 + JTDX FT8-deep
/// capture 2026-05-08. Frequencies are JTDX-reported center freqs.
///
/// The 1196 / 244 / 472 / 2039 Hz set is the explicit ROADMAP A0
/// list. The two extra entries (the 6/6 set) cover the full JTDX
/// AP-on extras list — `JTDX_AP_ON_EXTRAS` in the apon test — so we
/// see all of them at once. The exact numerical entries here are
/// **diagnostic targets**, not assertions; ROADMAP names "1196 /
/// 244 / 472 / 2039 Hz" as the four representatives but the actual
/// JTDX list is six rows long.
struct Target {
    label: &'static str,
    freq_hz: f32,
}
const TARGETS: &[Target] = &[
    Target {
        label: "CQ F5RXL IN94 / blind-CQ",
        freq_hz: 1196.0,
    },
    Target {
        label: "CQ EA2BFM IN83 / blind-CQ",
        freq_hz: 244.0,
    },
    Target {
        label: "K1JT HA5WA 73 / op-context",
        freq_hz: 472.0,
    },
    Target {
        label: "extra-2039 / op-context",
        freq_hz: 2039.0,
    },
];

/// Tolerance (Hz) for matching a coarse-sync candidate to a target.
/// Matches `ft8_decode_block_coarse_diag.rs`'s 6 Hz window.
const FREQ_TOL_HZ: f32 = 6.0;
/// Tolerance (s) on dt for the same matcher.
const DT_TOL_S: f32 = 0.5;

fn load_wav_i16(path: &Path) -> Vec<i16> {
    // Same RIFF chunk-walking loader as `ft8_qso3_apon_recall.rs`.
    let bytes = std::fs::read(path).expect("WAV present");
    assert_eq!(&bytes[0..4], b"RIFF", "not a RIFF/WAV file");
    let mut i = 12usize;
    let (mut data_off, mut data_len) = (0usize, 0usize);
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len;
        }
        i += len;
        if len % 2 == 1 {
            i += 1;
        }
    }
    assert!(data_off > 0, "no data chunk in WAV");
    bytes[data_off..data_off + data_len.min(bytes.len() - data_off)]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

/// Outcome of running the host per-candidate gate chain that
/// `decode::process_candidate` runs after coarse_sync. Mirrors the
/// stages literally — any change to `process_candidate` that adds
/// gates above the BP step needs a parallel update here.
#[derive(Debug)]
enum GateOutcome {
    /// `fine_refine_3stage` reported a frequency shift large enough
    /// that the candidate falls outside `process_candidate`'s
    /// implicit ±0.5 Hz refine envelope. (The host code doesn't
    /// reject explicitly — it shifts in place — but a large shift
    /// here is suggestive of the WSJT-X-faithful birdie filter
    /// re-snapping the candidate to a different bin.)
    RefineFineLargeShift { delf_hz: f32, score: f32 },
    /// `sync_quality(symbol_spectra)` <= 6 — the
    /// `decode.rs:493` early-return.
    NsyncBelowThreshold {
        nsync: u32,
        refined_freq_hz: f32,
        refined_dt_sec: f32,
    },
    /// All gates passed; the candidate would reach LLR / BP. (We
    /// don't run BP here — that's covered by the apon recall test.)
    PassesAllGates {
        nsync: u32,
        refined_freq_hz: f32,
        refined_dt_sec: f32,
    },
}

/// Replicates the prefix of `decode::process_candidate` up to the
/// `nsync <= 6` early return. We can't call `process_candidate`
/// directly (it's `fn`, not `pub`), but every helper it uses is
/// already public (`downsample`, `fine_refine_3stage`,
/// `symbol_spectra`, `sync_quality`).
fn run_gates(
    audio: &[i16],
    fft_cache: &[num_complex::Complex<f32>],
    cand_freq_hz: f32,
    cand_dt_sec: f32,
) -> GateOutcome {
    let (mut cd0, _) = downsample(audio, cand_freq_hz, Some(fft_cache));

    let r = fine_refine_3stage(&cd0, cand_dt_sec);
    let refined_freq = cand_freq_hz + r.delf_hz;
    let refined_dt = r.dt_sec;

    // Apply freq shift in place (mirrors decode.rs:476-484).
    if r.delf_hz.abs() > f32::EPSILON {
        let dt2 = 1.0_f32 / 200.0;
        for (k, c) in cd0.iter_mut().enumerate() {
            let phi = -core::f32::consts::TAU * r.delf_hz * (k as f32) * dt2;
            let rot = num_complex::Complex::new(phi.cos(), phi.sin());
            *c *= rot;
        }
    }

    // ROADMAP A0 names refine_fine as a suspect "killing real
    // signals above 2 kHz". The host doesn't reject on |delf| per
    // se, but a large shift means the candidate's claimed freq was
    // wrong — and `cand.freq_hz` (initial, NOT refined) is what
    // `process_candidate` later returns as `DecodeResult.freq_hz`.
    // Flag a >1.5 Hz shift so the diag highlights candidates that
    // refine_fine yanked off-bin.
    if r.delf_hz.abs() > 1.5 {
        return GateOutcome::RefineFineLargeShift {
            delf_hz: r.delf_hz,
            score: r.score,
        };
    }

    let i_start = ((refined_dt + 0.5) * 200.0).round() as usize;
    let cs_raw = symbol_spectra(&cd0, i_start);
    let nsync = sync_quality(&cs_raw);
    if nsync <= 6 {
        return GateOutcome::NsyncBelowThreshold {
            nsync,
            refined_freq_hz: refined_freq,
            refined_dt_sec: refined_dt,
        };
    }
    GateOutcome::PassesAllGates {
        nsync,
        refined_freq_hz: refined_freq,
        refined_dt_sec: refined_dt,
    }
}

#[derive(Debug)]
#[allow(dead_code)] // fields are surfaced via Debug-print in the diagnostic output
struct CandRef {
    rank: usize,
    freq_hz: f32,
    dt_sec: f32,
    score: f32,
    df_to_target: f32,
}

fn closest_to_target(cands: &[(usize, f32, f32, f32)], target_hz: f32) -> Option<CandRef> {
    let mut best: Option<CandRef> = None;
    for c in cands {
        let (rank, freq, dt, score) = *c;
        let df = (freq - target_hz).abs();
        if df <= FREQ_TOL_HZ && dt.abs() <= DT_TOL_S && best.as_ref().map(|b| df < b.df_to_target).unwrap_or(true) {
            best = Some(CandRef {
                rank,
                freq_hz: freq,
                dt_sec: dt,
                score,
                df_to_target: df,
            });
        }
    }
    best
}

#[test]
#[ignore = "diagnostic — run with --include-ignored --nocapture"]
fn ft8_qso3_coarse_sync_diag() {
    let path = Path::new(QSO3_PATH);
    if !path.exists() {
        // Match the soft-skip convention used by
        // `ft8_decode_block_coarse_diag.rs` so this still runs in
        // contributor environments without the WAV vendored in.
        println!("(skip {} — file not present)", path.display());
        return;
    }
    let slot = load_wav_i16(path);

    println!(
        "\n=== qso3_busy coarse_sync side-by-side diagnostic (issue #40) ===\n\
         WAV: {} samples ({:.2} s @ 12 kHz)\n",
        slot.len(),
        slot.len() as f32 / 12_000.0,
    );

    // ── Host coarse_sync, AP-on settings (matches apon test) ───────
    let host_apon = host_coarse_sync(&slot, 100.0, 3000.0, 1.3, None, 50);
    // ── Host coarse_sync, loose (matches block diag) ──────────────
    let host_loose = host_coarse_sync(&slot, 100.0, 3000.0, 0.5, None, 200);
    // ── Embedded coarse_sync (matches block diag) ─────────────────
    let spec = compute_spectrogram(&slot, 3000.0);
    let block_loose = block_coarse_sync(&spec, 100.0, 3000.0, 0.5, 200);

    println!(
        "host coarse_sync (sync_min=1.3, max_cand=50)  → {:>3} candidates",
        host_apon.len()
    );
    println!(
        "host coarse_sync (sync_min=0.5, max_cand=200) → {:>3} candidates",
        host_loose.len()
    );
    println!(
        "block coarse_sync (sync_min=0.5, max_cand=200) → {:>3} candidates",
        block_loose.len()
    );
    println!();

    // Re-shape so we can run them through the closest-to-target
    // helper uniformly.  Tuple = (rank, freq_hz, dt_sec, score).
    let host_apon_v: Vec<(usize, f32, f32, f32)> = host_apon
        .iter()
        .enumerate()
        .map(|(i, c)| (i, c.freq_hz, c.dt_sec, c.score))
        .collect();
    let host_loose_v: Vec<(usize, f32, f32, f32)> = host_loose
        .iter()
        .enumerate()
        .map(|(i, c)| (i, c.freq_hz, c.dt_sec, c.score))
        .collect();
    let block_loose_v: Vec<(usize, f32, f32, f32)> = block_loose
        .iter()
        .enumerate()
        .map(|(i, c)| (i, c.freq_hz, c.dt_sec, c.score))
        .collect();

    // FFT cache for the host gate-trace runs (downsample requires it).
    let fft_cache = build_fft_cache(&slot);

    println!(
        "── per-target match: closest candidate within ±{:.1} Hz, ±{:.2} s ──",
        FREQ_TOL_HZ, DT_TOL_S,
    );
    println!(
        "  {:<32} | {:>6} | {:^28} | {:^28} | {:^28}",
        "target", "freq", "host_apon (1.3/50)", "host_loose (0.5/200)", "block_loose (0.5/200)",
    );
    println!("  {}", "─".repeat(140));
    for tgt in TARGETS {
        let h_apon = closest_to_target(&host_apon_v, tgt.freq_hz);
        let h_loose = closest_to_target(&host_loose_v, tgt.freq_hz);
        let b_loose = closest_to_target(&block_loose_v, tgt.freq_hz);

        let fmt = |c: &Option<CandRef>| -> String {
            match c {
                Some(c) => format!(
                    "r{:>3} df={:+.1} s={:>4.0}",
                    c.rank + 1,
                    c.freq_hz - tgt.freq_hz,
                    c.score
                ),
                None => "       — MISSING —      ".to_string(),
            }
        };
        println!(
            "  {:<32} | {:>5.0}  | {:<28} | {:<28} | {:<28}",
            tgt.label,
            tgt.freq_hz,
            fmt(&h_apon),
            fmt(&h_loose),
            fmt(&b_loose),
        );
    }
    println!();

    // ── Per-target gate-trace on the LOOSE host candidates ────────
    // The AP-on settings (sync_min=1.3) trim the host pool too
    // aggressively to bisect the post-coarse stages, so we run the
    // gate trace against the larger 200-cand loose pool. This
    // reflects ROADMAP A0's "is the host bailing on candidates the
    // embedded path keeps?" question literally — the embedded path
    // uses sync_min=0.5.
    println!("── host-gate trace (sync_min=0.5, max_cand=200): which gate kills each target? ──");
    println!("  {:<32} | {:>6} | result", "target", "freq",);
    println!("  {}", "─".repeat(140));
    for tgt in TARGETS {
        let near: Vec<(usize, &mfsk_core::ft8::sync::SyncCandidate)> = host_loose
            .iter()
            .enumerate()
            .filter(|(_, c)| (c.freq_hz - tgt.freq_hz).abs() <= FREQ_TOL_HZ)
            .collect();
        if near.is_empty() {
            println!(
                "  {:<32} | {:>5.0}  | host coarse_sync emitted NO candidate within ±{:.1} Hz \
                 → root cause = stage 1 (coarse_sync) on this candidate",
                tgt.label, tgt.freq_hz, FREQ_TOL_HZ,
            );
            continue;
        }
        // Run gate-trace on every nearby candidate so we can see if
        // refine_fine pulls any of them onto/off the target.
        for (rank, c) in near.iter().take(5) {
            let outcome = run_gates(&slot, &fft_cache, c.freq_hz, c.dt_sec);
            let summary = match outcome {
                GateOutcome::RefineFineLargeShift { delf_hz, score } => format!(
                    "refine_fine large shift Δf={:+.2} Hz (final score={:.0}) — \
                     suspect: WSJT-X 3-stage filter re-snapping; \
                     `decode.rs:464–486`",
                    delf_hz, score,
                ),
                GateOutcome::NsyncBelowThreshold {
                    nsync,
                    refined_freq_hz,
                    refined_dt_sec,
                } => format!(
                    "nsync={} ≤ 6 → `decode.rs:493` early-return \
                     (refined: {:.1} Hz, dt={:+.2} s)",
                    nsync, refined_freq_hz, refined_dt_sec,
                ),
                GateOutcome::PassesAllGates {
                    nsync,
                    refined_freq_hz,
                    refined_dt_sec,
                } => format!(
                    "PASSES all pre-LLR gates (nsync={}, refined: {:.1} Hz, \
                     dt={:+.2} s) → failure must be downstream (LLR / BP / OSD / CRC)",
                    nsync, refined_freq_hz, refined_dt_sec,
                ),
            };
            println!(
                "  {:<32} | {:>5.0}  | rank {:>3} cand@{:>5.1} Hz dt={:+.2} s s={:>4.0} → {}",
                tgt.label,
                tgt.freq_hz,
                rank + 1,
                c.freq_hz,
                c.dt_sec,
                c.score,
                summary,
            );
        }
    }
    println!();

    // ── For context: the AP-on decode set the apon test would emit ─
    // If a target shows "PASSES all pre-LLR gates" above but is
    // missing from this set, the failure is downstream of nsync —
    // most likely BP/OSD/CRC failing to converge despite refined
    // sync. (AP-on rescue would normally pick those up; if it
    // doesn't, that's the iaptype-1 reach issue, not #40.)
    let ap = ApHint::new().with_call1(MYCALL).with_call2(HISCALL);
    let ap_on: Vec<String> = decode_frame_with_ap(
        &slot,
        100.0,
        3000.0,
        1.3,
        None,
        DecodeDepth::BpAllOsd,
        50,
        Some(&ap),
    )
    .into_iter()
    .filter_map(|r| {
        unpack77(&r.message77)
            .map(|m| format!("{:>5.0} Hz  dt={:+.2}s  {}", r.freq_hz, r.dt_sec, m))
    })
    .collect();
    println!(
        "── AP-on decode set (mycall={}, hiscall={}, sync_min=1.3, max_cand=50) — {} decode(s) ──",
        MYCALL,
        HISCALL,
        ap_on.len()
    );
    for line in &ap_on {
        println!("  {}", line);
    }
    println!();
}
