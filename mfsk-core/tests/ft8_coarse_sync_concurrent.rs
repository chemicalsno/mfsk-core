//! Host-side reproducer for the embedded Phase E2 dispatch race.
//!
//! On M5StickS3, dispatching `coarse_sync_with_allsum`'s tail half to
//! the APP_CPU worker corrupts recall on the rx_wavsim 2nd-and-later
//! slots (slot 1 OK, slots 2+ produce 0 candidate matches downstream).
//! If the bug is in `coarse_sync_with_allsum` itself (not thread-safe
//! between cores), running head/tail on two `std::thread`s here will
//! reproduce the divergence vs sequential.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8,fixed-point,fixed-point-llr,profile-coarse \
//!     --test ft8_coarse_sync_concurrent -- --include-ignored --nocapture
//! ```
//!
//! Expected if bug is in mfsk-core: parallel result ≠ sequential.
//! Expected if bug is embedded-specific: parallel result == sequential.
#![cfg(feature = "fixed-point")]

use std::path::Path;
use std::thread;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    BASIS_SCRATCH_LEN, coarse_sync_with_allsum, compute_spectrogram, precompute_coarse_allsum,
    process_candidates_into, refine_candidates_into,
};

const QSO_WAV: &str = "/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3/assets/qso3_busy.wav";
const QSO_WAVS: &[&str] = &[
    "/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3/assets/qso1.wav",
    "/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3/assets/qso2.wav",
    "/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3/assets/qso3_busy.wav",
];
const PASS1_LIMIT: usize = 30;
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const FREQ_MID: f32 = 1_550.0;
const SYNC_MIN: f32 = 1.0;

fn load_wav_i16(path: &Path) -> Vec<i16> {
    let bytes = std::fs::read(path).expect("read wav");
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"WAVE");
    let mut i = 12usize;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len =
            u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]]) as usize;
        if id == b"data" {
            data_off = i + 8;
            data_len = len;
            break;
        }
        i += 8 + len;
    }
    bytes[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn cands_eq(a: &[SyncCandidate], b: &[SyncCandidate]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b.iter()).all(|(x, y)| {
        x.freq_hz == y.freq_hz && x.dt_sec == y.dt_sec && (x.score - y.score).abs() < 1e-6
    })
}

fn print_first_n(label: &str, cands: &[SyncCandidate], n: usize) {
    println!("  {label}: len={}", cands.len());
    for (i, c) in cands.iter().take(n).enumerate() {
        println!(
            "    [{i}] freq={:.1} dt={:.3} score={:.4}",
            c.freq_hz, c.dt_sec, c.score
        );
    }
}

#[test]
#[ignore = "real-qso test; run with --include-ignored"]
fn coarse_sync_with_allsum_concurrent_matches_sequential() {
    let path = Path::new(QSO_WAV);
    if !path.exists() {
        println!("SKIP: {QSO_WAV} not found");
        return;
    }

    let audio = load_wav_i16(path);
    println!("loaded {QSO_WAV} — {} samples", audio.len());

    // Build spectrogram once (host one-shot). Both halves share it.
    let spec = compute_spectrogram(&audio[..], FREQ_MAX);
    println!("spec: n_freq={} n_time={}", spec.n_freq, spec.n_time);

    // Precompute per-half allsums (matches stage1_inc layout for
    // [100, 1550] and [1550, 3000]).
    let allsum_head = precompute_coarse_allsum(&spec, FREQ_MIN, FREQ_MID);
    let allsum_tail = precompute_coarse_allsum(&spec, FREQ_MID, FREQ_MAX);
    println!(
        "allsum: head_len={} tail_len={}",
        allsum_head.len(),
        allsum_tail.len()
    );

    // Reference: sequential 2× coarse_sync_with_allsum.
    let seq_head = coarse_sync_with_allsum(
        &spec,
        FREQ_MIN,
        FREQ_MID,
        SYNC_MIN,
        PASS1_LIMIT,
        &allsum_head,
    );
    let seq_tail = coarse_sync_with_allsum(
        &spec,
        FREQ_MID,
        FREQ_MAX,
        SYNC_MIN,
        PASS1_LIMIT,
        &allsum_tail,
    );
    println!("--- sequential ---");
    print_first_n("seq_head", &seq_head, 5);
    print_first_n("seq_tail", &seq_tail, 5);

    // Parallel: spawn worker for tail, main runs head, then join.
    // Repeat the cycle multiple times — the embedded bug is "slot 1
    // works, slots 2+ break", so we must run >1 round to reproduce.
    for round in 0..4 {
        // SAFETY: scoped threads keep `&spec` / `&allsum_*` borrows
        // valid for the worker's lifetime.
        let (par_head, par_tail) = thread::scope(|s| {
            let h = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MIN,
                    FREQ_MID,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_head,
                )
            });
            let t = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MID,
                    FREQ_MAX,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_tail,
                )
            });
            (h.join().unwrap(), t.join().unwrap())
        });

        let head_match = cands_eq(&seq_head, &par_head);
        let tail_match = cands_eq(&seq_tail, &par_tail);
        println!(
            "round {round}: head_match={head_match} tail_match={tail_match} \
             par_head.len={} par_tail.len={}",
            par_head.len(),
            par_tail.len()
        );

        if !head_match || !tail_match {
            print_first_n("par_head (DIFF)", &par_head, 5);
            print_first_n("par_tail (DIFF)", &par_tail, 5);
            panic!("round {round}: parallel result diverges from sequential");
        }
    }
    println!("--- all rounds match sequential ---");
}

/// Mirrors the rx_wavsim slot loop exactly: fresh `Spectrogram` and
/// per-half allsums every iteration (drop + reallocate, exposing any
/// heap-reuse / address-aliasing bug), 3-WAV cycle, parallel dispatch
/// of head/tail with sequential reference computed first.
#[test]
#[ignore = "real-qso test; run with --include-ignored"]
fn coarse_sync_with_allsum_per_slot_fresh_buffers_match() {
    let mut audios: Vec<Vec<i16>> = Vec::new();
    for path in QSO_WAVS {
        let p = Path::new(path);
        if !p.exists() {
            println!("SKIP: {path} not found");
            return;
        }
        audios.push(load_wav_i16(p));
    }

    // 6 rounds = 2 full WAV cycles (qso1, qso2, qso3, qso1, qso2, qso3).
    // The embedded bug appears on slot 4+ (same-WAV revisit), so we
    // need at least 4 to reproduce.
    for slot in 0..6 {
        let wav_idx = slot % audios.len();
        let audio = &audios[wav_idx];

        // Fresh build — drops previous slot's buffers, re-allocs.
        let spec = compute_spectrogram(&audio[..], FREQ_MAX);
        let allsum_head = precompute_coarse_allsum(&spec, FREQ_MIN, FREQ_MID);
        let allsum_tail = precompute_coarse_allsum(&spec, FREQ_MID, FREQ_MAX);

        // Sequential reference.
        let seq_head = coarse_sync_with_allsum(
            &spec,
            FREQ_MIN,
            FREQ_MID,
            SYNC_MIN,
            PASS1_LIMIT,
            &allsum_head,
        );
        let seq_tail = coarse_sync_with_allsum(
            &spec,
            FREQ_MID,
            FREQ_MAX,
            SYNC_MIN,
            PASS1_LIMIT,
            &allsum_tail,
        );

        // Parallel dispatch (mirrors `dual_core::coarse_sync_split_with_allsum`).
        let (par_head, par_tail) = thread::scope(|s| {
            let h = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MIN,
                    FREQ_MID,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_head,
                )
            });
            let t = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MID,
                    FREQ_MAX,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_tail,
                )
            });
            (h.join().unwrap(), t.join().unwrap())
        });

        let head_match = cands_eq(&seq_head, &par_head);
        let tail_match = cands_eq(&seq_tail, &par_tail);
        let seq_first_score = seq_head.first().map(|c| c.score).unwrap_or(0.0);
        let par_first_score = par_head.first().map(|c| c.score).unwrap_or(0.0);
        println!(
            "slot {slot} wav={} head_addr={:p} tail_addr={:p} \
             seq_head_score={:.0} par_head_score={:.0} \
             head_match={head_match} tail_match={tail_match}",
            wav_idx,
            allsum_head.as_ptr(),
            allsum_tail.as_ptr(),
            seq_first_score,
            par_first_score,
        );
        if !head_match || !tail_match {
            print_first_n("seq_head", &seq_head, 5);
            print_first_n("par_head", &par_head, 5);
            print_first_n("seq_tail", &seq_tail, 5);
            print_first_n("par_tail", &par_tail, 5);
            panic!("slot {slot} (wav {wav_idx}): parallel != sequential");
        }
    }
    println!("--- all 6 slots match sequential ---");
}

const MAX_CAND: usize = 15;

/// Full-pipeline host repro: every slot does coarse_sync_split +
/// pass2_split + stage3_split via `std::thread::scope`, mirroring
/// `dual_core::{coarse_sync_split_with_allsum,pass2_split,stage3_split}`.
/// Multi-WAV cycle, fresh buffers per slot. The embedded bug is "slot
/// 1 of qso1 decodes 3 messages; slot 4 (same qso1) decodes 0", so we
/// run 6 slots = 2 full cycles.
#[test]
#[ignore = "real-qso test; run with --include-ignored"]
fn full_pipeline_dispatch_matches_sequential() {
    let mut audios: Vec<Vec<i16>> = Vec::new();
    for path in QSO_WAVS {
        let p = Path::new(path);
        if !p.exists() {
            println!("SKIP: {path} not found");
            return;
        }
        audios.push(load_wav_i16(p));
    }

    for slot in 0..6 {
        let wav_idx = slot % audios.len();
        let audio = &audios[wav_idx];

        let spec = compute_spectrogram(&audio[..], FREQ_MAX);
        let allsum_head = precompute_coarse_allsum(&spec, FREQ_MIN, FREQ_MID);
        let allsum_tail = precompute_coarse_allsum(&spec, FREQ_MID, FREQ_MAX);

        // Reference: full sequential pipeline.
        let mut seq_pass1 = coarse_sync_with_allsum(
            &spec,
            FREQ_MIN,
            FREQ_MID,
            SYNC_MIN,
            PASS1_LIMIT,
            &allsum_head,
        );
        seq_pass1.extend(coarse_sync_with_allsum(
            &spec,
            FREQ_MID,
            FREQ_MAX,
            SYNC_MIN,
            PASS1_LIMIT,
            &allsum_tail,
        ));
        seq_pass1.sort_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(core::cmp::Ordering::Equal)
        });
        seq_pass1.truncate(PASS1_LIMIT);
        let seq_pass1_for_p2 = seq_pass1.clone();
        let seq_pass2 = {
            let mut basis_re = vec![0i16; BASIS_SCRATCH_LEN];
            let mut basis_im = vec![0i16; BASIS_SCRATCH_LEN];
            refine_candidates_into(
                &audio[..],
                seq_pass1_for_p2,
                MAX_CAND,
                &mut basis_re,
                &mut basis_im,
            )
        };
        let seq_stage3 = {
            let mut basis_re = vec![0i16; BASIS_SCRATCH_LEN];
            let mut basis_im = vec![0i16; BASIS_SCRATCH_LEN];
            process_candidates_into(
                &audio[..],
                seq_pass2.clone(),
                DecodeDepth::BpAll,
                &mut basis_re,
                &mut basis_im,
            )
        };

        // Parallel: dispatch each stage's tail to a worker thread.
        let par_pass1 = thread::scope(|s| {
            let h = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MIN,
                    FREQ_MID,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_head,
                )
            });
            let t = s.spawn(|| {
                coarse_sync_with_allsum(
                    &spec,
                    FREQ_MID,
                    FREQ_MAX,
                    SYNC_MIN,
                    PASS1_LIMIT,
                    &allsum_tail,
                )
            });
            let mut all = h.join().unwrap();
            all.extend(t.join().unwrap());
            all.sort_by(|a, b| {
                b.score
                    .partial_cmp(&a.score)
                    .unwrap_or(core::cmp::Ordering::Equal)
            });
            all.truncate(PASS1_LIMIT);
            all
        });
        let par_pass2 = thread::scope(|s| {
            let mid = par_pass1.len() / 2;
            let mut head = par_pass1.clone();
            let tail = head.split_off(mid);
            let h = s.spawn(move || {
                let mut br = vec![0i16; BASIS_SCRATCH_LEN];
                let mut bi = vec![0i16; BASIS_SCRATCH_LEN];
                refine_candidates_into(&audio[..], head, MAX_CAND, &mut br, &mut bi)
            });
            let t = s.spawn(move || {
                let mut br = vec![0i16; BASIS_SCRATCH_LEN];
                let mut bi = vec![0i16; BASIS_SCRATCH_LEN];
                refine_candidates_into(&audio[..], tail, MAX_CAND, &mut br, &mut bi)
            });
            let mut all = h.join().unwrap();
            all.extend(t.join().unwrap());
            all.sort_by(|a, b| b.2.cmp(&a.2));
            all.truncate(MAX_CAND);
            all
        });
        let par_stage3 = thread::scope(|s| {
            let mid = par_pass2.len() / 2;
            let mut head = par_pass2.clone();
            let tail = head.split_off(mid);
            let h = s.spawn(move || {
                let mut br = vec![0i16; BASIS_SCRATCH_LEN];
                let mut bi = vec![0i16; BASIS_SCRATCH_LEN];
                process_candidates_into(&audio[..], head, DecodeDepth::BpAll, &mut br, &mut bi)
            });
            let t = s.spawn(move || {
                let mut br = vec![0i16; BASIS_SCRATCH_LEN];
                let mut bi = vec![0i16; BASIS_SCRATCH_LEN];
                process_candidates_into(&audio[..], tail, DecodeDepth::BpAll, &mut br, &mut bi)
            });
            let mut all = h.join().unwrap();
            all.extend(t.join().unwrap());
            all
        });

        println!(
            "slot {slot} wav={} pass1: seq={} par={} | pass2: seq={} par={} | stage3: seq={} par={}",
            wav_idx,
            seq_pass1.len(),
            par_pass1.len(),
            seq_pass2.len(),
            par_pass2.len(),
            seq_stage3.len(),
            par_stage3.len()
        );
        if seq_stage3.len() != par_stage3.len() {
            for r in &seq_stage3 {
                println!("  seq: {:?}", r.message77);
            }
            for r in &par_stage3 {
                println!("  par: {:?}", r.message77);
            }
            panic!(
                "slot {slot}: stage3 result count diverges (seq={} par={})",
                seq_stage3.len(),
                par_stage3.len()
            );
        }
    }
    println!("--- all 6 slots match sequential pipeline ---");
}
