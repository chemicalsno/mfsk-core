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

use std::path::Path;
use std::thread;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode_block::{
    coarse_sync_with_allsum, compute_spectrogram, precompute_coarse_allsum,
};

const QSO_WAV: &str = "/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3/assets/qso3_busy.wav";
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
