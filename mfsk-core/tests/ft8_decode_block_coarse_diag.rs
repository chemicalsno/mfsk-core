//! Diagnostic: dump every candidate `decode_block::coarse_sync`
//! emits for the 3 real-QSO WAVs, cross-reference with the truth
//! freqs `decode_frame` finds. If a truth signal isn't even in the
//! coarse_sync output (top-N), the recall gap originates in stage 1+2,
//! not in BP/OSD.
//!
//! Run with `--features fft-rustfft,ft8,fixed-point` to test the
//! embedded i16 path; without `fixed-point` to test the f32 path.

use std::path::Path;

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram};
use mfsk_core::msg::wsjt77::unpack77;

const QSO_WAVS: &[&str] = &[
    "/home/minoru/src/rs-ft8n/ft8-bench/testdata/191111_110130.wav",
    "/home/minoru/src/rs-ft8n/ft8-bench/testdata/191111_110200.wav",
    "/home/minoru/src/WSJT-X/samples/FT8/210703_133430.wav",
];

fn load_wav_i16(path: &Path) -> Vec<i16> {
    let bytes = std::fs::read(path).expect("read wav");
    let payload = &bytes[44..];
    payload
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

#[test]
#[ignore = "diagnostic — run with --include-ignored --nocapture"]
fn coarse_sync_candidate_diag() {
    println!(
        "\n=== coarse_sync candidate diagnostic ===  (block path, max_cand=200, sync_min=0.5)\n"
    );
    #[cfg(feature = "fixed-point")]
    println!("(fixed-point feature ON — i16 spec / sc16 path)");
    #[cfg(not(feature = "fixed-point"))]
    println!("(fixed-point feature OFF — f32 spec)");

    for wav_path in QSO_WAVS {
        let path = Path::new(wav_path);
        let label = path.file_name().unwrap().to_string_lossy();
        let slot = load_wav_i16(path);

        // Frame's truth: every signal frame can find.
        let frame = decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
        let truth: Vec<(f32, f32, f32, String)> = frame
            .iter()
            .filter_map(|r| unpack77(&r.message77).map(|t| (r.freq_hz, r.dt_sec, r.snr_db, t)))
            .collect();

        // Block's candidates.
        let spec = compute_spectrogram(&slot, 3000.0);
        let cands = coarse_sync(&spec, 100.0, 3000.0, 0.5, 200);

        println!("── {} ─────────────────────────────────────", label);
        println!("  truth from decode_frame: {} signals", truth.len());
        println!("  block coarse_sync emitted: {} candidates", cands.len());

        // For each truth, find the closest candidate within 6 Hz / 0.5 s.
        println!(
            "\n  {:<32} | {:>7} | {:>5} | {:>4} | closest cand (df, ddt, score)",
            "truth msg", "freq", "SNR", "dt"
        );
        println!("  {}", "─".repeat(95));
        for (tf, tdt, tsnr, tmsg) in &truth {
            let mut best_df = f32::INFINITY;
            let mut best: Option<&_> = None;
            for c in &cands {
                let df = (c.freq_hz - tf).abs();
                let ddt = (c.dt_sec - tdt).abs();
                if df < 6.0 && ddt < 0.5 && df < best_df {
                    best_df = df;
                    best = Some(c);
                }
            }
            let mt = if tmsg.len() > 32 {
                &tmsg[..32]
            } else {
                tmsg.as_str()
            };
            match best {
                Some(c) => println!(
                    "  {:<32} | {:>5.0}Hz | {:>+4.0}  | {:>+4.2} | df={:+.1}Hz ddt={:+.2}s s={:.2}",
                    mt,
                    tf,
                    tsnr,
                    tdt,
                    c.freq_hz - tf,
                    c.dt_sec - tdt,
                    c.score,
                ),
                None => println!(
                    "  {:<32} | {:>5.0}Hz | {:>+4.0}  | {:>+4.2} | ✗ MISSING from coarse_sync",
                    mt, tf, tsnr, tdt,
                ),
            }
        }
        println!();
    }
}
