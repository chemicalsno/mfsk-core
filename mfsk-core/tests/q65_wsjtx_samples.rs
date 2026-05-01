//! Q65-30A real-world signal validation against WSJT-X sample
//! recordings.
//!
//! WSJT-X ships off-the-air capture .wav files at
//! `samples/Q65/30A_Ionoscatter_6m/*.wav` (12 kHz mono PCM-16, 30 s
//! each — Joe Taylor's reference dataset). These are real ionoscatter
//! signals on 6 m, captured by K1JT, with all the channel impairments
//! (Doppler, multipath, fading) absent from the synth-only tests.
//!
//! The test is conditionally skipped when the WSJT-X tree is not
//! present at the expected sibling path
//! (`../../WSJT-X/samples/Q65/30A_Ionoscatter_6m/`); developers
//! cloning only `mfsk-core` won't see a failure they can't fix.

#![cfg(feature = "q65")]

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use mfsk_core::fec::qra::FadingModel;
use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{
    Q65a30, Q65a60, decode_multi_period_for, decode_scan, decode_scan_fading_for,
    decode_scan_with_ap, decode_scan_with_ap_for,
};

/// Minimal WAV reader for WSJT-X's exact format: RIFF/WAVE header,
/// `fmt ` chunk = PCM (1 channel, 12 kHz, 16-bit), `data` chunk =
/// little-endian i16 samples. Anything else returns `None`.
fn read_wsjtx_wav(path: &Path) -> Option<Vec<f32>> {
    let mut file = File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    // Locate the `data` chunk after the standard 44-byte RIFF header.
    if bytes.len() < 44 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    // Confirm fmt chunk advertises mono / 12 kHz / 16-bit PCM.
    if &bytes[12..16] != b"fmt " {
        return None;
    }
    let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
    let sample_rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
    if channels != 1 || sample_rate != 12_000 || bits != 16 {
        return None;
    }
    if &bytes[36..40] != b"data" {
        return None;
    }
    let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
    let data = &bytes[44..44 + data_len.min(bytes.len() - 44)];
    let mut out = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        out.push(s as f32 / 32_768.0);
    }
    Some(out)
}

fn samples_dir(rel: &str) -> Option<PathBuf> {
    // Tests run from `mfsk-core/mfsk-core/`; the WSJT-X tree is at
    // `mfsk-core/../WSJT-X/`. Use `CARGO_MANIFEST_DIR` so the lookup
    // is independent of the caller's working directory.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65")
        .join(rel)
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

/// Smoke test: the Q65-30A *single-period* receive chain (plain BP +
/// AP + fast-fading metric) runs to completion against every WSJT-X
/// ionoscatter reference recording without panicking, and the WAV
/// reader handles the file format correctly.
///
/// **Single-period decode rate is not asserted** — these recordings
/// sit below the single-period decode threshold (each WAV produces
/// 0 decodes via plain / AP-CQ / fast-fading taken in isolation).
/// The averaged-decode gate lives in
/// `ionoscatter_6m_full_stack_decodes_via_averaging` below, which
/// stacks the slots and recovers them via the multi-period EMA path
/// (`decode_multi_period_for`). This test stays useful as a
/// regression catch on the per-slot chain (panic / WAV-reader /
/// search-params blow-up).
#[test]
fn ionoscatter_6m_receive_chain_runs() {
    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!(
            "skipping: WSJT-X sample tree not found at ../../WSJT-X/samples/Q65/30A_Ionoscatter_6m/"
        );
        return;
    };

    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    assert!(
        !entries.is_empty(),
        "WSJT-X Q65-30A sample dir contains no .wav files"
    );

    // ±50 symbols × 0.3 s/sym ≈ ±15 s around the slot midpoint.
    let nominal_mid = 12_000 * 15; // 15 s into the 30 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut wav_count = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => {
                eprintln!("skip {}: unsupported WAV format", path.display());
                continue;
            }
        };
        wav_count += 1;

        // Three receive paths must all complete without panic. Decode
        // counts are reported but not asserted — see this test's
        // docstring for why ionoscatter is currently a known gap.
        let plain = decode_scan(&audio, 12_000, nominal_mid, &params);
        let cq = decode_scan_with_ap(
            &audio,
            12_000,
            nominal_mid,
            &params,
            &ApHint::new().with_call1("CQ"),
        );
        let fading = decode_scan_fading_for::<Q65a30>(
            &audio,
            12_000,
            nominal_mid,
            &params,
            8.0,
            FadingModel::Gaussian,
            None,
        );
        eprintln!(
            "{}: plain={} cq={} fading_b90=8={}",
            path.file_name().unwrap().to_string_lossy(),
            plain.len(),
            cq.len(),
            fading.len(),
        );
    }
    assert!(
        wav_count > 0,
        "no readable WAVs in WSJT-X Q65-30A ionoscatter sample dir"
    );
}

/// Strict gate: stack the four ionoscatter recordings into one
/// running EMA via [`decode_multi_period_for`] and require at least
/// one decode total.
///
/// Mirrors WSJT-X's `iavg=1`/`iavg=2` averaged-decode flow from
/// `lib/q65_decode.f90` — the path that lets weak ionoscatter
/// signals decode when single-period BP/fading cannot. Our reduced
/// b90 sweep `{3, 8, 15} × {Gaussian, Lorentzian}` plus plain Bessel
/// fallback covers the realistic ionoscatter spread regime.
///
/// Currently no AP-list pair is supplied (the recordings are from
/// an unknown station so we don't know plausible call/grid pairs);
/// the fading + plain BP ladder must reach the threshold on its own.
/// If the expected call/grid pair becomes known, pass it through
/// [`mfsk_core::q65::standard_qso_codewords`] for the AP-list path.
#[test]
fn ionoscatter_6m_full_stack_decodes_via_averaging() {
    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!("skipping: WSJT-X sample tree not found");
        return;
    };
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    // Stable order so the EMA sees slots in chronological order.
    paths.sort();
    assert!(
        !paths.is_empty(),
        "WSJT-X Q65-30A sample dir contains no .wav files"
    );

    let audios: Vec<Vec<f32>> = paths.iter().filter_map(|p| read_wsjtx_wav(p)).collect();
    assert!(
        !audios.is_empty(),
        "no readable WAVs in WSJT-X Q65-30A ionoscatter sample dir"
    );
    let slot_refs: Vec<&[f32]> = audios.iter().map(|v| v.as_slice()).collect();

    // 15 s into the 30 s slot, ±15 s tolerance — covers the full
    // recording so the running-EMA coarse search has the whole slot.
    let nominal_mid = 12_000 * 15;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let decodes_no_ap =
        decode_multi_period_for::<Q65a30>(&slot_refs, 12_000, nominal_mid, &params, None);
    eprintln!(
        "[info] ionoscatter multi-period (no AP-list): {} unique decode(s) across {} slot(s)",
        decodes_no_ap.len(),
        slot_refs.len(),
    );
    for d in &decodes_no_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.message
        );
    }

    // Try the AP-list path with K1JT / K9AN — the call pair revealed
    // by the no-AP run above. WSJT-X normally builds this list from
    // the user's "Watch list" or last-seen-CQ; here we hard-code the
    // discovered pair to test the AP-list integration.
    use mfsk_core::q65::standard_qso_codewords;
    let ap_codewords = standard_qso_codewords("K1JT", "K9AN", "");
    let decodes_ap = decode_multi_period_for::<Q65a30>(
        &slot_refs,
        12_000,
        nominal_mid,
        &params,
        Some(&ap_codewords),
    );
    eprintln!(
        "[info] ionoscatter multi-period (AP-list K1JT/K9AN): {} unique decode(s)",
        decodes_ap.len(),
    );
    for d in &decodes_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.message
        );
    }

    // Goal: multi-period averaging recovers the signal in at least one
    // of the strategies. Single decode counts as success because the
    // running-EMA collapses repeated copies of the same QSO line into
    // one output entry — once a QSO has been recovered there's no
    // additional information from re-recovering it.
    assert!(
        !decodes_no_ap.is_empty() || !decodes_ap.is_empty(),
        "0/{} ionoscatter slots produced any decode through multi-period averaging \
         (neither without AP-list nor with K1JT/K9AN AP-list) — regression or \
         insufficient signal recovery in the EMA-on-spectrogram path",
        slot_refs.len(),
    );
}

#[test]
fn eme_6m_sample_yields_decode_with_ap() {
    // Q65-60A 6 m EME recording. With the AP path active we
    // should be able to recover at least one of the typical
    // call/CQ patterns even from this real-world weak signal.
    let Some(dir) = samples_dir("60A_EME_6m") else {
        eprintln!("skipping: WSJT-X 6m EME sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    assert!(
        !entries.is_empty(),
        "WSJT-X 6m EME sample dir contains no .wav files"
    );

    // The 60A frame can begin anywhere in a 60 s slot — wide tol.
    let nominal_mid = 12_000 * 30; // 30 s into the 60 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    // The 210106_1621.wav recording captures W7GJ working multiple
    // stations on 6 m EME (W7GJ is a well-known prolific 6 m EME
    // operator). Try the empty hint plus a hint locking call1 =
    // W7GJ (matching the actual exchange).
    let hints = [
        ("plain", ApHint::new()),
        ("W7GJ ??", ApHint::new().with_call1("W7GJ")),
    ];

    let mut plain_count = 0usize;
    let mut ap_count = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        for (label, hint) in &hints {
            use mfsk_core::q65::decode_scan_for;
            let decodes = if hint.has_info() {
                decode_scan_with_ap_for::<Q65a60>(&audio, 12_000, nominal_mid, &params, hint)
            } else {
                decode_scan_for::<Q65a60>(&audio, 12_000, nominal_mid, &params)
            };
            let names: Vec<String> = decodes.iter().map(|d| d.message.clone()).collect();
            println!(
                "{} [{label}]: {} decode(s) → {names:?}",
                path.file_name().unwrap().to_string_lossy(),
                decodes.len()
            );
            if hint.has_info() {
                ap_count += decodes.len();
            } else {
                plain_count += decodes.len();
            }
        }
    }
    // 6 m EME has the lowest Doppler spread in the EME band lineup,
    // so the AWGN-only metric already does a respectable job on
    // strong-ish signals — the published 210106_1621.wav reference
    // typically yields several W7GJ exchanges on first scan. We
    // require at least one decode to land via the plain or AP
    // path so a regression in the receive chain trips this test.
    assert!(
        plain_count + ap_count > 0,
        "6m EME reference recording produced no decodes via either \
         plain or AP — regression in the Q65-60A receive chain"
    );
    eprintln!("[info] 6m EME: plain {plain_count} decode(s), AP {ap_count} decode(s)");
}
