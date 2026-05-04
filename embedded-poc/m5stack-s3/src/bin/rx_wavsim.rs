//! WAV-fed streaming RX bench (M5StickS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::rx_wavsim`.
//!
//! Build: `cargo build --release --bin rx-wavsim`.

const QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../../assets/qso1.wav"),
    include_bytes!("../../../assets/qso2.wav"),
    include_bytes!("../../../assets/qso3_busy.wav"),
];

fn main() -> ! {
    // PASS1=30 / max_cand=15 — production setting.
    //
    // Bench-tested 2026-05-04 against the WSJT-X reference recording
    // (qso3 busy band): widening to PASS1=100 / max_cand=30 doubles
    // post-SlotEnd time on qso3 (0.71 s → 1.59 s) but recovers
    // **zero** extra qso3 callsigns — the missed signals are below
    // the coarse_sync top-100 entirely. The +1 recall gain is on
    // qso1 (OH3NIV -17 dB), and 1.59 s leaves only 0.4 s of UI budget
    // before the FT8 ~2 s QSO turnaround. Not worth it for a
    // qso1-only pickup. Log: `s3_pass100_max30_2026-05-04.log`.
    embedded_shared::apps::rx_wavsim::run(
        QSO_WAVS,
        30,
        15,
        mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH,
    )
}
