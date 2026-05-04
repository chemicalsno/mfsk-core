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
    embedded_shared::apps::rx_wavsim::run(
        QSO_WAVS,
        mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH,
    )
}
