//! WAV-fed streaming RX bench (M5Stack Core2 / ESP32-D0WD / LX6).
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
    // LX6 stays at the recall-floor (PASS1=30 / max_cand=15 / BP=30) —
    // at 1.4 s post-SlotEnd in this config it's already at the FT8
    // ~2 s QSO-turnaround ceiling. Going to PASS1=100 would push it to
    // ~2.4 s and miss the next slot's TX window. `bp_max_iter` is the
    // dominant scale knob inside stage 3: lowering to 20 cuts ~150 ms
    // off the LX6 budget at the cost of one borderline weak signal.
    embedded_shared::apps::rx_wavsim::run(
        QSO_WAVS,
        30,
        15,
        mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH,
        mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
    )
}
