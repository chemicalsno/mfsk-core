//! M5Stack Core2 (ESP32-D0WD-V3, LX6 dual-core, 8 MB QUAD PSRAM,
//! 16 MB Flash) FT8 compute bench. Thin shim — all logic in
//! `embedded_shared::apps::compute_bench`.

const QSO_WAVS: &[(&str, &[u8])] = &[
    ("qso1 (191111_110130)", include_bytes!("../../assets/qso1.wav")),
    ("qso2 (191111_110200)", include_bytes!("../../assets/qso2.wav")),
    (
        "qso3 busy band (210703)",
        include_bytes!("../../assets/qso3_busy.wav"),
    ),
];

fn main() -> ! {
    embedded_shared::apps::compute_bench::run("m5stack-core2", QSO_WAVS)
}
