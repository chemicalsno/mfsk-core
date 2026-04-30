# `mfsk-core` ESP32-S3 PoC

Proof-of-concept binary that exercises `mfsk-core`'s embedded port
(`embedded` branch) on real ESP32-S3 hardware.

This is **not** a separate crate — it lives under
`mfsk-core/embedded-poc/esp32s3/` and is excluded from the host
workspace so `cargo build` from the repo root keeps building only
the platform-agnostic library.

## What it does

1. Packs a 77-bit WSJT message (`CQ JA1NIE PM95`).
2. Synthesises one FT4 burst (~5 sec @ 12 kHz, 59 328 samples) into a
   pre-allocated buffer via the new caller-buffer API
   (`mfsk_core::ft4::encode::tones_to_f32_into`).
3. Runs the `mfsk_core::core::fft::FftPlanner` trait round-trip at
   1024 and 4096 points using the `EspDspPlanner` adapter that wraps
   `dsps_fft2r_fc32_ae32_` (esp-dsp ASM).

Reports timing for each step via `log::info!`. No hardware
peripherals required — runs entirely from the main task on power-up.

## Build / flash

From this directory (the `+esp` toolchain pin in `rust-toolchain.toml`
takes effect locally):

```bash
. ~/export-esp.sh                 # espup-installed Rust + Xtensa toolchain
cargo build --release             # ~5–15 min on first build (downloads ESP-IDF + esp-dsp)
espflash flash --monitor target/xtensa-esp32s3-espidf/release/mfsk-core-esp32s3
```

## Architecture

```
mfsk_core decode pipeline (sync / llr / build_fft_cache / …)
    │ calls
    ▼
mfsk_core::core::fft::default_planner()
    │ feature = "fft-extern"
    │ resolves to extern "Rust" symbol
    ▼
mfsk_core_make_default_fft_planner()      ← provided here
    │ in src/esp_dsp_fft.rs
    ▼
Box::new(EspDspPlanner::new())            ← FftPlanner trait impl
    │ wraps
    ▼
dsps_fft2r_fc32_ae32_  (esp-dsp ASM, Xtensa LX7 hand-written)
```

## Configuration

| Aspect | Setting |
|---|---|
| **mfsk-core features** | `alloc`, `ft4`, `ft8`, `wspr`, `fft-extern` |
| **FFT backend** | `EspDspPlanner` (esp-dsp ASM via FFI) |
| **ESP-IDF version** | v5.5.x (auto-fetched by `embuild` on first build) |
| **esp-dsp version** | `^1.4` (pulled via `[[package.metadata.esp-idf-sys.extra_components]]`) |
| **PSRAM** | enabled in `sdkconfig.defaults` (8 MB octal — typical WROOM-1) |
| **Optimisation** | `opt-level = 1` (conservative, see Cargo.toml comment) |

Approximate sizes from the last build (`xtensa-esp32s3-elf-size`):

| Section | Bytes |
|---|---:|
| `.text` (code) | 441 553 |
| `.data` (initialised) | 128 406 |
| `.bss` (zero-init / heap) | 966 323 |
| **Total** | **1 536 282** (~1.5 MB ELF) |

## Layout

```
embedded-poc/esp32s3/
├── Cargo.toml          # esp-idf-svc + mfsk-core (path = ../../mfsk-core)
├── build.rs            # embuild::espidf::sysenv::output()
├── rust-toolchain.toml # channel = "esp"
├── sdkconfig.defaults  # PSRAM, dsp tables, main-task stack
└── src/
    ├── main.rs         # PoC entry point
    ├── esp_dsp_fft.rs  # EspDspPlanner FftPlanner adapter
    └── bindings.h      # bindgen header for esp-dsp managed component
```

## License

GPL-3.0-or-later (matches `mfsk-core`).
