# mfsk-core M5Stack Core2 test bench

FT8 decode-on-hardware PoC for the **M5Stack Core2** (ESP32-D0WD-V3,
LX6 dual-core @ 240 MHz, 8 MB QUAD PSRAM, 16 MB Flash).

## Goal

Prove `mfsk_core::ft8::decode_block` — the embedded-friendly FT8
decoder that uses only power-of-two FFTs and the LDPC normalised
min-sum kernel — runs to completion on real silicon, and time each
stage so the eventual transceiver app can budget its slot.

`decode_block` is the *only* decode path used here; `decode_frame` /
`decode_sniper_*` rely on a 192 000-pt wide-band FFT cache that is
not power-of-two (esp-dsp can't run it) and a 3 840-pt per-symbol
FFT (also non-pow-2). Sensitivity vs `decode_frame` was characterised
on host AWGN sweeps (~0.7 dB threshold loss at 50 % decode rate) before
this binary was written — see `mfsk-core/tests/ft8_decode_block_snr_sweep.rs`.

## What it does

1. Synthesise a clean `CQ JL1NIE PM95` FT8 burst at 1500 Hz into a
   180 000-sample slot (PSRAM).
2. Run `decode_block` over the slot, with the binary-supplied
   `EspDspPlanner` (Xtensa AE32 ASM via the espressif/esp-dsp managed
   component) plumbed through `mfsk_core::core::fft::default_planner()`.
3. Print stage timing and every recovered message to UART.

The decoder returns the truth message → boot log says **PASS**;
otherwise **FAIL**. No LCD / touch / mic in this slice — that comes
once timing is confirmed.

## Build & flash

```sh
# from this directory
cargo run --release
```

The `+esp` Rust toolchain (espup-installed Xtensa Rust) is selected by
`rust-toolchain.toml`; the `xtensa-esp32-espidf` target by
`.cargo/config.toml`. ESP-IDF v5.5.3 will be downloaded by `embuild`
on first build (~2 GB checkout into `.embuild/`).

## Expected timing (LX6 @ 240 MHz, NFFT_SPEC=8192)

| stage              | wall-clock (estimated) |
|--------------------|------------------------|
| spectrogram        | ~3.0 s (372 × 8192-pt FFT) |
| coarse sync        | ~0.2 s |
| dt-refine + LLR    | ~0.5 s (5 dt × ≤5 cands × 20 ms DFT) |
| LDPC BP (NMS α=0.75) | ~0.25 s |
| **total**          | **~4.0 s** |

This is ~2 s over the 1.86 s in-slot budget (slot_end −
TX_end = 15 s − 13.14 s). Decode therefore spills into the next
slot's RX window — acceptable for a "decode every N-th slot" or
"always one slot behind" architecture; an issue for full-duty live
operation. Numbers will be replaced with the actual measurement
once the binary has been run.

## What's *not* in this PoC

- I2S mic capture (live RX).
- LCD output / touch input.
- Slot timing (NTP / GPS).
- TX path (already validated on the S3 PoC).
- FT4 / WSPR / Q65.
- `decode_sniper*` — by design, embedded uses `decode_block` only.

## Why ESP32 (not S3) here

M5Stack Core2 v1.x ships with the original ESP32-D0WD (Xtensa LX6),
not the LX7 ESP32-S3. Practical differences for this PoC:

- **PSRAM mode**: QUAD on Core2 vs OCT on the S3-WROOM-1 module.
  `sdkconfig.defaults` reflects this with `CONFIG_SPIRAM_MODE_QUAD=y`.
- **esp-dsp ASM**: `dsps_fft2r_fc32_ae32_` exists on both — the file
  `src/esp_dsp_fft.rs` is a verbatim copy of the S3 PoC's adapter.
- **FPU**: both have hardware single-precision FPU. The S3 also adds
  PIE vector instructions which esp-dsp uses on some routines, so the
  same FFT kernel is generally 1.5–2× faster on S3 than on LX6. The
  numbers above already account for that.

## Files

| file | role |
|------|------|
| `Cargo.toml` | crate manifest, `mfsk-core` path dep with `embedded-rx` features |
| `.cargo/config.toml` | `xtensa-esp32-espidf` target + ldproxy / espflash runner |
| `rust-toolchain.toml` | `channel = "esp"` |
| `sdkconfig.defaults` | PSRAM (QUAD) + extended esp-dsp twiddle table |
| `build.rs` | `embuild::espidf::sysenv::output()` |
| `src/bindings.h` | esp-dsp bindgen header |
| `src/esp_dsp_fft.rs` | `mfsk_core_make_default_fft_planner()` factory + `EspDspPlanner` (verbatim from S3 PoC) |
| `src/main.rs` | synth + decode + UART log |
