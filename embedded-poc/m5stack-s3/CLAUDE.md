# m5stack-s3 — agent build/flash notes

ESP32-S3 (LX7) クロス開発ワークフロー。m5stack-core2 (LX6) 用 CLAUDE.md
の S3 版 — toolchain や流儀は同じで、target / port / PSRAM mode のみ差分。
issue #15 Phase 2 baseline 取得を主目的に作成 (2026-05-04)。

## One-time setup (already done on this machine)

- `~/.rustup/toolchains/esp/` — Xtensa-fork Rust toolchain installed via
  [`espup`](https://github.com/esp-rs/espup). Selected automatically for
  this crate by `rust-toolchain.toml` (`channel = "esp"`)。LX7 ターゲット
  `xtensa-esp32s3-espidf` も同じ esp toolchain が rustc/std を提供する。
- `~/.espressif/` — esp-idf checkout / tools managed by `embuild`
  (downloaded on first build into `.embuild/` inside the crate).
- `~/export-esp.sh` — sets `PATH` and `LIBCLANG_PATH` for the Xtensa
  toolchain. **Must be sourced** before any cargo invocation in this
  crate, otherwise `bindgen` fails to find clang and the Xtensa GCC
  binutils aren't on PATH.
- `~/.cargo/bin/espflash` — flasher.

## Build + flash + monitor

```sh
# 1. Source the cross-dev env (PATH + LIBCLANG_PATH).
source ~/export-esp.sh

# 2. Build for xtensa-esp32s3-espidf. ~30 s clean release; ~3 s incremental.
cd embedded-poc/m5stack-s3
cargo build --release

# 3. Flash and open the serial monitor. S3 dev kits typically enumerate
#    as /dev/ttyACM0 (USB-Serial-JTAG, native S3) or /dev/ttyUSB0
#    (CP210x bridge on M5Stack S3 modules). Confirm with `dmesg | tail`
#    after plugging in, then pass --port explicitly.
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3
```

`cargo run --release` も interactive TTY なら同じく動く (`.cargo/config.toml`
の `runner` が `espflash flash --monitor`)。tee/pipe redirection をかける
場合のみ 3-step に分ける。

## Capturing logs

```sh
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3 \
    | tee logs/<descriptive>_$(date +%Y-%m-%d).log
```

Phase 2 baseline は `logs/s3_baseline_<date>.log` /
`logs/s3_baseline_llr_i8_<date>.log` のように命名。

## LX6 との差分メモ

- **PSRAM mode**: S3 は Octal (`CONFIG_SPIRAM_MODE_OCT=y`、~80 MB/s)。
  Quad PSRAM の S3 ボード (M5Stamp S3 等) を使う場合は `_MODE_QUAD=y`
  に切替。
- **内蔵 SRAM**: S3 ~512 KB DRAM (vs LX6 ~280 KB usable)。issue #15
  Phase 5 で stage 3 per-cand workspace を内蔵 pin する余地。
- **PIE SIMD**: S3 LX7 のみ。esp-dsp が S3 ビルド時に自動選択する
  ので、stage1 FFT / stage3 DFT は素のリビルドだけで速くなる見込み。
  手書き intrinsics は追求しない (issue #15 前提結論 1)。
- **dual_core dispatch + Phase E2 race**: LX6 で未解決のまま deferred
  (`project_decode_block_embedded.md` 参照)。S3 でも同じ症状が出る
  可能性が高いので、`rx_wavsim` は **sequential per-half on main** を
  そのまま継承して信頼運用。dispatch 経路は port するが配線しない。
- **opt-level**: m5stack-core2 と同じく `release.opt-level = 1`。
  `qsb_partial_gain` workaround は mfsk-core 側で入っているので "s"/"z"
  も技術的には通るはずだが、S3 で初めて build するときは 1 で安全側に。

## Trouble we've already debugged (LX6 から継承)

- **`espflash::no_serial`** — device not connected, or port permission
  denied (user not in `dialout`)。
- **`espflash::dialoguer_error: not a terminal`** — pass `--port` explicitly.
- **bindgen / `unable to find libclang`** — forgot to source
  `~/export-esp.sh`. `LIBCLANG_PATH` must point at the bundled
  esp-clang (not system clang).
- **`tlsf_malloc` heap corruption mid-sweep** — `decode_block` を
  main.rs から直接呼ぶと再現する既知バグ (memory
  `project_decode_block_embedded.md` item 2)。Production main.rs は
  D pattern を手で reproduce して回避。
- **dual-core 化で `SPIRAM_MALLOC_ALWAYSINTERNAL=4096` が必須** —
  16384 だと両 worker の cs Box (5KB×15×2) が内蔵 DRAM 行きで枯渇 →
  tlsf 破壊 → Guru Meditation。S3 でも継承して 4096 のまま。
