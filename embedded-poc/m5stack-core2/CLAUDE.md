# m5stack-core2 — agent build/flash notes

Quick-reference for the ESP32 cross-dev workflow this PoC uses. Forgetting
any of these has cost real time before; check this list before invoking
anything.

## One-time setup (already done on this machine)

- `~/.rustup/toolchains/esp/` — Xtensa-fork Rust toolchain installed via
  [`espup`](https://github.com/esp-rs/espup). Selected automatically for
  this crate by `rust-toolchain.toml` (`channel = "esp"`).
- `~/.espressif/` — esp-idf checkout / tools managed by `embuild`
  (downloaded on first build into `.embuild/` inside the crate).
- `~/export-esp.sh` — sets `PATH` and `LIBCLANG_PATH` for the Xtensa
  toolchain. **Must be sourced** before any cargo invocation in this
  crate, otherwise `bindgen` fails to find clang and the Xtensa GCC
  binutils aren't on PATH.
- `~/.cargo/bin/espflash` — flasher.

## Build + flash + monitor

The user's actual workflow has three steps, kept separate so each can
be re-run independently. Do **not** collapse them into `cargo run`
under a non-interactive shell — espflash's runner mode prompts for
disambiguation when multiple ports exist and fails with "not a
terminal" under `tee`/pipe redirection.

```sh
# 1. Source the cross-dev env (PATH + LIBCLANG_PATH).
source ~/export-esp.sh

# 2. Build for xtensa-esp32-espidf. ~30 s clean release; ~3 s incremental.
cd embedded-poc/m5stack-core2
cargo build --release

# 3. Flash and open the serial monitor. M5Stack Core2 enumerates as
#    /dev/ttyACM0; pass --port explicitly so espflash never prompts.
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32-espidf/release/mfsk-core-m5stack-core2
```

`cargo run --release` does work the same when run from an interactive
TTY (the `runner` in `.cargo/config.toml` invokes `espflash flash
--monitor`), but the explicit-3-step form is more robust under
automation / piped tees and matches what the user types.

## Capturing logs

Pipe espflash output to `logs/<descriptive>.log` to keep the on-device
sweep results — the bench logs every per-stage timing and per-message
SNR over UART, and we compare across runs (`grep -E 'WAV|stage|truth'`
across files).

```sh
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32-espidf/release/mfsk-core-m5stack-core2 \
    | tee logs/realqso_<config>_$(date +%Y-%m-%d).log
```

The serial monitor only exits on Ctrl-C; for long sweeps the user
typically lets the bench finish (logged "Sweep complete. Idling.")
then interrupts.

## Trouble we've already debugged

- **`espflash::no_serial`** — device not connected, or `/dev/ttyACM0`
  permission denied (user not in `dialout`).
- **`espflash::dialoguer_error: not a terminal`** — espflash auto-
  detected multiple ports and tried to prompt; pass `--port` explicitly.
- **bindgen / `unable to find libclang`** — forgot to source
  `~/export-esp.sh`. `LIBCLANG_PATH` must point at the bundled
  esp-clang (not system clang).
- **wrong toolchain (`error: toolchain 'esp' is not installed`)** —
  reinstall via `espup install`. Don't try to use stable Rust here.
- **`tlsf_malloc` heap corruption mid-sweep** — known bug when
  `decode_block` is called directly (see memory
  `project_decode_block_embedded.md` item 2). Production main.rs
  replicates the D pattern manually.
