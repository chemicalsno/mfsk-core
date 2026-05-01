# mfsk-core — agent notes

Repo-level notes for assistants working in this tree. Anything specific to
a sub-crate lives in its own `CLAUDE.md` or `README.md`; this file is for
cross-crate workflow that's easy to forget between sessions.

## Embedded targets

### `embedded-poc/m5stack-core2`

Real hardware: M5Stack Core2 (ESP32-D0WD-V3, Xtensa LX6, 8 MB PSRAM).

- **Build & flash via `espflash`**, not host cargo. The `.cargo/config.toml`
  in that crate sets `runner = "espflash flash --monitor"`, so the user's
  workflow is:
  ```sh
  cd embedded-poc/m5stack-core2
  cargo run --release          # builds + flashes + opens serial monitor
  ```
- The `+esp` Rust toolchain (Xtensa fork, espup-installed) is selected by
  `rust-toolchain.toml`. Host (`cargo check`/`build`) uses `xtensa-esp32-espidf`
  per `.cargo/config.toml`.
- `cargo check` from inside the crate is enough to validate code changes
  without flashing (~50 s with prebuilt esp-idf).
- Logs from the device land in `embedded-poc/m5stack-core2/logs/` —
  user has been capturing per-config sweep output there.
- The user has been actively flashing this throughout the
  decode_block-embedded work; do NOT assume "host check is enough" when
  changing main.rs or anything that affects the bench output. Offer to
  let the user flash and capture the new log.

## Memory

- `~/.claude/projects/-home-minoru-src-mfsk-core/memory/` holds the
  per-conversation auto-memory. `project_decode_block_embedded.md` is the
  authoritative log of the embedded-port performance journey — read it
  before touching `decode_block` or m5stack-core2/main.rs.
