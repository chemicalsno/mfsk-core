# mfsk-core

[![CI](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml/badge.svg)](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/mfsk-core.svg)](https://crates.io/crates/mfsk-core)
[![docs.rs](https://img.shields.io/docsrs/mfsk-core)](https://docs.rs/mfsk-core)
[![License](https://img.shields.io/badge/license-GPL--3.0--or--later-blue.svg)](LICENSE)

Pure-Rust library for **WSJT-family digital amateur-radio modes** — a
single crate that implements FT8, FT4, FST4, WSPR, JT9, JT65 and
Q65-30A decode / encode / synthesis on top of a small set of shared
primitives (DSP, sync correlation, LLR, LDPC / convolutional /
Reed-Solomon / QRA FEC, message codecs).

## Why this exists

[WSJT-X](https://sourceforge.net/projects/wsjt/) is the reference
implementation of these modes and will stay that way — it is
battle-tested on the desktop, heavily optimised, and the source of
truth for every protocol constant you will find in this crate. But
it is also a mixed Fortran / C / Qt application built around a
specific desktop workflow. That makes it a poor fit whenever you
want to run the decoders *somewhere else*:

- in a **browser** as a WASM PWA,
- on **Android or iOS** for portable operation, where linking a
  Fortran runtime is a non-starter,
- in a **headless Rust application** (skimmer, monitoring station,
  remote SDR front end),
- on **embedded MCUs** (ESP32-S3 with esp-dsp, RP2350 with CMSIS-DSP,
  Cortex-M) via `no_std + alloc` — the M5Stack Core2 PoC decodes 3–7
  FT8 results per 15 s cycle on Xtensa LX6 with the fixed-point hot
  path,
- or as the core of a **new protocol experiment** that reuses FT8's
  LDPC and sync machinery for a different modulation / FEC /
  message recipe.

The seven protocols share roughly 80 % of their signal path. In the
Fortran codebase that commonality is expressed by copy-and-paste
between per-mode source files; here it is expressed by traits.

## The abstraction

```text
         ┌────────────────────────────────────────────────────────┐
         │   ft8   ft4   fst4   wspr   jt9   jt65   q65           │  per-protocol ZSTs
         │        (each implements Protocol + FrameLayout)         │  (feature-gated)
         └─────────────┬─────────────────┬────────────────────────┘
                       │                 │
              ┌────────▼─────────┐  ┌────▼─────────┐
              │       msg        │  │     fec      │  shared codecs
              │  Wsjt77 · Jt72   │  │ LDPC · RS    │  behind traits
              │  Wspr50  · Q65   │  │ ConvFano·QRA │
              │  · Hash table    │  │              │
              └────────┬─────────┘  └────┬─────────┘
                       │                 │
                   ┌───▼─────────────────▼───┐
                   │          core           │  Protocol trait, DSP
                   │ sync · llr · equalize · │  (resample / GFSK /
                   │  pipeline · tx · dsp    │   downsample / subtract)
                   └─────────────────────────┘
```

Each protocol declares its slot length, tone count, Gray map, Costas
/ sync pattern, FEC codec and message codec at compile time via the
`Protocol` trait. The generic code in `core` — coarse sync, fine
sync, LLR computation, LDPC / RS / convolutional decode, GFSK
synthesis — works for any type that satisfies the trait. Dispatch is
monomorphised, so the machine code is byte-identical to a hand-
written per-protocol decoder.

Adding a new protocol is a trait impl on a ZST, not a cross-cutting
refactor: FST4-60A joined the crate post-hoc without changing any
shared pipeline code.

The receive path is a chain of free functions in `core::sync` →
`core::llr` → `core::equalize` → `core::pipeline` (each generic
over `P: Protocol`), not a `Demodulator` / `Receiver` trait. See
[`docs/LIBRARY.md` §4](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.md#4-shared-primitives-core)
for the data-flow diagram.

```toml
[dependencies]
mfsk-core = { version = "0.4", features = ["ft8", "ft4"] }
```

## Attribution

Every algorithm in this crate is derived from
[WSJT-X](https://sourceforge.net/projects/wsjt/) (Joe Taylor K1JT and
collaborators). Source files cite the corresponding upstream
`lib/ft8/*`, `lib/ft4/*`, `lib/fst4/*`, `lib/wsprd/*`, `lib/jt65_*`,
`lib/jt9_*`, `lib/packjt.f90`, etc. that they port from. This is a
Rust re-implementation aimed at broadening the set of platforms
(browser / WASM, Android, embedded) that can host the decoders —
**not** a replacement for WSJT-X itself, which remains the reference
implementation.

License matches upstream: **GPL-3.0-or-later**.

## Protocols

| Protocol   | Slot   | FEC                               | Message | Sync                   | Feature |
|------------|--------|-----------------------------------|---------|------------------------|---------|
| FT8        | 15 s   | LDPC(174, 91) + CRC-14            | 77 bit  | 3 × Costas-7           | `ft8`   |
| FT4        | 7.5 s  | LDPC(174, 91) + CRC-14            | 77 bit  | 4 × Costas-4           | `ft4`   |
| FST4-60A   | 60 s   | LDPC(240, 101) + CRC-24           | 77 bit  | 5 × Costas-8           | `fst4`  |
| WSPR       | 120 s  | Convolutional r=½ K=32 + Fano     | 50 bit  | Per-symbol LSB (npr3)  | `wspr`  |
| JT9        | 60 s   | Convolutional r=½ K=32 + Fano     | 72 bit  | 16 distributed slots   | `jt9`   |
| JT65       | 60 s   | Reed-Solomon(63, 12) GF(2⁶)       | 72 bit  | 63 distributed slots   | `jt65`  |
| Q65-30A    | 30 s   | QRA(15, 65) GF(2⁶) + CRC-12       | 77 bit  | 22 distributed slots   | `q65`   |
| Q65-60A‥E  | 60 s   | (same QRA codec)                  | 77 bit  | (same sync layout)     | `q65`   |

Seven protocol families, eleven wired ZSTs in the registry: Q65 contributes one
30-s sub-mode (Q65-30A) plus five 60-s EME sub-modes (Q65-60A‥E) that share
the FEC, message codec and sync layout but differ in NSPS / tone spacing.
[`PROTOCOLS`](https://docs.rs/mfsk-core/latest/mfsk_core/static.PROTOCOLS.html)
exposes one entry per wired ZST; `uvpacket` (when enabled) adds four
more for its rate ladder.

### Static set of protocols

`PROTOCOLS` is a `const` slice — the set of supported protocols is
fixed at compile time by Cargo features. There is no runtime
`register_protocol()` API by design: every wired ZST is verified by
`tests/protocol_invariants.rs` to satisfy the trait surface, and that
guarantee can't be extended to types unknown at compile time. UI /
FFI consumers should iterate `PROTOCOLS` (or filter via `by_id` /
`by_name`) at startup; if you need a new protocol, add the ZST + a
`protocol_meta!` line and rebuild.

### Applied example: `uvpacket` (experimental)

The `uvpacket` module (`--features uvpacket`, off by default) is
an in-tree example showing the trait abstractions extend beyond
WSJT-X — a π/4-DQPSK packet protocol for NFM / SSB voice channels
that reuses the shared `Ldpc240_101` codec. **Experimental and
its public API may change** — pin to an exact version. See
[`docs/UVPACKET.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/UVPACKET.md)
([日本語](https://github.com/jl1nie/mfsk-core/blob/main/docs/UVPACKET.ja.md))
for the design narrative, modulation / equaliser / framing details
and per-mode performance characterisation.

## Modules

- `mfsk_core::core` — protocol traits, DSP (resample / downsample /
  GFSK / subtract), sync, LLR, equaliser, pipeline driver.
- `mfsk_core::fec` — `Ldpc174_91` / `Ldpc240_101` / `ConvFano` /
  `ConvFano232` / `Rs63_12` / `qra::Q65Codec` (with the
  `qra15_65_64::QRA15_65_64_IRR_E23` code instance) for Q65.
- `mfsk_core::msg` — 77-bit (`Wsjt77Message`), 72-bit (`Jt72Codec`),
  50-bit (`Wspr50Message`) and Q65 (`Q65Message`, 77-bit ↔ 13-symbol
  packing helpers) message codecs; callsign hash table.
- `mfsk_core::{ft8, ft4, fst4, wspr, jt9, jt65, q65}` — per-protocol
  ZSTs, decoders and synthesisers (each feature-gated). The `q65`
  module exposes one ZST per wired sub-mode — `Q65a30` for
  terrestrial work, plus `Q65a60` / `Q65b60` / `Q65c60` / `Q65d60` /
  `Q65e60` for EME at 6 m through 10 GHz+ — with generic
  `synthesize_standard_for<P>` / `decode_at_for<P>` / `decode_scan_for<P>`
  helpers that pick the right NSPS and tone spacing from the type
  parameter.

## Features

| Feature       | Default | What it enables                              |
|---------------|---------|----------------------------------------------|
| `ft8`         | ✓       | FT8 decode / synth                           |
| `ft4`         | ✓       | FT4 decode / synth                           |
| `fst4`        |         | FST4-60A decode / synth                      |
| `wspr`        |         | WSPR decode / synth                          |
| `jt9`         |         | JT9 decode / synth                           |
| `jt65`        |         | JT65 decode / synth (+ erasure-aware RS)     |
| `q65`         |         | Q65-30A decode / synth (QRA soft-decision)   |
| `uvpacket`    |         | Applied example *(experimental)*: NFM voice-channel packet protocol (QPSK + LDPC), reuses `Ldpc240_101` |
| `full`        |         | Aggregate of all seven WSJT protocols + uvpacket + packet-bytes |
| `parallel`    | ✓       | Rayon-parallel candidate processing          |
| `fft-rustfft` | ✓       | Default host FFT backend (`rustfft`, requires `std`) |
| `fft-extern`  |         | Pluggable FFT trait — caller binary supplies an `FftPlanner` impl (esp-dsp on ESP32-S3, CMSIS-DSP on RP2350, …) |
| `fixed-point` |         | Embedded integer pipeline: u16 spectrogram + i16 DFT + Q3i8 LLR + integer NMS BP |
| `profile-coarse` |      | Always-on coarse_sync sub-stage profiling (host has `MFSK_PROFILE_COARSE` env var alternative) |

## Quick example

```rust
use mfsk_core::ft8::{
    decode::{decode_frame, DecodeDepth},
    wave_gen::{message_to_tones, tones_to_i16},
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. Decode it back.
for r in decode_frame(&audio, 100.0, 3_000.0, 1.0, None, DecodeDepth::BpAllOsd, 50) {
    if let Some(text) = unpack77(&r.message77) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

Each protocol module documents its top-level entry points and
carries its own Quick example:

- [`mfsk_core::ft8`](https://docs.rs/mfsk-core/latest/mfsk_core/ft8/)
  — `decode_frame` + `decode_sniper_ap` (narrow-band "sniper" mode)
- [`mfsk_core::ft4`](https://docs.rs/mfsk-core/latest/mfsk_core/ft4/)
  — `decode_frame`
- [`mfsk_core::fst4`](https://docs.rs/mfsk-core/latest/mfsk_core/fst4/)
  — FST4-60A `decode_frame`
- [`mfsk_core::wspr`](https://docs.rs/mfsk-core/latest/mfsk_core/wspr/)
  — `decode::decode_scan_default`
- [`mfsk_core::jt9`](https://docs.rs/mfsk-core/latest/mfsk_core/jt9/)
  — `decode_scan_default`
- [`mfsk_core::jt65`](https://docs.rs/mfsk-core/latest/mfsk_core/jt65/)
  — `decode_scan_default` + `decode_at_with_erasures` (for low SNR)
- [`mfsk_core::q65`](https://docs.rs/mfsk-core/latest/mfsk_core/q65/)
  — `decode_scan_default` (Q65-30A); generic `decode_scan_for<P>`
  for any wired sub-mode including the Q65-60A‥E EME variants;
  `decode_scan_with_ap` / `decode_scan_with_ap_for<P>` for AP-biased
  decoding (~2 dB threshold gain when call signs are known); and
  `decode_scan_fading_for<P>` for the fast-fading metric (Gaussian
  / Lorentzian channel models) that recovers 5–8 dB on Doppler-spread
  channels — required for microwave EME at 5.7 / 10 / 24 GHz; and
  `decode_scan_with_ap_list_for<P>` (paired with `standard_qso_codewords`)
  for BP-free template matching against the full WSJT-X "AP list"
  of standard exchanges (~3 dB threshold gain when the callsign pair
  is known up-front)

## C / C++ / Kotlin

The `mfsk-ffi` sibling crate in this repository builds a
`libmfsk.{so,a,dylib}` + `mfsk.h` (via `cbindgen`) that exposes the
same decoder and synthesiser surface through an opaque-handle C ABI.
It is not published to crates.io — consumers clone this repo and run:

```
cargo build -p mfsk-ffi --release
```

See `mfsk-ffi/examples/cpp_smoke/` for an end-to-end driver test
(including multi-threaded usage) and `mfsk-ffi/examples/kotlin_jni/`
for an Android/JNI skeleton.

## Architecture & ABI reference

For a deeper look at the design — trait hierarchy with worked
examples, shared DSP / sync / LLR / pipeline primitives, the C ABI
memory model, Kotlin/Android scaffolding — see the library
reference:

<!-- Absolute URLs so the links resolve from both GitHub and the
     crates.io README renderer (which otherwise rewrites
     "docs/LIBRARY.md" to mfsk-core/docs/... — see workspace
     layout: docs/ lives at the repo root, not under the crate). -->
- **English:** [`docs/LIBRARY.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.md)
- **日本語:** [`docs/LIBRARY.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.ja.md)
- **Embedded targets:**
  [English `docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
  / [日本語 `docs/EMBEDDED.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.ja.md)
  — generic-scalar architecture (one codebase for f32 host and
  fixed-point embedded), feature-flag map, FFT / dot-product extern
  contracts, BASIS scratch placement, Q-format reference, Core2 perf
  ballpark + footprint.

## Status

`0.5.x` — API is deliberately not frozen. Breaking changes follow
cargo-style minor bumps (`0.4 → 0.5`). The 0.4.x line landed the
embedded baseline (`no_std + alloc`, pluggable FFT backend,
caller-buffer TX APIs); 0.5.0 ships the first end-to-end real-audio
embedded port — M5Stack Core2 (Xtensa LX6) decoding 3-7 FT8 results
per 14 s slot in ~3.0–4.0 s wall-clock. See
[`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
for the integration contract; `embedded-poc/m5stack-core2/` is the
working example binary.

Algorithm correctness is covered by the workspace test suite,
including end-to-end synth → decode roundtrips for every protocol,
an AWGN sensitivity sweep that confirms Q65-30A hits its
WSJT-X-published −24 dB threshold, an AP-vs-plain comparison that
shows the expected ~2 dB gain from a-priori call sign information,
an AP-list (template matching) comparison that decodes 6/6 frames at
SNR −25 dB where plain BP fails 0/6, a real 6 m EME recording (W7GJ
exchanges from the WSJT-X reference set), and a real 10 GHz EME
recording that the fast-fading metric is required to decode. The
trait surface itself is pinned by `tests/protocol_invariants.rs` —
a single generic `<P: Protocol>` checker run across every wired ZST
so a new protocol gets structural validation without bespoke glue.
Run with `--features full` for the eleven-ZST coverage; the default
features (`ft8`, `ft4`) only exercise the two default protocols.
