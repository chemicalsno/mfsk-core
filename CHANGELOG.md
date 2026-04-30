# Changelog

## 0.4.2 — documentation consistency pass

Patch release. No public-API changes; host builds (`--features full`)
are byte-identical to 0.4.1. Brings the README, crate docs, mfsk-ffi
README and `docs/LIBRARY.{md,ja.md}` back in line with the 0.4.x
reality (embedded port, Q65 family, registry semantics).

### Documentation

- README, lib.rs feature table and CHANGELOG updated to reflect the
  full feature surface: `fft-rustfft` / `fft-extern` /
  `embedded-tx` / `embedded-rx` / `esp32s3` are now listed, the
  example `Cargo.toml` snippet uses `version = "0.4"`, and the
  `Status` section references the embedded port instead of the
  retired 0.3.x baseline.
- `docs/LIBRARY.md` §4 (and the Japanese mirror) gain a single
  receive-pipeline data-flow diagram covering the
  `samples → coarse_sync → refine → symbol_spectra → equalize_local
  → compute_llr → P::Fec::decode_soft → P::Msg::unpack` chain, plus
  a paragraph spelling out *why* there is no `Demodulator` /
  `Receiver` trait (the path is realised as free functions generic
  over `P: Protocol` so monomorphisation produces per-protocol code
  identical to a hand-written decoder).
- `FecCodec` trait docstring (`mfsk-core/src/core/protocol.rs`) now
  has a "Symbol granularity" section: the trait surface is bit-level
  by contract, non-binary codes (Q65 QRA over GF(2⁶), JT65 RS over
  GF(2⁶)) pack/unpack symbols inside their own `encode`, and
  `Q65Fec::decode_soft` returns `None` by design — the real Q65
  decode runs symbol-level via `crate::fec::qra::Q65Codec` from
  `crate::q65::rx::decode_at_for`.
- README adds a "Static set of protocols" callout: `PROTOCOLS` is
  fixed at compile time by Cargo features; there is no runtime
  `register_protocol()` API by design (every wired ZST is verified
  by `tests/protocol_invariants.rs` and that guarantee can't be
  extended to types unknown at compile time).
- `mfsk-ffi/README.md` protocol table gains the missing Q65 row
  plus the dedicated `mfsk_q65_decode{,_with_ap,_fading,_with_ap_list}`
  + `mfsk_encode_q65` ABI entries that 0.2.0 already shipped.

### Tests

- `tests/protocol_invariants.rs` cross-protocol asserts tightened:
  - `every_wired_protocol_has_a_unique_protocol_id` now derives the
    expected distinct-id count imperatively from the active feature
    flags, so the `unique.len() == expected` assertion is meaningful
    under any feature combination — not just `--features full`
    where it was previously gated.
  - `registry_entries_match_zst_trait_constants` now asserts that
    the count of verified entries equals `PROTOCOLS.len()`. Adding a
    new ZST + registry entry but forgetting the matching `check!`
    line trips this count cross-check instead of silently passing,
    and uvpacket sub-modes are now covered by their own `check!`
    lines.
  - Module doc updated to call out that the per-protocol invariant
    tests are feature-gated, so `cargo test --features full` is
    required for full eleven-ZST coverage.
- `mfsk-core/src/lib.rs` "Trait surface verification" section now
  reports the actual ~25 invariants split across modulation /
  frame-layout / codec layers (was "17") and notes the default
  `cargo test` only exercises FT8 + FT4.

### crates.io metadata

- `mfsk-core/Cargo.toml` `description` rewritten to mention the
  embedded-port story (`no_std + alloc`, pluggable FFT, ESP32-S3
  PoC) so the crates.io listing card matches the README.
- `categories` extended with `embedded` and `no-std` (now 5 of 5
  slots used).

### CI

- `feature-matrix` job adds `q65`, `uvpacket`, `embedded-tx` and
  `embedded-rx` rows. The embedded entries build the library
  no_std + alloc on the Linux host; the standalone `embedded-poc/`
  Xtensa binaries remain excluded from CI (PoC scope).

## 0.4.1 — embedded port (no_std + alloc, FFT trait, ESP32-S3 PoC)

Adds an embedded-target port without breaking the existing host
API. Host builds (`--features full`) are byte-identical to 0.4.0.

### What's new

- **`no_std + alloc` builds work end-to-end.** Default features
  still pull `std` so existing users see no behaviour change; new
  presets `embedded-tx` (TX synthesis only) and `embedded-rx`
  (full decode pipeline, requires caller-supplied FFT) build with
  `--no-default-features` against `xtensa-esp32s3-espidf`,
  `thumbv8m.main-none-eabihf`, etc.
- **Pluggable FFT backend via `mfsk_core::core::fft`.** New
  `Fft` / `FftPlanner` trait pair; the rustfft path stays the host
  default, embedded callers plug in their own impl through the
  `fft-extern` feature + an `extern "Rust"` factory function.
- **Caller-buffer TX APIs.** `*_into(out, …)` variants for FT8 /
  FT4 / WSPR / uvpacket synthesisers + `*_OUTPUT_LEN` constants
  let embedded callers drive I2S DMA buffers without per-burst
  `Vec` allocations. Vec-returning convenience wrappers preserved.
- **ESP32-S3 PoC binary** at `embedded-poc/esp32s3/` (excluded
  from the host workspace; uses `+esp` toolchain). Wires
  `mfsk-core --features fft-extern` to esp-dsp's hand-written
  Xtensa FFT (`dsps_fft2r_fc32_ae32_`) via `esp-idf-sys`'s
  managed-component pipeline. Validates the embedded port
  builds-and-links on real hardware.

### Workarounds bundled

- **Xtensa LLVM 19.1.2 codegen bug**: `if cond { 0.5_f32 }
  else { 1.0_f32 }` triggers `XtensaISD::PCREL_WRAPPER`
  instruction-selection SIGSEGV. `mfsk_core::ft8::decode` and
  `mfsk_core::core::pipeline` rewrite the gain calculation as
  `1.0 - 0.5 * (cond as u32 as f32)` (functionally identical;
  PER-sweep tests unchanged).

### New / changed features

| Feature | Default | Notes |
|---|:---:|---|
| `std` | ✓ | Already-on for host builds; bundles `alloc`. |
| `alloc` |   | Bare `no_std + alloc`. |
| `embedded-tx` |   | `alloc + ft8 + ft4 + wspr` (synth-only). |
| `embedded-rx` |   | `embedded-tx + fft-extern` (decode-capable). |
| `esp32s3` |   | Alias for `embedded-rx`. |
| `fft-rustfft` | ✓ | Host default; pulls `rustfft`. |
| `fft-extern` |   | Caller supplies `mfsk_core_make_default_fft_planner`. |
| `parallel` | ✓ | Now requires `std` (rayon is std-only). |

### Implementation notes

- `num-complex` and `crc` switched to `default-features = false`;
  `num-traits = "0.2", features = ["libm"]` added so call sites
  can `use num_traits::Float` under no_std.
- `std::*` references in the decode-side modules replaced with
  `core::*` / `alloc::*` equivalents. `std::collections::HashMap`
  in `msg::hash_table` swapped for `alloc::collections::BTreeMap`
  (small LRU bounded at 1000 entries; O(log n) lookups dwarfed
  by surrounding LDPC cost).
- `core::dsp::{downsample, subtract}`, `core::{sync, llr,
  pipeline}`, `wspr::{rx, spectrogram}`, etc. moved to the FFT
  trait via `core::fft::default_planner()`.

### Verified

- `cargo test --features full --release -- --include-ignored`
  passes (262 tests + the PER sweep cells unchanged).
- `cargo +esp build --target xtensa-esp32s3-espidf
   --no-default-features --features esp32s3 -Zbuild-std=core,alloc` ✓
- `cargo build --target thumbv8m.main-none-eabihf
   --no-default-features --features esp32s3` ✓
- ESP32-S3 PoC links the esp-dsp ASM FFT through the trait,
  ELF ~1.5 MB total / ~440 KB code.

## 0.4.0 — Q65 + abstraction unification

First release on the 0.4 line; cumulative since the 0.2.1 crates.io
publish. The headline is **the WSJT-family API surface**: a new
protocol (Q65-30A), trait-level cleanups that close abstraction
leaks the multi-protocol port surfaced, and a registry that gives
every protocol a uniform metadata view. The in-tree `uvpacket`
applied-example module is also rebuilt end-to-end (separate
section below) but it is gated behind `--features uvpacket` and
not part of the default-features API.

### WSJT-family additions (BREAKING vs 0.2.1)

- **Q65-30A** — full decode / encode / synthesis port from WSJT-X,
  including fast-fading log-likelihoods and AP-list handling. New
  `Q65a30` re-export, `--features q65`. (Cumulative across 0.3.x.)
- **`MessageCodec::verify_info`** — CRC verification lifted out of
  the LDPC layer into the message-codec trait, so the FEC code no
  longer has hard-coded knowledge of CRC-24 vs CRC-14 dispatch.
  Required because the same `Ldpc240_101` mother code is now
  shared across FST4 (CRC-24, 77-bit msg) and Q65 (CRC-14, 91-bit
  msg) and `uvpacket` (CRC-16, 96-bit raw bytes).
- **`Ldpc240_101` family unified** — single LDPC implementation
  used by FST4, Q65, and uvpacket (previously each had its own
  copy with subtle constant divergence).
- **`ProtocolMeta` registry** — every `Protocol` impl exposes a
  uniform metadata block (band rate, Costas pattern length,
  symbol count, …). Cross-protocol invariant tests assert the
  registry stays internally consistent (`tests/protocol_invariants.rs`).
- **`PacketBytesMessage`** — variable-length-bytes message codec,
  exposed as `--features packet-bytes`. Used as the byte-pipe
  building block for callers that want LDPC + interleaver + sync
  but do not need WSJT-77's structured-message dispatch.
- **`mfsk_core::VERSION`** — crate version constant, useful for
  FFI / WASM consumers verifying which build they linked against.

The trait reshuffle is the breaking part: `MessageCodec` impls
that were closed against `mfsk-core ≤ 0.2.1` need to add the new
`verify_info` method. Default implementations cover the common
"length-then-CRC" cases.

### `uvpacket` applied example (gated, redesigned)

`uvpacket` is an in-tree example of how the abstractions handle a
non-WSJT mode (3 kHz NFM / SSB voice-channel packet protocol).
**Breaking changes within `--features uvpacket` are expected
within the 0.4.x line** — pin the exact patch version if you depend
on it. ABI consolidation will follow in a future release.

The 0.4 redesign replaced the 0.3.x coherent-QPSK pipeline (which
failed over-the-air despite passing AWGN bench) with a
single-carrier **π/4-shifted DQPSK** modem at 1200 / 600 baud:

- 127-chip BPSK m-sequence preamble, **four primitive-polynomial
  variants** (one per `Mode`). Sync identifies the time offset
  and the payload mode in one matched-filter pass per centre.
- **9-tap T-spaced LMS equaliser** trained closed-form on the
  preamble. Differential demod is invariant to constant phase
  rotation and tolerates LO walk / clarifier offset to the AFC
  search-range limit; no pilots needed.
- **Dedicated header LDPC block** (Robust, unpunctured) carries
  `(block_count, app_type, sequence)` + CRC-16. Receiver decodes
  the header first (1 LDPC), reads `n_blocks`, then decodes the
  payload (`n_blocks` LDPCs). Total `1 + n_blocks` LDPC decodes
  per frame, vs ≤ 128 brute-force before.
- **AFC** at sync time, ±200 Hz default; callers widen for
  harsher channels.
- **UltraRobust** mode (header_code 0): half-baud (600 Hz)
  variant of Robust for marathon QSL on weak SSB / V-UHF mountain
  paths. ~4 dB tougher than Robust on every fading channel
  measured (Rayleigh, SSB realistic, FM realistic), see the
  positioning matrix in `docs/UVPACKET.md` §3.1.
- **WSJT-X-compatible SNR reporting** on every decoded frame
  (`DecodedFrame.snr_db`, dB / 2.5 kHz reference, −30 dB floor).
  Per-mode calibrated to ±0.3 dB residual against AWGN truth.
- **Shared-pair preamble correlator** — auto-detect path shares
  the differential pair products `aᵢ = mf[k]·conj(mf[k-1])`
  across the 3 NSPS_BASE preambles, ~36 % per-offset reduction at
  K=3. Bit-identical PER vs the per-preamble form (verified via
  `tests/uvpacket_per_modes_sweep`).

Removed from the 0.3 uvpacket:
- All coherent-QPSK encode / decode entry points; pilot symbols
  and the LMS phase tracker.
- 31-chip preamble + spread-header indirection (replaced by the
  4-variant 127-chip preamble + dedicated header block).
- Brute-force `(mode × n_blocks)` layout sweep.
- `framing::pack` / `framing::unpack` (replaced by `pack_header` /
  `unpack_header`; mode field removed from the header word).
- `UvFast` mode (header_code 2 ≤ 0.3.5); replaced by `UvUltraRobust`.

### Performance characterisation

PER thresholds (90 %, Eb/N0_info / SNR_2.5kHz dB) for the four
uvpacket modes on the channel models in
`mfsk-core/tests/common/air_channel.rs`:

| Mode (net bps) | AWGN | Rayleigh fd=5 | SSB realistic | FM realistic | Multipath 3-tap |
|---|---:|---:|---:|---:|---:|
| **UltraRobust** (504) | +4 / −3.7 | +8 / +0.3 | +4 / −3.7 | +6 / −1.7 | +6 / −1.7 |
| Robust (1008) | +6 / +1.3 | +12 / +7.3 | +8 / +3.3 | +10 / +5.3 | +8 / +3.3 |
| Standard (1200) | +8 / +4.0 | +12 / +8.0 | +8 / +4.0 | +10 / +6.0 | +10 / +6.0 |
| Express (1800) | +10 / +7.8 | +20 / +17.8 | >+15 / >+12.8 | +20 / +17.8 | fail |

Reproduce via `cargo test --release --features uvpacket --test
uvpacket_per_modes_sweep -- --ignored --nocapture`.

## 0.3.5 (continued) — 2026-04-29

uvpacket sync detector rewrite — replaces `|⟨preamble, mf_out⟩|²` as
the per-offset score with the **normalised coherence ratio**
`|⟨preamble, mf_out⟩|² / Σ|sᵢ|²`, fixing a structural false-sync
class that the 0.3.4 / 0.3.5 sync gate band-aids couldn't reach.

(In-place 0.3.5 update — no version bump to keep the published-crate
history clean. The earlier 0.3.5 entry below describes the
non-zero-median fix that this commit completes.)

### Background

The old detector summed `±sᵢ` for the 31 BPSK preamble bits and used
`|sum|²` as the match score. By Cauchy-Schwarz that magnitude is
bounded by `N·Σ|sᵢ|²`, but the bound is reached **only** when `sᵢ ∝
b̄ᵢ` for all i (the actual coherent-preamble signature). For a single
dominant sample (microphone click, USB plug-event, fan tick, …) the
sum is nearly as large as if the whole preamble had aligned, yet
*only one* sample contributed coherently. The old detector saw
"large magnitude" and accepted; the LDPC sweep then ran on noise.

uvpacket-web field reports showed `max/median = 139` from a single
field-amplitude impulse, vs `≤ 17` for proper noise. New direct
measurement: an isolated single-sample spike of 0.5 amplitude in
30 k samples of noise gives `max/median = 2209` under the old
detector — false sync every snapshot in environments with any
impulsive interference.

### Fixed

- New `preamble_coherence_score(mf_out, offset) -> f32` returns the
  normalised ratio. Bounded above by `PREAMBLE_LEN = 31`; saturates
  at 31 for a coherent BPSK preamble; collapses to ~1 for any
  single-sample dominance or random uncorrelated content.
- `rx::decode` and `rx::diag_sync_stats` now generate scores via
  `preamble_coherence_score` instead of `preamble_correlation(...).
  norm_sqr()`. The downstream `SYNC_PEAK_REL_TO_MEDIAN = 20×` gate
  and the threshold-relative-NMS peak picking are unchanged — only
  the *scoring metric* changed.

### Empirical (release, 30 000-sample buffers)

| scenario               | old detector | new detector |
|------------------------|--------------|--------------|
| pure white noise       | 13.5         | 12.3         |
| 1500 Hz tone           | 6.3          | 6.2          |
| 1200 Hz tone           | 2.5          | 2.8          |
| **noise + 0.5 click**  | **2 209**    | **10.6**     |
| AM(1500 Hz, 1200 Hz)   | 8.4          | 8.9          |
| strong tone @ 1500 Hz  | 6.2          | 7.1          |
| **real preamble +10 dB**| (varies)    | **46.5**     |

The impulse case dropped from 2 209 to 10.6 (well below the 20×
gate); the real-preamble case climbed to 46.5 (well above). Clean
separation, while every other point on the table is roughly
unchanged. All 271 existing uvpacket tests pass byte-identically —
the metric is mathematically equivalent for actual preambles.

### Roadmap note

A longer preamble (127 or 255 bits) would push the real-signal
saturation ratio higher (linear in `N`) without affecting the
noise floor, giving more headroom. That's a wire-format break and
deferred for now; the 31-bit + coherence-score combination already
restores the gate's intended noise rejection.

## 0.3.5 — 2026-04-29

uvpacket sync-gate hardening: 0.3.4's `max/median ≥ 20` rejection
collapsed to a no-op when the input buffer was partially zero (e.g.
the first few seconds of a fresh ring-buffer capture in uvpacket-web,
where the unfilled portion of the worklet's ring buffer was being
returned as zeros). With > 50 % of correlation scores at exactly 0,
`median(scores) = 0` and the defensive `if median <= 0 { return true }`
branch let noise through to the LDPC sweep — the very runaway 0.3.4
was supposed to fix.

### Fixed

- `global_max_is_sync_outlier` and `diag_sync_stats` now compute the
  median over **non-zero scores only**. An all-zero buffer (no audio
  at all) trivially rejects; a partially-zero buffer (e.g. ring-buffer
  pre-fill) produces a meaningful median from the real-audio portion.
- Adds `tests/uvpacket_noise_floor.rs::noise_floor_half_zero_buffer`
  as a regression test (7 s buffer, first half zeros, second half
  σ=0.003 noise). Confirmed: 0 frames, 2.3 ms decode.

No behaviour change for buffers without zero-padding artefacts (all
271 existing uvpacket tests still pass byte-identically).

## 0.3.4 — 2026-04-29

uvpacket RX: hard sync-rejection on the auto-detect path, fixing a
runaway-CPU bug discovered by uvpacket-web (https://jl1nie.github.io/webft8/uvpacket/)
under steady-state listening on noise-only audio.

### Fixed

- `uvpacket::rx::decode` and `uvpacket::rx::decode_multichannel` now
  short-circuit when the global preamble-correlation peak is not a
  clear outlier from the score-distribution median (≥ `20×` median).

  On pure χ²(2)-distributed noise the natural `max/median` ratio
  saturates around `ln(N)/ln(2) ≈ 17` (extreme-value statistics over
  `N ≈ 80 k` correlation offsets in a 7 s buffer); on real signal at
  +1 dB Eb/N0_info — Robust mode's 50 %-PER threshold — the ratio
  is `≈ 56`. The 20× gate cleanly separates them with a 4.5 dB
  signal-side margin (rejection at `−3.5 dB SNR`, well below any
  rate's actual decoding threshold).

  Without the gate, the 50 % relative-peak threshold left ~290 false
  NMS-survived peaks per 7 s noise buffer, each running a
  `4 modes × 32 n_blocks` LDPC BP+OSD-2 sweep — empirically 30–180 s
  of release-mode work per call. With the gate, a noise buffer
  short-circuits in `~330 µs` (≈ 7 000× speedup; new test
  `tests/uvpacket_noise_floor.rs`).

  No behaviour change for real signals — all 271 existing uvpacket
  tests still pass byte-identically.

## 0.3.3 — 2026-04-29

Multi-channel SSB receive + slotted-ALOHA TX primitives for
uvpacket. The 0.3.2 single-station SSB experience generalises
to a private group sharing one RF channel (e.g., 430.090 MHz
USB) where each TX picks a random free audio slot via LBT.

WSJT-family modes and the existing single-channel uvpacket API
are unchanged.

### Added

- `mfsk-core::uvpacket::rx::decode_multichannel(audio,
  &mc_opts, &fec_opts) -> Vec<(f32, DecodedFrame)>` — coarse-
  grid frequency sweep across the configured SSB passband,
  per-grid-point matched filter + preamble peak detection,
  frequency-axis NMS to drop adjacent-grid duplicates, and
  per-peak `(mode × n_blocks)` decode. Returns the detected
  audio centre alongside each decoded frame.
- `MultiChannelOpts { band_lo_hz, band_hi_hz, coarse_step_hz,
  nms_radius_hz, peak_rel_threshold }` with sensible defaults
  (300–2700 Hz / 25 Hz / 600 Hz / 0.5).
- `mfsk-core::uvpacket::rx::measure_slot_energies(audio,
  &mc_opts, slot_spacing_hz) -> Vec<SlotEnergy>` — per-slot
  mean matched-filter magnitude survey for the LBT step before
  a slotted-ALOHA TX. Policy-free: the helper just reports
  energies, the caller picks free-vs-busy by their own rule.
- `SlotEnergy { audio_centre_hz, mean_mf_magnitude }`.

### Operating concept

A private group shares one RF SSB channel. Inside the audio
passband the modem recognises a 1200 Hz slot grid (typically
800 Hz and 2000 Hz centres in 2.4 kHz SSB). Each TX:

1. Listens — captures a short audio buffer, runs
   `measure_slot_energies` to survey occupancy.
2. Picks a random free slot — uniform-random from the slots
   below an application-chosen energy threshold.
3. Transmits — `tx::encode(&header, &payload, picked_centre)`.

This is **slotted ALOHA on the audio-frequency axis**, plus
LBT. CSMA/CD proper isn't applicable to half-duplex SSB radio;
slotted ALOHA + LBT + ARQ at the application layer behaves
equivalently with much less mechanism, and lines up with the
natural amateur-radio "watch the frequency, find a clear spot,
transmit" practice.

mfsk-core supplies the primitives only; the application layer
owns the RNG, the ARQ ACK + retry state machine, and any
voice-mode coexistence policy.

### Cost

`decode_multichannel`: ~1 matched-filter pass per coarse-grid
step. With default settings (300–2700 Hz, 25 Hz step) ≈ 96
passes ≈ 70 ms in release per second of audio.
`measure_slot_energies`: 1 MF pass per slot, ~1 ms each at 1
sec audio. Effectively free.

### Empirical

- 2 simultaneous frames at 800 Hz / 2000 Hz centres in clean
  audio: both decoded with detected centres within ±50 Hz of
  truth.
- Same setup at +8 dB Eb/N0_info AWGN: both decoded.
- Slot survey with one busy slot at 800 Hz: busy slot's mean
  MF magnitude is > 5× the free slot's.

## 0.3.2 — 2026-04-29

Focused single-feature release on top of 0.3.1: **AFC (automatic
frequency control) for uvpacket** so the modem operates correctly
on SSB carriers without requiring TX/RX VFO-dial alignment.

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) and the
0.3.1-shipped uvpacket NFM path are unchanged. No breaking API
changes — AFC is opt-in via a new entry-point function.

### Added

- `mfsk-core::uvpacket::rx::decode_known_layout_with_afc(audio,
  sample_offset, audio_centre_hz, mode, n_blocks, &fec_opts,
  &afc_opts) -> Result<DecodedFrame, DecodeError>`. Runs the AFC
  search, then re-invokes the standard decoder at the corrected
  centre frequency.
- `mfsk-core::uvpacket::rx::AfcOpts { search_hz: f32 }` with
  `Default` returning `AfcOpts { search_hz: 200.0 }`. The total
  search window is `±search_hz`; 200 Hz covers typical SSB VFO
  mismatch worst-case.
- `pub fn diag_estimate_freq_offset` — test/characterisation hook
  that returns the AFC's Δf estimate without running the full
  decode roundtrip.
- `tests/uvpacket_afc.rs` — round-trip clean recover at ±150 Hz,
  baseline-fails-at-offset control, ±100 Hz at +6 dB AWGN
  (10/10), optional accuracy-print diagnostic.

### Algorithm

Frequency-grid preamble-correlation search at 25 Hz steps across
`[−search_hz, +search_hz]` (default 17 candidates). At each
candidate `audio_centre_hz + Δf_test`, run the matched filter and
take the best preamble-correlation magnitude over the ±NSPS
jitter window. Pick the coarse-grid winner, then parabolic-fit
the three adjacent magnitudes for sub-grid resolution. Re-run
the standard decoder at the corrected centre frequency.

The first attempt was an FFT-over-chip-rate-samples (cheap but
wrong): at non-trivial Δf the integer-sample preamble correlator
that picks `best_off` itself rolls off as `sinc(δ · 31 / 1200)`,
landing on noise samples for `|δ| ≳ 20 Hz` — the FFT then
operates on garbage. The frequency-grid search sidesteps this
because the preamble correlator magnitude itself peaks at the
correct Δf.

### Cost

~17× single-decode cost (full down-convert + matched-filter at
each grid point), ~50–100 ms total per attempted decode in
release mode. Tolerable for opportunistic SSB decode; can be
tightened by lazy-evaluating only enough grid points to
distinguish the winner from its neighbours, if profiling demands.

### Empirical accuracy

Clean-channel AFC estimate vs injected truth (search ±200 Hz):

```
Δf_true (Hz)  AFC_est (Hz)  decode
−150          −150.00       ✓
−100          −100.00       ✓
 −50           −49.99       ✓
 −20           −22.34       ✓ (mid-grid; LMS absorbs residual)
   0             0.00       ✓
 +20           +22.34       ✓
 +50           +50.00       ✓
+100          +100.00       ✓
+150          +150.00       ✓
+200          +200.00       ✓
```

≤ 0.01 Hz error at multi-of-25 Hz Δf; ≤ 2.5 Hz error at mid-grid
Δf. The LMS quadratic phase fit downstream absorbs the residual
without trouble (residual frequency offset over a 0.5 s burst is
within the LMS linear-term capacity).

### Operating envelope

With AFC, uvpacket decodes correctly across the full SSB VFO-
mismatch range (±200 Hz default; configurable). Combined with
the existing modem characterisation, this opens the modem up to
HF SSB weak-signal data and microwave SSB applications.

NFM users can keep using `decode_known_layout` — AFC is an extra
~50–100 ms per decode that's pure overhead on a static-VFO
channel.

### Known limitations

- AFC is per-frame static. Doppler-induced carrier drift across
  the burst is still absorbed by the LMS phase fit (constant +
  linear + quadratic), which works for ≤ ~10 Hz/s drift —
  typical for HF / VHF / UHF SSB.
- The auto-detect `decode()` path doesn't yet take an `AfcOpts`.
  Multi-frame SSB scans go through `decode_known_layout_with_afc`
  with caller-managed framing for now.

## 0.3.1 — 2026-04-29

Additive release on top of 0.3.0. Headline: the new `uvpacket`
applied-example module — a coherent QPSK + LDPC packet protocol
that fits inside an NFM voice passband (or SSB) and reuses the
WSJT FST4 LDPC mother code. Built end-to-end through a
modulation-pivot mid-cycle (initial 4-GFSK design failed
orthogonality at h=0.5, replaced by single-carrier QPSK + RRC +
m-sequence preamble + pilot phase tracking).

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) are
**unchanged** in this release. No breaking API changes.

### Added

- `mfsk-core::uvpacket` module (feature-gated `uvpacket`, off by
  default). Four-mode rate ladder (Robust/Standard/Fast/Express,
  1008–1800 net bps) with kSR-greedy puncture-set selection
  (Ha–McLaughlin) on the `Ldpc240_101` mother code's parity bits.
  Byte-pipe API (`app_type` 4-bit dispatch tag); bypasses the
  generic `MessageCodec` to fit non-WSJT use cases.
- TX (`uvpacket::tx::encode`): 31-bit BPSK m-sequence preamble
  → QPSK Gray-mapped data + pilots every 32 sym → RRC pulse
  shaping (α=0.5, span 6) → upconvert to 1500 Hz audio centre at
  12 kHz sample rate.
- RX (`uvpacket::rx::decode_known_layout` /
  `decode_known_layout_with_opts` / `decode`): 2× downconvert →
  matched filter → preamble correlation with parabolic sub-sample
  timing recovery → weighted LMS quadratic phase fit over all
  anchors (preamble centre + pilots) → magnitude-based σ²_n
  estimator from data symbols → σ-aware QPSK soft demap →
  per-LDPC-block decision-directed phase correction → BP+OSD-2
  decode (override via `&FecOpts` for OSD-3 etc.).
- AWGN + Rayleigh-flat-fading harness in
  `tests/common/channel.rs`. `awgn_sigma_for_eb_n0_info` now
  takes per-burst measured `signal_power` for cross-modulation-
  comparable Eb/N0_info numbers.
- Diagnostic test suites: `tests/uvpacket_ldpc_direct.rs`
  (modem-bypassed LDPC threshold sweep) and
  `tests/uvpacket_modem_diag.rs` (TX power audit, rx estimator
  audit, demod-only BER vs theory sweep).
- `mfsk-core/examples/uvpacket_samples.rs` — generates
  representative WAV files at `audio_samples/uvpacket/`
  (clean / +8 / +4 / +2 dB AWGN / 5 Hz Rayleigh / Express clean).
- `docs/UVPACKET.md` + `docs/UVPACKET.ja.md` — design narrative,
  characterisation tables (LDPC ceiling vs end-to-end), SNR
  calibration history, FM-threshold-margin analysis, SSB
  compatibility note.
- `docs/RELEASE_NOTES_0.3.1.md`.

### Characterisation

All numbers are Eb/N0 per **information bit** (WSJT cross-mode-
fair convention).

- **AWGN, 50 % PER**: Robust +1 dB, Standard / Fast +2 dB,
  Express +3 dB. 100 % PER at +4 dB across all modes.
- **AWGN, LDPC-only ceiling** (modem-bypassed): Robust 50 % PER
  at +0.5 dB, Express at +1.5 dB. End-to-end gap (modem
  implementation loss): 0.5–2 dB depending on mode.
- **Rayleigh, ≥ 90 % PER** (4-block, 20-byte payload): Robust at
  +10 dB / 5–10 Hz Doppler, +12 dB at 1 Hz; the higher-rate modes
  mostly +10 dB across.
- **Operating envelope**: Robust at −3.7 dB SNR_3kHz vs the NFM
  FM-threshold floor at ~+20 dB SNR_3kHz → ~24 dB margin. The
  channel CNR floor binds before the modem on NFM. On SSB the
  modem operates to its true threshold.

### Known limitations

- No automatic frequency control yet. SSB use requires both ends
  to agree on `audio_centre_hz` to within ~10 Hz. AFC is planned
  for a follow-up cycle.
- `Protocol::ID = ProtocolId::UvPacket` and several
  `ModulationParams` trait constants are decorative for uvpacket
  (the module bypasses the generic mfsk-core TX/RX pipeline).
  Documented as a scope-boundary trade-off in
  `mfsk-core/src/uvpacket/protocol.rs` and `docs/LIBRARY.md`
  §10.1 rather than spinning uvpacket out as a sibling crate.

## 0.3.0 — 2026-04-29

Internal cleanup release that closes long-standing abstraction
leaks in the FEC / message / pipeline layers, opens the door for
non-Wsjt77 message codecs, and lifts `coarse_sync` to handle
multi-frame chained signals at the same audio centre.

The 0.3.0-cycle uvpacket protocol prototype was developed and
abandoned within this cycle (an honest airtime comparison vs.
AFSK 1200 / AX.25 invalidated the original "drop-in" pitch); the
redesign for 0.3.1 is captured in `docs/0.3.1_PLAN.md`.

### Added

- `fec::ldpc::params::LdpcParams` sealed trait + generic
  `bp_decode_generic<P>` / `osd_decode_generic<P>` /
  `ldpc_encode_generic<P>`. Both `Ldpc174_91` and `Ldpc240_101`
  collapse onto the same algorithm code; ~600 lines of duplicate
  BP / OSD in `fec/ldpc240_101/{bp,osd}.rs` deleted.
- `MessageCodec::verify_info(&[u8]) -> bool` — message-level
  integrity verification hook. `Wsjt77Message` overrides to
  delegate to `check_crc14` / `check_crc24` (length-dispatched
  between K=91 and K=101). Future codecs with bespoke or no
  integrity field can opt out by overriding the default
  unconditional accept.
- `WsjtApCompatible` sealed marker trait on the AP module —
  `process_candidate_ap` / `decode_sniper_ap` / `ap_bits_for` only
  accept message codecs whose 77-bit field matches the Wsjt77
  layout. Codecs with different layouts (e.g. byte-oriented
  packet codecs) fail to compile against the AP path, surfacing
  the constraint at the type level instead of as a runtime panic.
- `PacketBytesMessage` byte-payload `MessageCodec` worked example
  (4-bit length + 80-bit payload + 7-bit CRC-7 in 91 info bits —
  the K of `Ldpc174_91`). Demonstrates that the trait
  accommodates byte-oriented protocols alongside the WSJT-77
  callsign-packing flavour. Gated on the new `packet-bytes`
  Cargo feature; not used by any wired protocol in 0.3.0.

### Changed

- `core::sync::coarse_sync` now emits multiple Costas peaks per
  frequency bin via greedy non-maximum suppression with ±MLAG
  spacing (cap 8 / bin). Strict superset of the previous
  one-or-two-peaks-per-bin behaviour: slot-based protocols
  (FT8/FT4/WSPR/JT9/JT65/Q65) keep byte-identical output because
  the second-best lag falls below `sync_min` after the
  noise-floor normalisation. Chained-frame protocols (multiple
  frames at the same audio centre, separated only in time) gain
  multi-frame discovery in a single pipeline pass.
- `pipeline::encode_tones_for_snr` drops its local `crc14` /
  `crc24` reconstruction and feeds `FecResult.info` straight back
  into `fec.encode`. The verifier-acceptance invariant guarantees
  this is bit-identical to the previous "extract msg77, recompute
  CRC, encode" path. Same simplification in
  `pipeline_ap::finalise_result`.
- `DecodeResult.message77: [u8; 77]` becomes `info: Box<[u8]>`
  carrying the FEC's full K information bits. The legacy 77-bit
  field survives as `DecodeResult::message77()` accessor for
  Wsjt77-family ergonomics that mfsk-ffi and the FT4 / FST4
  doctests rely on.
- `Q65Codec::decode` becomes CRC-agnostic: the trailing CRC-12
  check moves up to `Q65Message::verify_info`, mirroring the
  same shape as the LDPC families. `Q65DecodeError::CrcMismatch`
  is retained as a type but no longer produced internally.

### Removed

- The 0.3.0-cycle uvpacket protocol prototype (UvPacket150 / 300 /
  600 / 1200, the `mfsk-core/src/uvpacket/` module, the
  `docs/UVPACKET.{md,ja.md}` deep-dive, the
  `tests/uvpacket_roundtrip.rs` integration test). The
  redesign is in `docs/0.3.1_PLAN.md`.
- `ProtocolId::UvPacket` enum variant (will be reintroduced in
  0.3.1 with the new sub-mode tags).
- The `uvpacket` Cargo feature (its byte-codec content moved to
  the new `packet-bytes` feature).

### Internal

- `LIBRARY.{md,ja.md}` §11 motivating-example section reverted
  with the rest of the prototype.

## 0.2.1 — 2026-04-26

Patch release with no code changes — README hot-fix only.

The 0.2.0 README's `docs/LIBRARY.{md,ja.md}` links resolved to
`github.com/jl1nie/mfsk-core/blob/HEAD/mfsk-core/docs/LIBRARY.*`
when crates.io rendered the README, which is 404 because `docs/`
lives at the workspace root (the crate's
`readme = "../README.md"` pulls the workspace README in). Switched
both links to absolute `https://github.com/.../blob/main/docs/...`
URLs so they resolve from both crates.io and direct GitHub viewing.

## 0.2.0 — 2026-04-26

The Q65 wave: complete the WSJT-X Q65 family (terrestrial Q65-30A
plus EME Q65-60A‥E), expose all four Q65 decoder strategies through
the C ABI, and validate the trait surface end-to-end with a generic
checker plus a runtime registry. ~330 tests across the workspace
(up from ~230 at 0.1.0).

### Added — Q65 weak-signal decoder family complete

- `fec::qra::fast_fading` + `fading_tables` modules port
  `q65_intrinsics_fastfading`, `q65_esnodb_fastfading`,
  `fadengauss.c` and `fadenlorentz.c` from WSJT-X. Decodes the
  10 GHz EME reference recording (60D, VK7MO ↔ K6QPV) where the
  AWGN Bessel front end fails.
- `q65::ap_list::standard_qso_codewords` + `Q65Codec::decode_with_codeword_list`
  port `q65_decode_fullaplist` and `q65_set_list.f90` (the WSJT-X
  206-codeword "full AP list"). At SNR −25 dB (1 dB below the
  published Q65-30A threshold), AP-list decodes 6/6 frames where
  plain BP fails 0/6.
- New entry-point families in `q65::rx`, generic over the sub-mode
  ZST: `decode_at_fading_for<P>` / `decode_scan_fading_for<P>` and
  `decode_at_with_ap_list_for<P>` / `decode_scan_with_ap_list_for<P>`.

### Added — Q65 reaches C/C++/Kotlin via `mfsk-ffi`

- New `MfskProtocol::Q65a30 = 6` enum variant routes Q65-30A through
  the generic-handle path (`mfsk_decoder_new` + `mfsk_decode_f32`).
- Dedicated `mfsk_q65_decode{,_with_ap,_fading,_with_ap_list}`
  function family takes a `MfskQ65SubMode` parameter
  (`A30 / A60 / B60 / C60 / D60 / E60`) and reaches every sub-mode
  with every decoder strategy. New `MfskQ65FadingModel` enum
  (`Gaussian / Lorentzian`) for the fast-fading entry point.
- `mfsk_encode_q65` synthesises any sub-mode from
  `(call1, call2, grid_or_report)`.
- `mfsk-ffi` remains `publish = false` — consumers clone the
  workspace and `cargo build -p mfsk-ffi`.

### Added — Trait surface verified end-to-end

- New `mfsk_core::PROTOCOLS` static + `ProtocolMeta` struct +
  `by_id` / `by_name` / `for_protocol_id` lookup helpers
  (`mfsk-core/src/registry.rs`). Lets UI layers and FFI bridges
  enumerate the wired protocols at runtime; all six Q65 sub-modes
  appear as distinct entries (different NSPS / tone spacing) sharing
  `ProtocolId::Q65`.
- New `tests/protocol_invariants.rs` runs a single generic
  `assert_protocol_invariants::<P: Protocol>` against every wired
  ZST (FT8, FT4, FST4, WSPR, JT9, JT65, plus all six Q65 sub-modes
  — 11 in total) checking 17 trait-level invariants per ZST.
  Cross-checks every `PROTOCOLS` entry against its ZST through a
  separate code path so registry typos are caught.

### Changed

- `ModulationParams::GRAY_MAP` doc contract loosened from
  `len() == NTONES` to `len() ∈ [2^BITS_PER_SYMBOL, NTONES]` to
  match the actual range across protocols (JT9 trims its map to
  the 8 data tones; JT65 / Q65 extend with identity over the sync
  slots). Surfaced by the new invariants test.
- README + `docs/LIBRARY.{md,ja.md}` extended with new sections on
  Q65 decoder-strategy selection (when to use AWGN vs AP vs
  fast-fading vs AP-list) and on the runtime registry / invariants
  test.

### CI

- Heavy synthetic SNR / AP / fast-fading sweeps gated with
  `#[ignore = "slow: ..."]`; local `cargo test` skips them in
  debug mode (10+ min → seconds), CI runs them in release mode via
  `--include-ignored` (~10 s total).

## 0.1.0 — 2026-04-19

Initial release. Consolidates nine previously-separate workspace
crates from the `jl1nie/webft8` project into a single `mfsk-core`
crate with feature-gated protocol modules:

- `mfsk-core`, `mfsk-fec`, `mfsk-msg` → `core`, `fec`, `msg` modules
- `ft8-core`, `ft4-core`, `fst4-core`, `wspr-core`, `jt9-core`,
  `jt65-core` → per-protocol modules behind features of the same
  name

Features shipped at 0.1.0:

- FT8 (15 s, 8-GFSK, LDPC(174, 91))
- FT4 (7.5 s, 4-GFSK, LDPC(174, 91))
- FST4-60A (60 s, 4-GFSK, LDPC(240, 101))
- WSPR (120 s, 4-FSK, convolutional r=½ K=32 + Fano, incl. Type 1/2/3)
- JT9 (60 s, 9-FSK, convolutional r=½ K=32 + Fano)
- JT65 (60 s, 65-FSK, RS(63, 12) GF(2⁶), incl. erasure-aware decode)

Companion (not published): `mfsk-ffi` sibling crate exposing a
C ABI + `mfsk.h` header via cbindgen, with C++ driver and Kotlin
JNI example scaffolds.

Algorithms derived from WSJT-X (K1JT et al.); each source file
cites the corresponding upstream file.
