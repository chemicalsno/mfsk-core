# Embedded targets

`mfsk-core` is `no_std + alloc` capable: the FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of RAM when paired with caller-supplied FFT and dot-product
backends. This document covers what the library asks of an embedded
caller and what we don't ship.

## Architecture: how f32 and fixed-point share one codebase

The whole DSP / FEC pipeline is parameterised by **scalar traits**, so
the same source compiles to either a host-friendly f32 path or an
embedded-friendly integer path **with no duplicated code**:

- [`core::scalar::SpecScalar`] — spectrogram / DFT-output scalar
  (`f32` on host; `Q14i16` for embedded cs storage).
- [`core::scalar::LlrScalar`] — LLR scalar with wide-accumulator
  type (`f32` on host; `Q11i16` with i32 wide for embedded BP).
- [`core::scalar::Cmplx<S>`] — generic complex over a `SpecScalar`,
  `repr(C)` layout-compatible with `num_complex::Complex`.
- `compute_llr_generic<P, S, T>`, `compute_snr_db_generic<P, S>`,
  `bp_decode_generic_nms<P, T>` — all take the scalar types as
  generic parameters; one monomorphisation per `(P, S, T)` triple.

The `fixed-point` / `fixed-point-llr` / `fixed-point-cs` Cargo
features just **swap which scalar types the protocol glue picks**;
the generic body is unchanged. This means the embedded port shares
99 % of its code with the host build — bug fixes and optimisations
land once and apply everywhere.

### What the fixed-point switch is wired up to today

| Component | Generic over | Fixed-point switch wired? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ via `fixed-point-llr` |
| LLR computation (`core::llr`) | `SpecScalar` × `LlrScalar` | ✅ via `fixed-point-llr` |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — works for FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101) |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ via `fixed-point` |
| **FT4 / WSPR / Q65 / JT9 / JT65** | (host f32 only) | ❌ — these protocols don't go through `decode_block` today |

So: **the trait infrastructure is protocol-agnostic, but the only
protocol that actually flips into the integer path on the embedded
build is FT8.** Adding FT4 (next-most-likely candidate, since it
shares the same Costas/Gray/LDPC pieces) is a port of the
`decode_block` shape to FT4-specific symbol layout — nothing new
in the trait layer.

## What we test

| Target | MCU | Backend | Status |
|---|---|---|---|
| M5Stack Core2 | ESP32-D0WD-V3 (Xtensa LX6, single-issue f32 FPU) | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | Reference real-audio bench. Real-QSO 3-slot sweep ≈ 3.0 / 3.1 / 4.0 s wall-clock; see `embedded-poc/m5stack-core2/logs/`. |
| ESP32-S3 (DevKitC) | Xtensa LX7 + PIE | esp-dsp ASM | Earlier reference; same `fft-extern` contract. |

The `fft-extern` + `dotprod-extern` contracts are designed to be
target-portable (RP2040, RP2350-Hazard3, Cortex-M0/M3, etc.) but those
ports are not exercised in our CI. `embedded-poc/m5stack-core2/` is
the worked example to copy from.

## Cargo features for embedded use

Default features include `std`, `parallel`, and `fft-rustfft` — turn
those off and pick the embedded baseline:

```toml
[dependencies]
mfsk-core = { version = "0.5", default-features = false, features = [
    "alloc",            # Vec / Box / String — required for decode
    "ft8",              # FT8 protocol glue
    "fft-extern",       # caller supplies the FFT backend
    "fixed-point",      # u16 spectrogram + i16 internal DFT
    "fixed-point-llr",  # Q11 LLR + i16 BP NMS (FPU-friendly + smaller)
    # Optional:
    # "fixed-point-cs",            # Cmplx<Q14i16> cs storage (halves RAM)
    # "fixed-point-coarse-i32",    # i32 coarse_sync (ONLY for FPU-less MCUs)
    # "profile-coarse",            # always-on stage-2 sub-stage timing
] }
```

Feature reference:

| Feature | What it changes | When to use |
|---|---|---|
| `std` | Pulls in `std::env`, `std::time::Instant`. Decoupled from rustfft. | esp-idf-svc-style targets that have std. Optional on bare-metal. |
| `alloc` | `extern crate alloc` + Vec / Box. | All decode paths. |
| `fft-extern` | FFT backend via `mfsk_core_make_default_fft_planner` extern fn. | Any embedded target. |
| `fft-rustfft` | rustfft as the FFT backend. | Host only. |
| `fixed-point` | Spectrogram cells stored as `u16`, internal DFT in i16. | Embedded (halves PSRAM bandwidth). |
| `fixed-point-llr` | Q11 LLR + i16 NMS BP. | Embedded — match the rest of the integer pipeline. |
| `fixed-point-cs` | `Cmplx<Q14i16>` cs storage (4 KB instead of 8 KB per `Box<[[Cmplx<S>;8];79]>`). | RAM-tight embedded; LX6 is fine without. |
| `fixed-point-coarse-i32` | Stage-2 allsum / score in i32. | **FPU-less only** (RP2040, M0+). Hurts on LX6/LX7 (FPU+ALU parallelism collapses). |
| `profile-coarse` | Always emits coarse_sync sub-stage timings to stderr. | Diagnosis only. |
| `bp-crc-bail` | (reserved) | — |

## The two extern Rust contracts

### FFT backend

`mfsk_core::core::fft::FftPlanner` is the decode path's FFT trait.
Under `fft-extern`, the library expects the binary to provide an
`extern "Rust"` factory:

```rust
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::core::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}
```

`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs` is a working example
that bridges to esp-dsp's Xtensa ASM kernels (`dsps_fft2r_fc32_ae32`
+ `dsps_fft2r_sc16_ae32` for the i16 path). RP2040 / Cortex-M
implementations would bridge to CMSIS-DSP similarly.

### i16 × Q15 dot product

Under `fft-extern + fixed-point` the per-symbol DFT in
`ft8::decode_block` calls into `mfsk_core::core::dotprod::dot_q15_i32`,
which is a Rust scalar fallback by default but overridable via
another extern symbol:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "Rust" fn mfsk_core_dot_q15_i32(
    a: *const i16,
    b: *const i16,
    n: usize,
) -> i32 {
    // bridge to dsps_dotprod_s16_ae32 (LX6/LX7) or
    // arm_dot_prod_q15 (Cortex-M with CMSIS-DSP), etc.
}
```

This is the inner kernel of stage 3 — getting it onto the
target's native MAC unit is a big perf win on cached-RAM targets.
On LX6 with esp-dsp: roughly 1 cycle per 2 MACs.

## BASIS scratch

`fill_symbol_spectra_into` (and the wrappers `decode_block_into`,
`process_candidates_into`, `refine_candidates_into`) takes
caller-provided i16 scratch for the rotator basis:

```rust
const SCRATCH_LEN: usize = mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
static mut BASIS_RE: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
static mut BASIS_IM: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
```

`BASIS_SCRATCH_LEN = NTONES × NSPS = 15 360` (≈ 30 KB per axis,
60 KB total). This lives **in fast internal RAM** (`.bss`, not
PSRAM); on cached-PSRAM targets like Core2, putting basis in PSRAM
costs 5–10 cycles per dot-product term and tanks the ASM kernel.
The static-array form lands in `.bss` automatically. If you need
heap allocation, prefer
`heap_caps_malloc(BASIS_SCRATCH_LEN * 2, MALLOC_CAP_INTERNAL)` over
the default heap.

The `decode_block_into` / `process_candidates_into` /
`refine_candidates_into` family of `pub fn` exists specifically so
embedded callers can thread that scratch through without per-decode
allocation. The non-`_into` variants (`decode_block`, etc.) heap-
allocate at the default heap, which on ESP32 with PSRAM means a
slow basis on the hot path.

## Q-format quick reference

| Stage | Format | Range | File |
|---|---|---|---|
| Spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)` | `ft8::decode_block::Spectrogram` |
| DFT basis | Q15 i16 (cos, sin) | ±2¹⁵ ≈ ±1.0 | `fill_symbol_spectra_into` |
| Symbol cs | `Cmplx<f32>` (default) or `Cmplx<Q14i16>` (`fixed-point-cs`) | f32 unbounded; Q14 ±2 | `core::scalar::Cmplx` |
| LLR | f32 (host) or Q11 i16 (`fixed-point-llr`) | f32 unbounded; Q11 ±16 | `core::scalar::LlrScalar` |
| BP messages | T (same as LLR) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## What we don't ship

mfsk-core stops at the decode/encode pipeline. The following are
**deliberately out of scope** because hardware variation makes a
generic interface unhelpful:

- Audio capture (I2S, microphone gain, sample-rate clock recovery)
- Display / UI (TFT, OLED)
- Networking (Wi-Fi, BLE, MQTT)
- RTOS task wiring
- Time / clock synchronisation (NTP, GPS)
- Persistent storage / settings

`embedded-poc/m5stack-core2/src/main.rs` shows one way to wire all
of those (using esp-idf-svc) for one specific board — it is an
**example binary**, not a maintained application. Other targets
should expect to write their own glue. Reference what's there as a
template; copy what's useful.

## Performance ballpark (Core2 LX6, fixed-point + fixed-point-llr)

Three on-air recordings, baked into the m5stack-core2 binary as
WAV assets, decoded back-to-back:

| WAV | results | stage 1 | stage 2 | stage 3 | total |
|---|---|---|---|---|---|
| qso1 (mid-band) | 3 | 1.01 s | 0.76 s | 0.69 s | **2.88 s** |
| qso2 (mid-band) | 5 | 1.01 s | 0.76 s | 0.92 s | **3.10 s** |
| qso3 (busy band) | 7 | 1.01 s | 0.73 s | 1.83 s | **3.99 s** |

Stage 1 = spectrogram (92 × N=4096 FFT via two-for-one trick).
Stage 2 = coarse Costas correlation across 991 carrier bins × 27
lags. Stage 3 = per-candidate refine + LLR + BP staircase
(no OSD on Core2). Raw logs in
`embedded-poc/m5stack-core2/logs/stage3_lazy_llr_2026-05-02.log`
and friends.

## Known limitations on the embedded path

- **SNR estimate.** On the block-decode path, `DecodeResult.snr_db`
  reads ~4–12 dB low on strong signals compared to the host
  wide-band `decode_frame`. The block path uses direct DFT at the
  decoded freq and skips channel equalisation; the wide-band path
  uses a downsampled FFT plus per-tone Wiener equalisation that
  boosts strong-signal estimates. Same delta on host f32 and
  fixed-point — not a quantisation issue. Constant-offset
  workaround possible at the application layer; a proper fix needs
  the equalisation pass to run on the block path's cs (deferred
  post-0.5.0 — non-trivial PSRAM allocation pattern).
