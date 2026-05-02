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
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — confirmed by `espflash board-info`: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`. **Not** an ESP32-S2 (LX7, single-core, no BT) or S3. | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | Reference real-audio bench. See benchmark + footprint sections below. |
| ESP32-S3 (DevKitC) | Xtensa LX7 + PIE SIMD | esp-dsp ASM | Earlier reference; same `fft-extern` contract. |

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

## Using from C / C++ / non-Rust ESP-IDF projects (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
exposes a tiny C ABI for the FT8 block decoder slice. It is the
recommended way to call the embedded FT8 decoder from a non-Rust
ESP-IDF (or RP2040 / Cortex-M) project.

The crate is `no_std + alloc` under its `embedded-fixed-point`
feature so the resulting `libmfsk_ft8.a` doesn't carry Rust's `std`
runtime — drop-in linkable from C without the toolchain weirdness
that would come from mixing two libc layers.

**Verified end-to-end on ESP32 Core2** (m5stack-core2 example): a
separate `ffi_smoke_one` path calls `mfsk_ft8_decode_i16` (C ABI)
on the same baked WAVs as the direct-Rust `decode_one` path and
gets identical recall — qso1 (3 / 3), qso2 (5 / 5),
**qso3 busy band (7 / 7)**. With caller-managed BASIS scratch in
internal RAM the FFI path lands ~2.6 × faster than the same FFI
call with internal heap allocation (qso3 3.74 s vs 9.57 s) and
within ~5 % of the direct-Rust process_candidates_into path.
Logs:
- `embedded-poc/m5stack-core2/logs/ffi_into_2026-05-02.log`
  (recommended caller-scratch path)
- `embedded-poc/m5stack-core2/logs/ffi_smoke_2026-05-02.log`
  (heap-alloc reference for comparison)

### API at a glance

cbindgen-generated header — `mfsk-ffi-ft8/include/mfsk_ft8.h`,
regenerated on every build. The full surface is:

```c
typedef struct MfskFt8Result {
    char     text[40];   // NUL-terminated unpacked message
    float    freq_hz;    // carrier
    float    dt_sec;     // time offset relative to slot start
    float    snr_db;     // see "Known limitations" — embedded
                         // path reads ~4–12 dB low on strong sigs
    uint32_t hard_errors;
    uint8_t  pass;       // staircase stage (0=fast Bp, 1=full Bp,…)
} MfskFt8Result;

typedef struct MfskFt8ResultList {
    MfskFt8Result *items;
    size_t         len;
    size_t         _capacity;  // private
} MfskFt8ResultList;

// Required scratch length for the primary decode entry, in i16
// elements. Caller allocates two arrays of this length each.
size_t mfsk_ft8_basis_scratch_len(void);

// PRIMARY embedded entry. Caller-managed scratch — must live in
// fast internal RAM (NOT PSRAM) for the dot-product ASM kernel to
// hit peak throughput.
MfskFt8Status mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz, mono, ≥168 000
    float freq_min_hz, float freq_max_hz,     // typical 200, 3000
    float sync_min, int max_cand,             // typical 1.0, 30
    MfskFt8Depth depth,                       // 0=Bp, 1=BpAll, 2=BpAllOsd
    int16_t *basis_re, int16_t *basis_im,     // scratch
    MfskFt8ResultList *out);                  // populated by callee

// HOST-ONLY convenience — heap-allocs basis internally. Excluded
// from embedded builds: see "Why caller-supplied scratch" below.
#ifdef MFSK_FT8_HOST  // built with default `host` feature
MfskFt8Status mfsk_ft8_decode_i16_alloc(
    const int16_t *audio, size_t n_samples,
    float freq_min_hz, float freq_max_hz,
    float sync_min, int max_cand,
    MfskFt8Depth depth,
    MfskFt8ResultList *out);
#endif

void mfsk_ft8_result_list_free(MfskFt8ResultList *list);
```

### Why caller-supplied scratch (and why it's not optional on Core2)

The 60 KB `BASIS` scratch (cos/sin Q15 rotators × 8 tones × 1920
samples) is the **dot-product inner-loop hot data**. The esp-dsp
ASM kernel `dsps_dotprod_s16_ae32` runs at 1 cycle per 2 MACs only
when the basis sits in fast internal SRAM (DRAM). On a Core2 with
PSRAM enabled (the default), the standard `malloc` heap lands in
PSRAM, and PSRAM-resident reads cost **5–10 cycles/sample of stall**
through the cache, dropping the kernel to ~30 % of its rated speed.
Stage 3 wall-clock literally **doubles to triples** if BASIS is in
PSRAM.

You can't predict which heap a `malloc` call lands in from C —
ESP-IDF's heap allocator routes between internal RAM and PSRAM by
size and capability flags, and a 60 KB request lands in PSRAM
unless explicitly capped. So a "convenience" entry that hides a
60 KB `malloc` would silently de-tune any embedded caller. We
deliberately don't ship one for embedded — `mfsk_ft8_decode_i16`
takes the scratch as parameters, full stop. The caller decides:

```c
// The simplest correct pattern: static .bss arrays.
// They land in internal DRAM automatically and persist for the
// process lifetime.
#include "mfsk_ft8.h"
static int16_t basis_re[15360];   // = mfsk_ft8_basis_scratch_len()
static int16_t basis_im[15360];

MfskFt8ResultList results = {0};
MfskFt8Status st = mfsk_ft8_decode_i16(
    audio, n_samples,
    200.0f, 3000.0f, 1.0f, 30,
    MFSK_FT8_DEPTH_BP_ALL,
    basis_re, basis_im,
    &results);
```

If you must allocate dynamically, use ESP-IDF's capability-flagged
allocator: `heap_caps_malloc(15360 * sizeof(int16_t),
MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)`. Keep the buffers around
across decode calls — they don't need to be reset between slots.

### Build flags

#### Host (`libmfsk_ft8.so` / `libmfsk_ft8.a` for desktop testing)

```sh
cargo build -p mfsk-ffi-ft8 --release
# → target/release/libmfsk_ft8.{so,a}
# → mfsk-ffi-ft8/include/mfsk_ft8.h (cbindgen-generated)
```

Default features pull `mfsk-core/std + ft8 + fft-rustfft`. A C smoke
test linking the resulting `.so` lives at
`mfsk-ffi-ft8/tests/c_smoke/smoke.c`:

```sh
gcc -O2 -I mfsk-ffi-ft8/include \
    mfsk-ffi-ft8/tests/c_smoke/smoke.c \
    -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
    -Wl,-rpath,$PWD/target/release \
    -o /tmp/mfsk_smoke
/tmp/mfsk_smoke embedded-poc/m5stack-core2/assets/qso3_busy.wav
```

#### Embedded (Xtensa ESP32, `libmfsk_ft8.a` for ESP-IDF link)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf
# → target/xtensa-esp32-espidf/release/libmfsk_ft8.a
```

`-C panic=abort` is required because Rust unwinding panics need
`std`; embedded uses `panic = "abort"` everywhere. ESP-IDF projects
typically set this in their `.cargo/config.toml`:

```toml
[target.xtensa-esp32-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

#### Feature reference

| Feature | Default | Purpose |
|---|---|---|
| `host` | ✓ | Host build — pulls `mfsk-core/std + ft8 + fft-rustfft`. Both `mfsk_ft8_decode_i16` (caller scratch) and `mfsk_ft8_decode_i16_alloc` (heap convenience) are exported. |
| `embedded-fixed-point` | — | `no_std + alloc`. Pulls `mfsk-core/fft-extern + fixed-point + fixed-point-llr`. **Only `mfsk_ft8_decode_i16` is exported** — the heap-alloc convenience is excluded by design (see above). The linker must resolve `mfsk_core_make_default_fft_planner_*` and `mfsk_core_dot_q15_i32` (typically via a small Rust shim that bridges esp-dsp). |
| `embedded-runtime` | — | Provides default `#[panic_handler]` (calls libc `abort`) + `#[global_allocator]` (libc `malloc`/`free`). Needed for a self-contained `staticlib`; turn off when stacking another Rust runtime in the same image. |

### Linking it into an ESP-IDF (CMake) project

```
your-app/                          # esp-idf project root
├── main/main.c                    # calls mfsk_ft8_decode_i16(...)
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib component
│   ├── include/mfsk_ft8.h         # from mfsk-ffi-ft8 build
│   └── lib/libmfsk_ft8.a          # from mfsk-ffi-ft8 build
└── shim/                          # tiny Rust crate (esp-dsp bridges)
    ├── Cargo.toml                 # depends on mfsk-ffi-ft8
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # provides mfsk_core_make_default_fft_planner
                                   # and mfsk_core_dot_q15_i32 via esp-dsp
```

The `shim/` Rust crate is needed because mfsk-core's FFT-extern
contract uses `extern "Rust"` symbols (different ABI from `extern
"C"`), which a pure-C compilation unit can't satisfy. The shim is
~50 lines of Rust + a vendored copy of
`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs`.

`components/mfsk_ft8/CMakeLists.txt` minimal example:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_ft8_rust STATIC IMPORTED)
set_target_properties(mfsk_ft8_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_ft8.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_ft8_rust)
```

A worked skeleton lives at
[`embedded-poc/idf-component/`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/idf-component).

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

## Performance benchmark (Core2 LX6, `fixed-point` + `fixed-point-llr`)

Three on-air recordings baked into the m5stack-core2 binary as WAV
assets, decoded back-to-back over three full sweeps. Per-stage
breakdown from the first (cold-cache) iteration; total range is the
min–max across three iterations.

| WAV | results | stage 1 (spec) | stage 2 (sync) | stage 3 (refine + BP) | **total range** |
|---|---|---|---|---|---|
| qso1 (mid-band, 3 stations) | 3/3 vs `decode_frame` | 1.01 s | 0.77 s | 0.69 s | **2.87 – 3.24 s** |
| qso2 (mid-band, 5 stations) | 5/5 vs `decode_frame` | 1.01 s | 0.77 s | 0.92 s | **3.10 – 3.47 s** |
| qso3 (busy band, ≥7 of 10 stations) | 7 incl. block-only | 1.01 s | 0.75 s | 1.83 s | **3.99 – 4.36 s** |

- **Stage 1** = spectrogram, dominated by 92 × N=4096 i16 complex
  FFTs via the two-for-one real-FFT trick (see `compute_spectrogram`
  under `fixed-point`).
- **Stage 2** = coarse Costas correlation across 991 carrier bins ×
  27 lags. FPU-add bound on LX6 — the f32 path is faster than the
  i32 path here (see `fixed-point-coarse-i32` rationale above).
- **Stage 3** = per-candidate refine fill + LLR + BP staircase.
  OSD off on Core2 (`OSD_ENABLED=false` in the example main.rs);
  the spread between qso1 (3 results) and qso3 (7 results) is from
  the per-result fill + Step-2 BP variant cost.

Recall is preserved across all three iterations. Iteration-to-
iteration drift (~10 %) on later runs is allocator and PSRAM cache
warm-up.

Raw monitor logs:
`embedded-poc/m5stack-core2/logs/release_0_5_0_2026-05-02.log`
(latest 0.5.0 release sweep) plus per-commit perf-chain logs
(`stage3_bp_pool`, `stage3_syncblocks12`, `stage3_lazy_llr`,
`two_for_one`, `phase3_coarse_i32`).

## Binary footprint (Core2 reference, `xtensa-esp32-elf-size -A`)

| Region | Size | Contents |
|---|---|---|
| **IRAM** (`.iram0.text` + `.iram0.vectors`) | **69 KB** | Internal-RAM code: esp-idf interrupt handlers, Wi-Fi/BT IRAM-resident routines |
| **DRAM** (`.dram0.data` + `.dram0.bss`) | **76 KB** | Internal-RAM static data: BASIS scratch (60 KB) + spectrogram cache + esp-idf statics |
| **Flash text** (`.flash.text`) | **448 KB** | App + esp-idf code |
| **Flash rodata** (`.flash.rodata`) | **1.21 MB** | Read-only data — **incl. the three baked WAVs (~1.08 MB)** for the offline real-audio bench |
| **Total app binary** | **1.997 MB** | What `espflash flash` writes |

Subtracting the baked WAV assets (1.08 MB) and the bundled esp-idf
runtime, `mfsk-core` itself plus the M5Stack Core2 example glue
contributes roughly **150–200 KB** of flash text. The IRAM/DRAM
totals shown include esp-idf — the library proper has no IRAM
requirement and the only DRAM requirement is the 60 KB BASIS scratch
plus an optional 8 KB-per-candidate cs Box (~120 KB peak across
~15 surviving candidates), comfortably fitting the on-chip 320 KB
SRAM budget on bare ESP32 (no PSRAM needed strictly for the decode
path; PSRAM helps only for the wider spectrogram).

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
