# Changelog

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
