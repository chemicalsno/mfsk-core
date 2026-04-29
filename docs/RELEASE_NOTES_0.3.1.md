# Release notes — mfsk-core 0.3.1 (devel)

> Status: pre-release on the `devel` branch. Tag + crates.io publish
> follow the same workflow as 0.3.0 (push devel → PR to main → merge
> → tag).

The 0.3.1 cycle is **additive** with respect to 0.3.0 — no breaking
changes to the WSJT-X family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65).
The headline deliverable is the new `uvpacket` module, an in-tree
applied example of how the FEC infrastructure can be reused outside
the WSJT family.

## Library scope (clarified)

The README and crate-level docs now state explicitly:

- **Primary scope**: WSJT-X family digital modes
  (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) ported to Rust.
- **Secondary**: the FEC building blocks (`Ldpc240_101` and the
  shared BP/OSD machinery) are re-exported and can be reused by
  third-party protocols. `uvpacket` is one such reuse, kept in-tree
  as a worked example rather than spun out as a sibling crate.

The `Protocol` trait was designed to express the WSJT family
cleanly; uvpacket sits at its boundary and ends up with several
"decorative" `ModulationParams` constants. This is documented as
a scope-boundary trade-off rather than disguised — see
`mfsk-core/src/uvpacket/protocol.rs`'s preamble.

## New: `uvpacket` (feature-gated, off by default)

A four-mode packet protocol for narrow-FM voice channels. Targets
private-group ham messaging (signed QSL exchange, position beacons,
short text). **Not** a public APRS replacement, not a voice mode,
not a wideband mode.

### Design

- **Modulation**: single-carrier coherent QPSK at 1200 baud, RRC
  pulse (α = 0.5, span 6 sym), audio centre 1500 Hz, 12 kHz
  sample rate.
- **Sync**: 31-bit BPSK m-sequence preamble + known QPSK pilot
  every 32 symbols (~3 % overhead).
- **FEC**: `Ldpc240_101` (rate-0.42 mother code from FST4) with
  kSR-greedy puncture-set selection. Four sub-modes:

  | Sub-mode | rate | Net bps |
  |---|---:|---:|
  | Robust | 0.42 | 1008 |
  | Standard | 0.50 | 1200 |
  | Fast | 0.66 | 1600 |
  | Express | 0.75 | 1800 |
- **Frame**: 1–32 LDPC blocks per frame, 4-byte CRC-16-protected
  header, block-interleaver across all codewords.
- **API**: byte-pipe with 4-bit `app_type` tag — no
  `MessageCodec` indirection.

### Characterisation (Phase 2'a / 2'b)

σ formula calibrated from per-burst measured signal power for
cross-modulation comparability.

- **AWGN**: 50 % PER at +3.7 dB Eb/N0_info, 100 % PER at +6–8 dB
  across all four modes.
- **Rayleigh** (4-block, 20-byte payload, ≥ 90 % PER):
  Robust at +10–12 dB / 1–5 Hz Doppler, +12–15 dB at 10 Hz;
  Express at +15 dB across all Doppler.
- **LDPC-only ceiling** (modem-bypassed): Robust 50 % PER at
  +0.5 dB, Express at +1.5 dB. The ~3 dB end-to-end gap is the
  QPSK modem implementation loss — the dominant remaining work for
  Phase 3+.

### Modulation pivot history

The 0.3.1 cycle's first design was 4-GFSK at h = 0.5. Phase 2
characterisation revealed `sinc(0.5) ≈ 0.637` adjacent-tone leakage
breaking max-likelihood symbol detection. The redesign to coherent
QPSK + RRC matched filter was committed mid-cycle. See
`docs/0.3.1_PLAN.md` for the full chronology.

### Audio samples

Representative WAV files at `audio_samples/uvpacket/` for ear-level
inspection (clean, AWGN at three Eb/N0 points, Rayleigh fading,
Express). Regenerate with:

```sh
cargo run --release --features uvpacket --example uvpacket_samples
```

### Documentation

- [`docs/UVPACKET.md`](UVPACKET.md) /
  [`docs/UVPACKET.ja.md`](UVPACKET.ja.md) — full positioning
  narrative, design choices, characterisation tables for AWGN +
  Rayleigh + LDPC-only ceiling, and the known modem
  implementation-loss section.
- [`docs/0.3.1_PLAN.md`](0.3.1_PLAN.md) — design plan and
  modulation-pivot rationale (kept for posterity).

## Other changes

- **Test harness**: `awgn_sigma_for_eb_n0_info` now takes a
  `signal_power` argument for cross-modulation comparable Eb/N0.
  `signal_power(audio)` helper exposed for callers.
- **Test cleanup**: 4-FSK-specific demod-path diagnostics
  (`per_tone_magnitude_distribution`, `demod_only_ber_at_known_eb_n0`)
  removed — they have no QPSK meaning.
- **Multi-peak coarse_sync** (already in 0.3.0): exercised by
  uvpacket's auto-detect path, which scans for multiple preamble
  hits in the same buffer.

## Migration

No code changes required for downstream consumers of the WSJT
family modes. New consumers wanting `uvpacket` add the feature:

```toml
mfsk-core = { version = "0.3.1", features = ["uvpacket"] }
```

(Pulls in `fst4` automatically because it shares `Ldpc240_101`.)

## Tracked, not yet shipped

- 0.3.0 release tag + crates.io publish (still pending the push
  green light from the maintainer).
- Phase 3+: signed QSL application layer, IC-705 BLE PTT helper,
  webft8-style PWA shell. These are user-product-direction work,
  not modem internals; they will land in a later cycle.
