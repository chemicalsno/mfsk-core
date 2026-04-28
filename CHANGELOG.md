# Changelog

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
