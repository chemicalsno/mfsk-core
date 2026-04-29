// SPDX-License-Identifier: GPL-3.0-or-later
//! # `uvpacket` — applied example: a NFM-voice-channel packet protocol
//!
//! **Scope note.** This module is **not** a member of the WSJT-X
//! mode family that the rest of `mfsk-core` ports. It is an in-tree
//! *applied example* of how the FEC infrastructure
//! ([`crate::fec::Ldpc240_101`], BP, OSD-2) can be reused outside
//! that family. It targets a different design point — narrow-FM
//! voice channels (HT/mobile, ~3 kHz audio passband) with private-
//! group amateur-radio messaging — and consequently diverges from
//! WSJT-X assumptions in almost every layer above the FEC:
//!
//! | Layer | WSJT-X family | uvpacket |
//! |---|---|---|
//! | Modulation | M-ary tone FSK / GFSK | single-carrier coherent **QPSK** + RRC |
//! | Demod | non-coherent symbol-power detect | matched-filter + pilot-aided phase track |
//! | Slot | 7.5 / 15 / 60 / 120 s | variable-length burst |
//! | Sync | tone-index Costas blocks | 31-bit BPSK m-sequence preamble |
//! | Message | structured (callsign + grid) | byte-pipe (`app_type` tag) |
//! | TX/RX path | generic `mfsk-core` pipeline | bespoke ([`tx::encode`] / [`rx::decode`]) |
//!
//! What is **shared**: the LDPC mother code (`Ldpc240_101`, ported
//! from FST4), the `FecCodec`/`FecOpts` API surface, and OSD-2
//! soft-decoding. Everything else is uvpacket-local.
//!
//! ## Why this lives in-tree
//!
//! Splitting it into a sibling crate would add maintenance overhead
//! disproportionate to the deliverable. Keeping it here lets the
//! LDPC reuse story be demonstrated end-to-end without crate-
//! boundary friction. The cost is that `Protocol::ID =
//! ProtocolId::UvPacket` and the `ModulationParams` trait constants
//! (`NTONES = 4`, `GFSK_BT`, `TONE_SPACING_HZ`, …) are
//! **decorative** for this module — they exist only to satisfy the
//! trait signature and the `protocol_invariants` checker, and are
//! never consulted by the bespoke TX/RX paths. See
//! [`mod@protocol`] for the explicit list.
//!
//! ## Sub-modes (all share modem + preamble + FEC mother code)
//!
//! - [`UvRobust`] — `Ldpc240_101` native rate 0.42, 1008 net bps.
//!   Mountain / weak-signal posture.
//! - [`UvStandard`] — punctured to rate 1/2, 1200 net bps. Typical
//!   NFM with fading.
//! - [`UvFast`] — rate 2/3, 1600 net bps (+33 %).
//! - [`UvExpress`] — rate 3/4, 1800 net bps (+50 %). OSD-2 is
//!   essentially mandatory at the BP threshold; viable only thanks to
//!   kSR-greedy puncture-set selection.
//!
//! ## Modulation
//!
//! Single-carrier coherent QPSK at 1200 baud (10 samples/symbol at
//! 12 kHz), root-raised-cosine pulse (α = 0.5, span 6 sym), audio
//! centre 1500 Hz. The QPSK constellation is Gray-mapped and the
//! TX/RX paths use a 31-bit BPSK m-sequence preamble + periodic QPSK
//! pilot symbols (one every 32 sym, ≈ 3 % overhead) for symbol
//! timing, frame detection and decision-directed phase tracking.
//!
//! ## FEC
//!
//! Reuses the WSJT-X FST4 hand-tuned irregular
//! [`crate::fec::Ldpc240_101`] as the rate-0.42 mother code. The
//! three higher-rate sub-modes apply kSR-greedy puncturing to the
//! 139 parity bits.
//!
//! ## Frame structure
//!
//! - Variable length, 1–32 LDPC blocks per frame.
//! - Each LDPC block carries 96 info bits (12 byte) padded to the
//!   FEC's 101-bit input.
//! - 4-byte frame header: mode (2b) + block count (5b) + app type
//!   (4b) + sequence (5b) + CRC-16 (16b).
//! - Block-interleaver across all codewords in the frame spreads
//!   fade-burst erasures across every codeword.
//!
//! ## Application API
//!
//! Byte-pipe — bypasses [`crate::core::MessageCodec`]. Callers
//! deliver raw bytes plus a 4-bit `app_type` tag; the modem doesn't
//! know or care what's inside.
//!
//! ```ignore
//! use mfsk_core::uvpacket;
//! let audio = uvpacket::tx::encode(&header, payload, 1500.0);
//! let frames = uvpacket::rx::decode(&audio, 1500.0);
//! for f in frames { /* dispatch on f.app_type */ }
//! ```
//!
//! ## Characterisation (post LMS phase tracker)
//!
//! σ formula calibrated from per-burst signal power
//! (`tests/common/channel.rs`):
//!
//! - **AWGN**: 50 % PER at +1 dB Eb/N0_info Robust, +2 dB Standard
//!   / Fast, +3 dB Express. 100 % PER at +4 dB across all modes.
//! - **Rayleigh** (4-block, 20-byte payload, ≥ 90 % PER):
//!   Robust at +10 dB / 5–10 Hz Doppler, +12 dB at 1 Hz; the
//!   higher-rate modes ~+10 dB across most Doppler.
//! - **LDPC-only ceiling**: Robust 50 % PER at +0.5 dB, Express at
//!   +1.5 dB. End-to-end gap is now 0.5–2 dB (down from ~3 dB
//!   pre-LMS).
//! - **FM threshold margin**: Robust at −3.7 dB SNR_3kHz vs the
//!   NFM FM-threshold floor at ~+20 dB SNR_3kHz → **~24 dB
//!   margin**. The channel CNR floor binds before the modem.
//!
//! Representative WAV samples for ear-level inspection live at
//! `audio_samples/uvpacket/` in the repository.
//!
//! See [`docs/UVPACKET.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/UVPACKET.md)
//! for the full design narrative, the modulation-pivot history,
//! and the implementation-loss breakdown.

pub mod framing;
pub mod interleaver;
pub mod message;
pub mod protocol;
pub mod puncture;
pub mod rx;
pub mod sync_pattern;
pub mod tx;

pub use message::UvPacketRawMessage;
pub use protocol::{AUDIO_CENTRE_HZ, UvExpress, UvFast, UvRobust, UvStandard};
pub use puncture::Mode;
