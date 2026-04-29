// SPDX-License-Identifier: GPL-3.0-or-later
//! # `uvpacket` — U/VHF NFM packet protocol family
//!
//! Four-mode 4-GFSK packet protocol designed for narrow-FM voice
//! channels with private-group ham-radio messaging in mind. Targets
//! channel conditions where AX.25 / AFSK 1200 fails (Rayleigh fading,
//! weak signals, mountain operation), while matching or beating AFSK
//! airtime in cleaner conditions.
//!
//! ## Sub-modes (all share modem + sync + FEC mother code)
//!
//! - [`UvRobust`] — `Ldpc240_101` native rate 0.42, 1008 net bps.
//!   Mountain / weak-signal mode; AFSK has no equivalent.
//! - [`UvStandard`] — punctured to rate 1/2, 1200 net bps. AFSK 1200
//!   throughput parity plus FEC.
//! - [`UvFast`] — punctured to rate 2/3, 1600 net bps (+33 %).
//! - [`UvExpress`] — punctured to rate 3/4, 1800 net bps (+50 %).
//!   Headline-fast strong-signal mode; OSD-2 essentially mandatory.
//!
//! ## Modulation
//!
//! 4-GFSK at 1200 baud (10 samples/symbol at 12 kHz), tone spacing
//! 600 Hz (h = 0.5), Gaussian shaping BT = 0.5, audio centre 1700 Hz.
//! Tones land at 800 / 1400 / 2000 / 2600 Hz, fitting an NFM voice
//! passband while clearing the 300–500 Hz HPF on cheap handhelds.
//!
//! ## FEC
//!
//! Reuses the WSJT-X FST4 hand-tuned irregular `Ldpc240_101`
//! ([`crate::fec::Ldpc240_101`]) as the rate-0.42 mother code. The
//! three higher-rate sub-modes apply puncturing to the 139 parity
//! bits.
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
//! - Costas-4 sync (`[0, 1, 3, 2]`, FT4 reuse) prefixes each LDPC
//!   block; the bespoke RX path stitches consecutive Costas-prefixed
//!   blocks into a multi-block frame.
//!
//! ## Application API
//!
//! Byte-pipe — bypasses [`crate::core::MessageCodec`]. Callers
//! deliver raw bytes plus a 4-bit `app_type` tag; the modem doesn't
//! know or care what's inside.
//!
//! ```ignore
//! use mfsk_core::uvpacket;
//! let audio = uvpacket::tx::encode(&header, payload, 1700.0);
//! let frames = uvpacket::rx::decode(&audio);
//! for (app_type, bytes) in frames {
//!     /* dispatch on app_type */
//! }
//! ```

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
