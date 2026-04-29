// SPDX-License-Identifier: GPL-3.0-or-later
//! Protocol markers + trait wiring for the four uvpacket modes.
//!
//! All four modes share the same modem (4-GFSK at 1200 baud,
//! 600 Hz tone spacing, h=0.5, BT=0.5), the same FEC mother code
//! (`Ldpc240_101`), the same Costas-4 head sync, and the same
//! per-LDPC-block frame layout at the [`Protocol`] trait level. They
//! differ only in the puncturing applied to the FEC parity bits,
//! which lives outside the trait constants in
//! [`crate::uvpacket::puncture`].
//!
//! | ZST            | rate | net bps (at 4-GFSK 2400 ch bps) | use |
//! |----------------|-----:|--------------------------------:|-----|
//! | [`UvRobust`]   | 0.42 | 1008 | mountain / weak signal / deep fading |
//! | [`UvStandard`] | 0.50 | 1200 | typical NFM with fading             |
//! | [`UvFast`]     | 0.66 | 1600 | good-signal default                  |
//! | [`UvExpress`]  | 0.75 | 1800 | strong-signal headline-fast mode (OSD-2 essentially mandatory) |
//!
//! Higher-rate modes use kSR-greedy puncture-set selection (see
//! [`crate::uvpacket::puncture`]) — the empirical AWGN sweep showed
//! ~1–3 dB Eb/N0 gain over uniform-spread at the deeper puncture
//! rates, which makes `UvExpress` (76 % parity puncturing) viable.
//!
//! Note: at the [`Protocol`] level, all four ZSTs claim the same
//! `N_DATA = 120` (= unpunctured codeword 240 ch bits / 2 bits/sym).
//! The actual on-air block length post-puncture is shorter for
//! Standard / Fast / Express and is handled by the bespoke TX/RX
//! paths in [`crate::uvpacket::tx`] / [`crate::uvpacket::rx`]. The
//! Protocol-level constants describe the *unpunctured* codeword so
//! the standard mfsk-core invariants (FEC fits in N_DATA × bits/sym)
//! hold.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Ldpc240_101;

use super::message::UvPacketRawMessage;
use super::puncture::Mode;
use super::sync_pattern::UVPACKET_SYNC_BLOCKS;

/// Identity Gray map for 4-FSK (FT4 uses the same).
const GRAY_4: [u8; 4] = [0, 1, 3, 2];

/// Audio-domain centre frequency at synth time (Hz). Tones land at
/// 800 / 1400 / 2000 / 2600 Hz, comfortably inside the typical NFM
/// HT audio passband while clearing the 300–500 Hz HPF found on
/// cheaper handhelds.
pub const AUDIO_CENTRE_HZ: f32 = 1700.0;

/// Define a uvpacket sub-mode ZST with all four trait impls.
///
/// All sub-modes share modulation, frame layout, FEC, message codec,
/// and sync. The only per-mode datum is the inherent `MODE` constant
/// pointing at the puncturing variant.
macro_rules! uvpacket_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        mode = $mode:expr,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl $name {
            /// Puncturing posture for this sub-mode. Used by the
            /// bespoke TX / RX paths to pick the right puncture
            /// table.
            pub const MODE: Mode = $mode;
        }

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            /// 1200 baud at 12 kHz sample rate → 10 samples / symbol.
            const NSPS: u32 = 10;
            const SYMBOL_DT: f32 = 1.0 / 1200.0;
            /// h = 0.5 → tone spacing = baud × h = 600 Hz.
            const TONE_SPACING_HZ: f32 = 600.0;
            const GRAY_MAP: &'static [u8] = &GRAY_4;
            const GFSK_BT: f32 = 0.5;
            const GFSK_HMOD: f32 = 0.5;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 4;
            const NSTEP_PER_SYMBOL: u32 = 2;
            /// 12000 / 4 = 3000 Hz baseband window — clears the
            /// 800–2600 Hz tone span with margin.
            const NDOWN: u32 = 4;
        }

        impl FrameLayout for $name {
            /// 240 codeword bits / 2 bits-per-symbol = 120 data symbols
            /// per LDPC block. (Unpunctured. Higher-rate modes
            /// transmit fewer ch bits per block but the trait-level
            /// constant describes the mother codeword.)
            const N_DATA: u32 = 120;
            /// One Costas-4 at the head of each LDPC block.
            const N_SYNC: u32 = 4;
            const N_SYMBOLS: u32 = 124;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&UVPACKET_SYNC_BLOCKS);
            /// uvpacket frames are not slot-aligned — value is
            /// informational only. Use the duration of one
            /// LDPC-block-sized "protocol unit" so callers that
            /// expect a non-zero T_SLOT_S see something reasonable.
            const T_SLOT_S: f32 = 124.0 / 1200.0;
            const TX_START_OFFSET_S: f32 = 0.0;
        }

        impl Protocol for $name {
            type Fec = Ldpc240_101;
            type Msg = UvPacketRawMessage;
            const ID: ProtocolId = ProtocolId::UvPacket;
        }
    };
}

uvpacket_submode! {
    /// **Robust** — rate 0.42 (unpunctured `Ldpc240_101`).
    /// 1008 net bps. For mountain / weak-signal / deep-fading
    /// channels where AFSK 1200 cannot deliver. AFSK has no
    /// equivalent mode — this is the design's headline value-prop.
    UvRobust, mode = Mode::Robust,
}

uvpacket_submode! {
    /// **Standard** — punctured to rate 1/2. 1200 net bps.
    /// Throughput parity with AFSK 1200 plus FEC for typical NFM
    /// channels.
    UvStandard, mode = Mode::Standard,
}

uvpacket_submode! {
    /// **Fast** — punctured to rate 2/3. 1600 net bps (+33 % vs
    /// AFSK 1200). Good-signal default. 63 % parity puncturing;
    /// kSR-greedy puncture selection delivers ~1 dB Eb/N0 gain
    /// over uniform-spread at the BP threshold.
    UvFast, mode = Mode::Fast,
}

uvpacket_submode! {
    /// **Express** — punctured to rate 3/4. 1800 net bps (+50 % vs
    /// AFSK 1200). Strong-signal headline-fast mode. 76 % parity
    /// puncturing — OSD-2 is essentially mandatory at the BP
    /// threshold (~+3 dB Eb/N0 with OSD-2; BP-only needs ~+5 dB).
    /// Viable only thanks to kSR-greedy puncture selection
    /// (uniform-spread fails at this rate).
    UvExpress, mode = Mode::Express,
}
