// SPDX-License-Identifier: GPL-3.0-or-later
//! Common test fixtures: channel models, RNG helpers, asset paths.

pub mod air_channel;
pub mod channel;

/// Build a path to an asset under `embedded-poc/assets/` that resolves
/// regardless of where the crate is checked out (CI runners, contributor
/// dev boxes, the maintainer's `/home/ubuntu/...` tree). Equivalent to
/// `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/", $asset)`.
#[macro_export]
macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

/// On-air FT8 recordings used by the `decode_block` performance and
/// recall sweeps. Two consecutive 15 s slots from `jl1nie/RustFT8` plus
/// the WSJT-X-distributed reference recording.
///
/// `qso3_busy.wav` is bit-identical to WSJT-X
/// `samples/FT8/210703_133430.wav` (verified via `cmp` 2026-05-04, see
/// the module doc on `ft8_reference_suite_recall.rs`).
//
// `dead_code`: each integration test is its own crate, so consumers
// that only need the channel helpers (e.g. uvpacket tests) see this
// const as unused.
#[allow(dead_code)]
pub const REAL_QSO_WAVS: &[&str] = &[
    asset_path!("191111_110130.wav"),
    asset_path!("191111_110200.wav"),
    asset_path!("qso3_busy.wav"),
];
