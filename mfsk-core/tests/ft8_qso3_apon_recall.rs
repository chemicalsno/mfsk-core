//! Placeholder — WSJT-X **AP-on** regression for the reference WAV
//! (`samples/FT8/210703_133430.wav` = `embedded-poc/assets/qso3_busy.wav`).
//!
//! AP-on (a-priori decoding) is `ft8b.f90` ipass 5..8 in WSJT-X: each
//! candidate's BP is re-run with my_call / dx_call / report bits
//! pinned via `apmask` + `llrz` substitution, recovering signals the
//! AP-off pipeline can't. The golden set is a strict superset of the
//! AP-off golden — adds at least the operator's own outgoing-QSO
//! signals at the bottom of the SNR floor.
//!
//! This test is `#[ignore]` until `mfsk-core` ports the AP-list
//! machinery. Track in Task #29; AP infrastructure also reuses the
//! `ApHint` plumbing in `decode.rs::process_candidate`.
//!
//! Run (once AP lands):
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_apon_recall -- --include-ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

#[test]
#[ignore = "AP-list (ft8b.f90 ipass 5..8) not yet ported — see Task #29"]
fn qso3_apon_meets_wsjtx_golden_floor() {
    // TODO: wire up `decode_frame` (or future AP-aware decode_block
    // variant) with my_call / dx_call hint, run on qso3_busy.wav,
    // assert against the AP-on golden set captured from WSJT-X.
    //
    // The AP-on reference must be re-captured: the AP-off 8-entry
    // table in `reference_qso3_busy_wsjtx_decode.md` is *not* the
    // AP-on truth. AP introduces extra decodes that depend on
    // operator context (my_call etc.) — the test fixture should
    // mirror the WSJT-X UI's "Use AP for Decoding" toggle exactly.
    panic!("AP-list not ported yet");
}
