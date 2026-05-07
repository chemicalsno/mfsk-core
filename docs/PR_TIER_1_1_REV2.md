# PR #20 (`feat/ft4-subtract`) — Tier 1.1 Review Rev 2

Follow-up to Gemini review 2026-05-07 06:25 UTC (5 medium comments) on
`mfsk-core/src/ft4/subtract.rs`. All addressed in commit
`fix(ft4): improve subtract precision and freq search robustness`.

## Comments addressed

### Precision (L167, L172, L193, L199) — fixed

Both unit tests (`subtract_with_exact_timing_near_zero`,
`subtract_reduces_power`) summed `(s as f32).powi(2)` over ~90,000 i16
samples in an `f32` accumulator. Each squared i16 reaches ~`2^30`; once
the running partial sum exceeds ~`2^24` the additions silently round,
producing a non-deterministic `power_before` and a flaky pass/fail
ratio.

Fix: switched both accumulators (and the divisor in
`subtract_reduces_power`) to `f64`. Added an inline comment explaining
why on the first occurrence and a back-reference on the second.

Scope check: I grepped the entire FT4 PR diff for any other
many-sample `f32` reductions (sum, mean, RMS, dot products). The four
flagged lines were the only such accumulators introduced — all
non-test code in `subtract.rs` delegates to `core::dsp::subtract`,
which is unchanged in this PR.

### Frequency search radius (L121) — fixed (widened to ±5 Hz)

`refine_signal_freq` previously searched ±2.5 Hz. FT4's coarse-sync
output is on a `12000/2304 ≈ 5.208 Hz` grid (`NFFT1 = 4 × NSPS = 2304`
at 12 kHz), so ±2.5 Hz only covers ~±0.48 bin. By contrast, FT8's
coarse-sync grid is 2.93 Hz/bin (NFFT_SPEC=4096), so the FT8 ±2.5 Hz
window already covers ±0.85 bin.

Fix: widened FT4 radius to **±5 Hz** (≈ ±0.96 bin), matching FT8's
per-bin coverage. Step size unchanged at 0.1 Hz, so the cost roughly
doubles from ~50 to ~100 GFSK reference builds (~2 ms per signal on
host f32 — still well under the 1-call-per-decoded-result budget). The
docstring now spells out the bin-width derivation and cites WSJT-X
`lib/ft4/sync4d.f90`'s `ctwk` table, which uses an equivalent ±5 Hz
window for the same reason.

## Pushbacks / non-changes

- **FT8 has the same f32 accumulator pattern in its tests** (`ft8/subtract.rs`
  L104–105, L120–121, L140, L154). Out of scope for this PR (which is
  branch `feat/ft4-subtract`); the FT8 tests have not failed in CI to
  date because the constants happen to align. Worth a follow-up commit
  on `main`. Not changed here to keep the diff focused on FT4.
- **The `core::dsp::subtract::refine_freq` signature was not changed.**
  Both FT8 and FT4 still call it with their own `(radius_hz, step_hz)`
  pair, which is the right design — the bin-width-vs-protocol tradeoff
  is a property of the calling protocol's coarse-sync, not of the
  generic refinement routine.

## CI gates run locally

| Gate | Result |
| --- | --- |
| `cargo fmt --check` | clean |
| `cargo build -p mfsk-core --features full --release` | ok |
| `cargo clippy -p mfsk-core --features full --all-targets -- -D warnings` | clean |
| `cargo test -p mfsk-core --features full --release --lib ft4` | 2/2 pass |
| `cargo test -p mfsk-core --features full --release` | only pre-existing `ft8_qso3_apoff_recall` failure (missing `/home/minoru/...` WAV — same failure on `main`, not introduced by this PR) |

## Files touched

- `mfsk-core/src/ft4/subtract.rs` — `f64` accumulators in tests, ±5 Hz
  radius in `refine_signal_freq`, expanded docstring.
- `docs/PR_TIER_1_1_REV2.md` — this document.

## Critical correction (post-review WSJT-X parity audit)

After REV2 doc-only landing, an out-of-band WSJT-X-source verification
audit (Anthropic verification agent, 2026-05-07) caught two parity bugs
that this PR's earlier "BT unification" commit (`b652100`) and original
`subtract_signal_lpf` introduction had silently introduced. Both are
now fixed in this same amended commit.

### Bug 1 — GFSK BT was wrong direction (2.0, should be 1.0)

The `b652100` commit "unified" FT4 GFSK on `BT = 2.0`, citing parity
with WSJT-X. That citation was wrong — it confused FT4 with FT8:

- WSJT-X `lib/ft4/gen_ft4wave.f90` calls `gfsk_pulse(1.0, tt)` —
  the literal `1.0` is the BT.
- WSJT-X `lib/ft4/subtractft4.f90` declares `bt=1.0` at the top.
- FT8 (not FT4) uses `BT = 2.0`.

Both implementations use the same `gfsk_pulse` formula
`0.5·[erf(C·bt·(t+½)) − erf(C·bt·(t−½))]` so the parameter is directly
comparable. Fixed in `mfsk-core/src/ft4/encode.rs` (`FT4_GFSK.bt`),
`mfsk-core/src/ft4/decode.rs` (`FT4_SUBTRACT.gfsk.bt`), and
`mfsk-core/src/ft4/mod.rs` (`Ft4::GFSK_BT`) — all flipped 2.0 → 1.0.
Doc-comments updated to cite `gen_ft4wave.f90` / `subtractft4.f90`.

The original "unification" claim that BT alignment dropped the
self-cancellation residual from 9.13e-3 to ~1e-12 is still valid: the
two sides are still aligned, just at the correct WSJT-X value.
Empirical residual ratio after this fix:
**5.06e-13** (`subtract_with_exact_timing_near_zero`).

### Bug 2 — `subtract_signal_lpf` half-window 2.86× too wide

The original `subtract_signal_lpf` passed `lpf_half = 2000`
(full window 4000 samples = 333 ms @ 12 kHz) with a doc-comment
claiming this matched WSJT-X `NFILT = 4000`. That was again FT8's
NFILT, not FT4's. WSJT-X `lib/ft4/subtractft4.f90` uses
`NFILT = 1400` (half = 700 samples = 58 ms). Fixed in
`mfsk-core/src/ft4/subtract.rs:90`: `2000` → `700`. Surrounding
doc-comments at `:14-23` and `:70-75` updated accordingly.

### Files touched by the correction

- `mfsk-core/src/ft4/encode.rs` — `FT4_GFSK.bt` 2.0 → 1.0 + doc.
- `mfsk-core/src/ft4/decode.rs` — `FT4_SUBTRACT.gfsk.bt` 2.0 → 1.0
  + doc.
- `mfsk-core/src/ft4/mod.rs` — `Ft4::GFSK_BT` 2.0 → 1.0.
- `mfsk-core/src/ft4/subtract.rs` — `lpf_half` 2000 → 700 + doc
  citations.
- `docs/PR_TIER_1_1_REV2.md` — this section.

### CI gates after correction

| Gate | Result |
| --- | --- |
| `cargo fmt --check` | clean |
| `cargo build -p mfsk-core --features full --release` | ok |
| `cargo clippy -p mfsk-core --features full --all-targets -- -D warnings` | clean |
| `cargo test -p mfsk-core --features full --release` | 328 pass, only pre-existing `ft8_qso3_apoff_recall` missing-WAV failure |
| `ft4_wsjtx_sample_recall_vs_golden` (real-WAV round-trip) | pass |
| `subtract_with_exact_timing_near_zero` residual | 5.06e-13 (pass; was the headline ~1e-12 result, still satisfied at the correct BT) |

No tests were tuned to BT=2.0; flipping back to BT=1.0 left every
existing FT4 test passing without threshold adjustment.

## Known coverage gaps

- The new `refine_signal_freq` ±5 Hz widening is justified by the bin-math
  argument in the docstring, but is not exercised by a behavioural test
  that drives a real off-bin signal through subtract and asserts residual
  reduction. The existing tests are compile-shape / sanity-only and would
  not catch a regression from ±5 Hz back to ±2.5 Hz on real FT4 audio.
  Acknowledged; tracked as a follow-up under harness-driven SIC tests
  rather than expanding scope of this PR.
