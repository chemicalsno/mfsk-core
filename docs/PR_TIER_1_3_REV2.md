# PR #21 (Tier 1.3) — Rev 2: `_with_options` parity fix

## Why

Gemini's 2026-05-07 review on PR #21 flagged signature drift: the
`_with_options` siblings introduced in commit `36a3c5e1` dropped the
`freq_hint: Option<f32>` parameter present on the original
`decode_frame_with_options` (commit `4a32995`). This breaks the
"`_with_options` is a strict superset of its non-options sibling +
`depth` + `strictness`" invariant the rest of the API follows.

## Audit

For every pair below, `_with_options` must equal the non-options
parameters PLUS `depth: DecodeDepth`, `strictness: DecodeStrictness`,
plus (where the original API exposes it) `freq_hint: Option<f32>`.
`_and_options` cache variants additionally return the `FftCache`.

| pair | non-options params | `_with_options` adds | drift before fix |
|---|---|---|---|
| `ft4::decode_frame[_with_options]` | audio, freq_min, freq_max, sync_min, max_cand | freq_hint, depth, strictness | (none — was correct from rev 1) |
| `ft4::decode_frame_with_cache[_and_options]` | audio, freq_min, freq_max, sync_min, max_cand | freq_hint, depth, strictness | missing `freq_hint` |
| `ft4::decode_frame_subtract[_with_options]` | audio, freq_min, freq_max, sync_min, max_cand | freq_hint, depth, strictness | missing `freq_hint` |
| `ft4::decode_sniper_ap[_with_options]` | audio, target_freq, max_cand, eq_mode, ap_hint | depth, strictness | none (sniper uses `target_freq`, no `freq_hint`) |
| `fst4::decode_frame[_with_options]` | audio, freq_min, freq_max, sync_min, max_cand | freq_hint, depth, strictness | (none — was correct from rev 1) |
| `fst4::decode_frame_with_cache[_and_options]` | audio, freq_min, freq_max, sync_min, max_cand | freq_hint, depth, strictness | missing `freq_hint` |

## Changes

- `mfsk-core/src/ft4/decode.rs`
  - `decode_frame_with_cache_and_options`: add `freq_hint: Option<f32>`,
    forward to `pipeline::decode_frame`.
  - `decode_frame_subtract_with_options`: add `freq_hint: Option<f32>`,
    forward to `pipeline::decode_frame_subtract`.
  - Legacy shims `decode_frame_with_cache` and `decode_frame_subtract`
    pass `None` to keep their behaviour unchanged.
  - Compile-shape tests for both updated `_with_options` variants pass
    `None` for `freq_hint` while iterating the 3×3 depth × strictness
    matrix.
- `mfsk-core/src/fst4/decode.rs`
  - `decode_frame_with_cache_and_options`: add `freq_hint: Option<f32>`,
    forward to `pipeline::decode_frame`.
  - Legacy shim `decode_frame_with_cache` passes `None`.
  - Compile-shape test updated.

## Verification

Local CI gates on the user's macOS dev box (Rust release toolchain):

- `cargo build -p mfsk-core --features full --release` — pass.
- `cargo clippy -p mfsk-core --features full --all-targets -- -D warnings` — pass.
- `cargo fmt --check` — pass.
- `cargo test -p mfsk-core --features full --release --no-fail-fast` — pass except for the
  two pre-existing missing-WAV failures (`qso3_apoff_meets_wsjtx_golden_floor`,
  `qso3_apoff_meets_jtdx_recall_floor`), which depend on test fixtures
  not present in the repo and are unrelated to this change.

No external callers of these functions exist inside the workspace, so
the parameter additions do not require updates outside `mfsk-core`.

## Known coverage gaps

Acknowledging review limitations to preempt the next round:

- The 3×3 depth × strictness compile-shape tests cover only the
  `freq_hint = None` path. `freq_hint = Some(_)` is **not exercised**
  against the three new siblings (`decode_frame_with_cache_and_options`
  in ft4/fst4 and the ft4 `_subtract_with_options` variant). A silent-drop
  regression — where the parameter is accepted but never reaches
  `pipeline::decode_frame` — would not be caught at this layer.
- Marginal-signal behavioural tests for the depth-vs-strictness coupling
  (e.g. proving `Deep + Strict` recovers a frame that `Normal + Loose`
  does not) are **out of scope** for this plumbing PR. The new parameters
  forward to existing, separately-tested pipeline machinery; behavioural
  validation belongs in a follow-up that runs the harness against a
  known-marginal corpus.
- Separately, the `freq_hint` docstring on the new options entrypoints
  describes the parameter as "narrowing coarse_sync". The current
  implementation in `core/sync.rs` actually re-sorts candidates by
  ±10 Hz proximity rather than truncating the search. This is a
  pre-existing mfsk-core-wide doc/impl drift, not introduced by this
  PR; tracked for a docs-only follow-up.
