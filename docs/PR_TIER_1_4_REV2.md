# PR #22 — Rev 2 fix pass

Branch: `feat/ft8-decode-frame-with-ap`

This revision addresses the second round of Gemini review feedback on top of the
Rev 1 patch (which added `decode_frame_with_ap_full`,
`decode_frame_subtract_with_ap`, `decode_frame_subtract_with_known_and_ap`, and
the `ap_hint` doc paragraph on `decode_frame_inner`).

## Issues addressed

### 1. HIGH — caller-supplied `known` signals were never subtracted from the SIC residual

Location: `mfsk-core/src/ft8/decode.rs::decode_frame_subtract_with_known_and_ap`
(and, by extension, the `decode_frame_subtract_with_known` shim that now
forwards into it).

#### Symptom

In a Phase 1 → Phase 2 pipelined decode, `decode_frame_subtract_with_known` is
called with `known = phase1_results`. The intent of the function is successive
interference cancellation: strong signals already known from Phase 1 should be
subtracted from the residual so that Phase 2's three additional subtract passes
operate on a cleaner band where weaker signals are no longer masked.

The Rev 1 implementation pushed `known` into `all_results` for dedup purposes
but never subtracted them from `residual`. Every Pass 1 / Pass 2 iteration ran
on the original audio — defeating the purpose of pipelined SIC.

#### Provenance

The bug pre-dates this PR. It was introduced in commit `48b1f37`
(`feat(ft8): WSJT-X-faithful decode pipeline + JTDX-level recall`). The Rev 1
AP refactor inherited the same logic verbatim. Both functions are fixed by the
same patch since the non-AP entry point is now a shim into the AP version.

#### Fix

After Pass 0 (which still uses the precomputed FFT against the original audio
to discover signals Phase 1 missed), every signal in `known` is subtracted
from `residual` using `subtract_signal_weighted` with the standard `qsb_partial_gain`
weighting. The same is done for the newly discovered Pass 0 signals before
Pass 1 begins. A `residual_dirty` flag tracks whether the FFT cache is still
valid for reuse on Pass 0; if Pass 0 itself dirtied the residual via
subtraction, we'd already be past the cache-reuse window anyway.

```text
before:
  Pass 0: decode(residual=audio, fft=cache, known)              -> new
          subtract(new from residual)                            // known never subtracted
  Pass 1: decode(residual /* still has known */, fft=fresh, ...)
  Pass 2: decode(residual /* still has known */, fft=fresh, ...)

after:
  Pass 0: decode(residual=audio, fft=cache, known)              -> new
          subtract(known from residual)                          // <-- THE FIX
          subtract(new   from residual)
  Pass 1: decode(residual /* clean */, fft=fresh, ...)
  Pass 2: decode(residual /* clean */, fft=fresh, ...)
```

### 2. MEDIUM — redundant FFT-cache clone in `decode_frame_inner`

Location: `mfsk-core/src/ft8/decode.rs::decode_frame_inner`

The function had two `match precomputed_fft { Some(c) => c.to_vec(), None => build_fft_cache(audio) }`
blocks: one inside the `candidates.is_empty()` early-exit branch and one for
the main path. On the candidates path the cache was built (or cloned) just
once, but the early-exit branch was a duplicate code site. Hoisting the
construction above the branch removes the duplication.

### 3. Audit beyond the literal comments

- `decode_frame_subtract` and `decode_frame_subtract_with_ap` do not accept a
  `known` parameter, so the SIC bug does not apply to them. Their internal
  passes already subtract every signal they discover.
- `decode_frame_with_ap_full`, `decode_frame_with_ap`, and `decode_frame_with_cache`
  all forward into `decode_frame_inner` with `known = &[]` and consistent
  `EqMode::Off` / `precomputed_fft = None`. The FFT cache returned by
  `decode_frame_with_ap_full` is the same `Vec<Complex<f32>>` produced by
  `decode_frame_inner` (no extra clone in the wrapper).
- Parameter parity between every `_with_ap` function and its non-AP sibling
  was rechecked. Each AP version takes the non-AP signature plus an
  `ap_hint: Option<&ApHint>` last parameter. Strictness, depth, max_cand,
  freq_min/max, sync_min, freq_hint, eq_mode are all preserved.

## Regression test

`decode_frame_subtract_with_known_and_ap_subtracts_known_before_phase2`
(in `mfsk-core/src/ft8/decode.rs::tests`) directly exercises the bug.

Strategy:

1. Synthesize a strong clean FT8 signal A at 1500 Hz inside a 15 s frame.
2. Phase-1-decode it via `decode_frame` to obtain a real `DecodeResult`
   (the `sync_cv`, `freq_hz`, `dt_sec` it carries are required for accurate
   waveform reconstruction inside `subtract_signal_weighted`).
3. Call a `#[cfg(test)]` wrapper
   `decode_frame_subtract_with_known_and_ap_debug_residual` that mirrors the
   production function but additionally returns the post-pass residual
   buffer.
4. Compute narrow-band (±1 Hz) DFT energy at A's center frequency in the
   original audio vs. the residual.
5. Assert `residual_energy * 2 < original_energy`.

Verified failure-with-bug: temporarily stripping the `known` subtract block
from the debug variant (leaving everything else intact) makes the test fail
with `e_after = e_before` (no reduction whatsoever). With the fix the residual
energy at the carrier drops by orders of magnitude.

## CI gates (local)

- `cargo build -p mfsk-core --features full --release` — pass
- `cargo clippy -p mfsk-core --features full --all-targets -- -D warnings` — pass
- `cargo fmt --check` — pass
- `cargo test -p mfsk-core --features full --release` — 332 lib tests pass,
  all integration tests pass, only `qso3_apoff_meets_wsjtx_golden_floor` fails
  due to the missing on-air WAV fixture (pre-existing, unrelated).
