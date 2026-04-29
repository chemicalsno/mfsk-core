# Release notes — mfsk-core 0.3.2

> Status: pre-release on the `devel` branch. Tag + crates.io publish
> follow the same workflow as 0.3.1.

Focused single-feature release on top of 0.3.1: **AFC (automatic
frequency control) for uvpacket**, opening up SSB use without
requiring TX/RX VFO-dial alignment.

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) and the
0.3.1-shipped uvpacket NFM path are unchanged. No breaking API
changes — AFC is opt-in via a new entry-point function.

## What's new

```rust
use mfsk_core::uvpacket::rx::{decode_known_layout_with_afc, AfcOpts};

let frame = decode_known_layout_with_afc(
    &audio,
    sample_offset,
    audio_centre_hz_nominal,
    mode,
    n_blocks,
    &fec_opts,
    &AfcOpts::default(),  // ±200 Hz search window
)?;
```

The AFC sweeps `audio_centre_hz + Δf_test` in 25 Hz steps across
`[−search_hz, +search_hz]`, runs the matched filter at each
candidate, and picks the Δf where the preamble-correlation
magnitude peaks. Parabolic refinement of the three adjacent
coarse-grid magnitudes drives the final Δf to within a fraction
of the grid spacing.

Empirical: ≤ 0.01 Hz error at multi-of-25-Hz Δf, ≤ 2.5 Hz error
at mid-grid Δf — well inside the LMS phase fit's residual
absorption capacity.

## Why frequency-grid and not FFT-over-chip-rate

The first-attempt FFT approach (cheap and natural-looking) fails
because the integer-sample preamble correlator that picks
`best_off` itself rolls off as `sinc(δ · 31 / 1200)`. At
`|δ| ≳ 20 Hz` the sinc dives below 0.5 and `best_off` lands on
noise samples — the FFT then operates on garbage. The
frequency-grid search sidesteps this entirely by searching for
the Δf at which the preamble correlator magnitude itself peaks.

## Cost

~17× single-decode cost (full down-convert + matched-filter at
each grid point), ~50–100 ms total per attempted decode in
release mode. Tolerable for opportunistic SSB decode; can be
tightened if profiling demands.

NFM users keep using `decode_known_layout` — AFC is pure
overhead on a static-VFO channel.

## Operating envelope (combined with 0.3.1's modem)

| Channel | Modem threshold | Channel CNR floor | Margin |
|---|---:|---:|---:|
| NFM | −3.7 dB SNR_3kHz Robust | +20 dB SNR_3kHz (FM threshold) | ~24 dB |
| SSB (with AFC) | −3.7 dB SNR_3kHz Robust | (no FM threshold floor) | modem-limited |

On SSB the modem operates to its true threshold; HF
weak-signal data and microwave SSB are in scope.

## Known limitations

- AFC is per-frame static. Doppler-induced carrier drift across
  the burst is still absorbed by the LMS phase fit (constant +
  linear + quadratic), which copes with ≤ ~10 Hz/s drift —
  typical for HF / VHF / UHF SSB.
- The auto-detect `decode()` path doesn't yet take an `AfcOpts`.
  Multi-frame SSB scans go through `decode_known_layout_with_afc`
  with caller-managed framing for now.
