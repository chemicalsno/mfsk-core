# Release notes — mfsk-core 0.3.3

> Status: pre-release on the `multichannel-aloha` branch.

Multi-channel SSB receive + slotted-ALOHA TX primitives for
uvpacket. Builds on 0.3.2 AFC for SSB single-station use, and
generalises to a private group sharing one RF SSB channel
(e.g., 430.090 MHz USB) where each TX picks a random free
audio slot via LBT.

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) and the
existing single-channel uvpacket API are unchanged. No breaking
API changes — the new functions are additive.

## What's new

```rust
use mfsk_core::uvpacket::rx::{
    MultiChannelOpts, SlotEnergy, decode_multichannel, measure_slot_energies,
};

// RX: decode every frame in the SSB passband, returning each
// frame's detected audio centre alongside the frame.
let frames: Vec<(f32, DecodedFrame)> = decode_multichannel(
    &audio,
    &MultiChannelOpts::default(),  // 300–2700 Hz, 25 Hz coarse, 600 Hz NMS
    &fec_opts,
);

// TX-side LBT survey: mean MF energy per 1200 Hz slot.
let slots: Vec<SlotEnergy> = measure_slot_energies(
    &recent_audio,
    &MultiChannelOpts::default(),
    1200.0,
);
```

The application combines `measure_slot_energies` with its own
RNG to implement the slotted-ALOHA TX step:

```rust
let slots = measure_slot_energies(&recent_audio, &mc, 1200.0);
let mut mags: Vec<f32> = slots.iter().map(|s| s.mean_mf_magnitude).collect();
mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
let median = mags[mags.len() / 2];
let free: Vec<f32> = slots
    .iter()
    .filter(|s| s.mean_mf_magnitude < median * 2.0)  // 3 dB threshold
    .map(|s| s.audio_centre_hz)
    .collect();
let centre = match free.len() {
    0 => return Err(AllSlotsBusy),  // back off, retry
    n => free[rng.gen_range(0..n)],
};
let burst = tx::encode(&header, &payload, centre)?;
```

mfsk-core ships **no RNG dependency** — the application supplies
its own randomness (`rand::Rng`, browser `crypto.getRandomValues`,
`/dev/urandom`, …).

## Algorithm

`decode_multichannel`:

1. Coarse-grid frequency scan at `coarse_step_hz` (default 25 Hz)
   across `[band_lo_hz, band_hi_hz]`. At each candidate centre,
   matched-filter the audio and find time-axis preamble peaks.
2. Frequency-axis NMS — drop peaks within `nms_radius_hz`
   (default 600 Hz) of a stronger peak; this collapses
   adjacent-grid-point detections of the same signal.
3. Per-peak decode at the picked centre via the existing
   `decode_known_layout_with_opts` (no inner AFC needed —
   the LMS phase fit absorbs the ≤ 12.5 Hz coarse-grid
   residual).

`measure_slot_energies`:

1. Enumerate slot centres at `slot_spacing_hz` spacing inside
   the band, starting half a slot in.
2. At each centre, matched-filter the audio and report
   `mean(|mf|²)` as the slot's energy.
3. Caller decides what counts as "free" / "busy".

## Why slotted ALOHA + LBT, not CSMA/CD

CSMA/CD proper isn't applicable to half-duplex SSB radio (can't
reliably listen while keying the transmitter). Slotted ALOHA +
LBT + application-layer ARQ behaves equivalently with much less
mechanism, and lines up with the natural amateur-radio "watch
the frequency, find a clear spot, transmit" practice.

## Channel grid

| SSB passband | Slots | Slot centres (Hz) |
|---|---:|---|
| 2.4 kHz | 2 | 800, 2000 |
| 2.7 kHz | 2 | 850, 2150 |
| 3.0 kHz | 2 | 900, 2100 |

A single uvpacket signal occupies ~1800 Hz end-to-end. 1200 Hz
adjacent-slot separation gives < −20 dB inter-slot
interference — comfortable.

## Cost

- `decode_multichannel`: ~96 matched-filter passes per
  call with the default 300–2700 Hz / 25 Hz config. ≈ 70 ms in
  release for a 1-second audio buffer.
- `measure_slot_energies`: 1 MF pass per slot. ~2 ms for a
  2-slot grid. Effectively free vs the RX scan.
- Per-peak decode: the existing `decode_known_layout_with_opts`
  cost (~10 ms × `(mode × n_blocks)` worst-case ≈ 0.3–1 s on
  a marginal-SNR signal).

## Empirical validation

- Two simultaneous frames at 800 Hz and 2000 Hz centres in clean
  audio: both decoded with detected centres within ±50 Hz of
  the true value.
- Same setup at +8 dB Eb/N0_info AWGN: both decoded.
- One frame at 800 Hz, slot survey across the default band: the
  900 Hz slot reports > 5× the energy of the 2100 Hz slot.

## Known limitations / out of scope

- Voice-mode coexistence (FFT-based wideband detector for mixed
  voice + data on a public channel): future cycle. The MF-based
  survey is biased toward narrowband uvpacket-shaped signals.
- ARQ / retransmit logic: application-layer.
- RNG: application-layer.
- Adaptive slot spacing: stays at 1200 Hz (mc_opts band edges
  let you place slots wherever, but no narrower-than-1200 Hz
  inter-channel-interference correction).

## Migration

No code changes required for downstream consumers. New
consumers wanting multi-channel pull in 0.3.3 unconditionally
(no new feature flag).
