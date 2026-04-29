# uvpacket — NFM voice-channel packet protocol (applied example)

> **日本語版:** [UVPACKET.ja.md](UVPACKET.ja.md)

`uvpacket` is an in-tree applied example of how `mfsk-core`'s FEC
infrastructure (`Ldpc240_101`, belief propagation, OSD-2) can be
reused outside the WSJT-X family. It is **not** a member of that
family. It targets a different design point — narrow-FM voice
channels (HT/mobile, ~3 kHz audio passband) intended for private-
group amateur-radio messaging (signed QSL exchange, short text,
position reports).

This document covers the positioning, the design choices, the
characterisation results, and the comparison with AX.25 / AFSK 1200
(the only fair baseline at this design point). For the API surface
see the in-source rustdoc.

## 1. Positioning

### 1.1 What problem this solves

AX.25 / AFSK 1200 has been the de facto NFM digital protocol for
~40 years. It has two characteristics that motivated this
experiment:

1. **No FEC.** A single bit error anywhere in the frame is fatal.
   On a clean channel that's fine; on a fading channel a single
   deep null kills frames that would otherwise carry useful range.
2. **Non-coherent BFSK** at 1200 baud, audio centre 1700 Hz. The
   demod has no phase reference and pays the textbook ~3 dB
   non-coherent penalty.

If you accept those two as fixed, the only knob you can turn for
better range is transmit power. uvpacket asks: *if we replace the
modem and add an LDPC code that fits inside the same NFM audio
passband, how much margin do we get?*

### 1.2 What this is **not**

- Not a public APRS replacement. Wide deployment requires
  interoperability with existing TNCs and that ship has sailed.
  uvpacket is for **private groups** running the same software on
  both ends.
- Not a voice mode. M17, D-STAR, DMR / NXDN are voice-primary
  protocols with optional data subchannels. uvpacket is data-only.
- Not a wideband mode. VARA FM uses ~12.5 kHz of bandwidth and
  delivers ~25 kbps; uvpacket fits in NFM voice (~3 kHz) and
  delivers 1–1.8 kbps. Different leagues.

### 1.3 Where this fits

| Mode | net bps | Bandwidth | AWGN threshold (SNR_3kHz) | FEC | Open source |
|---|---:|---|---:|---|:-:|
| AX.25 / AFSK 1200 | 1200 | NFM (~3 kHz) | +10 dB | none | ✓ |
| uvpacket Robust | 1008 | NFM | **+3 dB** | LDPC 0.42 | ✓ |
| uvpacket Express | 1800 | NFM | **+5–6 dB** | LDPC 0.75 | ✓ |
| M17 (4-FSK) | 4800 | ~9 kHz | +5–7 dB | conv | ✓ |
| D-STAR DV | 4800 | 6.25 kHz | ~+10 dB CNR | Golay | partial |
| DMR / NXDN | 4-FSK | 6.25 / 12.5 kHz | ~+7–8 dB CNR | BCH | (commercial) |
| VARA FM | ~25000 | 12.5 kHz | +10 dB | proprietary | ✗ |

uvpacket sits in a niche that turns out to be relatively empty: an
**open-source data-only NFM digital protocol with FEC and decent
fading tolerance, in the AX.25 audio-bandwidth slot**. It is not
revolutionary versus M17/VARA on different design points, but it is
a clear improvement over AX.25 in its own.

## 2. Design

### 2.1 Modulation

Single-carrier coherent **QPSK at 1200 baud**, root-raised-cosine
pulse (α = 0.5, span 6 sym), audio centre 1500 Hz at 12 kHz sample
rate. The QPSK constellation is Gray-mapped:

| `(b1, b0)` | constellation point |
|---:|:--|
| (0, 0) | +1 + 0j |
| (0, 1) | 0 + 1j |
| (1, 0) | 0 − 1j |
| (1, 1) | −1 + 0j |

The TX peak-normalises the burst envelope to ≤ 1; RMS sits around
0.2–0.5 (~7 dB PAPR is normal for RRC-shaped QPSK at α = 0.5).

### 2.2 Preamble + pilots

Frame head is a **31-bit BPSK m-sequence** (Fibonacci LFSR,
polynomial x⁵ + x² + 1, initial state `[0, 0, 0, 0, 1]`). 31 chips
× 1 sym/chip = 26 ms preamble at 1200 baud. Cyclic autocorrelation
sidelobes are bounded by 1/31 ≈ −15 dB amplitude — clean correlator
peak for symbol-timing acquisition, frame detection, and initial
carrier-phase reference.

After the preamble, **one known QPSK pilot symbol every 32
transmitted symbols** (≈ 3 % overhead). The pilot constellation
point is +1 + 0j. The RX builds a per-symbol phase reference by
linearly interpolating between consecutive pilot anchors; a full
decision-directed PLL is overkill at this pilot density and channel
coherence time.

### 2.3 FEC

Reuses [`Ldpc240_101`] from FST4 as the rate-0.42 mother code (101
info bits → 240 channel bits per block). The four sub-modes apply
puncturing chosen by **kSR-greedy puncture-set selection**
(Ha–McLaughlin) to the 139 parity bits:

| Sub-mode | rate | Puncture | Net bps | Posture |
|---|---:|---:|---:|---|
| Robust | 0.42 | 0 % | 1008 | mountain / weak signal / deep fading |
| Standard | 0.50 | 30 % | 1200 | typical NFM with fading |
| Fast | 0.66 | 63 % | 1600 | good-signal default |
| Express | 0.75 | 76 % | 1800 | strong-signal headline-fast (OSD-2 mandatory) |

kSR-greedy delivers ~1–3 dB Eb/N0 gain over uniform-spread
puncturing at the deeper rates, which is what makes Express viable
at all (uniform-spread fails to converge at 76 % parity puncture).

### 2.4 Frame structure

- Variable length: 1–32 LDPC blocks per frame.
- Each LDPC block carries 96 info bits (12 byte) padded to the
  FEC's 101-bit input. The remaining 5 bits per block carry a
  **D-iii spread copy** of the 32-bit frame header (header replicated
  ~7 times across the frame for slow-path recovery — currently the
  fast path simply uses block 0's CRC-validated header).
- 4-byte frame header: mode (2b) + block count (5b) + app type
  (4b) + sequence (5b) + CRC-16 (16b).
- **Block-interleaver** across all codewords in the frame spreads
  fade-burst erasures across every codeword — the deeper the
  Rayleigh null, the more it gets diluted.

### 2.5 Application API

Byte-pipe — bypasses `mfsk-core`'s `MessageCodec`. Callers deliver
raw bytes plus a 4-bit `app_type` tag. The modem does not interpret
the bytes. Suggested allocation:

| `app_type` | Use |
|---:|---|
| 0 | raw / experimentation |
| 1 | signed QSL exchange |
| 2 | position beacon |
| 3 | short text |
| 4 | ARQ ACK |
| 5–15 | user-defined |

## 3. Characterisation

### 3.1 AWGN

Phase 2'a sweep, 30 trials per cell, 4-block frame, 44-byte payload.
Eb/N0 is **per information bit** (cross-mode-fair convention used
throughout the WSJT family). σ is calibrated from per-burst
measured signal power — see [§4](#4-snr-calibration-history) for
why this matters.

```
mode      eb/n0 (dB)  -2  0  2  4  6  8 10 12 14 16 18 20 22
─────────────────────────────────────────────────────────────
Robust                 0  0  0  9 26 29 30 30 30 30 30 30 30
Standard               0  0  1 15 27 30 30 30 30 30 30 30 30
Fast                   0  0  1 19 27 30 30 30 30 30 30 30 30
Express                0  0  1 16 29 30 30 30 30 30 30 30 30
```

All four modes hit **50 % PER at +4 dB** Eb/N0_info, **100 % PER
at +8 dB**. This matches QPSK + rate-0.42–0.75 LDPC theory (~1–2 dB
code gain over uncoded QPSK at 1e-2 BER).

### 3.2 Rayleigh flat fading

Phase 2'b sweep, 30 trials per cell, 4-block frame, 20-byte payload.

```
mode      doppler  +10  +12  +15  +20  +25  +30  +35  (Eb/N0_info dB)
──────────────────────────────────────────────────────────────────
Robust    1 Hz     25   26   30   30   30   30
Robust    5 Hz     24   30   30   30   30   30
Robust    10 Hz    19   21   26   29   30   30
Standard  1 Hz     23   27   29   30   30   30
Standard  5 Hz     23   30   30   30   30   30
Standard  10 Hz    19   21   27   30   30   30
Fast      1 Hz     17        29   30   30   30   30
Fast      5 Hz     23        29   30   30   30   30
Fast      10 Hz    22        29   30   30   30   30
Express   1 Hz     16        26   30   30   30   30
Express   5 Hz     19        29   30   30   30   30
Express   10 Hz    19        29   30   30   30   30
```

≥ 90 % PER thresholds:

- **Robust / Standard**: +12 dB at 1–5 Hz Doppler, +15 dB at 10 Hz
- **Fast / Express**: +15 dB across all Doppler

These are realistic fading-tolerance numbers for VHF/UHF mobile NFM
channels.

### 3.3 Comparison with AX.25 / AFSK 1200

For a 256-byte AX.25 frame at FER ≤ 1 %, BER must be ≤ ~5e-6. Non-
coherent BFSK reaches that BER at Eb/N0 ≈ 14 dB. Translating to
SNR in a 3 kHz audio passband:

- **AFSK 1200**: 14 + 10·log₁₀(1200/3000) = **+10 dB SNR_3kHz**
- **uvpacket Robust**: 8 + 10·log₁₀(1008/3000) = **+3 dB SNR_3kHz**
- **uvpacket Express**: ~8 + 10·log₁₀(1800/3000) = **+6 dB SNR_3kHz**

uvpacket Robust is **~7 dB** better than AX.25 at comparable
throughput; Express is **~4 dB** better while delivering 50 % more
net bps. On Rayleigh fading the gap widens (AX.25 has no FEC and
its frames are atomic against any single-bit erasure; uvpacket's
FEC + interleaver dilute fade bursts across every codeword).

### 3.4 The +9–10 dB FM-threshold floor

Both modems sit on top of FM detection. Below CNR ≈ +9–10 dB the FM
discriminator output is dominated by impulse noise and **both
modems fail catastrophically**. The audio-domain Eb/N0 numbers
above are meaningful only above the FM threshold; below it, neither
protocol decodes. This is a property of the channel, not the
modems.

To get below the FM threshold you need a different on-air
modulation (SSB digital, direct IQ digital), which is outside the
scope of this experiment.

## 4. SNR calibration history

The Phase 1 4-FSK design (h = 0.5, GFSK BT = 0.5) showed a ~+11 dB
SNR threshold gap versus theory. Two contributions:

1. **Tone non-orthogonality**: at h = 0.5, the orthogonality
   integral `sinc(Δf · T_sym) = sinc(0.5) ≈ 0.637` — adjacent tones
   leak 64 % of their energy, breaking max-likelihood symbol
   detection. This was the root cause and motivated the modulation
   pivot to QPSK (I/Q axes are orthogonal by construction).
2. **σ-formula miscalibration**: the AWGN harness assumed a
   constant-envelope `P = 0.5` signal. After the QPSK pivot, RRC-
   shaped QPSK has RMS ≈ 0.22 (peak-normalised to 1), so the stated
   Eb/N0 was off by ~10 dB. Phase 2'a recalibrated the formula to
   take per-burst measured signal power.

The numbers in §3 are post-recalibration and are cross-modulation-
comparable. The tx-side burst power is measured via
`signal_power(audio) = mean(audio²)` and fed into
`awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power)` — see
`mfsk-core/tests/common/channel.rs`.

## 5. Audio samples

Representative WAV files for ear-level inspection live at
`audio_samples/uvpacket/` in this repository. All are 12 kHz mono
16-bit PCM, with 200 ms of leading/trailing silence:

| File | Mode | Channel | Decode |
|---|---|---|:-:|
| `uv_robust_clean.wav` | Robust, 4 blocks, 20 B | clean | ✓ |
| `uv_robust_awgn_+8db.wav` | Robust | AWGN +8 dB Eb/N0 | ✓ |
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (50 % PER region) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✗ |
| `uv_robust_rayleigh_5hz_+15db.wav` | Robust | 5 Hz Rayleigh, +15 dB | ✓ |
| `uv_express_clean.wav` | Express, 4 blocks, 20 B | clean | ✓ |

To regenerate them:

```sh
cargo run --release --features uvpacket --example uvpacket_samples
```

The clean Robust burst is ~440 ms; Express is ~270 ms. The audible
character is "narrow-band data buzz" — the RRC pulse spreads each
QPSK symbol across multiple tones, so the spectrum is roughly
flat across `[1500 − 600, 1500 + 600] Hz` with raised-cosine
shoulders. To a human ear it sounds fairly close to AFSK 1200,
slightly more "smeared" because the constellation moves through
all four phases rather than two tones.

## 6. Implementation pointers

| Layer | File |
|---|---|
| Protocol ZSTs / sub-mode parameters | [`mfsk-core/src/uvpacket/protocol.rs`](../mfsk-core/src/uvpacket/protocol.rs) |
| Frame header + CRC + bit packing | [`mfsk-core/src/uvpacket/framing.rs`](../mfsk-core/src/uvpacket/framing.rs) |
| Puncture sets (kSR-greedy) | [`mfsk-core/src/uvpacket/puncture.rs`](../mfsk-core/src/uvpacket/puncture.rs) |
| Block interleaver | [`mfsk-core/src/uvpacket/interleaver.rs`](../mfsk-core/src/uvpacket/interleaver.rs) |
| Preamble + pilot definitions | [`mfsk-core/src/uvpacket/sync_pattern.rs`](../mfsk-core/src/uvpacket/sync_pattern.rs) |
| TX (bytes → audio) | [`mfsk-core/src/uvpacket/tx.rs`](../mfsk-core/src/uvpacket/tx.rs) |
| RX (audio → bytes) | [`mfsk-core/src/uvpacket/rx.rs`](../mfsk-core/src/uvpacket/rx.rs) |
| AWGN + Rayleigh harness | [`mfsk-core/tests/common/channel.rs`](../mfsk-core/tests/common/channel.rs) |
| Threshold sweeps | [`mfsk-core/tests/uvpacket_awgn.rs`](../mfsk-core/tests/uvpacket_awgn.rs), [`uvpacket_rayleigh.rs`](../mfsk-core/tests/uvpacket_rayleigh.rs) |

## 7. License

GPL-3.0-or-later, matching the rest of `mfsk-core`. The LDPC mother
code is derived from WSJT-X (`lib/fst4/`).
