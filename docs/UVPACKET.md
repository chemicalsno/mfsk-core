# uvpacket — applied example: NFM voice-channel packet protocol

> **日本語版:** [UVPACKET.ja.md](UVPACKET.ja.md)

`uvpacket` is an in-tree applied example of how `mfsk-core`'s FEC
infrastructure (`Ldpc240_101`, belief propagation, OSD-2/3) can be
reused outside the WSJT-X family. It is **not** a member of that
family. It targets a different design point — narrow-FM voice
channels (HT/mobile, ~3 kHz audio passband) intended for private-
group amateur-radio messaging (signed QSL exchange, short text,
position reports).

This document covers the design choices, the characterisation
results, and the known modem implementation loss. For the API
surface see the in-source rustdoc.

## 1. Scope

### 1.1 What this is

A **four-mode packet modem** that fits inside an NFM voice
passband, layered on a hand-tuned irregular LDPC mother code from
FST4. Designed for **private groups** running the same software on
both ends — not a public protocol replacement, not interoperable
with anything else.

### 1.2 What this is not

- Not an interoperable mode. No standardisation, no TNC support.
- Not a voice mode. Data only.
- Not a wideband mode. Fits in NFM voice (~3 kHz), nominal 1–1.8
  kbps net throughput. Different design point from M17 / D-STAR /
  DMR / VARA FM.
- Not a weak-signal mode. Aimed at the operating envelope above the
  FM threshold (CNR ≥ +9–10 dB), which is the channel's own
  irreducible floor for any FM-detected mode.

### 1.3 Where it sits

uvpacket fills an open-source niche: an LDPC-coded data-only NFM
modem with a coherent QPSK physical layer, sub-second burst
duration, and graceful rate ladder for opportunistic throughput.

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
sidelobes are bounded by 1/31 ≈ −15 dB amplitude — a clean
correlator peak for symbol-timing acquisition, frame detection,
and initial carrier-phase reference.

After the preamble, **one known QPSK pilot symbol every 32
transmitted symbols** (≈ 3 % overhead). The pilot constellation
point is +1 + 0j. The RX builds a per-symbol phase reference by
linearly interpolating between consecutive pilot anchors. A
**per-block decision-directed correction** (see §4) is then applied
on top of the pilot interpolation to absorb the average within-
block phase-tracking residual.

### 2.3 FEC

Reuses [`Ldpc240_101`] from FST4 as the rate-0.42 mother code (101
info bits → 240 channel bits per block). The four sub-modes apply
puncturing chosen by **kSR-greedy puncture-set selection**
(Ha–McLaughlin) to the 139 parity bits:

| Sub-mode | rate | Puncture | Net bps | Posture |
|---|---:|---:|---:|---|
| Robust | 0.42 | 0 % | 1008 | maximum-margin posture |
| Standard | 0.50 | 30 % | 1200 | typical NFM with fading |
| Fast | 0.66 | 63 % | 1600 | good-signal default |
| Express | 0.75 | 76 % | 1800 | strong-signal headline (OSD-3 mandatory) |

kSR-greedy delivers ~1–3 dB Eb/N0 gain over uniform-spread
puncturing at the deeper rates, which is what makes Express viable
at all (uniform-spread fails to converge at 76 % parity puncture).

### 2.4 Frame structure

- Variable length: 1–32 LDPC blocks per frame.
- Each LDPC block carries 96 info bits (12 byte) padded to the
  FEC's 101-bit input. The remaining 5 bits per block carry a
  **D-iii spread copy** of the 32-bit frame header (header
  replicated ~7 times across the frame for slow-path recovery —
  the current fast path simply uses block 0's CRC-validated header).
- 4-byte frame header: mode (2b) + block count (5b) + app type
  (4b) + sequence (5b) + CRC-16 (16b).
- **Block-interleaver** across all codewords in the frame spreads
  fade-burst erasures across every codeword.

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

### 3.1 LDPC layer (modem-bypassed reference)

`tests/uvpacket_ldpc_direct.rs` feeds Gaussian-noise LLRs straight
into the LDPC decoder, calibrated for `Eb/N0_info` per channel bit.
This isolates the FEC from the modem and gives the **theoretical
ceiling** the QPSK end-to-end pipeline aspires to:

```
mode      eb/n0 (dB)  -2  -1   0   1   2   3   4
─────────────────────────────────────────────────
Robust                 0   2   6  21  28  30  30
Standard               0   1   5  20  29  30  30
Fast                   0   1   6  22  26  30  30
Express                0   0   0  14  24  29  30
```

50 % PER thresholds: Robust ≈ +0.5 dB, Standard / Fast ≈ +0.7 dB,
Express ≈ +1.5 dB. The mother code's design rate is 0.42 so Robust
holds a ~1 dB lead at the FEC layer.

### 3.2 QPSK end-to-end (modem + FEC)

`tests/uvpacket_demod_diagnostic::awgn_threshold_finder_per_mode`,
30 trials per cell, 4-block frame, 44-byte payload, OSD-3:

```
mode      eb/n0 (dB)  -2  0  2  4  6  8 10 12 14 16 18 20 22
─────────────────────────────────────────────────────────────
Robust                 0  0  3 19 27 30 30 30 30 30 30 30 30
Standard               0  0  4 21 30 30 30 30 30 30 30 30 30
Fast                   0  0  3 20 29 30 30 30 30 30 30 30 30
Express                0  0  1 19 30 30 30 30 30 30 30 30 30
```

50 % PER threshold ≈ **+3.7 dB** Eb/N0_info, 100 % PER threshold ≈
**+6–8 dB**. Modem implementation loss versus the LDPC-only ceiling
is **~3 dB across all modes**.

The Robust LDPC advantage from §3.1 is largely consumed by the
modem implementation loss — see §4. Robust does win below the
modem floor where the higher-rate modes catastrophically fail, but
that regime is operationally narrow.

### 3.3 Rayleigh flat fading

`tests/uvpacket_rayleigh.rs`, 30 trials per cell, 4-block frame,
20-byte payload:

```
mode       Doppler  +10  +12  +15  +20  +25  +30  +35  (Eb/N0_info dB)
──────────────────────────────────────────────────────────────────
Robust     1 Hz     —    —    30   30   30   30   —
Robust     5 Hz     28   30   30   30   30   30   —
Robust    10 Hz     20   24   29   30   30   30   —
Standard   1 Hz     24   28   29   30   30   30   —
Standard   5 Hz     26   30   30   30   30   30   —
Standard  10 Hz     19   21   27   30   30   30   —
Fast       1 Hz     —    —    —    30   30   30   30
Fast       5 Hz     23   —    29   30   30   30   30
Fast      10 Hz     23   —    30   30   30   30   30
Express    1 Hz     18   —    26   30   30   30   30
Express    5 Hz     22   —    29   30   30   30   30
Express   10 Hz     22   —    29   30   30   30   30
```

≥ 90 % PER thresholds: **Robust ~+10–12 dB at 1–5 Hz Doppler,
~+15 dB at 10 Hz**; Express ~+15 dB across all Doppler. Realistic
fading-tolerance for VHF/UHF mobile NFM channels.

### 3.4 The +9–10 dB FM-threshold floor

The modem sits on top of FM detection. Below CNR ≈ +9–10 dB the FM
discriminator output is dominated by impulse noise and **the modem
fails catastrophically**. The audio-domain Eb/N0 numbers above are
meaningful only above the FM threshold; below it, no audio-domain
demod recovers. This is a property of the channel, not the modem.

To get below the FM threshold a different on-air modulation (SSB
digital, direct IQ digital) is needed, which is outside the scope
of this experiment.

## 4. Known modem implementation loss

The ~3 dB gap between the LDPC-only threshold (§3.1) and the QPSK
end-to-end threshold (§3.2) is the modem implementation loss. It
breaks down approximately as:

- **~1.5 dB irreducible floor** at moderate-to-high SNR — likely a
  combination of finite-span RRC mismatch, `signal_power` formula
  treating preamble/pilot energy as data energy, and amplitude /
  σ_n estimator noise.
- **~1.5 dB low-SNR penalty** that shrinks as SNR rises — pilot-
  interpolated phase tracking has variance proportional to channel
  σ²_n, so pilot phase estimates are noisier exactly when the
  modem is closest to its threshold. Robust suffers most because
  its σ_n is largest at fixed Eb/N0_info.

The current rx (commit history through `Phase 2'a improvements`)
implements:

- σ-aware LLR scaling: `LLR = (A / σ²_n) · qpsk_max_log(r_derot)`
- Magnitude-based σ²_n estimator from data symbols:
  `σ²_n = (E[|r|²] − A²) / 2`
- One-pass per-block decision-directed phase correction (DDPT):
  hard-decide each data symbol, accumulate residual phase per LDPC
  block, apply as a constant correction on top of pilot interp
- OSD-3 fallback (mandatory for Express, marginal-but-positive for
  the others)

These took ~0.4 dB out of the original ~3.5 dB modem loss. The
remaining ~1.5–3 dB requires deeper changes (denser pilots → TX
format change, proper decision-directed PLL with per-symbol phase
update, sub-sample timing recovery). Phase 3+ work, not bug-fix.

## 5. Modulation pivot history

The first 0.3.1 design was 4-GFSK at h = 0.5. Phase 2 found the
orthogonality integral `sinc(0.5) ≈ 0.637` left adjacent tones
leaking 64 % of their energy, breaking max-likelihood symbol
detection. The redesign to coherent QPSK + RRC matched filter was
committed mid-cycle. See `docs/0.3.1_PLAN.md` for the chronology.

The σ formula in `tests/common/channel.rs` was also recalibrated
in Phase 2'a to take per-burst measured signal power, so stated
Eb/N0_info numbers are now cross-modulation comparable.

## 6. Audio samples

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
QPSK symbol across multiple tones, giving an approximately flat
spectrum across `[1500 − 600, 1500 + 600] Hz` with raised-cosine
shoulders.

## 7. Implementation pointers

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
| LDPC-only sweep (modem-bypassed) | [`mfsk-core/tests/uvpacket_ldpc_direct.rs`](../mfsk-core/tests/uvpacket_ldpc_direct.rs) |
| Modem TX/RX diagnostics | [`mfsk-core/tests/uvpacket_modem_diag.rs`](../mfsk-core/tests/uvpacket_modem_diag.rs) |
| AWGN / Rayleigh threshold sweeps | [`mfsk-core/tests/uvpacket_awgn.rs`](../mfsk-core/tests/uvpacket_awgn.rs), [`uvpacket_rayleigh.rs`](../mfsk-core/tests/uvpacket_rayleigh.rs) |

## 8. License

GPL-3.0-or-later, matching the rest of `mfsk-core`. The LDPC mother
code is derived from WSJT-X (`lib/fst4/`).
