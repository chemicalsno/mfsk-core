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
30 trials per cell, 4-block frame, 44-byte payload, OSD-2 (default):

```
mode      eb/n0 (dB)  -2  0  2  4  6  8 10 12 14 16 18 20 22
─────────────────────────────────────────────────────────────
Robust                 0  0 14 29 30 30 30 30 30 30 30 30 30
Standard               0  0 10 30 30 30 30 30 30 30 30 30 30
Fast                   0  0 12 29 30 30 30 30 30 30 30 30 30
Express                0  0  3 29 30 30 30 30 30 30 30 30 30
```

50 % PER thresholds:

- **Robust**: ~+1 dB
- **Standard / Fast**: ~+2 dB
- **Express**: ~+3 dB

The textbook rate ordering (lower rate → lower threshold) is
recovered: Robust beats Express by ~2 dB at the threshold, matching
the LDPC-layer advantage from §3.1.

100 % PER thresholds: Robust / Standard / Fast / Express all at
~+4 dB. Modem implementation loss versus the LDPC-only ceiling is
**~0.5–2 dB** depending on mode (was ~3 dB before the Phase 2'b
phase-tracker rework — see §4 for the breakdown).

### 3.3 Rayleigh flat fading

`tests/uvpacket_rayleigh.rs`, 30 trials per cell, 4-block frame,
20-byte payload:

```
mode       Doppler  +10  +12  +15  +20  +25  +30  +35  (Eb/N0_info dB)
──────────────────────────────────────────────────────────────────
Robust     1 Hz     —    28   30   30   30   30   —
Robust     5 Hz     30   30   30   30   30   30   —
Robust    10 Hz     28   30   30   30   30   30   —
Standard   1 Hz     27   28   30   30   30   30   —
Standard   5 Hz     30   30   30   30   30   30   —
Standard  10 Hz     28   30   30   30   30   30   —
Fast       1 Hz     —    —    30   30   30   30   30
Fast       5 Hz     27   —    30   30   30   30   30
Fast      10 Hz     29   —    30   30   30   30   30
Express    1 Hz     25   —    29   30   30   30   30
Express    5 Hz     27   —    30   30   30   30   30
Express   10 Hz     28   —    30   30   30   30   30
```

≥ 90 % PER thresholds (post-LMS phase tracker, OSD-2): **Robust
≈ +10 dB at 5–10 Hz Doppler, ~+12 dB at 1 Hz**; Standard / Fast /
Express all at ~+10 dB across most Doppler / a touch higher at
1 Hz Express. The phase-tracker rework dropped the Rayleigh
thresholds by 2–5 dB depending on (mode × Doppler).

### 3.4 The FM-threshold floor — and why it makes the modem
###     implementation loss operationally invisible

The modem sits on top of FM detection. Below CNR ≈ +9–10 dB the
FM discriminator output is dominated by impulse noise and **any**
audio-domain modem fails catastrophically. The audio-domain Eb/N0
numbers above are meaningful only above the FM threshold.

**At the FM threshold**, post-detection audio SNR (in a 3 kHz
passband) is roughly `CNR_threshold + FM_SNR_improvement ≈ +9 +
10·log₁₀(B_IF/B_audio · 3) ≈ +9 + 11 ≈ +20 dB SNR_3kHz`.

Translating uvpacket Robust's 50 % PER threshold (+1 dB
Eb/N0_info) to the same units:

```
SNR_3kHz_Robust = +1 + 10·log₁₀(1008 / 3000) = −3.7 dB
```

Margin from the FM threshold floor down to the Robust modem
threshold: **~+24 dB**. The 0.5–2 dB residual modem implementation
loss in §4 is operationally invisible — it sits well below the
channel's own irreducible CNR floor, where no audio modem of any
kind decodes.

The FM threshold is the binding constraint for NFM voice
channels.

### 3.5 SSB compatibility

The modem is an audio-domain QPSK + RRC processor (signal
occupies ~1200–1800 Hz around the 1500 Hz centre, well inside a
typical SSB passband). On SSB the FM-threshold floor goes away
and the modem operates at its true ~−3.7 dB SNR_3kHz Robust
threshold — a useful weak-signal data envelope, especially on HF.

What's missing for production SSB use is **automatic frequency
control (AFC)**: the current rx assumes `audio_centre_hz` is known
exactly, while real SSB receivers see a ±50–100 Hz centre offset
from VFO-dial mismatches between TX and RX. Adding AFC is a
~50–100-line change (frequency-search the preamble correlator over
a configurable window, then track the offset through the LMS
phase fit) — planned for a follow-up cycle, not 0.3.1.

Until AFC lands, SSB usage requires both ends to dial in the same
frequency to within ~10 Hz. With a digital VFO and CAT control
that's straightforward; with a manual dial it's not.

## 4. Modem implementation loss

The gap between the LDPC-only threshold (§3.1) and the QPSK
end-to-end threshold (§3.2) is the modem implementation loss.
Phase 2'b reworked the rx phase tracker and brought the gap from
~3 dB down to **0.5–2 dB** (mode-dependent: lowest for Robust which
benefits most from coherent integration of all anchors).

The current rx implements:

- **Weighted-LMS quadratic phase fit** over all anchors (preamble
  centre + each pilot). Replaces the previous linear interpolation
  between adjacent pilots; provides global averaging of the noisy
  pilot phase estimates while still capturing slow Doppler drift
  via the second-order term. The preamble anchor gets weight √31
  (number of chips it averages); pilots get weight 1.
- **σ-aware LLR scaling**:
  `LLR = (A / σ²_n) · qpsk_max_log(r_derot)`.
- **Magnitude-based σ²_n estimator** from data symbols:
  `σ²_n = (E[|r|²] − A²) / 2`. Captures the total noise on data
  symbols, including any residual phase-tracking jitter.
- **Per-LDPC-block decision-directed correction (DDPT)** stacked on
  top of the LMS track: hard-decide each data symbol, accumulate
  the complex residual per block, take its arg as a per-block
  constant phase correction.
- **OSD-2** by default (good cost/performance balance);
  `decode_known_layout_with_opts` accepts `&FecOpts` for callers
  who want OSD-3 (~30 × slower per decode, ~10–15 % better PER
  near threshold for the higher-rate modes).

The remaining 0.5–2 dB loss is dominated by:

- σ²_n estimator noise at low SNR (the magnitude-based estimator
  has finite-sample variance that becomes a meaningful fraction of
  the true variance at threshold-level SNR).
- Finite-span RRC matching loss (~0.05 dB) plus finite-precision
  arithmetic accumulating over the LDPC block.
- Sample-timing rounded to integer samples — addressed by the
  sub-sample timing recovery (parabolic peak fit on the preamble
  correlation magnitude → fractional offset applied via linear
  interpolation of the matched-filter output). Empirical gain at
  the threshold is ~0.1 dB, at the lower end of the predicted
  range. Mixed at higher SNR (Rayleigh fading sometimes shows ±1
  trial of 30, well within statistical noise).

These are sub-1-dB-each engineering knobs rather than structural
bugs. Closing them is Phase 3+ work; the current modem already
delivers a meaningful Robust > Standard / Fast > Express ordering
at the threshold, matching the LDPC theory.

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
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (97 % per-frame after LMS) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✓ (53 % per-frame statistic; this seed lands on the OK side after sub-sample timing) |
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
