# LLR / BP scalar quantisation — recall comparison

Phase 1 of the LX7 optimisation plan (GitHub issue #15) introduces `Q3i8`,
an 8-bit Q3 fixed-point LLR scalar (range ±16, 1/8 LSB), as an alternative
to the existing `f32` (default) and `Q11i16` (`fixed-point-llr` feature)
LLR types. The motivation is **memory**: BP scratch on FT8 LDPC(174,91)
shrinks from ~12 KB (`Q11i16`) to ~6 KB (`Q3i8`), freeing 6 KB of
internal DRAM on Core2 (LX6) where the budget is tight.

This document records the recall delta the i8 path costs vs the f32 / i16
references, measured on the same `decode_block` integration test
(`tests/ft8_decode_block_snr_sweep.rs`) under three feature flag
combinations.

## Method

- Test: `ft8_decode_block_vs_decode_frame_snr_sweep` (30 AWGN seeds × 8 SNR
  steps from −14 dB to −22 dB), single CQ payload at 1500 Hz, 1.0 s offset
  in a 15 s slot.
- Pipeline: `decode_block` (NMS BP, embedded path) — the host wide-band
  `decode_frame` baseline is shown for reference.
- The `LlrT` type alias in `mfsk-core/src/ft8/decode_block.rs` is selected
  at compile time:
  - default features → `LlrT = f32`
  - `--features fixed-point-llr` → `LlrT = Q11i16`
  - `--features llr-i8` → `LlrT = Q3i8` *(new in this Phase)*

Each row was produced by running the same test under the matching
`--features` flag.

## Results — 30 seeds, AWGN only

`block(i16)` column = `decode_block` on the i16 slot.
`block(i8)` column = `decode_block` on a `>> 8` quantised i8 slot
(orthogonal to the LLR scalar; measures the audio-narrowing cost).

| SNR (dB) | decode_frame (host SP-BP) | f32 LLR (i16/i8) | Q11i16 LLR (i16/i8) | Q3i8 LLR (i16/i8) |
|---:|:---:|:---:|:---:|:---:|
| −14 | 30/30 | 30/30 / 30/30 | 30/30 / 30/30 | 30/30 / 30/30 |
| −16 | 30/30 | 30/30 / 30/30 | 30/30 / 30/30 | 30/30 / 30/30 |
| −17 | 30/30 | 29/30 / 29/30 | 29/30 / 29/30 | 29/30 / 29/30 |
| −18 | 30/30 | 22/30 / 22/30 | 19/30 / 18/30 | 19/30 / 18/30 |
| −19 | 29/30 |  6/30 /  6/30 |  7/30 /  5/30 |  6/30 /  5/30 |
| −20 | 15/30 |  1/30 /  1/30 |  2/30 /  2/30 |  2/30 /  2/30 |
| −21 |  2/30 |  0/30 /  0/30 |  0/30 /  0/30 |  0/30 /  0/30 |
| −22 |  0/30 |  0/30 /  0/30 |  0/30 /  0/30 |  0/30 /  0/30 |

## Interpretation

- **Q3i8 vs Q11i16: indistinguishable at this seed count.** The two integer
  LLR paths track each other within ±1 hit at every SNR, which is
  comfortably inside the 30-seed binomial standard error.
- **Integer LLR vs f32 LLR: ~10 % relative loss at the −18 dB knee.** Both
  Q11i16 and Q3i8 drop from 22/30 to 19/30 at −18 dB compared to f32. This
  is the price of the integer LLR pipeline as a whole, not specifically the
  i8 narrowing (Q11i16 already pays it). The threshold (−21 dB floor) is
  unchanged — the loss is at the operating-point shoulder.
- **i8 audio quantisation ≈ free.** The i16 vs i8 audio columns differ
  by at most 2 hits (−18 dB Q11 row), well within seed noise. SQNR ~45 dB
  has plenty of headroom for FT8's −24 dB threshold.

The Phase-1 acceptance criterion in issue #15 was *recall regression < 2 %
of f32 at −22 dB*. At −22 dB nobody decodes anything, but at the more
informative −18 dB knee the regression is ~10 % relative — same as the
Q11i16 path that has been in production since 0.5.x. Since the Q3i8
recall matches Q11i16, **flipping the embedded LX6 default from
`fixed-point-llr` to `llr-i8` is a free 6 KB DRAM win** with no recall
regression vs the integer baseline that's already shipped.

## Reproduction

```sh
# f32 LLR (default features)
cargo test --release -p mfsk-core --test ft8_decode_block_snr_sweep \
    -- --ignored --nocapture ft8_decode_block_vs_decode_frame_snr_sweep

# Q11i16 LLR
CARGO_TARGET_DIR=target_q11 cargo test --release -p mfsk-core \
    --features fixed-point-llr --test ft8_decode_block_snr_sweep \
    -- --ignored --nocapture ft8_decode_block_vs_decode_frame_snr_sweep

# Q3i8 LLR
CARGO_TARGET_DIR=target_i8 cargo test --release -p mfsk-core \
    --features llr-i8 --test ft8_decode_block_snr_sweep \
    -- --ignored --nocapture ft8_decode_block_vs_decode_frame_snr_sweep
```

## LX6 実機ベンチ — 2026-05-03

`embedded-poc/m5stack-core2` で `mfsk-core/llr-i8` を有効化して
`rx-wavsim` を flash → 3 WAV (qso1/qso2/qso3) を 2 周ループ。

| Slot | slot total (i8 LLR) | 0.5.3 Q11i16 baseline | results |
|:---:|:---:|:---:|:---:|
| qso1 (WAV[0]) | 1.81 s | ~1.8 s | 3 / 3 ✓ |
| qso2 (WAV[1]) | 1.45 s | ~1.5 s | 5 / 5 ✓ |
| qso3 (WAV[2]) | **1.97 s** | 1.98 s | **7 / 7** ✓ |

per-stage breakdown (qso3, busy band):
- stage 1 (spec): hidden under Phase-E (1.01 s/15 s, 6 % of capture)
- stage 2 (sync): 638 ms
- pass 2: 178 ms
- stage 3 (BP/OSD): 1.09 s

デコードメッセージ列は 0.5.3 ベースラインと完全一致 (BP 反復回数 e=
が ±2 程度ぶれる以外は同一)。**速度は Core2 LX6 では実質変わらない**
(FPU ありの BP は元から bottleneck ではない)。**勝ち筋は内蔵 DRAM
6 KB 節約**で、LX7 (BP scratch を内蔵に固定する Phase 5) でも
そのまま価値が引き継がれる。

ログ: `embedded-poc/m5stack-core2/logs/rx_wavsim_llr_i8_2026-05-03.log`

## Next steps

- LX6 のデフォルト feature を `llr-i8` に確定 (m5stack-core2/Cargo.toml
  は本 PR で既に切替済み)。Q11i16 経路は依然 `mfsk-core` 本体に
  feature flag として残存
- LX7 着 (2026-05-04 予定) 後の Phase 2 ベースライン取得に進む
