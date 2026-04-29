# uvpacket — NFM音声チャンネル向けパケットプロトコル（応用例）

> **English:** [UVPACKET.md](UVPACKET.md)

`uvpacket` は `mfsk-core` の FEC 基盤（`Ldpc240_101`、BP、OSD-2）を
WSJT-X 系の外で再利用する**応用例**として in-tree に置かれている
モジュールです。WSJT-X 系のメンバーでは**ありません**。設計対象
は別のもの — 狭帯域FM音声チャンネル（HT/モバイル、~3 kHz音声
帯域）でのプライベートグループ向けアマチュア無線メッセージング
（署名付き QSL 交換、短文、位置レポート）です。

このドキュメントでは立ち位置、設計上の選択、特性測定結果、そして
AX.25 / AFSK 1200（この設計対象でフェアな唯一のベースライン）と
の比較を扱います。API は in-source rustdoc を参照してください。

## 1. 立ち位置

### 1.1 何を解決するのか

AX.25 / AFSK 1200 は ~40 年にわたり NFM のデファクトデジタル
プロトコルでした。この実験の動機となったのは以下の2点:

1. **FECなし**。フレーム内のたった1ビットエラーで致命傷。クリーン
   な channel では問題ないが、フェージング channel で深いnullを
   1つ食らうとフレームが落ちる。
2. **非コヒーレント BFSK** at 1200 baud, 音声中心 1700 Hz。位相
   基準を持たないので教科書通りの ~3 dB 非コヒーレント損失を払う。

これらを固定されたものとして受け入れる場合、距離を伸ばす唯一の
ノブは送信電力です。uvpacket は問います: **同じNFM音声帯域に
収まる範囲で modem を置き換えて LDPC を載せた場合、どれだけ
margin が稼げるか?**

### 1.2 これは「ない」もの

- パブリック APRS の置換ではない。広範な展開には既存 TNC との
  相互運用が必要だがその船は既に出航済み。uvpacket は**両端で
  同じソフトウェア**を動かすプライベートグループ向け。
- 音声モードではない。M17 / D-STAR / DMR / NXDN は音声を主用途と
  するプロトコルでデータはサブチャンネル。uvpacket はデータ専用。
- 広帯域モードではない。VARA FM は ~12.5 kHz の帯域で ~25 kbps
  を出すが、uvpacket は NFM 音声 (~3 kHz) に収まり 1–1.8 kbps。
  土俵が違う。

### 1.3 ここに収まる

| モード | net bps | 帯域 | AWGN閾値 (SNR_3kHz) | FEC | OSS |
|---|---:|---|---:|---|:-:|
| AX.25 / AFSK 1200 | 1200 | NFM (~3 kHz) | +10 dB | なし | ✓ |
| uvpacket Robust | 1008 | NFM | **+3 dB** | LDPC 0.42 | ✓ |
| uvpacket Express | 1800 | NFM | **+5–6 dB** | LDPC 0.75 | ✓ |
| M17 (4-FSK) | 4800 | ~9 kHz | +5–7 dB | conv | ✓ |
| D-STAR DV | 4800 | 6.25 kHz | ~+10 dB CNR | Golay | 部分的 |
| DMR / NXDN | 4-FSK | 6.25 / 12.5 kHz | ~+7–8 dB CNR | BCH | (商用) |
| VARA FM | ~25000 | 12.5 kHz | +10 dB | プロプライエタリ | ✗ |

uvpacket が収まるのは存外空いていたニッチ: **AX.25 と同じ音声
帯域スロットで FEC とまともなフェージング耐性を持つ、データ
専用のオープンソース NFM デジタルプロトコル**。M17/VARA を別の
土俵で打ち負かすものではないが、自分の土俵では AX.25 を明確に
改善している。

## 2. 設計

### 2.1 変調

単一搬送波**コヒーレント QPSK 1200 baud**、ルートレイズドコサイン
パルス（α = 0.5、span 6 sym）、音声中心 1500 Hz、サンプリング
12 kHz。QPSK constellation は Gray マッピング:

| `(b1, b0)` | 信号点 |
|---:|:--|
| (0, 0) | +1 + 0j |
| (0, 1) | 0 + 1j |
| (1, 0) | 0 − 1j |
| (1, 1) | −1 + 0j |

TX はバーストエンベロープのピークを ≤ 1 に正規化。RMS は 0.2–0.5
程度（α = 0.5 の RRC で QPSK の場合 PAPR ~7 dB は標準的）。

### 2.2 プリアンブル + パイロット

フレーム頭は **31-bit BPSK m-sequence**（Fibonacci LFSR、多項式
x⁵ + x² + 1、初期状態 `[0, 0, 0, 0, 1]`）。31 chip × 1 sym/chip
= 1200 baud で 26 ms のプリアンブル。巡回自己相関のサイドローブは
1/31 ≈ −15 dB 振幅で抑えられる — シンボルタイミング獲得、
フレーム検出、初期搬送波位相基準のためのきれいな相関ピーク。

プリアンブル後は**32 シンボルごとに 1 つの既知 QPSK パイロット**
（オーバーヘッド ~3 %）。パイロット信号点は +1 + 0j。RX は連続
する pilot anchor を線形補間してシンボル単位の位相基準を構築 —
このパイロット密度と channel のコヒーレンス時間では完全な
decision-directed PLL は overkill。

### 2.3 FEC

FST4 由来の [`Ldpc240_101`] をレート 0.42 の親コードとして再利用
（情報 101 bit → channel 240 bit / block）。4つのサブモードは
**kSR-greedy puncture set 選択**（Ha–McLaughlin）で 139 parity bit
にパンクチャを適用:

| サブモード | rate | パンクチャ | Net bps | 想定姿勢 |
|---|---:|---:|---:|---|
| Robust | 0.42 | 0 % | 1008 | 山岳 / 弱信号 / 深いフェージング |
| Standard | 0.50 | 30 % | 1200 | フェージング有の典型的NFM |
| Fast | 0.66 | 63 % | 1600 | 良好な信号でのデフォルト |
| Express | 0.75 | 76 % | 1800 | 強信号での最速（OSD-2必須） |

kSR-greedy は深い rate で uniform-spread に対し ~1–3 dB の Eb/N0
gain を出し、これが Express をそもそも成立させている（uniform-
spread は 76 % parity puncture で BP threshold で収束しない）。

### 2.4 フレーム構造

- 可変長: フレームあたり 1–32 LDPC ブロック。
- 各 LDPC ブロックは 96 情報 bit (12 byte) を運び、FEC の 101 bit
  入力にパディング。残りの block あたり 5 bit は 32 bit フレーム
  ヘッダの **D-iii 拡散コピー**（ヘッダがフレーム全体に ~7 回複製
  されてスローパス復元用 — 現状 fast path はブロック 0 の
  CRC 検証済みヘッダを使用）。
- 4 byte フレームヘッダ: mode (2b) + block count (5b) + app type
  (4b) + sequence (5b) + CRC-16 (16b)。
- フレーム内全 codeword をまたぐ**ブロックインターリーバー**が
  fade burst の erasure を全 codeword に拡散 — Rayleigh の null
  が深いほど薄まる。

### 2.5 アプリケーション API

バイトパイプ — `mfsk-core` の `MessageCodec` をバイパス。呼び出し
側は raw bytes と 4 bit `app_type` タグを渡す。modem は中身を
解釈しない。推奨割り当て:

| `app_type` | 用途 |
|---:|---|
| 0 | raw / 実験 |
| 1 | 署名付き QSL 交換 |
| 2 | 位置ビーコン |
| 3 | 短文 |
| 4 | ARQ ACK |
| 5–15 | ユーザー定義 |

## 3. 特性測定

### 3.1 AWGN

Phase 2'a sweep、cell ごとに 30 trials、4 ブロックフレーム、44 byte
ペイロード。Eb/N0 は**情報 1 bit あたり**（WSJT 系で標準的な
モード間フェアな convention）。σ はバーストごとに測定された信号
電力で校正 — なぜ重要かは [§4](#4-snr-校正の経緯) 参照。

```
mode      eb/n0 (dB)  -2  0  2  4  6  8 10 12 14 16 18 20 22
─────────────────────────────────────────────────────────────
Robust                 0  0  0  9 26 29 30 30 30 30 30 30 30
Standard               0  0  1 15 27 30 30 30 30 30 30 30 30
Fast                   0  0  1 19 27 30 30 30 30 30 30 30 30
Express                0  0  1 16 29 30 30 30 30 30 30 30 30
```

4 モードすべてが **+4 dB Eb/N0_info で 50 % PER**、**+8 dB で
100 % PER** に到達。QPSK + rate-0.42–0.75 LDPC の理論（無符号
QPSK の 1e-2 BER に対して ~1–2 dB code gain）と一致。

### 3.2 Rayleigh フラットフェージング

Phase 2'b sweep、cell ごとに 30 trials、4 ブロックフレーム、20 byte
ペイロード。

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

≥ 90 % PER 閾値:

- **Robust / Standard**: 1–5 Hz Doppler で +12 dB、10 Hz で +15 dB
- **Fast / Express**: 全 Doppler で +15 dB

VHF/UHF モバイル NFM channel に対する現実的なフェージング耐性。

### 3.3 AX.25 / AFSK 1200 との比較

256 byte の AX.25 フレームで FER ≤ 1 % に対して BER ≤ ~5e-6 が
必要。非コヒーレント BFSK が BER 5e-6 に到達するのは Eb/N0 ≈
14 dB。3 kHz 音声帯域での SNR に変換すると:

- **AFSK 1200**: 14 + 10·log₁₀(1200/3000) = **+10 dB SNR_3kHz**
- **uvpacket Robust**: 8 + 10·log₁₀(1008/3000) = **+3 dB SNR_3kHz**
- **uvpacket Express**: ~8 + 10·log₁₀(1800/3000) = **+6 dB SNR_3kHz**

uvpacket Robust は同等スループットで AX.25 より **~7 dB** 良い。
Express は 50 % 多い net bps を出しながら **~4 dB** 良い。Rayleigh
フェージングではこの差が広がる（AX.25 は FEC を持たずフレームが
任意の 1 bit erasure に対して原子的、uvpacket は FEC + interleaver
が fade burst を全 codeword に薄める）。

### 3.4 +9–10 dB の FM 閾値フロア

両 modem とも FM 検波の上に乗る。CNR ≈ +9–10 dB を下回ると FM
discriminator 出力はインパルスノイズ支配となり、**両 modem とも
壊滅的に失敗**する。上の音声領域 Eb/N0 数値は FM 閾値より上で
のみ意味があり、その下ではどちらのプロトコルも復号しない。これは
channel の性質であり modem の性質ではない。

FM 閾値より下に行くには別の on-air 変調（SSB digital、direct IQ
digital）が必要で、本実験のスコープ外。

## 4. SNR 校正の経緯

Phase 1 の 4-FSK 設計（h = 0.5、GFSK BT = 0.5）は理論に対して
~+11 dB の SNR 閾値ギャップを示しました。寄与は2つ:

1. **Tone 非直交性**: h = 0.5 で直交性積分 `sinc(Δf · T_sym) =
   sinc(0.5) ≈ 0.637` — 隣接 tone がエネルギーの 64 % を漏らし、
   最尤シンボル検出を破壊する。これが root cause で QPSK への
   変調ピボット動機（I/Q 軸は構成的に直交）。
2. **σ-formula 校正不良**: AWGN ハーネスは constant-envelope
   `P = 0.5` を仮定。QPSK ピボット後 RRC-shaped QPSK は RMS ≈
   0.22（peak を 1 に正規化）なので、表示 Eb/N0 が ~10 dB ずれて
   いた。Phase 2'a で per-burst 測定信号電力を取る formula に再校正。

§3 の数値は再校正後で、変調間で比較可能。tx 側のバースト電力は
`signal_power(audio) = mean(audio²)` で測定し、
`awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power)` に
渡される — `mfsk-core/tests/common/channel.rs` を参照。

## 5. 音声サンプル

リポジトリの `audio_samples/uvpacket/` に耳での確認用 WAV を配置。
すべて 12 kHz mono 16-bit PCM、200 ms の前後無音付き:

| ファイル | モード | チャンネル | 復号 |
|---|---|---|:-:|
| `uv_robust_clean.wav` | Robust, 4 blocks, 20 B | clean | ✓ |
| `uv_robust_awgn_+8db.wav` | Robust | AWGN +8 dB Eb/N0 | ✓ |
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (50 % PER 域) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✗ |
| `uv_robust_rayleigh_5hz_+15db.wav` | Robust | 5 Hz Rayleigh, +15 dB | ✓ |
| `uv_express_clean.wav` | Express, 4 blocks, 20 B | clean | ✓ |

再生成は:

```sh
cargo run --release --features uvpacket --example uvpacket_samples
```

クリーンな Robust バーストは ~440 ms、Express は ~270 ms。可聴
キャラクターは「狭帯域データバズ」 — RRC pulse が各 QPSK シンボル
を複数 tone にまたがって広げるので、スペクトラムは
`[1500 − 600, 1500 + 600] Hz` でほぼ平坦、レイズドコサインの
shoulder 付き。人間の耳には AFSK 1200 にかなり近く聞こえ、4 つの
位相を巡回するので少し「smeared」感があります。

## 6. 実装ポインタ

| Layer | File |
|---|---|
| Protocol ZST / サブモードパラメータ | [`mfsk-core/src/uvpacket/protocol.rs`](../mfsk-core/src/uvpacket/protocol.rs) |
| フレームヘッダ + CRC + bit packing | [`mfsk-core/src/uvpacket/framing.rs`](../mfsk-core/src/uvpacket/framing.rs) |
| Puncture sets (kSR-greedy) | [`mfsk-core/src/uvpacket/puncture.rs`](../mfsk-core/src/uvpacket/puncture.rs) |
| ブロックインターリーバー | [`mfsk-core/src/uvpacket/interleaver.rs`](../mfsk-core/src/uvpacket/interleaver.rs) |
| プリアンブル + パイロット定義 | [`mfsk-core/src/uvpacket/sync_pattern.rs`](../mfsk-core/src/uvpacket/sync_pattern.rs) |
| TX (bytes → audio) | [`mfsk-core/src/uvpacket/tx.rs`](../mfsk-core/src/uvpacket/tx.rs) |
| RX (audio → bytes) | [`mfsk-core/src/uvpacket/rx.rs`](../mfsk-core/src/uvpacket/rx.rs) |
| AWGN + Rayleigh ハーネス | [`mfsk-core/tests/common/channel.rs`](../mfsk-core/tests/common/channel.rs) |
| 閾値 sweep | [`mfsk-core/tests/uvpacket_awgn.rs`](../mfsk-core/tests/uvpacket_awgn.rs)、[`uvpacket_rayleigh.rs`](../mfsk-core/tests/uvpacket_rayleigh.rs) |

## 7. ライセンス

GPL-3.0-or-later、`mfsk-core` の他と同じ。LDPC 親コードは WSJT-X
(`lib/fst4/`) からの派生。
