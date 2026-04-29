# uvpacket — 応用例: NFM 音声チャンネル向けパケットプロトコル

> **English:** [UVPACKET.md](UVPACKET.md)

`uvpacket` は `mfsk-core` の FEC 基盤（`Ldpc240_101`、BP、OSD-2/3）
を WSJT-X 系の外で再利用する **応用例** として in-tree に置かれて
いるモジュールです。WSJT-X 系のメンバーでは**ありません**。設計
対象は別 — 狭帯域 FM 音声チャンネル（HT/モバイル、~3 kHz 音声
帯域）でのプライベートグループ向けアマチュア無線メッセージング
（署名付き QSL 交換、短文、位置レポート）です。

このドキュメントでは設計上の選択、特性測定結果、既知の modem
実装損失をまとめます。API は in-source rustdoc を参照。

## 1. スコープ

### 1.1 これが何か

NFM 音声帯域に収まる **4 モードのパケット modem**。FST4 由来の
hand-tuned irregular LDPC を親コードとして使用。**両端で同じ
ソフトウェア**を動かすプライベートグループ向け — 公的な互換
プロトコル置換ではなく、既存 TNC とも互換性なし。

### 1.2 これは「ない」もの

- 相互運用モードではない。標準化なし、TNC サポートなし。
- 音声モードではない。データ専用。
- 広帯域モードではない。NFM 音声 (~3 kHz) に収まり、ネット
  スループット 1–1.8 kbps。M17 / D-STAR / DMR / VARA FM とは
  別の土俵。
- 弱信号モードではない。FM 閾値（CNR ≥ +9–10 dB）より上の運用
  envelope を狙うもので、それ以下では FM 検波系のどんな modem
  でも崩壊する不可避フロアがチャンネル側にある。

### 1.3 立ち位置

uvpacket はオープンソースで空いていたニッチを埋める: **コヒー
レント QPSK 物理層、サブ秒バースト時間、機会的スループット用の
段階的レートラダー**を持つ LDPC 符号化されたデータ専用 NFM modem。

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

フレーム頭は **31 bit BPSK m-sequence**（Fibonacci LFSR、多項式
x⁵ + x² + 1、初期状態 `[0, 0, 0, 0, 1]`）。31 chip × 1 sym/chip
= 1200 baud で 26 ms のプリアンブル。巡回自己相関のサイドローブ
は 1/31 ≈ −15 dB で抑えられる — シンボルタイミング獲得、フレーム
検出、初期搬送波位相基準のためのきれいな相関ピーク。

プリアンブル後は**32 シンボルごとに 1 つの既知 QPSK パイロット**
（オーバーヘッド ~3 %）。パイロット信号点は +1 + 0j。RX は連続
する pilot anchor を線形補間してシンボル単位の位相基準を構築。
その上で **ブロック単位の decision-directed 補正**（§4 参照）を
適用して、ブロック内の平均位相追跡残差を吸収します。

### 2.3 FEC

FST4 由来の [`Ldpc240_101`] をレート 0.42 の親コードとして再利用
（情報 101 bit → channel 240 bit / block）。4 つのサブモードは
**kSR-greedy puncture set 選択**（Ha–McLaughlin）で 139 parity
bit にパンクチャを適用:

| サブモード | rate | パンクチャ | Net bps | 想定姿勢 |
|---|---:|---:|---:|---|
| Robust | 0.42 | 0 % | 1008 | 最大マージン姿勢 |
| Standard | 0.50 | 30 % | 1200 | フェージング有の典型的 NFM |
| Fast | 0.66 | 63 % | 1600 | 良好な信号でのデフォルト |
| Express | 0.75 | 76 % | 1800 | 強信号での最速（OSD-3 必須） |

kSR-greedy は深い rate で uniform-spread に対し ~1–3 dB の Eb/N0
gain を出し、これが Express をそもそも成立させている。

### 2.4 フレーム構造

- 可変長: フレームあたり 1–32 LDPC ブロック。
- 各 LDPC ブロックは 96 情報 bit (12 byte) を運び、FEC の 101 bit
  入力にパディング。残りの block あたり 5 bit は 32 bit フレーム
  ヘッダの **D-iii 拡散コピー**（ヘッダがフレーム全体に ~7 回複製
  されてスローパス復元用 — 現状 fast path はブロック 0 の
  CRC 検証済みヘッダを使用）。
- 4 byte フレームヘッダ: mode (2b) + block count (5b) + app type
  (4b) + sequence (5b) + CRC-16 (16b)。
- フレーム内全 codeword をまたぐ **ブロックインターリーバー**が
  fade burst の erasure を全 codeword に拡散。

### 2.5 アプリケーション API

バイトパイプ — `mfsk-core` の `MessageCodec` をバイパス。呼び出し
側は raw bytes と 4 bit `app_type` タグを渡す。modem は中身を解釈
しない。推奨割り当て:

| `app_type` | 用途 |
|---:|---|
| 0 | raw / 実験 |
| 1 | 署名付き QSL 交換 |
| 2 | 位置ビーコン |
| 3 | 短文 |
| 4 | ARQ ACK |
| 5–15 | ユーザー定義 |

## 3. 特性測定

### 3.1 LDPC レイヤー（modem バイパス参照）

`tests/uvpacket_ldpc_direct.rs` は Gaussian noise の LLR を直接
LDPC デコーダに食わせる（channel bit 単位の `Eb/N0_info` で校正）。
これで FEC を modem から分離し、QPSK end-to-end が目指す**理論的
上限**を出す:

```
mode      eb/n0 (dB)  -2  -1   0   1   2   3   4
─────────────────────────────────────────────────
Robust                 0   2   6  21  28  30  30
Standard               0   1   5  20  29  30  30
Fast                   0   1   6  22  26  30  30
Express                0   0   0  14  24  29  30
```

50 % PER 閾値: Robust ≈ +0.5 dB, Standard / Fast ≈ +0.7 dB,
Express ≈ +1.5 dB。親コードの設計レートは 0.42 なので Robust が
FEC レイヤーで ~1 dB のリードを保つ。

### 3.2 QPSK end-to-end (modem + FEC)

`tests/uvpacket_demod_diagnostic::awgn_threshold_finder_per_mode`、
cell ごとに 30 trials、4 ブロックフレーム、44 byte ペイロード、
OSD-2 (default):

```
mode      eb/n0 (dB)  -2  0  2  4  6  8 10 12 14 16 18 20 22
─────────────────────────────────────────────────────────────
Robust                 0  0 14 29 30 30 30 30 30 30 30 30 30
Standard               0  0 10 30 30 30 30 30 30 30 30 30 30
Fast                   0  0 12 29 30 30 30 30 30 30 30 30 30
Express                0  0  3 29 30 30 30 30 30 30 30 30 30
```

50 % PER 閾値:

- **Robust**: ~+1 dB
- **Standard / Fast**: ~+2 dB
- **Express**: ~+3 dB

教科書通りの rate ordering (低 rate ほど低閾値) が復活: Robust が
Express を ~2 dB 引き離し、§3.1 の LDPC レイヤー優位と一致。

100 % PER 閾値: 全モードで ~+4 dB。Modem 実装損失は LDPC のみの
上限に対して **モード別に ~0.5–2 dB** (Phase 2'b の位相追跡器
書き直し前は ~3 dB だった — 内訳は §4)。

### 3.3 Rayleigh フラットフェージング

`tests/uvpacket_rayleigh.rs`、cell ごとに 30 trials、4 ブロック
フレーム、20 byte ペイロード:

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

≥ 90 % PER 閾値（LMS 位相追跡後、OSD-2）: **Robust は 5–10 Hz
Doppler で +10 dB、1 Hz で +12 dB**; Standard / Fast / Express
もほぼ +10 dB(1 Hz Express だけ少し高め)。位相追跡器書き直しで
Rayleigh 閾値が (mode × Doppler) ごとに 2–5 dB 削れた。

### 3.4 FM 閾値フロア — そして modem 実装損失が運用上不可視な理由

modem は FM 検波の上に乗る。CNR ≈ +9–10 dB を下回ると FM
discriminator 出力はインパルスノイズ支配となり、**どんな**
audio-domain modem も壊滅的に失敗する。上の音声領域 Eb/N0 数値
は FM 閾値より上でのみ意味を持つ。

**FM 閾値の地点**で、検波後の音声 SNR (3 kHz パスバンド換算) は
おおよそ `CNR_threshold + FM_SNR_improvement ≈ +9 +
10·log₁₀(B_IF/B_audio · 3) ≈ +9 + 11 ≈ +20 dB SNR_3kHz`。

uvpacket Robust の 50 % PER 閾値 (+1 dB Eb/N0_info) を同じ単位に
換算:

```
SNR_3kHz_Robust = +1 + 10·log₁₀(1008 / 3000) = −3.7 dB
```

FM 閾値フロアから Robust modem 閾値までのマージン: **~+24 dB**。
§4 の残り 0.5–2 dB の実装損失は運用上**不可視** — チャンネル側
の不可避 CNR フロアより遥か下で、そこではどんな audio modem も
復号しない。

FM 閾値が NFM 音声チャンネルの拘束条件。これより下に行くには
別の on-air 変調（SSB digital、direct IQ digital）が必要で、
本実験のスコープ外。

## 4. modem 実装損失

§3.1 の LDPC のみ閾値と §3.2 の QPSK end-to-end 閾値のギャップ
が modem 実装損失。Phase 2'b で rx 位相追跡器を書き直して
~3 dB から **0.5–2 dB** に削った（モード依存: 全 anchor の
コヒーレント積分から最も恩恵を受ける Robust が最小）。

現在の rx 実装:

- **全 anchor (preamble centre + 各 pilot) の重み付き LMS
  二次フィット**。隣接 pilot 間の線形補間を置き換え、雑音が
  多い pilot 位相推定の **大域平均化**を行いながら、二次項で
  緩い Doppler ドリフトも捕捉。preamble anchor は重み √31
  (averaging する chip 数)、pilot は重み 1。
- **σ-aware LLR スケーリング**:
  `LLR = (A / σ²_n) · qpsk_max_log(r_derot)`
- データシンボル **magnitude ベース σ²_n 推定**:
  `σ²_n = (E[|r|²] − A²) / 2`。残留位相追跡ジッタを含む
  データシンボル上の総雑音を捕捉。
- LMS トラックの上に積む**ブロック単位 decision-directed 補正
  (DDPT)**: 各データシンボルを hard-decide、ブロックごとに
  複素残差を累積、その arg をブロック単位の定数位相補正として
  適用。
- デフォルトは **OSD-2** (cost / 性能のバランス)。
  `decode_known_layout_with_opts` が `&FecOpts` を受け取り、
  OSD-3 を選びたい呼び出し側に開放（~30× 遅いが高 rate モードの
  閾値近傍で ~10–15 % PER 改善）。

残り 0.5–2 dB の主要因:

- 低 SNR での σ²_n 推定器ノイズ（magnitude ベース推定器の
  有限サンプル分散が閾値レベル SNR で真の分散に対して有意に
  なる）。
- 有限長 RRC マッチング損失 (~0.05 dB) と LDPC ブロックを通じた
  有限精度演算の積み重ね。
- 整数サンプル粒度だったタイミング — sub-sample timing recovery
  を実装済み (preamble 相関 magnitude の三点放物線フィットで
  fractional offset を解析的に取得、その offset で MF 出力を
  線形補間でサンプリング)。閾値での実測ゲインは ~0.1 dB、予想
  範囲の下端。高 SNR の Rayleigh で ±1 trial / 30 程度ばらつく
  が統計ノイズ範囲内。

これらは構造的バグではなく sub-1-dB クラスの調整項目。閉じる
作業は Phase 3+ で、現状の modem は閾値で意味のある Robust >
Standard / Fast > Express の順序を提供しており LDPC 理論と一致。

## 5. 変調ピボットの経緯

0.3.1 サイクルの最初の設計は h = 0.5 の 4-GFSK。Phase 2 で
直交性積分 `sinc(0.5) ≈ 0.637` が隣接 tone エネルギーを 64 %
漏らし、最尤シンボル検出を破壊することが判明。コヒーレント
QPSK + RRC 整合フィルタへの再設計をサイクル中盤でコミット。
完全な経緯は `docs/0.3.1_PLAN.md`。

`tests/common/channel.rs` の σ 公式も Phase 2'a で per-burst の
測定信号電力を取るよう再校正したので、表示 Eb/N0_info は変調間
で比較可能。

## 6. 音声サンプル

リポジトリの `audio_samples/uvpacket/` に耳での確認用 WAV を
配置。すべて 12 kHz mono 16-bit PCM、200 ms の前後無音付き:

| ファイル | モード | チャンネル | 復号 |
|---|---|---|:-:|
| `uv_robust_clean.wav` | Robust, 4 blocks, 20 B | clean | ✓ |
| `uv_robust_awgn_+8db.wav` | Robust | AWGN +8 dB Eb/N0 | ✓ |
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (LMS 後 97% per-frame) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✓ (53 % per-frame 統計; この seed は sub-sample timing で OK 側) |
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
shoulder 付き。

## 7. 実装ポインタ

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
| LDPC のみ sweep (modem バイパス) | [`mfsk-core/tests/uvpacket_ldpc_direct.rs`](../mfsk-core/tests/uvpacket_ldpc_direct.rs) |
| Modem TX/RX 診断 | [`mfsk-core/tests/uvpacket_modem_diag.rs`](../mfsk-core/tests/uvpacket_modem_diag.rs) |
| AWGN / Rayleigh 閾値 sweep | [`mfsk-core/tests/uvpacket_awgn.rs`](../mfsk-core/tests/uvpacket_awgn.rs)、[`uvpacket_rayleigh.rs`](../mfsk-core/tests/uvpacket_rayleigh.rs) |

## 8. ライセンス

GPL-3.0-or-later、`mfsk-core` の他と同じ。LDPC 親コードは WSJT-X
(`lib/fst4/`) からの派生。
