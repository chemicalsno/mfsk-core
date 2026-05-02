# 組み込みターゲット

`mfsk-core` は `no_std + alloc` 対応です。FT8 デコードパス
(`mfsk_core::ft8::decode_block`) は、呼び出し側が FFT と内積の
バックエンドを供給する条件で、~150 KB の RAM 環境でも動作します。
本文書は組み込みコール元に求められる契約と、ライブラリ側で**提供しない**
ものを説明します。

## アーキテクチャ: f32 と固定小数点が同一コードベースで共存

DSP / FEC パイプライン全体が **scalar trait** 化されており、
同じソースがホスト用 f32 パスと組み込み用整数パスの **どちらにも
コード重複なしでコンパイル**されます:

- [`core::scalar::SpecScalar`] — スペクトログラム / DFT 出力 scalar
  (ホスト: `f32`; 組み込み cs 格納: `Q14i16`)
- [`core::scalar::LlrScalar`] — wide-accumulator 型を含む LLR scalar
  (ホスト: `f32`; 組み込み BP: `Q11i16` + i32 wide)
- [`core::scalar::Cmplx<S>`] — `SpecScalar` 上の generic 複素数。
  `repr(C)` で `num_complex::Complex` とレイアウト互換。
- `compute_llr_generic<P, S, T>`, `compute_snr_db_generic<P, S>`,
  `bp_decode_generic_nms<P, T>` — すべて scalar 型を generic 引数に
  取り、`(P, S, T)` の組ごとに 1 monomorphisation。

`fixed-point` / `fixed-point-llr` / `fixed-point-cs` の Cargo feature
は **protocol glue がどの scalar 型を選ぶかを切り替える**だけで、
generic 本体は変わりません。組み込みポートはコードの 99 % を
ホストビルドと共有 — バグ修正・最適化は一回当てればどちらにも効きます。

### 現状で fixed-point 切替が wired up されている範囲

| コンポーネント | generic 対象 | fixed-point 切替 |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ `fixed-point-llr` |
| LLR 計算 (`core::llr`) | `SpecScalar` × `LlrScalar` | ✅ `fixed-point-llr` |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — FT8 LDPC(174,91) と FST4/uvpacket LDPC(240,101) 両方 |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ `fixed-point` |
| **FT4 / WSPR / Q65 / JT9 / JT65** | (ホスト f32 のみ) | ❌ — これらは `decode_block` を経由していない |

つまり **trait インフラは protocol-agnostic だが、組み込みビルドで
整数パスに切り替わる protocol は現状 FT8 のみ**。
FT4 (次の有力候補。Costas/Gray/LDPC の部品を共有) を加えるのは
`decode_block` の形を FT4 のシンボル配置に合わせて移植する作業で、
trait 層に新規追加は不要です。

## 検証済みターゲット

| ターゲット | MCU | バックエンド | 状況 |
|---|---|---|---|
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, 単発 f32 FPU, 16 MB flash, 約 4 MB PSRAM) — `espflash board-info` で確認: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`。**ESP32-S2 ではない** (S2 は LX7 single-core で BT 無し) し S3 でもない | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | リファレンス実音声ベンチ。下のベンチ + フットプリント節参照 |
| ESP32-S3 (DevKitC) | Xtensa LX7 + PIE SIMD | esp-dsp ASM | 旧リファレンス。同じ `fft-extern` 契約 |

`fft-extern` + `dotprod-extern` 契約は他ターゲット (RP2040,
RP2350-Hazard3, Cortex-M0/M3 など) にもポータブルな設計ですが、
CI では検証していません。`embedded-poc/m5stack-core2/` が
コピー元の見本です。

## 組み込み用 Cargo feature

デフォルトの `std`, `parallel`, `fft-rustfft` を切り、組み込み
ベースラインを選びます:

```toml
[dependencies]
mfsk-core = { version = "0.5", default-features = false, features = [
    "alloc",            # Vec / Box / String — decode で必須
    "ft8",              # FT8 protocol glue
    "fft-extern",       # FFT backend は呼び出し側提供
    "fixed-point",      # u16 spectrogram + i16 内部 DFT
    "fixed-point-llr",  # Q11 LLR + i16 BP NMS (FPU 軽負荷 + 省メモリ)
    # 任意:
    # "fixed-point-cs",            # Cmplx<Q14i16> cs 格納 (RAM 半減)
    # "fixed-point-coarse-i32",    # i32 coarse_sync (FPU 無し MCU 専用)
    # "profile-coarse",            # stage-2 サブステージ常時計測
] }
```

Feature 一覧:

| Feature | 効果 | 用途 |
|---|---|---|
| `std` | `std::env`, `std::time::Instant` を入れる。rustfft とは分離。 | std を持つ esp-idf-svc 系ターゲット。bare-metal では任意 |
| `alloc` | `extern crate alloc` + Vec / Box | 全 decode パス |
| `fft-extern` | `mfsk_core_make_default_fft_planner` extern fn 経由で FFT backend | 組み込み全般 |
| `fft-rustfft` | rustfft を FFT backend | ホスト専用 |
| `fixed-point` | spectrogram cell を `u16`、内部 DFT を i16 | 組み込み (PSRAM 帯域半減) |
| `fixed-point-llr` | Q11 LLR + i16 NMS BP | 組み込み — 整数パイプラインを揃える |
| `fixed-point-cs` | `Cmplx<Q14i16>` cs 格納 (8 KB → 4 KB / Box) | RAM タイトな組み込み。LX6 は無くても動く |
| `fixed-point-coarse-i32` | stage-2 allsum / score を i32 | **FPU 無しのみ** (RP2040, M0+)。LX6/LX7 では FPU+ALU 並列性が崩れて遅くなる |
| `profile-coarse` | coarse_sync サブステージ計測を常時出力 | 診断専用 |

## 2 つの extern Rust 契約

### FFT backend

`mfsk_core::core::fft::FftPlanner` は decode パスの FFT trait です。
`fft-extern` 下では、バイナリ側に `extern "Rust"` factory を
要求します:

```rust
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::core::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}
```

`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs` が esp-dsp Xtensa
ASM kernel (`dsps_fft2r_fc32_ae32` + i16 パスの
`dsps_fft2r_sc16_ae32`) へ橋渡しする実装例です。RP2040 / Cortex-M
は同じ要領で CMSIS-DSP に橋渡しできます。

### i16 × Q15 内積

`fft-extern + fixed-point` 下では、`ft8::decode_block` のシンボル
DFT が `mfsk_core::core::dotprod::dot_q15_i32` を呼びます。
標準は Rust スカラ実装ですが、別の extern symbol で上書き可能:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "Rust" fn mfsk_core_dot_q15_i32(
    a: *const i16,
    b: *const i16,
    n: usize,
) -> i32 {
    // dsps_dotprod_s16_ae32 (LX6/LX7) や
    // arm_dot_prod_q15 (Cortex-M with CMSIS-DSP) などへ橋渡し
}
```

これが stage 3 の最内ループです。ターゲットネイティブの MAC ユニットへ
橋渡しすれば、cached-RAM ターゲットで大きく速くなります。LX6 +
esp-dsp で MAC 1 サイクルあたり約 2 mac/cycle。

## BASIS scratch

`fill_symbol_spectra_into` (および `decode_block_into` /
`process_candidates_into` / `refine_candidates_into` ラッパー群) は
基底ベクター用の i16 scratch を呼び出し側から受け取ります:

```rust
const SCRATCH_LEN: usize = mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
static mut BASIS_RE: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
static mut BASIS_IM: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
```

`BASIS_SCRATCH_LEN = NTONES × NSPS = 15 360` (約 30 KB / 軸、
2 軸合計 60 KB)。これは **internal RAM (`.bss`、PSRAM 不可)** に
置く必要があります。Core2 のような cached-PSRAM ターゲットでは
PSRAM 上の basis は内積 1 項あたり 5–10 サイクル余分にかかり、
ASM kernel が遅くなります。`static` 配列形式は `.bss` に自動配置されます。
ヒープ確保したい場合は `heap_caps_malloc(BASIS_SCRATCH_LEN * 2,
MALLOC_CAP_INTERNAL)` を使ってください。

`decode_block_into` / `process_candidates_into` /
`refine_candidates_into` の `pub fn` 群は、組み込みコール元が
basis を decode 毎にアロケートせず通せるよう用意されています。
非 `_into` 版 (`decode_block` など) は標準ヒープに確保するため、
PSRAM 構成の ESP32 では低速 basis でホットパスが走ります。

## Q-format 早見表

| ステージ | 形式 | レンジ | ファイル |
|---|---|---|---|
| spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)` | `ft8::decode_block::Spectrogram` |
| DFT 基底 | Q15 i16 (cos, sin) | ±2¹⁵ ≈ ±1.0 | `fill_symbol_spectra_into` |
| シンボル cs | `Cmplx<f32>` (デフォルト) または `Cmplx<Q14i16>` (`fixed-point-cs`) | f32 制限なし、Q14 ±2 | `core::scalar::Cmplx` |
| LLR | f32 (ホスト) または Q11 i16 (`fixed-point-llr`) | f32 制限なし、Q11 ±16 | `core::scalar::LlrScalar` |
| BP メッセージ | T (LLR と同じ) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## ライブラリで提供しないもの

mfsk-core はデコード/エンコードパイプラインまでです。以下は
ハードウェア依存が大きく汎用 API が役に立たないため、**意図的に
スコープ外**です:

- 音声入出力 (I2S, マイクゲイン, sample-rate clock recovery)
- ディスプレイ / UI (TFT, OLED)
- ネットワーク (Wi-Fi, BLE, MQTT)
- RTOS タスク連携
- 時刻同期 (NTP, GPS)
- 永続化 / 設定ストレージ

`embedded-poc/m5stack-core2/src/main.rs` が 1 ターゲット (esp-idf-svc)
での全部入り例ですが、これは **整合した動作するサンプルバイナリ**で
あって、メンテされるアプリケーションではありません。他ターゲットは
独自の glue を書いてください。中身を template として参照してください。

## パフォーマンスベンチマーク (Core2 LX6, `fixed-point` + `fixed-point-llr`)

m5stack-core2 バイナリにベイクされた 3 つの実 QSO 録音を、3 sweep
連続デコード。stage 内訳は 1 周目 (cold cache)、合計レンジは 3 sweep
の最小〜最大。

| WAV | 結果 | stage 1 (spec) | stage 2 (sync) | stage 3 (refine + BP) | **合計レンジ** |
|---|---|---|---|---|---|
| qso1 (mid-band, 3 局) | 3/3 vs `decode_frame` | 1.01 s | 0.77 s | 0.69 s | **2.87 – 3.24 s** |
| qso2 (mid-band, 5 局) | 5/5 vs `decode_frame` | 1.01 s | 0.77 s | 0.92 s | **3.10 – 3.47 s** |
| qso3 (busy band, 10 局のうち ≥7) | 7 (block-only 含む) | 1.01 s | 0.75 s | 1.83 s | **3.99 – 4.36 s** |

- **Stage 1** = spectrogram。92 × N=4096 i16 complex FFT を two-for-one
  real-FFT trick で実行 (`compute_spectrogram` under `fixed-point`)。
- **Stage 2** = 991 carrier bin × 27 lag の Costas 粗同期。LX6 では
  FPU-add 律速 — i32 パスより f32 パスのほうが速い
  (`fixed-point-coarse-i32` の節参照)。
- **Stage 3** = 候補ごとの refine fill + LLR + BP staircase。
  Core2 では OSD off (`OSD_ENABLED=false` in 例の main.rs)。
  qso1 (3 結果) と qso3 (7 結果) の差は、結果あたりの fill +
  Step-2 BP variant コスト。

3 sweep 通して recall は維持。後続 sweep の ~10 % drift は
allocator と PSRAM cache の warm-up。

生モニタログ:
`embedded-poc/m5stack-core2/logs/release_0_5_0_2026-05-02.log`
(0.5.0 リリース sweep) + コミットごとの perf-chain ログ
(`stage3_bp_pool`, `stage3_syncblocks12`, `stage3_lazy_llr`,
`two_for_one`, `phase3_coarse_i32`)。

## バイナリフットプリント (Core2 リファレンス, ELF)

`xtensa-esp32-elf-size -A` 計測:

| 領域 | サイズ | 内容 |
|---|---|---|
| IRAM (`.iram0.text` + vectors) | **69 KB** | 内部 RAM コード (esp-idf 割り込みハンドラ等) |
| DRAM (`.dram0.data` + `.bss`) | **76 KB** | 内部 RAM 静的領域 (BASIS scratch 60 KB + spectrogram キャッシュ等) |
| Flash (`.flash.text`) | **448 KB** | アプリコード |
| Flash (`.flash.rodata`) | **1.21 MB** | 読み出し専用データ (うちベイクした実 QSO WAV 約 1.08 MB) |
| **合計バイナリ** | **1.997 MB** | |

実 QSO WAV (1.08 MB) を除けば mfsk-core + esp-idf + アプリ全部で
約 920 KB。ライブラリ側だけのコードサイズはおおよそ **150-200 KB**
の見積もり (esp-idf を分離して測ると正確)。

## 組み込みパスの既知制限

- **SNR 推定値**: ブロックデコードパスの `DecodeResult.snr_db` は
  強信号で `decode_frame` (ホスト広帯域) より 4–12 dB 低めに出ます。
  ブロックパスは exact tone freq での直接 DFT を使い、
  `decode_frame` が使う 200 Hz baseband ダウンサンプル + Wiener
  チャネルイコライズを通っていないため。ホスト f32 と固定小数点
  パスで同じ delta が出るので量子化問題ではありません。アプリ
  レイヤで定数オフセットを当てる workaround は可能。proper fix は
  ブロック cs に対してイコライズを走らせる必要があり (PSRAM
  確保パターンが重い) post-0.5.0 の課題。
