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

`fixed-point` の Cargo feature
は **protocol glue がどの scalar 型を選ぶかを切り替える**だけで、
generic 本体は変わりません。組み込みポートはコードの 99 % を
ホストビルドと共有 — バグ修正・最適化は一回当てればどちらにも効きます。

### 現状で fixed-point 切替が wired up されている範囲

| コンポーネント | generic 対象 | fixed-point 切替 |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ `fixed-point` |
| LLR 計算 (`core::llr`) | `SpecScalar` × `LlrScalar` | ✅ `fixed-point` |
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

### 他ターゲット — 検証状況 vs 構想

`fft-extern` + `dotprod-extern` 契約は他ターゲットへの portable な
**設計**で、`mfsk-ffi-ft8` は非 Xtensa MCU 向けにも clean に cross-build します:

| ターゲット | `cargo build` 通る | FFT/dotprod shim 提供 | 実機検証 |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 PoC) | ✅ 旧リファレンス |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ | ❌ 候補: pico-sdk-rs 経由で CMSIS-DSP | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ | ❌ DSP ライブラリ無し。FFT は `microfft`、dot は scalar Rust | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | 未試行 | ❌ 候補: CMSIS-DSP の `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | 未試行 | ❌ scalar Rust のみ (DSP ユニット無し) | ❌ |

**実機 end-to-end 動作確認は ESP32 / ESP32-S3 (Xtensa LX6 / LX7) のみ**。
他ターゲットは cross-build できる (試: `cargo build -p mfsk-ffi-ft8
--release --no-default-features --features embedded-fixed-point,
embedded-runtime --target <T>`) が、2 つの extern Rust シンボルを
ユーザ側で用意する必要あり。RP2040 / RP2350 / Cortex-M 用の具体的
shim は将来の作業です。

`embedded-poc/m5stack-core2/` が コピー元の見本です。

## 組み込み用 Cargo feature

デフォルトの `std`, `parallel`, `fft-rustfft` を切り、組み込み
ベースラインを選びます:

```toml
[dependencies]
mfsk-core = { version = "0.5", default-features = false, features = [
    "alloc",            # Vec / Box / String — decode で必須
    "ft8",              # FT8 protocol glue
    "fft-extern",       # FFT backend は呼び出し側提供
    "fixed-point",      # u16 spec + i16 DFT + Q3i8 LLR + i16 NMS BP
    # 任意:
    # "profile-coarse",            # stage-2 サブステージ常時計測
] }
```

Stage-3 の感度は Cargo feature ではなく `process_candidates_into` の
ランタイム引数 `q_thresh: u32` で渡します。
[`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] は 12 で、現状
出荷している全 target で full recall。Phase E + work-stealing 後の
LX6 (Core2) / LX7 (M5StickS3) 実機で 12 vs 14 を A/B 測定し
(`logs/core2_q_sweep_2026-05-04.log`、`logs/s3_q_sweep_2026-05-04.log`)、
relaxed q=14 は **qso3 でのみ 0–78 ms 削減**するが、各 chip で **qso3 の弱
信号 1 局を失う** (S3: W1DIG -15.5 dB, Core2: N1PJT -18 dB) — 12
で post-SlotEnd 1.5 s 切り (Core2) / 0.8 s 切り (S3) を達成済みなので、
recall を犠牲にする価値は無く q=12 デフォルト推奨。

Feature 一覧:

| Feature | 効果 | 用途 |
|---|---|---|
| `std` | `std::env`, `std::time::Instant` を入れる。rustfft とは分離。 | std を持つ esp-idf-svc 系ターゲット。bare-metal では任意 |
| `alloc` | `extern crate alloc` + Vec / Box | 全 decode パス |
| `fft-extern` | `mfsk_core_make_default_fft_planner` extern fn 経由で FFT backend | 組み込み全般 |
| `fft-rustfft` | rustfft を FFT backend | ホスト専用 |
| `fixed-point` | u16 spec + i16 内部 DFT + Q3i8 LLR + i16 NMS BP の組み込み整数パイプライン | どの組み込みでも。host f32 と recall 同等で PSRAM 帯域半減、BP scratch ~6 KB |
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
| シンボル cs | `Cmplx<f32>` (デフォルト) または `Cmplx<Q14i16>` (manual via `core::scalar`) | f32 制限なし、Q14 ±2 | `core::scalar::Cmplx` |
| LLR | f32 (ホスト) または Q3i8 (`fixed-point`) | f32 制限なし、Q3 ±16, 1/8 LSB | `core::scalar::LlrScalar` |
| BP メッセージ | T (LLR と同じ) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## C / C++ / 非 Rust ESP-IDF プロジェクトから使う (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
は FT8 ブロックデコーダ部分の小さな C ABI です。組み込み (ESP-IDF /
RP2040 / Cortex-M) C/C++ プロジェクトから FT8 デコードを使う
推奨方法。

`embedded-fixed-point` feature 下で `no_std + alloc` 構成のため、
出力される `libmfsk_ft8.a` は Rust の `std` ランタイムを抱え込まず、
C 側から libc 二重リンク等の問題なく drop-in リンク可能。

**ESP32 Core2 で end-to-end 動作検証済み** (m5stack-core2 例):
別途 `ffi_smoke_one` パスが `mfsk_ft8_decode_i16` (C ABI) を同じ
ベイク済み WAV に対して呼んだ結果、直接 Rust の `decode_one` パスと
完全一致 — qso1 (3/3)、qso2 (5/5)、**qso3 busy band (7/7)**。
caller-managed BASIS scratch を内部 RAM に置けば、同じ FFI 呼び出しを
内部 heap-alloc で行ったケースの **約 2.6 倍速い** (qso3 で
3.74 s vs 9.57 s)、直接 Rust process_candidates_into パスとも 5 % 以内。
ログ:
- `embedded-poc/m5stack-core2/logs/ffi_into_2026-05-02.log`
  (推奨 caller-scratch)
- `embedded-poc/m5stack-core2/logs/ffi_smoke_2026-05-02.log`
  (heap-alloc 比較用)

### API 概要

cbindgen 生成 header — `mfsk-ffi-ft8/include/mfsk_ft8.h`、ビルド毎に
再生成。全 surface:

```c
typedef struct MfskFt8Result {
    char     text[40];   // NUL-終端 unpack メッセージ
    float    freq_hz;    // キャリア周波数
    float    dt_sec;     // スロット先頭からの時間オフセット
    float    snr_db;     // 既知制限 — embedded path は強信号で
                         // ~4–12 dB 低めに出る
    uint32_t hard_errors;
    uint8_t  pass;       // staircase ステージ (0=fast Bp, 1=full Bp,…)
} MfskFt8Result;

typedef struct MfskFt8ResultList {
    MfskFt8Result *items;
    size_t         len;
    size_t         _capacity;  // 内部使用
} MfskFt8ResultList;

// プライマリ decode 関数の所要 scratch 長 (i16 個数)。
// 同じ長さの 2 配列を呼び出し側で確保。
size_t mfsk_ft8_basis_scratch_len(void);

// PRIMARY 組み込みエントリ。caller-managed scratch — dot-product ASM
// kernel をピーク速度で回すには、必ず内部 RAM に置く (PSRAM 不可)。
MfskFt8Status mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz mono、≥168 000
    float freq_min_hz, float freq_max_hz,     // 典型 200, 3000
    float sync_min, int max_cand,             // 典型 1.0, 30
    MfskFt8Depth depth,                       // 0=Bp, 1=BpAll, 2=BpAllOsd
    int16_t *basis_re, int16_t *basis_im,     // scratch
    MfskFt8ResultList *out);                  // 結果

// HOST 専用簡便版 — basis を内部 heap-alloc。組み込みビルドでは
// 意図的に提供しない (下記「caller-supply scratch の理由」参照)。
#ifdef MFSK_FT8_HOST  // default `host` feature 有効時
MfskFt8Status mfsk_ft8_decode_i16_alloc(
    const int16_t *audio, size_t n_samples,
    float freq_min_hz, float freq_max_hz,
    float sync_min, int max_cand,
    MfskFt8Depth depth,
    MfskFt8ResultList *out);
#endif

void mfsk_ft8_result_list_free(MfskFt8ResultList *list);
```

### なぜ caller-supply scratch (組み込みでは選択肢ですらない)

60 KB の `BASIS` scratch (cos/sin Q15 rotator × 8 tone × 1920 sample) は
**dot-product 内ループの hot data** です。esp-dsp の ASM kernel
`dsps_dotprod_s16_ae32` が 2 MAC / cycle のピークで回るのは、
basis が**高速な内部 SRAM (DRAM)** にあるときのみ。Core2 で PSRAM が
有効 (デフォルト) の場合、標準 `malloc` heap は PSRAM に流れ、
PSRAM-resident な読み出しはキャッシュ越しで **5–10 cycle/sample の
ストール**を発生させ、kernel が定格の ~30 % まで落ちます。BASIS が
PSRAM にあると stage 3 wall-clock が文字通り **2〜3 倍**になります。

C から `malloc` の戻り先は予測できません — ESP-IDF の heap は
size と capability flag で内部 RAM/PSRAM をルーティングし、60 KB
要求は明示的に絞らない限り PSRAM に行きます。なので 60 KB malloc を
裏で隠す「簡便版」は組み込み側で必ず性能を毀損する。組み込みには
わざと用意せず、`mfsk_ft8_decode_i16` は scratch を引数で受け取る
形に統一しています。呼び出し側がポリシーを決める:

```c
// 一番シンプルで正しいパターン: static .bss 配列。
// 自動的に内部 DRAM に乗り、プロセス寿命中保持される。
#include "mfsk_ft8.h"
static int16_t basis_re[15360];   // = mfsk_ft8_basis_scratch_len()
static int16_t basis_im[15360];

MfskFt8ResultList results = {0};
MfskFt8Status st = mfsk_ft8_decode_i16(
    audio, n_samples,
    200.0f, 3000.0f, 1.0f, 30,
    MFSK_FT8_DEPTH_BP_ALL,
    basis_re, basis_im,
    &results);
```

動的確保したいなら ESP-IDF の capability-flag アロケータを:
`heap_caps_malloc(15360 * sizeof(int16_t),
MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)`。複数 decode 呼び出しを
跨いで保持してください — slot ごとに reset 不要。

### Build flag

#### Host (`libmfsk_ft8.so` / `libmfsk_ft8.a` ホストテスト用)

```sh
cargo build -p mfsk-ffi-ft8 --release
# → target/release/libmfsk_ft8.{so,a}
# → mfsk-ffi-ft8/include/mfsk_ft8.h (cbindgen 生成)
```

デフォルト feature で `mfsk-core/std + ft8 + fft-rustfft` が入る。
`.so` を C テストから link する例が
`mfsk-ffi-ft8/tests/c_smoke/smoke.c`:

```sh
gcc -O2 -I mfsk-ffi-ft8/include \
    mfsk-ffi-ft8/tests/c_smoke/smoke.c \
    -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
    -Wl,-rpath,$PWD/target/release \
    -o /tmp/mfsk_smoke
/tmp/mfsk_smoke embedded-poc/m5stack-core2/assets/qso3_busy.wav
```

#### 組み込み (Xtensa ESP32, `libmfsk_ft8.a` を ESP-IDF link)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf
# → target/xtensa-esp32-espidf/release/libmfsk_ft8.a
```

`-C panic=abort` 必須 — Rust unwinding panic は `std` を要求し、
組み込みでは `panic = "abort"` 一択。ESP-IDF プロジェクトでは通常
`.cargo/config.toml` に書く:

```toml
[target.xtensa-esp32-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

#### Feature 一覧

| Feature | Default | 用途 |
|---|---|---|
| `host` | ✓ | ホストビルド — `mfsk-core/std + ft8 + fft-rustfft`。`mfsk_ft8_decode_i16` (caller scratch) と `mfsk_ft8_decode_i16_alloc` (heap 簡便) の**両方**を export |
| `embedded-fixed-point` | — | `no_std + alloc`。`mfsk-core/fft-extern + fixed-point`。**`mfsk_ft8_decode_i16` のみ** export — heap-alloc 簡便版は意図的に除外 (上記参照)。`mfsk_core_make_default_fft_planner_*` と `mfsk_core_dot_q15_i32` を linker が解決する必要 (典型的には小さな Rust shim が esp-dsp に橋渡し) |
| `embedded-runtime` | — | crate 内に default `#[panic_handler]` (libc `abort` 呼び) + `#[global_allocator]` (libc `malloc`/`free`) を提供。自己完結 `staticlib` 用。同じイメージに別の Rust ランタイムを重ねるときは OFF |

### ESP-IDF (CMake) プロジェクトへの組み込み

```
your-app/                          # esp-idf project ルート
├── main/main.c                    # mfsk_ft8_decode_i16(...) を呼ぶ
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib コンポーネント
│   ├── include/mfsk_ft8.h         # mfsk-ffi-ft8 ビルド成果物
│   └── lib/libmfsk_ft8.a          # mfsk-ffi-ft8 ビルド成果物
└── shim/                          # 小さな Rust crate (esp-dsp bridge)
    ├── Cargo.toml                 # mfsk-ffi-ft8 に依存
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # mfsk_core_make_default_fft_planner と
                                   # mfsk_core_dot_q15_i32 を esp-dsp 経由で提供
```

shim/ の Rust crate が必要なのは、mfsk-core の FFT-extern 契約が
`extern "Rust"` (C と ABI が違う) を使うため、純 C コンパイルユニットからは
提供不可。shim は ~50 行 Rust + `embedded-poc/m5stack-core2/src/esp_dsp_fft.rs`
の vendor copy で済みます。

`components/mfsk_ft8/CMakeLists.txt` 最小例:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_ft8_rust STATIC IMPORTED)
set_target_properties(mfsk_ft8_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_ft8.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_ft8_rust)
```

完成 skeleton は
[`embedded-poc/idf-component/`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/idf-component)。

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

## パフォーマンスベンチマーク (Core2 LX6, `fixed-point`)

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
- **Stage 2** = 991 carrier bin × 27 lag の Costas 粗同期。LX6/LX7 では
  FPU-add 律速 — f32 allsum を FPU で動かしつつインデックス計算を ALU で
  並列化できる。i32 化すると両方が ALU に直列化して ~25 % stage-2
  wall-clock を浪費するため、整数 coarse_sync feature は廃止済み。
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

### Dual-core + Phase E (0.5.3)

A 〜 E まで完全に積んだ pipeline 並列化 (FFT prewarm; per-core BASIS
scratch; Pass 2 / Stage 3 / Stage 2 を PRO_CPU と APP_CPU で半分割;
incremental stage 1 を capture window に隠蔽) で、同じ 3 録音が
Core2 LX6 上で **sub-2 s** 帯に落ちます。`rx-wavsim` streaming bench
(baked WAV を `mfsk_ft8_stream_*` ring に real-time pace で push、
WAV 完了 notify で 1 slot decode) end-to-end の数値:

| WAV | 結果 | stage 1 (take) | stage 2 | pass 2 | stage 3 | **合計** |
|---|---|---|---|---|---|---|
| qso1 (mid-band, 3 局) | 3/3 ✓ | 0.05 s | 0.68 s | 0.18 s | 0.92 s | **1.83 s** |
| qso2 (mid-band, 5 局) | 5/5 ✓ | 0.05 s | 0.68 s | 0.18 s | 0.54 s | **1.45 s** |
| qso3 (busy band, 7 局) | 7/7 ✓ | 0.05 s | 0.65 s | 0.18 s | 1.10 s | **1.98 s** |

Recall: 3 WAV 通算 15/15 ground-truth callsign 全て decode、qso3 の
-18.2 dB `N1PJT` や qso2 の -17.9 / -18.0 dB (`OH3NIV`, `LZ1JZ`) も
含む。phantom 無し。

- **Stage 1** は `take_spec()` が返すだけ。実体は `stage1_inc.rs` の
  APP_CPU worker が slot capture (15 s) の間に増分計算済み。
  Per-pair FFT (~5 ms each) は audio chunk 到着のたびに発火、累計
  ~1.0 s 分の compute が 15 s window に分散 (~6 % CPU 利用)。decode
  latency budget からは実質ゼロコスト。
- **Stage 2** は `coarse_sync_split` で carrier bin range を半分割
  (worker が 1 550 Hz 以上、main が 下半。`score` 降順マージ→
  PASS1_LIMIT 切り詰め)。`coarse_sync` は setup / sort / dedupe が
  支配的なので gain は控えめ (-9 % 単独)。
- **Pass 2** (sync_quality_block0 re-rank) と **Stage 3** (refine
  + BP staircase) も per-cand で半分割。各 core が専用 60 KB の
  Q15 BASIS scratch (内部 DRAM 常駐) を持つため、esp-dsp asm dot
  product が両 core で 1 cycle/sample で走る。

生ログ: `embedded-poc/m5stack-core2/logs/rx_wavsim_phaseE_2026-05-03.log`。

Compute bench (`main.rs` sweep、streaming overlap 無し) の同時期
スナップショットは dual-core only の数値 (qso3 3.22 s) を出します;
log: `dual_core_phase_abcd_2026-05-03.log`。この 3.22 s と streaming
版 1.98 s の 1.24 s 差が、まさに Phase E が capture window 下に
隠した stage 1 の wall-clock です。

### S3 LX7 (M5StickS3) — post-SlotEnd で sub-1 s

同じ pipeline を ESP32-S3 LX7 (stage 1/2 の dot product に PIE SIMD
が効く以外は Core2 と同一) で動かすと、3 つの WAV 全てが
**post-SlotEnd 1 秒以内** で復号完了します (relaxed-recall 不使用):

| WAV | 結果 | post-SlotEnd | stage 2 (capture 中) | pass 2 | stage 3 |
|---|---|---:|---:|---:|---:|
| qso1 (mid-band, 3 局) | 3/3 ✓ | **0.574 s** | 0.18 s (隠蔽) | 0.12 s | 0.44 s |
| qso2 (mid-band, 5 局) | 5/5 ✓ | **0.370 s** | 0.18 s (隠蔽) | 0.12 s | 0.24 s |
| qso3 (busy band, 7 局) | 7/7 ✓ | **0.707 s** | 0.18 s (隠蔽) | 0.12 s | 0.58 s |

Core2 の数値と比べた wall-clock 改善は 2 つ:

1. **Stage 2 が capture 末尾と並列実行され隠蔽される。**
   `stage1_inc` が pair 92 完成時 (SlotEnd の ~200 ms 前) に
   `SpecBundle` (spec + per-half allsum) を `spec_q` queue で送出し、
   main 側は audio capture 末尾と並列で `coarse_sync_split_with_allsum`
   を実行。post-SlotEnd の budget には入らない。
   `embedded_shared::pipeline::SpecBundle` と
   `embedded_shared::stage1_inc::worker_main` の `emit_spec_bundle`
   呼び出し参照。
2. **Stage 3 work-stealing。** `dual_core::stage3_split` は
   candidate を head / tail に事前分割しない。PRO_CPU と APP_CPU が
   共有 `Vec<Option<RefinedCandidate>>` から `AtomicUsize::fetch_add(1)`
   で動的に取り合う。失敗 cand が偏って一方が手待ちになる事態が
   原理的に発生しない。qso3 (15 cand 中 7 が失敗、4 LLR variant
   全部走る) で stage 3 単独 0.81 s → 0.58 s (-28 %)。

生ログ: `embedded-poc/m5stack-s3/logs/s3_workstealing_2026-05-04.log`。

Recall は完全保持 (decode メッセージ・SNR・hard error count が固定
分割版と bit-identical)。MAX_CAND=15、PASS1_LIMIT=30、relaxed-recall
は使用していない。

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
