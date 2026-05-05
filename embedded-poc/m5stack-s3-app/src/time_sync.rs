//! Slot-boundary time sync (Phase 2 連動).
//!
//! 二系統の補正を持つ:
//!
//! 1. **GPS UTC offset (CI-V 経由)**
//!    - `civ.rs` が CI-V `0x23 0x00` (MY_POSIT_READ) を 30 s 間隔で送出
//!    - notify で BCD UTC が返る (`docs/ble-transport.js:_parseCivGpsTime`)
//!    - `embassy_time::Instant::now()` との offset を保持し、FT8 スロット
//!      境界 (15 s 区切り) を offset 込みで生成
//!
//! 2. **Median DT estimation across decoded messages (REQUIRED)**
//!    - 各スロットでデコードされた message ごとに DT (time-of-arrival
//!      offset, sec) が出る
//!    - そのスロットの全 message を集めて **median(DT)** を取り、
//!      ローカルクロック ↔ バンド合意 のオフセットとする
//!    - 移動平均でなく **median** を使う理由は、信号弱の誤デコードや
//!      個別 fader の影響に頑健にするため。`mfsk-core` の dt-estimator
//!      が同じ理由で median を採用しているのと一致 (commit 269ba0a)。
//!    - 用途:
//!        a. GPS が無い (BLE 切断 or IC-705 GPS unfix) ときのフォール
//!           バック時刻源として median(DT) を使う
//!        b. GPS あり時も sanity check に使い、|GPS - median(DT)| が
//!           大きすぎる場合は GPS パケットを破棄
//!
//! - **DF (audio frequency offset) は追跡しない**: S3 側に独立 LO は無く、
//!   IC-705 が提供する USB Audio がそのまま baseband。LO ドリフトは
//!   IC-705 内 (TCXO + GPS-disciplinable) で完結する。デコード結果の
//!   DF は表示用にのみ使う。
//!
//! 公開 API (Phase 2 で実装):
//!   pub fn update_gps_utc(gps_utc_ms: u64);
//!   pub fn record_decode_dt(dt_sec: f32);            // 1 デコードごと
//!   pub fn finalize_slot();                          // スロット終端で median 算出
//!   pub fn slot_dt_offset() -> f32;                  // 直近スロット median(DT)
//!   pub fn next_slot_boundary() -> embassy_time::Instant;
//!   pub fn slot_index() -> u32;                      // 0/1/2/3 (0/15/30/45 sec)
//!
//! 実装メモ: median は heapless::Vec<f32, N> を sort して中央値、N は
//! 1 スロットの最大同時デコード数 (50 程度) で固定。allocator-free。
//!
//! Phase 0 ではプレースホルダのみ。
