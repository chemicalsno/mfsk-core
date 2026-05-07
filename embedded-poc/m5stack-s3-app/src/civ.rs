//! BLE GATT CI-V transport for IC-705 (Phase 2).
//!
//! 参照実装: `/home/ubuntu/src/rs-ft8n/docs/ble-transport.js` (K7MDL2 方式)。
//! Service UUID: 14cf8001-1ec2-d408-1b04-2eb270f14203
//! Char UUID:    14cf8002-1ec2-d408-1b04-2eb270f14203
//!
//! ペアリング順序:
//!   msg1 = FE F1 00 0x61 + UUID(41B ASCII)            + FD
//!   msg2 = FE F1 00 0x62 + DEVICE_NAME(16B padded)    + FD
//!   msg3 = FE F1 00 0x63 + PAIR_TOKEN(4B)             + FD
//!   notify FE F1 00 0x64 = CI-V bus access granted
//!
//! 以降 `write_value_without_response` で CI-V フレーム
//!   (FE FE 0xA4 0xE0 ... FD) を送出、notify で応答 (FE FE 0xE0 0xA4 ...)
//! を受け取る。
//!
//! API (Phase 2 で実装):
//!   pub async fn connect() -> Result<CivTransport>
//!   pub async fn read_freq(&self) -> Result<u64>
//!   pub async fn set_freq(&self, hz: u64) -> Result<()>
//!   pub async fn set_ptt(&self, on: bool) -> Result<()>
//!   pub async fn set_mode(&self, mode: Mode, filter: Filter) -> Result<()>
//!   pub async fn request_position(&self) -> Result<()>      // notify 経由で UTC が来る
//!
//! クレート選定: `esp32-nimble` を central として使用。Phase 0 では
//! 依存追加はせず、Phase 2 着手時に Cargo.toml の該当行を有効化する。
