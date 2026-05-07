//! QSO finite state machine (Phase 4).
//!
//! 参照: `/home/ubuntu/src/rs-ft8n/docs/qso.js` クラス `QsoManager`。
//! 状態遷移: IDLE → CALLING → REPORT → FINAL → DONE → IDLE
//!
//! - CALLING: 自局 CQ または指定局呼出。retry 5 × 15 s
//! - REPORT:  受信報告返信 (SNR を txReport に変換)
//! - FINAL:   73 送信。retry 3 回
//! - DONE:    1 record を ADIF logger に渡して IDLE に戻す
//!
//! Phase 4 で実装。Phase 0 では型のみ用意。

use heapless::String;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QsoState {
    Idle,
    Calling,
    Report,
    Final,
    Done,
}

#[allow(dead_code)]
pub struct QsoManager {
    pub state: QsoState,
    pub my_call: String<11>,
    pub my_grid: String<6>,
    pub dx_call: Option<String<11>>,
    pub dx_grid: Option<String<6>>,
    pub rx_report: Option<i8>,
    pub tx_report: Option<i8>,
    pub retries: u8,
}

impl QsoManager {
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            state: QsoState::Idle,
            my_call: String::new(),
            my_grid: String::new(),
            dx_call: None,
            dx_grid: None,
            rx_report: None,
            tx_report: None,
            retries: 0,
        }
    }
}
