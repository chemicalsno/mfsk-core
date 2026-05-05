//! BtnA / BtnB input handling (Phase 6).
//!
//! 2 ボタンしかないので **モード状態機** で意味を切替:
//!   Monitor          : BtnA = (no-op)        BtnB = カーソル移動
//!   Cursor on row    : BtnA = Call station   BtnB = カーソル移動
//!   QSO active       : BtnA = confirm/abort  BtnB = abort QSO
//!   Menu             : BtnA = 項目決定       BtnB = 項目移動
//! 長押し A = menu open / close  (800 ms threshold)
//!
//! GPIO ピン: M5StickS3 の BtnA = GPIO37, BtnB = GPIO39 (要確認)。
//! Debounce 10 ms。`embassy_time::Timer` で長押し判定。
//!
//! Phase 0 ではプレースホルダのみ。
