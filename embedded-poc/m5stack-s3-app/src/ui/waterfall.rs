//! Waterfall renderer (Phase 3).
//!
//! 入力: 12 kHz audio stream → STFT → magnitude bins (200..2700 Hz の窓)。
//! 出力: 100 px 高 ring buffer に最新 line を最下行へ書き、上スクロール。
//!
//! 参照: `/home/minoru/src/rs-ft8n/docs/waterfall.js` (boxcar decimate, 配色)。
