//! USB Audio Class host capture (Phase 1).
//!
//! IC-705 を ESP32-S3 USB-OTG host で UAC class device として認識し、
//! isochronous IN endpoint から 16 kHz mono i16 を吸い上げて
//! `mfsk_ft8_stream_push_i16` (12 kHz リサンプル経由) に流す。
//!
//! 実装メモ:
//! - 参照: espressif/esp_usb_audio (recipe.c) — Rust から `unsafe extern "C"`
//!   で叩く薄いラッパーを書く。
//! - サンプリング: IC-705 USB Audio は 48 kHz / 16 kHz が選べる。電力と
//!   帯域の都合で 16 kHz を選択し、`embedded_shared` の linear resampler
//!   I16→12k で 12 kHz に揃える。
//! - 検証: PC で 1500 Hz tone を IC-705 に注入 → S3 側で `peek_latest`
//!   → FFT magnitude が 1500 Hz に立つことを USB-CDC log で確認。
//!
//! Phase 0 ではプレースホルダのみ。
