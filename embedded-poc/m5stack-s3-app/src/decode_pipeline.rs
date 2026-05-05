//! WAV-fed decode pipeline (Phase 3).
//!
//! `embedded_shared::apps::rx_wavsim::run` 相当を本クレート内に inline:
//! - EspLogger init は外す (FanoutLogger と競合するため main で先に install 済)
//! - 結果は `log::info!` で吐き、FanoutLogger 経由で LCD scroll panel に流れる
//! - Phase 3 の次イテレーションで `tx_picker::OccupancyMap` / `snr_norm::NoiseFloorTracker`
//!   への ingest を追加し、専用 UI region に流し込む

extern crate alloc;

use alloc::vec::Vec;
use core::fmt::Write as _;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{BASIS_SCRATCH_LEN, DEFAULT_Q_THRESH, NFFT_SPEC};
use mfsk_core::msg::wsjt77::unpack77;
use mfsk_ft8::mfsk_ft8_basis_scratch_len;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

use crate::ui::state::{DecodedRow, UI};

/// 開発用 WAV (qso3_busy のみ単独ループ — UI 構築用に最も多くのデコード結果を出す)。
static QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../assets/qso3_busy.wav"),
];

/// Per-core BASIS scratch (main side)。60 KB × 2、内部 DRAM 配置。
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// 別スレッドから呼ぶ。返らない。
pub fn run() -> ! {
    let need = mfsk_ft8_basis_scratch_len();
    assert!(BASIS_SCRATCH_LEN >= need, "BASIS_SCRATCH_LEN too small");

    // wav_sim (4) / stage1_inc (3) より高い優先度。
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(NFFT_SPEC);
    dual_core::init();

    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    stage1_inc::spawn(chunk_q, slot_q, spec_q);
    wav_sim::spawn(QSO_WAVS, chunk_q);

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz)");

    let mut slot_seq: u32 = 0;
    loop {
        // 広帯域受信 (200..3000 Hz)。phantom (qso3_busy で 3/7 件) は
        // UI に "?" 付きで表示するが、QSO FSM / TX picker は
        // multi-slot persistence test (連続 2-of-3 slot で同 callsign-pair
        // 検出) で gate するため自動応答に流れない。設計の根拠は host 側
        // 実測: 200..2000 で phantom 0 だが 2 kHz 超の real (qso1/qso2 R6WA)
        // を逃す → ユーザを "聞こえてる気"にさせない方が大事。
        let spec = pipeline::recv_box::<pipeline::SpecBundle>(spec_q);
        let pass1: Vec<SyncCandidate> = dual_core::coarse_sync_split_with_allsum(
            &spec.spec,
            100.0,
            3_000.0,
            1.0,
            PASS1_LIMIT,
            &spec.allsum_head,
            &spec.allsum_tail,
        );
        drop(spec);

        let slot = pipeline::recv_box::<pipeline::Slot>(slot_q);
        let wav_idx = slot.wav_idx;
        let n_pass1 = pass1.len();

        #[allow(static_mut_refs)]
        let pass2 = unsafe {
            dual_core::pass2_split(
                &slot.audio,
                pass1,
                MAX_CAND,
                &mut BASIS_RE,
                &mut BASIS_IM,
            )
        };

        let depth = DecodeDepth::BpAll;
        #[allow(static_mut_refs)]
        let results = unsafe {
            dual_core::stage3_split(
                &slot.audio,
                pass2,
                depth,
                DEFAULT_Q_THRESH,
                mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
                &mut BASIS_RE,
                &mut BASIS_IM,
            )
        };

        log::info!("WAV[{wav_idx}] p1={n_pass1} dec={}", results.len());
        slot_seq = slot_seq.wrapping_add(1);
        // Push every CRC-passing decode to the UI ring. The UI side
        // gates re-render on `dirty_seq` so this is one lock per
        // result + zero LCD redraws if nothing changed.
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(&r.message77) {
                    let mut msg: heapless::String<22> = heapless::String::new();
                    let take = text.len().min(msg.capacity());
                    let _ = msg.push_str(&text[..take]);
                    let row = DecodedRow {
                        df_hz: r.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                        snr_db: r.snr_db.round().clamp(-128.0, 127.0) as i8,
                        hard_errors: r.hard_errors.min(255) as u8,
                        msg,
                        slot_seq,
                    };
                    ui.push_decode(row);
                    log::info!("{:4.0}Hz {:+5.1}dB {}", r.freq_hz, r.snr_db, text);
                }
            }
        }
    }
}
