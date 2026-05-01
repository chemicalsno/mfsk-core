//! FT8 sensitivity comparison: `decode_block` (embedded path, pow-of-2
//! FFT only, NMS BP) vs `decode_frame` (host path, 192k-pt FFT cache,
//! sum-product BP).
//!
//! Run with:
//!   cargo test --release -p mfsk-core --features fft-rustfft,ft8 \
//!       ft8_decode_block_snr_sweep --include-ignored -- --nocapture
//!
//! Threshold loss interpretation:
//!   - 0–1 dB worse than `decode_frame`: acceptable, ship it
//!   - 1–2 dB worse: investigate (likely the rectangular window leakage
//!     at NFFT_SPEC=8192 with fractional tone bin alignment)
//!   - more than 2 dB worse: redesign needed (Hann window, multi-bin tone
//!     sum, or a finer-resolution NFFT)

use std::f32::consts::PI;

use mfsk_core::core::{MessageCodec, MessageFields};
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult, decode_frame};
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::ft8::wave_gen;

const FS: f32 = 12_000.0;
const REF_BW: f32 = 2_500.0;
const SLOT: usize = 180_000;
const SEEDS: u64 = 30;

struct Lcg {
    s: u64,
    spare: Option<f32>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            s: seed.wrapping_add(1),
            spare: None,
        }
    }
    fn next(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn uniform(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u = self.uniform();
        let v = self.uniform();
        let mag = (-2.0 * u.ln()).sqrt();
        self.spare = Some(mag * (2.0 * PI * v).sin());
        mag * (2.0 * PI * v).cos()
    }
}

fn pack_cq(call: &str, grid: &str) -> [u8; 77] {
    let bits = mfsk_core::msg::Wsjt77Message
        .pack(&MessageFields {
            call1: Some("CQ".into()),
            call2: Some(call.into()),
            grid: Some(grid.into()),
            ..Default::default()
        })
        .unwrap();
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn make_slot(msg77: &[u8; 77], freq_hz: f32, snr_db: f32, seed: u64) -> Vec<i16> {
    let mut mix = vec![0.0f32; SLOT];
    let snr_lin = 10f32.powf(snr_db / 10.0);
    // WSJT-X SNR convention: A = sqrt(4·SNR·B/FS) with σ_noise = 1.
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();
    let itone = wave_gen::message_to_tones(msg77);
    let pcm = wave_gen::tones_to_f32(&itone, freq_hz, amp);
    let start = (0.5 * FS) as usize;
    let n = pcm.len().min(SLOT - start);
    for i in 0..n {
        mix[start + i] += pcm[i];
    }
    let mut rng = Lcg::new(seed);
    for s in mix.iter_mut() {
        *s += rng.gauss();
    }
    let peak = mix.iter().map(|x| x.abs()).fold(0.0f32, f32::max).max(1e-6);
    let scale = 29_000.0 / peak;
    mix.iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

fn hit(results: &[DecodeResult], truth: &[u8; 77]) -> bool {
    results.iter().any(|r| &r.message77 == truth)
}

/// Quantise i16 slot to i8 — embedded path divides by 256 (drops the
/// low byte) to fit in internal SRAM. SQNR ~45 dB; FT8's -24 dB
/// threshold has plenty of headroom but we measure the actual cost
/// here so the embedded port doesn't ship blind.
fn to_i8(slot_i16: &[i16]) -> Vec<i8> {
    slot_i16.iter().map(|&s| (s >> 8) as i8).collect()
}

#[test]
#[ignore = "slow: 8 SNR × 10 seeds × 2 decoders. Release build: ~2 min. CI runs via --include-ignored."]
fn ft8_decode_block_vs_decode_frame_snr_sweep() {
    let msg = pack_cq("JA1ABC", "PM95");

    println!("\n=== FT8 decode_block vs decode_frame ({SEEDS} seeds/SNR) ===");
    println!("  SNR     decode_frame   block(i16)   block(i8, embedded)");

    for snr in [-14, -16, -17, -18, -19, -20, -21, -22] {
        let mut ok_frame = 0;
        let mut ok_block_i16 = 0;
        let mut ok_block_i8 = 0;
        for seed in 0..SEEDS {
            let audio_i16 = make_slot(&msg, 1500.0, snr as f32, 0xF80000 + seed);
            let audio_i8 = to_i8(&audio_i16);

            // decode_frame: host wide-band path with 192k cache + SP BP.
            if hit(
                &decode_frame(&audio_i16, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAll, 50),
                &msg,
            ) {
                ok_frame += 1;
            }
            // decode_block on i16 (host reference for the new pipeline).
            if hit(
                &decode_block(&audio_i16, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 30),
                &msg,
            ) {
                ok_block_i16 += 1;
            }
            // decode_block on i8 (embedded actual: half storage, ~45 dB SQNR).
            if hit(
                &decode_block(&audio_i8, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 30),
                &msg,
            ) {
                ok_block_i8 += 1;
            }
        }
        println!(
            "  {:>3} dB     {:>3}/{:<2}        {:>3}/{:<2}        {:>3}/{:<2}",
            snr, ok_frame, SEEDS, ok_block_i16, SEEDS, ok_block_i8, SEEDS
        );
    }
}
