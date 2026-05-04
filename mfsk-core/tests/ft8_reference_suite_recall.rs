//! FT8 reference WAV: host full-effort vs embedded equivalent
//! wall-clock + recall comparison.
//!
//! The only formally-distributed FT8 reference recording from the
//! WSJT-X project is `samples/FT8/210703_133430.wav` (a busy-band
//! 7-station slot). It's the same audio as `embedded-poc/assets/
//! qso3_busy.wav` (verified via `cmp` 2026-05-04).
//!
//! mfsk-core's `decode_block` under `fixed-point` is bit-identical
//! between host and Xtensa builds (Issue #15 Phase 1 verified
//! 2026-05-03), so the host `decode_block` row is a faithful proxy
//! for the LX6 / LX7 production decoder's recall — only the
//! wall-clock differs (host is ~50× faster than 240 MHz Xtensa).
//! Real on-hardware times come from the rx-wavsim bench logs
//! committed in this branch.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_reference_suite_recall \
//!     -- --include-ignored --nocapture
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "fixed-point"))]

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

/// (label, path, hardware timing if measured for this WAV).
type Entry = (&'static str, &'static str, Option<HwTiming>);

#[derive(Copy, Clone)]
struct HwTiming {
    /// post-SlotEnd ms on M5Stack Core2 LX6 at q_thresh=12 (full).
    core2_post_slotend_ms: u32,
    /// post-SlotEnd ms on M5StickS3 LX7 at q_thresh=12 (full).
    s3_post_slotend_ms: u32,
}

/// Reference WAVs.
///
/// `qso3_busy` is the **WSJT-X formally-distributed FT8 reference
/// sample** (`samples/FT8/210703_133430.wav`). The other two
/// (`qso1`, `qso2`) are private on-air recordings carried in this
/// repo for breadth — they're informational, not "reference" in the
/// formal sense.
const ENTRIES: &[Entry] = &[
    // WSJT-X reference recording (busy band, 7 stations).
    (
        "qso3 busy band  (WSJT-X 210703_133430.wav)",
        "/home/minoru/src/mfsk-core/embedded-poc/assets/qso3_busy.wav",
        Some(HwTiming {
            core2_post_slotend_ms: 1434,
            s3_post_slotend_ms: 707,
        }),
    ),
    (
        "qso1 mid-band   (informational, on-air capture)",
        "/home/minoru/src/mfsk-core/embedded-poc/assets/qso1.wav",
        Some(HwTiming {
            core2_post_slotend_ms: 1303,
            s3_post_slotend_ms: 574,
        }),
    ),
    (
        "qso2 mid-band   (informational, on-air capture)",
        "/home/minoru/src/mfsk-core/embedded-poc/assets/qso2.wav",
        Some(HwTiming {
            core2_post_slotend_ms: 632,
            s3_post_slotend_ms: 370,
        }),
    ),
];

fn load_wav_i16(path: &Path) -> Option<Vec<i16>> {
    let bytes = std::fs::read(path).ok()?;
    if &bytes[0..4] != b"RIFF" {
        return None;
    }
    let mut i = 12usize;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len;
        }
        i += len;
        if len % 2 == 1 {
            i += 1;
        }
    }
    if data_off == 0 {
        return None;
    }
    Some(
        bytes[data_off..data_off + data_len.min(bytes.len() - data_off)]
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect(),
    )
}

#[test]
#[ignore = "reference recall + wall-clock; run with --include-ignored"]
fn ft8_reference_host_vs_embedded() {
    println!("\n=== FT8 reference: host full-decode vs embedded ===\n");
    println!(
        "  Host:    Ryzen / x86_64 desktop, single-thread, rustfft.\n  \
         Embed:   M5Stack Core2 (ESP32 LX6 dual-core 240 MHz, 8 MB QUAD PSRAM)\n  \
                  / M5StickS3   (ESP32-S3 LX7 dual-core 240 MHz + PIE SIMD,\n  \
                                 8 MB Octal PSRAM)\n  \
           Embedded ms = post-SlotEnd wall-clock from rx-wavsim bench\n  \
                         (stage 2 hidden under capture; pass 2 + stage 3\n  \
                         only). Logged in `logs/{{core2_q_sweep,\n  \
                         s3_workstealing}}_2026-05-04.log`."
    );
    println!();

    let bar = "─".repeat(108);
    println!(
        "  {:<48}  {:>5}  {:>5}/{:<3}  {:>6}  {:>6}/{:<3}  {:>6}  {:>6}",
        "WAV", "truth", "host", "rec", "host_ms", "embed", "rec", "core2_ms", "s3_ms"
    );
    println!("  {bar}");

    let mut sum_truth = 0usize;
    let mut sum_host = 0usize;
    let mut sum_embed = 0usize;
    let mut sum_host_ms = 0u128;
    let mut sum_blk_ms = 0u128;
    let mut sum_core2 = 0u32;
    let mut sum_s3 = 0u32;

    for (label, path, hw) in ENTRIES {
        let Some(slot) = load_wav_i16(Path::new(path)) else {
            println!("  {label:<48}  (load failed: {path})");
            continue;
        };

        // Host wide-band reference: BpAllOsd, max_cand=200.
        let t0 = Instant::now();
        let truth: BTreeSet<String> =
            decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();
        let host_ms = t0.elapsed().as_millis();

        // Embedded equivalent: BpAll, max_cand=15, PASS1=30 (default),
        // q_thresh=12 (default). Bit-identical to LX6/LX7 binary
        // under `fixed-point`.
        let t1 = Instant::now();
        let embed: BTreeSet<String> =
            decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 15)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();
        let blk_ms = t1.elapsed().as_millis();

        let host_n = truth.len();
        let embed_n = embed.len();
        let embed_hit = embed.intersection(&truth).count();
        // Host always equals truth (it produced it), so host_recall = 100 %.

        let core2_ms = hw.map(|h| h.core2_post_slotend_ms).unwrap_or(0);
        let s3_ms = hw.map(|h| h.s3_post_slotend_ms).unwrap_or(0);

        println!(
            "  {label:<48}  {host_n:>5}  {host_n:>5}/{host_n:<3}  {host_ms:>6}  {embed_n:>6}/{embed_hit:<3}  {core2_ms:>6}  {s3_ms:>6}"
        );

        sum_truth += host_n;
        sum_host += host_n;
        sum_embed += embed_hit;
        sum_host_ms += host_ms;
        sum_blk_ms += blk_ms;
        sum_core2 += core2_ms;
        sum_s3 += s3_ms;
    }

    println!("  {bar}");
    let recall_pct = if sum_truth > 0 {
        100.0 * sum_embed as f64 / sum_truth as f64
    } else {
        0.0
    };
    println!(
        "  {:<48}  {sum_truth:>5}  {sum_host:>5}/{sum_truth:<3}  {sum_host_ms:>6}  {sum_embed:>6}/{sum_embed:<3}  {sum_core2:>6}  {sum_s3:>6}",
        "TOTAL"
    );
    println!(
        "\n  Embedded recall vs host wide-band: {sum_embed} / {sum_truth} = {recall_pct:.1} %"
    );
    println!("  Host wide-band total ms (host CPU):       {sum_host_ms}");
    println!("  Host fixed-point  total ms (host CPU):    {sum_blk_ms}");
    println!(
        "  Core2 LX6  total ms (post-SlotEnd, real HW): {sum_core2}  (×{:.1} vs host wide-band)",
        sum_core2 as f64 / sum_host_ms as f64
    );
    println!(
        "  S3    LX7  total ms (post-SlotEnd, real HW): {sum_s3}  (×{:.1} vs host wide-band)",
        sum_s3 as f64 / sum_host_ms as f64
    );
}
