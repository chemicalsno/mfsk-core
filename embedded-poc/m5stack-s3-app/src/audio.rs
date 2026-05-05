//! ES8311 audio codec init + I2S TX playback (Phase 3 demo).
//!
//! M5StickS3 carries an ES8311 low-power codec on the shared I2C
//! bus (addr 0x18) and a PDM-driven internal speaker. For Phase 3 we
//! play back the same `qso3_busy.wav` the decoder is consuming so
//! the device feels alive — useful as both a sanity check on the
//! audio path and a demo while QSO-receive logic is wired in later
//! phases.
//!
//! Pinout (per `reference_m5stick_s3_pinout.md`):
//!   ES8311 MCLK = GPIO18      ES8311 BCLK  = GPIO17
//!   ES8311 LRCK = GPIO15      ES8311 DIN   = GPIO16  (S3 → codec)
//!
//! ES8311 init register sequence is a stripped-down port of the
//! M5Unified Arduino reference — minimum subset to get DAC →
//! speaker sounding correct at 12 kHz / 16-bit / mono. Full register
//! map at <https://docs.m5stack.com> ES8311 datasheet.

use core::sync::atomic::{AtomicBool, Ordering};

use anyhow::{Context, Result};
use esp_idf_hal::{
    delay::TickType,
    i2c::I2cDriver,
    i2s::{I2sDriver, I2sTx},
};

/// Audio playback gate. `true` (default) = stream WAV samples,
/// `false` = emit silence. The decode pipeline flips this off
/// around `pass2_split`+`stage3_split` because the BP stage
/// sequesters both LX7 cores hard enough that the I2S DMA buffer
/// underruns and the speaker emits buzz/clicks (user reported as
/// "ぶつぶつ"). Silence avoids the audible glitch without disabling
/// the I2S channel itself (which would re-introduce a transient
/// pop on enable/disable).
pub static AUDIO_GATE: AtomicBool = AtomicBool::new(true);

const ES8311_ADDR: u8 = 0x18;
/// PMIC (M5PM1) at I2C 0x6E. GPIO3 (bit 3 of reg 0x11) drives the
/// onboard speaker amplifier's enable line on M5StickS3 — high to
/// unmute, low to mute. Identical to the `py32pmic_i2c_addr` constant
/// in the M5Unified board source.
const PMIC_ADDR: u8 = 0x6E;
const I2C_TIMEOUT: u32 = 100;

/// Verbatim port of M5Unified's `_speaker_enabled_cb_sticks3`
/// register sequence. Programs the codec into "MCLK=BCLK" mode so
/// the I2S master can drive both clocks off the same pin pair, then
/// powers up the analog stage and the headphone-drive amplifier and
/// unmutes the DAC. Caller is responsible for asserting the PA
/// enable on PMIC GPIO3 (`pa_enable`) right after this returns.
pub fn init_es8311(i2c: &mut I2cDriver) -> Result<()> {
    i2c.write(ES8311_ADDR, &[0x00], I2C_TIMEOUT)
        .context("ES8311 not found at I2C 0x18")?;

    // Configure PMIC GPIO3 as a push-pull output, idle low.
    // Mirrors the StickS3 PA-control init in M5Unified.cpp:
    //   reg 0x16 bit3 = 0  → GPIO3 = GPIO function (not alt)
    //   reg 0x10 bit3 = 1  → GPIO3 = output
    //   reg 0x13 bit3 = 0  → push-pull
    //   reg 0x11 bit3 = 0  → output low (PA off until we play)
    pmic_bit_off(i2c, 0x16, 1 << 3)?;
    pmic_bit_on(i2c, 0x10, 1 << 3)?;
    pmic_bit_off(i2c, 0x13, 1 << 3)?;
    pmic_bit_off(i2c, 0x11, 1 << 3)?;

    // ES8311 minimum init for playback (8 registers — same set the
    // M5Unified `_speaker_enabled_cb_sticks3` writes when it enables
    // the codec at runtime).
    let seq: &[(u8, u8)] = &[
        (0x00, 0x80), // RESET + CSM power on
        (0x01, 0xB5), // CLOCK_MANAGER: MCLK source = BCLK
        (0x02, 0x18), // CLOCK_MANAGER: MULT_PRE = 3
        (0x0D, 0x01), // SYSTEM: power up analog
        (0x12, 0x00), // SYSTEM: power up DAC
        (0x13, 0x10), // SYSTEM: enable output to HP drive
        // DAC volume — ES8311 reg 0x32. M5Unified board init uses
        // 0xBF and labels it "0 dB"; the M5PaperColor board's
        // `+16 dB` value 0xCF gives a 1 dB/step calibration.
        // 0xB5 = 0xBF − 10 = ~ -10 dB ≈ 1/3 linear of M5Unified's
        // 0 dB reference (user's preferred listening level on the
        // tiny built-in speaker after the qso3 WAV bring-up).
        (0x32, 0xA9), // DAC volume: ~ -22 dB (= 0xB5 −12 dB, two halvings)
        (0x37, 0x08), // DAC: bypass equalizer
    ];
    for &(reg, val) in seq {
        i2c.write(ES8311_ADDR, &[reg, val], I2C_TIMEOUT)
            .with_context(|| format!("ES8311 reg 0x{reg:02X} write failed"))?;
    }

    log::info!("ES8311 init OK (M5Unified-port; PA disabled at PMIC GPIO3)");
    Ok(())
}

/// Drive the speaker amplifier's enable pin (PMIC GPIO3 high). Call
/// this once just before starting playback; pair with [`pa_disable`]
/// when the audio thread shuts down.
pub fn pa_enable(i2c: &mut I2cDriver) -> Result<()> {
    pmic_bit_on(i2c, 0x11, 1 << 3)
}

#[allow(dead_code)]
pub fn pa_disable(i2c: &mut I2cDriver) -> Result<()> {
    pmic_bit_off(i2c, 0x11, 1 << 3)
}

fn pmic_bit_on(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<()> {
    let mut buf = [0u8; 1];
    i2c.write_read(PMIC_ADDR, &[reg], &mut buf, I2C_TIMEOUT)
        .with_context(|| format!("PMIC read 0x{reg:02X}"))?;
    let v = buf[0] | mask;
    i2c.write(PMIC_ADDR, &[reg, v], I2C_TIMEOUT)
        .with_context(|| format!("PMIC write 0x{reg:02X}"))
}

fn pmic_bit_off(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<()> {
    let mut buf = [0u8; 1];
    i2c.write_read(PMIC_ADDR, &[reg], &mut buf, I2C_TIMEOUT)
        .with_context(|| format!("PMIC read 0x{reg:02X}"))?;
    let v = buf[0] & !mask;
    i2c.write(PMIC_ADDR, &[reg, v], I2C_TIMEOUT)
        .with_context(|| format!("PMIC write 0x{reg:02X}"))
}

/// Loop the supplied 12 kHz mono 16-bit WAV out the I2S TX driver,
/// upsampling 4× to 48 kHz stereo on the fly (zero-order hold).
/// The codec is in `MCLK=BCLK` mode so the actual sample rate is
/// whatever I2S generates — caller has set the I2S driver to 48 kHz.
///
/// 4× ZOH is audibly fine for a band-monitor demo (the FT8 audio
/// already sits below 3 kHz, so the 6 kHz Nyquist of the source
/// doesn't fold any spectral content into a problematic region).
/// Switch to a polyphase filter later if a follow-up needs better
/// transient response.
///
/// Digital attenuation of -18 dBFS (shift right by 3) sits the
/// playback at the same listening level the user OK'd in the sine
/// test (ES8311 DAC at 0x80 = -32 dB analog).
pub fn audio_thread(mut i2s: I2sDriver<'static, I2sTx>, wav: &'static [u8]) -> ! {
    // Bump our FreeRTOS priority above the dual_core worker (= 5)
    // and stage1_inc (= 3) so stage 3 BP can't starve the I2S DMA
    // refill — that starvation drains the DMA buffer to underrun
    // and the codec emits a click when audio resumes. Priority 8
    // lands between the dual_core worker and the watchdog.
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }

    i2s.tx_enable().expect("I2S tx_enable");
    log::info!("audio: streaming WAV (12 kHz mono → 48 kHz stereo, 4× ZOH, prio 8)");

    // Skip the 44-byte RIFF/fmt/data header. qso3_busy.wav is
    // canonical 12 kHz mono i16 LE.
    let pcm = if wav.len() > 44 { &wav[44..] } else { wav };

    // Output chunk = 80 ms of 48 kHz stereo i16 = 48000 × 0.08 × 4 bytes.
    // Input  chunk = 80 ms of 12 kHz mono i16 = 12000 × 0.08 × 2 = 1920 B
    //                                             = 960 samples.
    const IN_SAMPLES: usize = 960;
    const OUT_BYTES: usize = IN_SAMPLES * 4 /*upsample*/ * 4 /*stereo i16*/;
    let mut out = vec![0u8; OUT_BYTES];

    // Per-input-sample envelope, ramped towards the gate's target
    // (1.0 = play, 0.0 = mute). 600-sample ramp at the 12 kHz input
    // rate ≈ 50 ms — long enough that the I2S DMA buffer (~80 ms)
    // can drain through the ramped tail without a step discontinuity
    // even when the audio thread loses CPU to stage 3 BP for a
    // moment. (5 ms was too short: the DMA queue carries enough
    // pre-mute audio that the speaker still saw a step.)
    let mut env: f32 = 1.0;
    const ENV_STEP: f32 = 1.0 / 600.0;

    // Loop-boundary fade window — # of input samples over which we
    // ramp at the start and end of each WAV cycle. 600 samples at
    // 12 kHz = 50 ms each side. The FT8 slot has ~0.5 s of silence
    // around the tone block in the source WAV, so this fade fits
    // entirely inside the natural quiet zone and the user doesn't
    // hear a level dip on the signal itself — only the discontinuity
    // at byte_pos wrap-around is smoothed away.
    const LOOP_FADE_SAMPLES: usize = 600;
    let total_samples = pcm.len() / 2;

    let mut byte_pos = 0usize;
    loop {
        let target_env: f32 = if AUDIO_GATE.load(Ordering::Acquire) {
            1.0
        } else {
            0.0
        };
        let mut o = 0usize;
        for _ in 0..IN_SAMPLES {
            if byte_pos + 2 > pcm.len() {
                byte_pos = 0; // loop the WAV
            }
            let s = i16::from_le_bytes([pcm[byte_pos], pcm[byte_pos + 1]]);
            let sample_idx = byte_pos / 2;
            byte_pos += 2;

            // Gate envelope — walks toward the AUDIO_GATE target one
            // step per input sample (= 1/12 kHz tick).
            if env < target_env {
                env = (env + ENV_STEP).min(target_env);
            } else if env > target_env {
                env = (env - ENV_STEP).max(target_env);
            }

            // Loop-boundary envelope — 1.0 in the middle of the
            // WAV, ramping linearly to 0 in the LOOP_FADE_SAMPLES
            // closest to either end. Multiplied with the gate
            // envelope so loop discontinuity and gate transitions
            // are both smoothed.
            let dist_to_end = total_samples.saturating_sub(sample_idx);
            let loop_env = if sample_idx < LOOP_FADE_SAMPLES {
                sample_idx as f32 / LOOP_FADE_SAMPLES as f32
            } else if dist_to_end <= LOOP_FADE_SAMPLES {
                dist_to_end as f32 / LOOP_FADE_SAMPLES as f32
            } else {
                1.0
            };

            // -18 dBFS digital attenuation × gate × loop envelope.
            let attenuated = ((s >> 3) as f32 * env * loop_env) as i16;
            let attn = attenuated.to_le_bytes();
            for _ in 0..4 {
                out[o] = attn[0];
                out[o + 1] = attn[1];
                out[o + 2] = attn[0];
                out[o + 3] = attn[1];
                o += 4;
            }
        }
        if let Err(e) = i2s.write_all(&out, TickType::new_millis(500).ticks()) {
            log::warn!("audio: i2s write err {e:?}");
        }
    }
}
