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

use anyhow::{Context, Result};
use esp_idf_hal::{
    delay::TickType,
    i2c::I2cDriver,
    i2s::{I2sDriver, I2sTx},
};

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
        // DAC volume — ES8311 reg 0x32 is unsigned 0..0xFF, each
        // step = 0.5 dB, with 0xBF ≈ 0 dBFS. Knock 32 dB off so the
        // sine test (and subsequent qso3 playback) sit at a polite
        // listening level on the M5StickS3's tiny speaker.
        (0x32, 0x80), // DAC volume: −32 dB (0x80)
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

/// Sine-wave test source for the audio path. 1 kHz at 44.1 kHz
/// stereo, ±0x4000 amplitude (= -6 dBFS). Use this to confirm the
/// ES8311 init + PA enable chain before plugging in real WAV data
/// — if a tone is audible the codec / amplifier / speaker chain is
/// good and any silence on real WAV is purely a sample-rate /
/// resample issue.
///
/// `wav` arg is currently unused — kept so the call site doesn't
/// need to change when we switch back to `qso3_busy.wav` playback
/// after the path is proven.
pub fn audio_thread(mut i2s: I2sDriver<'static, I2sTx>, _wav: &'static [u8]) -> ! {
    i2s.tx_enable().expect("I2S tx_enable");
    log::info!("audio: I2S TX enabled, streaming 1 kHz sine @ 44.1 kHz stereo");

    // Pre-generate one full cycle so the loop is just a memcpy.
    // 44100 / 1000 = 44.1 samples per cycle — round to 441 samples
    // = exactly 10 cycles, repeats seamlessly.
    const SR: usize = 44_100;
    const CYCLE: usize = 441; // 10 cycles
    const TWO_PI: f32 = 2.0 * core::f32::consts::PI;
    let mut buf = vec![0u8; CYCLE * 4]; // 4 bytes per stereo sample
    for n in 0..CYCLE {
        let phase = (n as f32) * TWO_PI * 1000.0 / SR as f32;
        let s = (phase.sin() * 4_096.0) as i16; // -18 dBFS digital
        let bytes = s.to_le_bytes();
        // Stereo: L, R, L, R, ... — same sample on both channels.
        buf[n * 4..n * 4 + 2].copy_from_slice(&bytes);
        buf[n * 4 + 2..n * 4 + 4].copy_from_slice(&bytes);
    }

    loop {
        if let Err(e) =
            i2s.write_all(&buf, TickType::new_millis(500).ticks())
        {
            log::warn!("audio: i2s write err {e:?}");
        }
    }
}
