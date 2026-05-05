//! M5PM1 PMIC bring-up (I2C 0x6E on bus0 SDA=47 SCL=48).
//!
//! M5StickS3 では LCD 電源が PMIC 経由で制御されている。電源 ON しない
//! 限り SPI 経路が完璧でも画面は黒のまま — これが Phase 0.5 LCD bring-up
//! 初回試行で表示が出なかった真の原因。
//!
//! 移植元: m5stack/M5GFX/src/M5GFX.cpp の `board_M5StickS3` 分岐
//! (PM1_G2 = LCD Power Enable シーケンス、`lgfx::i2c::bitOff/bitOn`)。
//!
//! 手順:
//!   reg 0x16 bit2 = 0   : GPIO2 を GPIO 機能に設定 (alt function 解除)
//!   reg 0x10 bit2 = 1   : GPIO2 = output
//!   reg 0x13 bit2 = 0   : GPIO2 push-pull
//!   reg 0x11 bit2 = 1   : GPIO2 output HIGH → LCD 電源 ON
//!   reg 0x09     = 0x00 : I2C idle sleep を disable (PMIC が深い idle に
//!                          落ちて以降の通信不能になるバグへの workaround)
//! その後 100ms 待ち。

use anyhow::{anyhow, Context};
use esp_idf_hal::{
    gpio::{Gpio47, Gpio48},
    i2c::{I2cConfig, I2cDriver, I2C1},
    units::FromValueType,
};

const ADDR: u8 = 0x6E;

/// M5GFX が `I2C_NUM_1` (= `I2C1`) を使うのに合わせる。`I2C0` だと
/// ESP_ERR_TIMEOUT で PMIC に到達できなかった。
pub fn init_lcd_power<'d>(
    i2c1: I2C1<'d>,
    sda: Gpio47<'d>,
    scl: Gpio48<'d>,
) -> Result<I2cDriver<'d>, anyhow::Error> {
    let cfg = I2cConfig::new().baudrate(100_u32.kHz().into());
    let mut i2c = I2cDriver::new(i2c1, sda, scl, &cfg).context("I2C1 driver init")?;

    // ── Bus scan: 0x03..0x77 を 0 バイト write で probe (NACK = 不在) ──
    // 何が応答するか実機で見て、想定 (0x6E PMIC, 0x68 IMU) と突合する。
    log::info!("I2C bus scan:");
    let mut found: heapless::Vec<u8, 16> = heapless::Vec::new();
    for addr in 0x03u8..=0x77 {
        if i2c.write(addr, &[], 50).is_ok() {
            let _ = found.push(addr);
        }
    }
    if found.is_empty() {
        log::warn!("  (no devices responded — bus dead or no pull-ups)");
    } else {
        for a in &found {
            log::info!("  device @ {:#04x}", a);
        }
    }

    // M5PM1 device id read (reg 0x00) — sanity check.
    let mut dev_id = [0u8; 1];
    i2c.write_read(ADDR, &[0x00], &mut dev_id, 100)
        .map_err(|e| anyhow!("M5PM1 device id read failed: {e:?}"))?;
    log::info!("M5PM1 device id @0x00 = {:#04x}", dev_id[0]);

    bit_off(&mut i2c, 0x16, 1 << 2)?; // GPIO2 = function GPIO
    bit_on(&mut i2c, 0x10, 1 << 2)?; // GPIO2 = output
    bit_off(&mut i2c, 0x13, 1 << 2)?; // GPIO2 push-pull
    bit_on(&mut i2c, 0x11, 1 << 2)?; // GPIO2 = HIGH → LCD power
    write_reg(&mut i2c, 0x09, 0x00)?; // I2C idle sleep off

    std::thread::sleep(std::time::Duration::from_millis(100));
    log::info!("M5PM1 LCD power enabled (PM1_G2=HIGH)");
    Ok(i2c)
}

fn read_reg(i2c: &mut I2cDriver, reg: u8) -> Result<u8, anyhow::Error> {
    let mut buf = [0u8; 1];
    i2c.write_read(ADDR, &[reg], &mut buf, 100)
        .map_err(|e| anyhow!("M5PM1 read reg {reg:#04x}: {e:?}"))?;
    Ok(buf[0])
}

fn write_reg(i2c: &mut I2cDriver, reg: u8, val: u8) -> Result<(), anyhow::Error> {
    i2c.write(ADDR, &[reg, val], 100)
        .map_err(|e| anyhow!("M5PM1 write reg {reg:#04x}={val:#04x}: {e:?}"))
}

fn bit_on(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<(), anyhow::Error> {
    let cur = read_reg(i2c, reg)?;
    write_reg(i2c, reg, cur | mask)
}

fn bit_off(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<(), anyhow::Error> {
    let cur = read_reg(i2c, reg)?;
    write_reg(i2c, reg, cur & !mask)
}
