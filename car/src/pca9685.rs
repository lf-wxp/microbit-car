#![allow(dead_code)]
//! PCA9685 16-channel 12-bit PWM I2C driver
//!
//! Fully ported from pca9685.py, provides low-level PWM control.
//!
//! # Features
//!
//! - Configurable PWM frequency (24Hz ~ 1526Hz)
//! - 16 independent PWM output channels
//! - Per-channel ON/OFF timing control (0-4095)
//! - Convenient duty setting (ON=0, OFF=duty)
//!
//! # Register Layout
//!
//! ```text
//! 4 bytes per channel:
//!   base = (channel << 2) + LED0_ON_L
//!   [ON_L, ON_H, OFF_L, OFF_H]
//! ```

use embassy_nrf::twim::Twim;
use embassy_time::{Duration, Timer};

use defmt::{info, warn};

// --- PCA9685 Register Addresses ---

/// MODE1 register
pub const REG_MODE1: u8 = 0x00;
/// MODE2 register
#[allow(dead_code)]
pub const REG_MODE2: u8 = 0x01;
/// Sub-address 1
#[allow(dead_code)]
pub const REG_SUBADR1: u8 = 0x02;
/// Sub-address 2
#[allow(dead_code)]
pub const REG_SUBADR2: u8 = 0x03;
/// Sub-address 3
#[allow(dead_code)]
pub const REG_SUBADR3: u8 = 0x04;
/// Prescaler register (writable only in sleep mode)
pub const REG_PRESCALE: u8 = 0xFE;
/// Channel 0 ON low byte
pub const REG_LED0_ON_L: u8 = 0x06;
/// Channel 0 ON high byte
#[allow(dead_code)]
pub const REG_LED0_ON_H: u8 = 0x07;
/// Channel 0 OFF low byte
#[allow(dead_code)]
pub const REG_LED0_OFF_L: u8 = 0x08;
/// Channel 0 OFF high byte
#[allow(dead_code)]
pub const REG_LED0_OFF_H: u8 = 0x09;
/// All channels ON low byte
#[allow(dead_code)]
pub const REG_ALL_LED_ON_L: u8 = 0xFA;
/// All channels ON high byte
#[allow(dead_code)]
pub const REG_ALL_LED_ON_H: u8 = 0xFB;
/// All channels OFF low byte
#[allow(dead_code)]
pub const REG_ALL_LED_OFF_L: u8 = 0xFC;
/// All channels OFF high byte
#[allow(dead_code)]
pub const REG_ALL_LED_OFF_H: u8 = 0xFD;

// --- MODE1 Register Bits ---

/// ALLCALL bit (bit0)
const MODE1_ALLCALL: u8 = 0x01;
/// Sleep mode bit (bit4)
const MODE1_SLEEP: u8 = 0x10;
/// Auto-Increment bit (bit5)
const MODE1_AI: u8 = 0x20;
/// Restart bit (bit7)
const MODE1_RESTART: u8 = 0x80;

// --- PWM Output Channel Numbers (CH0~CH15) ---

/// PWM channel 0
#[allow(dead_code)]
pub const CH0: u8 = 0;
/// PWM channel 1
#[allow(dead_code)]
pub const CH1: u8 = 1;
/// PWM channel 2
#[allow(dead_code)]
pub const CH2: u8 = 2;
/// PWM channel 3
#[allow(dead_code)]
pub const CH3: u8 = 3;
/// PWM channel 4
#[allow(dead_code)]
pub const CH4: u8 = 4;
/// PWM channel 5
#[allow(dead_code)]
pub const CH5: u8 = 5;
/// PWM channel 6
#[allow(dead_code)]
pub const CH6: u8 = 6;
/// PWM channel 7
#[allow(dead_code)]
pub const CH7: u8 = 7;
/// PWM channel 8
#[allow(dead_code)]
pub const CH8: u8 = 8;
/// PWM channel 9
#[allow(dead_code)]
pub const CH9: u8 = 9;
/// PWM channel 10
#[allow(dead_code)]
pub const CH10: u8 = 10;
/// PWM channel 11
#[allow(dead_code)]
pub const CH11: u8 = 11;
/// PWM channel 12
#[allow(dead_code)]
pub const CH12: u8 = 12;
/// PWM channel 13
#[allow(dead_code)]
pub const CH13: u8 = 13;
/// PWM channel 14
#[allow(dead_code)]
pub const CH14: u8 = 14;
/// PWM channel 15
#[allow(dead_code)]
pub const CH15: u8 = 15;

/// Total number of channels
pub const NUM_CHANNELS: u8 = 16;

/// PWM maximum value (12-bit resolution)
pub const PWM_MAX: u16 = 4095;

/// PCA9685 default I2C address
const DEFAULT_ADDR: u8 = 0x40;

/// PCA9685 internal oscillator frequency (25MHz)
const OSCILLATOR_FREQ: u32 = 25_000_000;

/// PCA9685 driver struct
///
/// Encapsulates I2C communication, provides PWM frequency setting and channel control.
pub struct Pca9685<'a> {
  twim: &'a mut Twim<'static>,
  addr: u8,
  frequency_hz: u16,
}

impl<'a> Pca9685<'a> {
  /// Create and initialize PCA9685 driver
  ///
  /// Initialization sequence matches pca9685.py `__init__`:
  /// 1. Write MODE1 = 0x00 to reset
  /// 2. Set frequency to 50Hz
  /// 3. Clear all 16 channels' duty to 0
  pub async fn new(twim: &'a mut Twim<'static>) -> Self {
    Self::with_address(twim, DEFAULT_ADDR).await
  }

  /// Resume initialized PCA9685 driver (skip initialization)
  ///
  /// Used when PCA9685 has already been initialized via `new()` and needs to be reused.
  /// This is a zero-cost reference wrapper, no I2C communication performed.
  pub fn resume(twim: &'a mut Twim<'static>) -> Self {
    Self::resume_with_address(twim, DEFAULT_ADDR)
  }

  /// Resume initialized PCA9685 driver with specified address
  pub fn resume_with_address(twim: &'a mut Twim<'static>, addr: u8) -> Self {
    Self {
      twim,
      addr,
      frequency_hz: 50, // Assume already initialized to 50Hz
    }
  }

  /// Create and initialize PCA9685 driver with specified I2C address
  pub async fn with_address(twim: &'a mut Twim<'static>, addr: u8) -> Self {
    let mut driver = Self {
      twim,
      addr,
      frequency_hz: 0,
    };

    driver.write_reg(REG_MODE1, 0x00).await;
    Timer::after(Duration::from_millis(5)).await;

    // Set default frequency to 50Hz
    driver.set_frequency_hz(50).await;

    // Clear all channels' duty
    for ch in 0..NUM_CHANNELS {
      driver.duty(ch, 0).await;
    }

    info!("PCA9685 initialized: addr=0x{:02X}, freq=50Hz", addr);
    driver
  }

  /// Get current PWM frequency
  pub fn frequency_hz(&self) -> u16 {
    self.frequency_hz
  }

  /// Set PWM frequency
  ///
  /// Exactly matches pca9685.py `frequency_hz.setter`:
  /// ```text
  /// prescaler = round(25000000 / 4096 / freq - 1)
  /// ```
  ///
  /// # Arguments
  /// - `freq`: Target frequency (Hz), valid range approx. 24~1526Hz
  pub async fn set_frequency_hz(&mut self, freq: u16) {
    let prescaler = ((OSCILLATOR_FREQ + (4096 * freq as u32) / 2) / (4096 * freq as u32) - 1) as u8;

    // Read current MODE1
    let old_mode = self.read_reg(REG_MODE1).await;
    // Enter sleep mode (set bit4, clear bit7)
    let sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP;
    self.write_reg(REG_MODE1, sleep_mode).await;
    // Write prescale (only modifiable in sleep mode)
    self.write_reg(REG_PRESCALE, prescaler).await;
    // Restore old mode
    self.write_reg(REG_MODE1, old_mode).await;
    Timer::after(Duration::from_millis(5)).await;
    // Enable restart + auto-increment + allcall (old_mode | 0xA1)
    self
      .write_reg(
        REG_MODE1,
        old_mode | MODE1_RESTART | MODE1_AI | MODE1_ALLCALL,
      )
      .await;

    self.frequency_hz = freq;
  }

  /// Set channel PWM ON/OFF timing points
  ///
  /// Exactly matches pca9685.py `pwm(channel, on, off)`.
  /// Writes 4 bytes: struct.pack("<HH", on, off)
  ///
  /// # Arguments
  /// - `channel`: Channel number (0-15)
  /// - `on`: ON timing point (0-4095), position in PWM cycle where high level starts
  /// - `off`: OFF timing point (0-4095), position in PWM cycle where low level starts
  ///
  /// # Panics
  /// If channel > 15 or on/off > 4095
  pub async fn pwm(&mut self, channel: u8, on: u16, off: u16) {
    debug_assert!(channel < NUM_CHANNELS, "channel out of range (0-15)");
    debug_assert!(on <= PWM_MAX, "on value out of range (0-4095)");
    debug_assert!(off <= PWM_MAX, "off value out of range (0-4095)");

    let reg_base = (channel << 2) + REG_LED0_ON_L;

    // Write 4 bytes: ON_L, ON_H, OFF_L, OFF_H (little-endian u16 pair)
    let buf: [u8; 5] = [
      reg_base,
      (on & 0xFF) as u8,
      ((on >> 8) & 0x0F) as u8,
      (off & 0xFF) as u8,
      ((off >> 8) & 0x0F) as u8,
    ];

    if self.twim.write(self.addr, &buf).await.is_err() {
      warn!("PCA9685: I2C write failed for channel {}", channel);
    }
  }

  /// Set channel duty cycle (ON=0, OFF=duty)
  ///
  /// Equivalent to pca9685.py `duty(channel, duty)`, i.e., `pwm(channel, 0, duty)`.
  ///
  /// # Arguments
  /// - `channel`: Channel number (0-15)
  /// - `value`: Duty cycle (0-4095)
  pub async fn duty(&mut self, channel: u8, value: u16) {
    self.pwm(channel, 0, value.min(PWM_MAX)).await;
  }

  /// Read a single register
  async fn read_reg(&mut self, reg: u8) -> u8 {
    let mut buf = [0u8; 1];
    if self
      .twim
      .write_read(self.addr, &[reg], &mut buf)
      .await
      .is_err()
    {
      warn!("PCA9685: I2C read failed for reg 0x{:02X}", reg);
      return 0;
    }
    buf[0]
  }

  /// Write a single register
  async fn write_reg(&mut self, reg: u8, value: u8) {
    let buf: [u8; 2] = [reg, value];
    if self.twim.write(self.addr, &buf).await.is_err() {
      warn!("PCA9685: I2C write failed for reg 0x{:02X}", reg);
    }
  }
}
