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
//! # Default carrier frequency
//!
//! On power-up the driver picks [`DEFAULT_PWM_FREQ_HZ`] (currently
//! 1500 Hz) instead of the more common 50 Hz hobby-servo default.
//! At 50 Hz the H-bridge / motor inductance resonates audibly and
//! sounds exactly like a stuck buzzer; pushing the carrier above
//! ~1 kHz moves the audible component out of the most sensitive band
//! of human hearing. PCA9685's hardware ceiling is ~1526 Hz, so 1500
//! is the practical sweet spot for DC-motor-only setups.
//!
//! Note that the chip uses a single shared frequency for **all** 16
//! channels, so this default is incompatible with RC hobby servos
//! (which need 50 Hz). Switch back to 50 Hz with
//! [`Pca9685::set_frequency_hz`] if you need to drive servos.
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
pub const REG_MODE2: u8 = 0x01;
/// Sub-address 1
pub const REG_SUBADR1: u8 = 0x02;
/// Sub-address 2
pub const REG_SUBADR2: u8 = 0x03;
/// Sub-address 3
pub const REG_SUBADR3: u8 = 0x04;
/// Prescaler register (writable only in sleep mode)
pub const REG_PRESCALE: u8 = 0xFE;
/// Channel 0 ON low byte
pub const REG_LED0_ON_L: u8 = 0x06;
/// Channel 0 ON high byte
pub const REG_LED0_ON_H: u8 = 0x07;
/// Channel 0 OFF low byte
pub const REG_LED0_OFF_L: u8 = 0x08;
/// Channel 0 OFF high byte
pub const REG_LED0_OFF_H: u8 = 0x09;
/// All channels ON low byte
pub const REG_ALL_LED_ON_L: u8 = 0xFA;
/// All channels ON high byte
pub const REG_ALL_LED_ON_H: u8 = 0xFB;
/// All channels OFF low byte
pub const REG_ALL_LED_OFF_L: u8 = 0xFC;
/// All channels OFF high byte
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
pub const CH0: u8 = 0;
/// PWM channel 1
pub const CH1: u8 = 1;
/// PWM channel 2
pub const CH2: u8 = 2;
/// PWM channel 3
pub const CH3: u8 = 3;
/// PWM channel 4
pub const CH4: u8 = 4;
/// PWM channel 5
pub const CH5: u8 = 5;
/// PWM channel 6
pub const CH6: u8 = 6;
/// PWM channel 7
pub const CH7: u8 = 7;
/// PWM channel 8
pub const CH8: u8 = 8;
/// PWM channel 9
pub const CH9: u8 = 9;
/// PWM channel 10
pub const CH10: u8 = 10;
/// PWM channel 11
pub const CH11: u8 = 11;
/// PWM channel 12
pub const CH12: u8 = 12;
/// PWM channel 13
pub const CH13: u8 = 13;
/// PWM channel 14
pub const CH14: u8 = 14;
/// PWM channel 15
pub const CH15: u8 = 15;

/// Total number of channels
pub const NUM_CHANNELS: u8 = 16;

/// PWM maximum value (12-bit resolution)
pub const PWM_MAX: u16 = 4095;

/// PCA9685 default I2C address
const DEFAULT_ADDR: u8 = 0x40;

/// PCA9685 internal oscillator frequency (25MHz)
const OSCILLATOR_FREQ: u32 = 25_000_000;

/// Default PWM carrier frequency for DC-motor use (Hz).
///
/// At ~1500 Hz the PCA9685 is at its hardware ceiling
/// (`25 MHz / 4096 / 4 ≈ 1525 Hz`), well above the audible "buzz"
/// band of the H-bridge + motor coil resonance. See the long-form
/// rationale in [`Pca9685::with_address`].
pub const DEFAULT_PWM_FREQ_HZ: u16 = 1500;

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
  /// 2. Set frequency to [`DEFAULT_PWM_FREQ_HZ`]
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
      // Must match the value used in `with_address` so that downstream
      // duty calculations (e.g. servo angle conversion) stay consistent
      // when callers pick up an already-initialized chip.
      frequency_hz: DEFAULT_PWM_FREQ_HZ,
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

    // Set default frequency to ~1500 Hz, the upper limit of PCA9685.
    //
    // Why not the original 50 Hz? At 50 Hz the H-bridge / motor inductance
    // resonates audibly (the 50 Hz fundamental and its odd harmonics fall
    // inside the most sensitive part of human hearing, ~150-500 Hz),
    // producing a continuous chirp that sounds like a stuck buzzer.
    // Pushing the carrier above ~1 kHz moves the audible component out
    // of the high-sensitivity band; 1500 Hz is the chip's hard ceiling
    // (25 MHz / 4096 / (prescaler+1), with prescaler_min = 3).
    //
    // CAVEAT: PCA9685 uses one shared frequency for all 16 channels. RC
    // hobby servos require a 50 Hz / 20 ms frame, so this default is
    // incompatible with the MotorBit servo headers (S1-S8). This robot
    // only drives DC motors via the M1-M4 H-bridge, where any frequency
    // in the kHz range works fine, so we trade servo support for silent
    // operation. If servos are needed later, expose `set_frequency_hz`
    // and call it with 50 before driving them.
    driver.set_frequency_hz(DEFAULT_PWM_FREQ_HZ).await;

    // Clear all channels' duty
    for ch in 0..NUM_CHANNELS {
      driver.duty(ch, 0).await;
    }

    info!(
      "PCA9685 initialized: addr=0x{:02X}, freq={}Hz",
      addr, DEFAULT_PWM_FREQ_HZ
    );
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

  /// Force a channel into "full OFF" state (datasheet section 7.3.3).
  ///
  /// Setting bit 4 of the `LEDx_OFF_H` register puts the output into
  /// **always-low** mode, bypassing the PWM modulator entirely. This is
  /// fundamentally different from writing `duty(0)`: a 0% duty cycle still
  /// makes the modulator perform an internal compare every PWM period,
  /// which on some H-bridges (TB6612 and friends) leaks audible switching
  /// noise into the motor coils. Full-OFF cuts the modulator completely,
  /// driving the output to a clean DC ground level.
  ///
  /// We use this whenever a DC motor is commanded to stop, so the chassis
  /// is silent when standing still even with a kHz-range PWM carrier.
  ///
  /// Per the datasheet, `LEDx_OFF_H` bit 4 has priority over `LEDx_ON_H`
  /// bit 4, so we only need to write the OFF register. Writing all four
  /// bytes anyway (with the FULL_OFF bit set in OFF_H) keeps the register
  /// pair internally consistent and matches the auto-increment write
  /// pattern used elsewhere.
  pub async fn set_full_off(&mut self, channel: u8) {
    debug_assert!(channel < NUM_CHANNELS, "channel out of range (0-15)");

    let reg_base = (channel << 2) + REG_LED0_ON_L;

    // ON  = 0x0000
    // OFF_L = 0x00, OFF_H = 0b0001_0000 (FULL_OFF bit)
    let buf: [u8; 5] = [reg_base, 0x00, 0x00, 0x00, 0x10];

    if self.twim.write(self.addr, &buf).await.is_err() {
      warn!("PCA9685: I2C write failed for FULL_OFF channel {}", channel);
    }
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
