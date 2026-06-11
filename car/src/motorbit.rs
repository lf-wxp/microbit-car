#![allow(dead_code)]
//! MotorBit expansion board driver
//!
//! Fully ported from motorbit.py, provides DC motor and servo control.
//!
//! # Hardware Layout
//!
//! MotorBit board provides via PCA9685:
//! - 4x DC motors (M1-M4), each using 2 PWM channels (forward/reverse)
//! - 8x Servos (S1-S8), each using 1 PWM channel
//!
//! # Channel Mapping
//!
//! ```text
//! DC Motors:
//!   M1: positive=CH0,  negative=CH1
//!   M2: positive=CH2,  negative=CH3
//!   M3: positive=CH4,  negative=CH5
//!   M4: positive=CH6,  negative=CH7
//!
//! Servos:
//!   S1=CH8, S2=CH9, S3=CH10, S4=CH11
//!   S5=CH12, S6=CH13, S7=CH14, S8=CH15
//! ```

use crate::pca9685::{self, PWM_MAX, Pca9685};
use defmt::trace;

// --- DC Motor Index ---

/// Motor M1 index
pub const M1: u8 = 0;
/// Motor M2 index
pub const M2: u8 = 1;
/// Motor M3 index
pub const M3: u8 = 2;
/// Motor M4 index
pub const M4: u8 = 3;

// --- DC Motor Channel Mapping ---

/// Motor channel mapping: [(positive_channel, negative_channel); 4]
/// Matches DcMotor initialization parameters in motorbit.py
const DC_MOTOR_CHANNELS: [(u8, u8); 4] = [
  (pca9685::CH0, pca9685::CH1), // M1: A01, A02
  (pca9685::CH2, pca9685::CH3), // M2: A03, A04
  (pca9685::CH4, pca9685::CH5), // M3: B01, B02
  (pca9685::CH6, pca9685::CH7), // M4: B03, B04
];

// --- Servo Channel Mapping ---

/// Servo S1 channel
pub const S1: u8 = pca9685::CH8;
/// Servo S2 channel
pub const S2: u8 = pca9685::CH9;
/// Servo S3 channel
pub const S3: u8 = pca9685::CH10;
/// Servo S4 channel
pub const S4: u8 = pca9685::CH11;
/// Servo S5 channel
pub const S5: u8 = pca9685::CH12;
/// Servo S6 channel
pub const S6: u8 = pca9685::CH13;
/// Servo S7 channel
pub const S7: u8 = pca9685::CH14;
/// Servo S8 channel
pub const S8: u8 = pca9685::CH15;

/// Number of servos
pub const NUM_SERVOS: usize = 8;

/// Servo channel list
pub const SERVO_CHANNELS: [u8; NUM_SERVOS] = [S1, S2, S3, S4, S5, S6, S7, S8];

// --- Servo Configuration ---

/// Servo configuration parameters
///
/// Matches default parameters in motorbit.py `Servo.__init__`
#[derive(Clone, Copy)]
pub struct ServoConfig {
  /// Maximum rotation angle (degrees), default 180
  pub max_rotation_angle: u16,
  /// Minimum pulse width (microseconds), default 500
  pub min_pulse_width_us: u16,
  /// Maximum pulse width (microseconds), default 2500
  pub max_pulse_width_us: u16,
}

impl Default for ServoConfig {
  fn default() -> Self {
    Self {
      max_rotation_angle: 180,
      min_pulse_width_us: 500,
      max_pulse_width_us: 2500,
    }
  }
}

// --- MotorBit Main Struct ---

/// MotorBit expansion board driver
///
/// Provides complete control for 4 DC motors and 8 servos.
/// Holds a mutable reference to the PCA9685 driver internally.
pub struct MotorBit<'a, 'b> {
  pca9685: &'b mut Pca9685<'a>,
  servo_configs: [ServoConfig; NUM_SERVOS],
}

impl<'a, 'b> MotorBit<'a, 'b> {
  /// Create MotorBit driver instance
  ///
  /// PCA9685 should already be initialized to 50Hz.
  pub fn new(pca9685: &'b mut Pca9685<'a>) -> Self {
    Self {
      pca9685,
      servo_configs: [ServoConfig::default(); NUM_SERVOS],
    }
  }

  /// Get mutable reference to PCA9685 driver (for direct low-level PWM access)
  pub fn pca9685_mut(&mut self) -> &mut Pca9685<'a> {
    self.pca9685
  }

  // ==================== DC Motor Control ====================

  /// Set DC motor speed
  ///
  /// Exactly matches motorbit.py `DcMotor.speed.setter`:
  /// - speed >= 0: positive_channel = |speed|, negative_channel = 0
  /// - speed < 0:  negative_channel = |speed|, positive_channel = 0
  ///
  /// # Arguments
  /// - `motor`: Motor index (M1=0, M2=1, M3=2, M4=3)
  /// - `speed`: Speed value (-4095 ~ +4095)
  ///
  /// # Panics
  /// If motor > 3 or |speed| > 4095
  pub async fn set_dc_motor_speed(&mut self, motor: u8, speed: i16) {
    debug_assert!(motor <= M4, "motor index out of range (0-3)");
    debug_assert!(
      (-4095..=4095).contains(&speed),
      "speed out of range (-4095 ~ 4095)"
    );

    let (pos_ch, neg_ch) = DC_MOTOR_CHANNELS[motor as usize];

    if speed >= 0 {
      self.pca9685.duty(pos_ch, speed.unsigned_abs()).await;
      self.pca9685.duty(neg_ch, 0).await;
    } else {
      self.pca9685.duty(neg_ch, speed.unsigned_abs()).await;
      self.pca9685.duty(pos_ch, 0).await;
    }

    trace!("DC motor M{}: speed={}", motor + 1, speed);
  }

  /// Stop specified DC motor
  pub async fn stop_dc_motor(&mut self, motor: u8) {
    self.set_dc_motor_speed(motor, 0).await;
  }

  /// Stop all DC motors
  pub async fn stop_all_motors(&mut self) {
    for m in M1..=M4 {
      self.set_dc_motor_speed(m, 0).await;
    }
  }

  // ==================== Servo Control ====================

  /// Set servo configuration
  ///
  /// Corresponds to Servo property setters in motorbit.py:
  /// - max_rotation_angle
  /// - min_pulse_width_us
  /// - max_pulse_width_us
  ///
  /// # Arguments
  /// - `servo_index`: Servo index (0-7, corresponds to S1-S8)
  /// - `config`: Servo configuration
  pub fn set_servo_config(&mut self, servo_index: u8, config: ServoConfig) {
    debug_assert!(
      (servo_index as usize) < NUM_SERVOS,
      "servo index out of range (0-7)"
    );
    self.servo_configs[servo_index as usize] = config;
  }

  /// Get servo configuration
  pub fn servo_config(&self, servo_index: u8) -> &ServoConfig {
    &self.servo_configs[servo_index as usize]
  }

  /// Set servo angle
  ///
  /// Exactly matches motorbit.py `Servo.angle.setter`:
  /// ```text
  /// duty = (min_pulse_us + angle / max_angle * (max_pulse_us - min_pulse_us))
  ///        / (1000000 / frequency_hz) * 4095
  /// ```
  ///
  /// # Arguments
  /// - `servo_index`: Servo index (0-7, corresponds to S1-S8)
  /// - `angle`: Target angle (0 ~ max_rotation_angle)
  ///
  /// # Panics
  /// If angle > max_rotation_angle
  pub async fn set_servo_angle(&mut self, servo_index: u8, angle: u16) {
    debug_assert!(
      (servo_index as usize) < NUM_SERVOS,
      "servo index out of range (0-7)"
    );

    let config = self.servo_configs[servo_index as usize];
    debug_assert!(
      angle <= config.max_rotation_angle,
      "angle out of range (0 ~ max_rotation_angle)"
    );

    let channel = SERVO_CHANNELS[servo_index as usize];
    let freq = self.pca9685.frequency_hz() as u32;

    // pulse_us = min_pulse_us + angle * (max_pulse_us - min_pulse_us) / max_angle
    // period_us = 1_000_000 / freq
    // duty = pulse_us / period_us * 4095
    //      = pulse_us * freq * 4095 / 1_000_000

    let min_us = config.min_pulse_width_us as u32;
    let max_us = config.max_pulse_width_us as u32;
    let max_angle = config.max_rotation_angle as u32;

    // Use integer arithmetic to avoid floating point
    let pulse_us = min_us + (angle as u32) * (max_us - min_us) / max_angle;
    let duty = (pulse_us * freq * PWM_MAX as u32) / 1_000_000;

    self.pca9685.duty(channel, duty as u16).await;

    trace!("Servo S{}: angle={}, duty={}", servo_index + 1, angle, duty);
  }

  /// Set servo raw PWM duty cycle
  ///
  /// Directly control servo channel PWM value, bypassing angle calculation.
  ///
  /// # Arguments
  /// - `servo_index`: Servo index (0-7)
  /// - `duty`: PWM duty cycle (0-4095)
  pub async fn set_servo_duty(&mut self, servo_index: u8, duty: u16) {
    debug_assert!(
      (servo_index as usize) < NUM_SERVOS,
      "servo index out of range (0-7)"
    );
    let channel = SERVO_CHANNELS[servo_index as usize];
    self.pca9685.duty(channel, duty.min(PWM_MAX)).await;
  }
}
