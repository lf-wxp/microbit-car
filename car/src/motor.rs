//! Motor driver module - Mecanum wheel motion control
//!
//! This module integrates PCA9685 low-level driver and MotorBit board-level abstraction,
//! providing high-level motion control interface for Mecanum wheel chassis.
//!
//! # Architecture
//!
//! ```text
//! MotorDriver (kinematics + I2C init)
//!     └── MotorBit (motor/servo control)
//!           └── Pca9685 (low-level PWM)
//! ```
//!
//! # Mecanum Wheel Inverse Kinematics
//!
//! ```text
//!   motor_fl (M1) = vx - vy - k * omega
//!   motor_fr (M2) = vx + vy + k * omega
//!   motor_rl (M3) = vx + vy - k * omega
//!   motor_rr (M4) = vx - vy + k * omega
//! ```
//!
//! Where vx/vy/omega range [-100, 100], output mapped to motor speed [-4095, 4095].

use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{Peri, bind_interrupts, peripherals};
use static_cell::StaticCell;

use defmt::{info, trace};
use protocol::MotionPayload;

use crate::motorbit::{self, MotorBit};
use crate::pca9685::Pca9685;

/// Mecanum wheel kinematics geometry factor K
/// Used to adjust rotation sensitivity, range 0.5~1.5, tune based on chassis dimensions
const GEOMETRY_FACTOR_K: i32 = 1;

/// Speed percentage to PWM value mapping scale (4095 / 100)
const SPEED_TO_PWM_SCALE: i32 = 40; // approx 4095/100 ≈ 40.95, use 40

// --- I2C Interrupt Binding ---

bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// I2C TX RAM buffer size
const TX_BUF_SIZE: usize = 16;

/// Motor driver struct
///
/// Encapsulates I2C communication, PCA9685 driver, MotorBit board-level control,
/// and Mecanum wheel inverse kinematics.
/// Also exposes low-level MotorBit interface for direct access to servos and other peripherals.
pub struct MotorDriver {
  twim: Twim<'static>,
}

impl MotorDriver {
  /// Initialize motor driver
  ///
  /// Configures PCA9685 with 50Hz PWM output frequency.
  /// micro:bit v2 edge connector I2C pins: P19(SCL)=P0.26, P20(SDA)=P1.00
  pub async fn new(
    twispi0: Peri<'static, peripherals::TWISPI0>,
    scl: Peri<'static, peripherals::P0_26>,
    sda: Peri<'static, peripherals::P1_00>,
  ) -> Self {
    // Allocate TX RAM buffer (nRF DMA requires data in RAM)
    static TX_BUF: StaticCell<[u8; TX_BUF_SIZE]> = StaticCell::new();
    let tx_buf = TX_BUF.init([0u8; TX_BUF_SIZE]);

    let config = twim::Config::default();
    let mut twim = Twim::new(twispi0, Irqs, sda, scl, config, tx_buf);

    // Initialize PCA9685 (50Hz, all channels cleared)
    {
      let mut pca = Pca9685::new(&mut twim).await;
      // Ensure all motors are stopped
      let mut mb = MotorBit::new(&mut pca);
      mb.stop_all_motors().await;
    }

    info!("MotorDriver initialized (PCA9685 @ 50Hz, all motors stopped)");
    Self { twim }
  }

  /// Apply MotionPayload to Mecanum wheel motors
  ///
  /// Performs inverse kinematics calculation, converts (vx, vy, omega) to 4 motor speed values,
  /// and outputs PWM signals via PCA9685.
  pub async fn apply_motion(&mut self, motion: &MotionPayload) {
    // Fast path: all zeros means stop immediately
    if motion.vx == 0 && motion.vy == 0 && motion.omega == 0 {
      self.stop_all().await;
      return;
    }

    // Inverse kinematics calculation (input range [-100, 100])
    let vx = motion.vx as i32;
    let vy = motion.vy as i32;
    let omega = motion.omega as i32;

    let raw_fl = vx - vy - GEOMETRY_FACTOR_K * omega;
    let raw_fr = vx + vy + GEOMETRY_FACTOR_K * omega;
    let raw_rl = vx + vy - GEOMETRY_FACTOR_K * omega;
    let raw_rr = vx - vy + GEOMETRY_FACTOR_K * omega;

    // Normalization: if any motor value exceeds [-100, 100], scale proportionally
    let max_abs = raw_fl
      .abs()
      .max(raw_fr.abs())
      .max(raw_rl.abs())
      .max(raw_rr.abs());

    let (fl, fr, rl, rr) = if max_abs > 100 {
      (
        (raw_fl * 100 / max_abs) as i16,
        (raw_fr * 100 / max_abs) as i16,
        (raw_rl * 100 / max_abs) as i16,
        (raw_rr * 100 / max_abs) as i16,
      )
    } else {
      (raw_fl as i16, raw_fr as i16, raw_rl as i16, raw_rr as i16)
    };

    trace!("Motor speeds: FL={}, FR={}, RL={}, RR={}", fl, fr, rl, rr);

    // Map percentage [-100, 100] to motor speed [-4095, 4095]
    let fl_pwm = percent_to_motor_speed(fl);
    let fr_pwm = percent_to_motor_speed(fr);
    let rl_pwm = percent_to_motor_speed(rl);
    let rr_pwm = percent_to_motor_speed(rr);

    // Set motor speed via MotorBit
    // Note: creating temporary Pca9685/MotorBit here is zero-cost (just reference wrappers)
    let mut pca = Pca9685::resume(&mut self.twim);
    let mut mb = MotorBit::new(&mut pca);
    mb.set_dc_motor_speed(motorbit::M1, fl_pwm).await;
    mb.set_dc_motor_speed(motorbit::M2, fr_pwm).await;
    mb.set_dc_motor_speed(motorbit::M3, rl_pwm).await;
    mb.set_dc_motor_speed(motorbit::M4, rr_pwm).await;
  }

  /// Stop all motors
  pub async fn stop_all(&mut self) {
    let mut pca = Pca9685::resume(&mut self.twim);
    let mut mb = MotorBit::new(&mut pca);
    mb.stop_all_motors().await;
    trace!("All motors stopped");
  }

  /// Get low-level TWIM reference for direct PCA9685/MotorBit access
  ///
  /// Usage example:
  /// ```ignore
  /// let mut pca = Pca9685::resume(motor_driver.twim_mut());
  /// let mut mb = MotorBit::new(&mut pca);
  /// mb.set_servo_angle(0, 90).await; // S1 rotate to 90°
  /// ```
  #[allow(dead_code)]
  pub fn twim_mut(&mut self) -> &mut Twim<'static> {
    &mut self.twim
  }
}

/// Map percentage speed [-100, 100] to motor speed [-4095, 4095]
fn percent_to_motor_speed(percent: i16) -> i16 {
  (percent as i32 * SPEED_TO_PWM_SCALE).clamp(-4095, 4095) as i16
}
