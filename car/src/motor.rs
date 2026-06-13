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
//! Body-frame convention used by the controller protocol:
//! - `vx > 0` → forward
//! - `vy > 0` → strafe **right**
//! - `omega > 0` → rotate clockwise (viewed from above)
//!
//! With the chassis wheel layout
//!
//! ```text
//!   M1 (front-left)    M2 (front-right)
//!   M3 (rear-left)     M4 (rear-right)
//! ```
//!
//! the inverse kinematics become:
//!
//! ```text
//!   motor_fl (M1) = vx + vy - k * omega
//!   motor_fr (M2) = vx - vy + k * omega
//!   motor_rl (M3) = vx - vy - k * omega
//!   motor_rr (M4) = vx + vy + k * omega
//! ```
//!
//! Quick sanity checks:
//! - Pure right strafe (`vy = +1`): FL = +1, FR = -1, RL = -1, RR = +1
//!   → left wheels roll forward, right wheels roll backward, which
//!   pushes the chassis to the right. ✓
//! - Pure forward (`vx = +1`): all four = +1. ✓
//! - Pure CW rotation (`omega = +1`): FL = -k, FR = +k, RL = -k, RR = +k
//!   → right side forward, left side backward → CW. ✓
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

/// Sentinel value for "no speed has ever been written for this motor".
///
/// Any real motor speed is in `[-4095, 4095]`, so `i16::MIN` is
/// guaranteed to differ from the first command, forcing an actual
/// I²C write on the first `apply_motion` / `stop_all` call after boot.
const SPEED_CACHE_UNINIT: i16 = i16::MIN;

/// Motor driver struct
///
/// Encapsulates I2C communication, PCA9685 driver, MotorBit board-level control,
/// and Mecanum wheel inverse kinematics.
/// Also exposes low-level MotorBit interface for direct access to servos and other peripherals.
///
/// # Idempotent writes
///
/// `last_speeds` caches the most recently written speed for each of the four
/// motors (M1..M4). `apply_motion` and `stop_all` skip the I²C transaction
/// for any motor whose target speed has not changed. This is critical for
/// audible noise: the controller streams motion packets at ~50 Hz even when
/// the joystick is centred (vx=vy=ω=0), and re-writing the PCA9685 channel
/// registers at that rate makes the I²C lines toggle constantly, which the
/// MotorBit board's power filtering picks up as a faint kHz-range hum even
/// though the PWM outputs themselves are in FULL_OFF.
pub struct MotorDriver {
  twim: Twim<'static>,
  /// Cached most-recently-written speed for [M1, M2, M3, M4].
  /// Initialised to [`SPEED_CACHE_UNINIT`] so the first command always writes.
  last_speeds: [i16; 4],
}

impl MotorDriver {
  /// Initialize motor driver
  ///
  /// Configures PCA9685 with the default PWM carrier frequency for DC
  /// motors (see `pca9685::DEFAULT_PWM_FREQ_HZ`).
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

    // Initialize PCA9685 (default high-frequency carrier, all channels cleared)
    {
      let mut pca = Pca9685::new(&mut twim).await;
      // Ensure all motors are stopped
      let mut mb = MotorBit::new(&mut pca);
      mb.stop_all_motors().await;
    }

    info!(
      "MotorDriver initialized (PCA9685 @ {}Hz, all motors stopped)",
      crate::pca9685::DEFAULT_PWM_FREQ_HZ
    );
    Self {
      twim,
      last_speeds: [SPEED_CACHE_UNINIT; 4],
    }
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

    // Inverse kinematics calculation (input range [-100, 100]).
    // See module-level docs for the convention and derivation.
    let vx = motion.vx as i32;
    let vy = motion.vy as i32;
    let omega = motion.omega as i32;

    let raw_fl = vx + vy - GEOMETRY_FACTOR_K * omega;
    let raw_fr = vx - vy + GEOMETRY_FACTOR_K * omega;
    let raw_rl = vx - vy - GEOMETRY_FACTOR_K * omega;
    let raw_rr = vx + vy + GEOMETRY_FACTOR_K * omega;

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
    let targets = [
      percent_to_motor_speed(fl),
      percent_to_motor_speed(fr),
      percent_to_motor_speed(rl),
      percent_to_motor_speed(rr),
    ];

    self.write_motor_speeds(targets).await;
  }

  /// Stop all motors
  pub async fn stop_all(&mut self) {
    if self.last_speeds == [0; 4] {
      // Already stopped — skip the bus traffic entirely.
      return;
    }

    {
      let mut pca = Pca9685::resume(&mut self.twim);
      let mut mb = MotorBit::new(&mut pca);
      mb.stop_all_motors().await;
    }
    self.last_speeds = [0; 4];
    trace!("All motors stopped");
  }

  /// Write target speeds to PCA9685, skipping any motor whose target
  /// is exactly equal to the cached previous value.
  ///
  /// This is the single place where I²C writes are emitted, so the
  /// idempotency check covers both `apply_motion` and `stop_all` and
  /// keeps the bus silent when the chassis is commanded to the same
  /// state repeatedly (typical when the controller is connected and
  /// the joystick is centred → 50 Hz of identical "stop" packets).
  async fn write_motor_speeds(&mut self, targets: [i16; 4]) {
    // Channel order is M1..M4, matching `motorbit::M1..M4`.
    const MOTOR_IDS: [u8; 4] = [motorbit::M1, motorbit::M2, motorbit::M3, motorbit::M4];

    // Fast path: if every motor already matches the cache, don't
    // even bother instantiating the Pca9685/MotorBit wrappers —
    // keeps the common "still stopped / still cruising" hot path
    // branch-only, no I²C activity.
    if targets == self.last_speeds {
      return;
    }

    let mut pca = Pca9685::resume(&mut self.twim);
    let mut mb = MotorBit::new(&mut pca);

    for ((&target, cached), &motor_id) in targets
      .iter()
      .zip(self.last_speeds.iter_mut())
      .zip(MOTOR_IDS.iter())
    {
      if target == *cached {
        continue;
      }
      mb.set_dc_motor_speed(motor_id, target).await;
      *cached = target;
    }
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
