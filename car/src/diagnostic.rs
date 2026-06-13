//! Motor wiring diagnostic mode.
//!
//! Entered by tapping the on-board **A button** (`P0_14`) within a
//! short window after boot. Once active, the firmware drives each
//! Mecanum wheel motor (M1 → M2 → M3 → M4) independently with a fixed
//! PWM duty, first forward then backward, and logs the current motor
//! under test so the operator can visually verify wiring polarity and
//! channel mapping without involving the radio link or the controller.
//!
//! Physical layout reminder (looking down on the chassis from above):
//!
//! ```text
//!   front-left  M1   M2  front-right
//!   rear-left   M3   M4  rear-right
//! ```
//!
//! "Forward" here means the direction reported as positive by
//! [`MotorBit::set_dc_motor_speed`] (i.e. the `pos_ch` PWM line is
//! driven). Whether that physically corresponds to the wheel rolling
//! the chassis forward depends on motor wiring polarity — that is
//! exactly what this routine is meant to expose.
//!
//! The loop runs forever; power-cycle or reset the board to leave
//! diagnostic mode.

use defmt::info;
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::{Peri, peripherals};
use embassy_time::{Duration, Timer};

use crate::motor::MotorDriver;
use crate::motorbit::{self, MotorBit};
use crate::pca9685::Pca9685;

/// PWM duty applied to the motor under test. PCA9685 PWM range is
/// 0..=4095; ~50% gives a clearly visible spin without being violent
/// on a bench.
const TEST_SPEED: i16 = 2048;

/// How long each direction is held (ms).
const RUN_MS: u64 = 1000;

/// Pause between phases / motors (ms). Long enough that the human
/// observer can clearly distinguish each step.
const PAUSE_MS: u64 = 500;

/// Inter-cycle gap (ms).
const CYCLE_GAP_MS: u64 = 2000;

/// How long after boot to keep watching for an A-button press (ms).
const REQUEST_WINDOW_MS: u64 = 2000;

/// Polling interval inside the request window (ms). 50 ms × 40 ticks
/// = 2 s window, with comfortably crisp button responsiveness.
const POLL_INTERVAL_MS: u64 = 50;

/// Watch the on-board **A button** (active-low) for a short window
/// after boot and report whether the operator pressed it. Should be
/// called early in `main`, *before* the pin is consumed by anything
/// else.
///
/// micro:bit v2 wires the A button to `P0_14` with an external pull-up
/// already on the board; we still enable the internal pull-up as a
/// belt-and-braces measure. Returns as soon as a press is detected,
/// so a quick tap is enough — there is no need to hold the button or
/// coordinate with the reset key.
pub async fn is_diagnostic_requested(pin: Peri<'static, peripherals::P0_14>) -> bool {
  let button = Input::new(pin, Pull::Up);
  let ticks = REQUEST_WINDOW_MS / POLL_INTERVAL_MS;

  info!(
    "Press A within {}ms to enter diagnostic mode",
    REQUEST_WINDOW_MS
  );

  for _ in 0..ticks {
    if button.is_low() {
      info!("Diagnostic mode requested (A button pressed during boot window)");
      return true;
    }
    Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
  }

  false
}

/// Run the motor wiring diagnostic loop forever.
///
/// `motor_driver` must already be initialised — we only need its
/// underlying I2C peripheral so we can talk to PCA9685 / MotorBit
/// directly, bypassing the Mecanum kinematics.
pub async fn run(motor_driver: &mut MotorDriver) -> ! {
  info!("=== DIAGNOSTIC MODE ENTERED ===");
  info!("Cycling M1 -> M2 -> M3 -> M4, forward then backward, repeatedly.");
  info!("Power-cycle the board to exit.");

  // Map motor index -> human-readable position label. Order matches
  // the chassis layout the operator sees from above.
  const MOTORS: [(u8, &str); 4] = [
    (motorbit::M1, "M1 front-left"),
    (motorbit::M2, "M2 front-right"),
    (motorbit::M3, "M3 rear-left"),
    (motorbit::M4, "M4 rear-right"),
  ];

  let mut cycle: u32 = 0;

  loop {
    cycle += 1;
    info!("--- diagnostic cycle #{} ---", cycle);

    for (idx, label) in MOTORS.iter() {
      run_single_motor(motor_driver, *idx, label).await;
      pause(PAUSE_MS).await;
    }

    info!(
      "--- cycle #{} done, restarting in {}ms ---",
      cycle, CYCLE_GAP_MS
    );
    pause(CYCLE_GAP_MS).await;
  }
}

/// Drive a single motor forward then backward, then stop. All other
/// motors are held at 0 throughout.
async fn run_single_motor(motor_driver: &mut MotorDriver, motor: u8, label: &str) {
  // Forward.
  info!(
    "Testing {}: FORWARD (+{}) for {}ms",
    label, TEST_SPEED, RUN_MS
  );
  set_only(motor_driver, motor, TEST_SPEED).await;
  Timer::after(Duration::from_millis(RUN_MS)).await;

  // Brief stop so the operator can clearly see the direction flip.
  stop_all(motor_driver).await;
  Timer::after(Duration::from_millis(PAUSE_MS)).await;

  // Backward.
  info!(
    "Testing {}: BACKWARD (-{}) for {}ms",
    label, TEST_SPEED, RUN_MS
  );
  set_only(motor_driver, motor, -TEST_SPEED).await;
  Timer::after(Duration::from_millis(RUN_MS)).await;

  // Stop before moving on to the next motor.
  stop_all(motor_driver).await;
}

/// Set exactly one motor to `speed`, force every other motor to 0.
async fn set_only(motor_driver: &mut MotorDriver, motor: u8, speed: i16) {
  let mut pca = Pca9685::resume(motor_driver.twim_mut());
  let mut mb = MotorBit::new(&mut pca);
  for m in motorbit::M1..=motorbit::M4 {
    let target = if m == motor { speed } else { 0 };
    mb.set_dc_motor_speed(m, target).await;
  }
}

/// Stop every motor.
async fn stop_all(motor_driver: &mut MotorDriver) {
  let mut pca = Pca9685::resume(motor_driver.twim_mut());
  let mut mb = MotorBit::new(&mut pca);
  mb.stop_all_motors().await;
}

/// Sleep helper that reads slightly nicer than the raw timer call.
async fn pause(ms: u64) {
  Timer::after(Duration::from_millis(ms)).await;
}
