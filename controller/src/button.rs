//! Button input module for the controller firmware.
//!
//! Reads two edge-connector buttons used to control the rotational
//! velocity (omega) of the Mecanum-wheel car:
//!
//! - C button: P16 (P0.09) -> rotate counter-clockwise (omega negative)
//! - D button: P15 (P0.13) -> rotate clockwise         (omega positive)
//!
//! Buttons are wired active-low with the nRF52833 internal pull-up
//! enabled, so a pressed button reads `Level::Low`.
//!
//! A small per-tick step (`OMEGA_STEP`) is applied so that omega ramps
//! smoothly between 0 and the target value instead of snapping, which
//! gives a softer rotation feel and reduces motor stress.
//!
//! When both C and D are pressed simultaneously they cancel out and the
//! target omega becomes 0 (treated as an "anti-spin brake").

use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use defmt::{info, trace};

use crate::mode;

/// Maximum absolute omega magnitude produced by the buttons.
const OMEGA_MAX: i8 = 100;

/// Per-tick ramp step. With a 20 ms tick this means the omega reaches
/// the maximum value in roughly `OMEGA_MAX / OMEGA_STEP * 20 ms` =
/// 5 * 20 ms = 100 ms, which feels responsive without being abrupt.
const OMEGA_STEP: i8 = 20;

/// Button polling interval (ms). Matches the joystick sampling rate so
/// the two control sources update at the same cadence.
const POLL_INTERVAL_MS: u64 = 20;

/// Channel used to publish the current omega value to the fusion task.
/// Capacity 4 absorbs short bursts without blocking the producer.
pub static OMEGA_CHANNEL: Channel<CriticalSectionRawMutex, i8, 4> = Channel::new();

/// Mode-switch debounce window (ms): a press only counts if the line
/// has been stable in the "released" state for at least this long.
const MODE_DEBOUNCE_MS: u64 = 50;

/// Polling interval for the mode-switch button (ms).
const MODE_POLL_INTERVAL_MS: u64 = 10;

/// Configure the controller-board A button as a debounced mode-toggle
/// input.
///
/// The shield's A button is wired to edge-connector P13 (`P0.17`).
/// The extension board does not include a hardware pull-up on this
/// line, so we enable the nRF52833 internal pull-up. Pressed = low.
pub fn init_mode_switch(pin: Peri<'static, peripherals::P0_17>) -> Input<'static> {
  let input = Input::new(pin, Pull::Up);
  info!("Mode-switch button initialized (A=P13/P0.17)");
  input
}

/// Inputs owned by the button task.
pub struct ButtonInputs {
  /// C button (P16 / P0.09) -> CCW rotation
  pub ccw: Input<'static>,
  /// D button (P15 / P0.13) -> CW rotation
  pub cw: Input<'static>,
}

/// Configure the C/D edge-connector pins as pulled-up digital inputs.
pub fn init(
  pin_ccw: Peri<'static, peripherals::P0_09>, // P16 / C button
  pin_cw: Peri<'static, peripherals::P0_13>,  // P15 / D button
) -> ButtonInputs {
  let ccw = Input::new(pin_ccw, Pull::Up);
  let cw = Input::new(pin_cw, Pull::Up);
  info!("Buttons initialized (C=P16/P0.09 CCW, D=P15/P0.13 CW)");
  ButtonInputs { ccw, cw }
}

/// Compute the new omega value, ramping `current` toward `target`
/// by at most `OMEGA_STEP` per tick.
fn step_toward(current: i8, target: i8) -> i8 {
  if current == target {
    return current;
  }
  let diff = target as i16 - current as i16;
  let step = OMEGA_STEP as i16;
  let next = if diff.abs() <= step {
    target as i16
  } else if diff > 0 {
    current as i16 + step
  } else {
    current as i16 - step
  };
  next.clamp(-(OMEGA_MAX as i16), OMEGA_MAX as i16) as i8
}

/// Determine the omega target from the current button states.
///
/// Active-low: a pressed button drives the line to `Level::Low`, so we
/// invert when reading.
fn target_from_buttons(ccw_pressed: bool, cw_pressed: bool) -> i8 {
  match (ccw_pressed, cw_pressed) {
    (true, true) => 0,
    (true, false) => -OMEGA_MAX,
    (false, true) => OMEGA_MAX,
    (false, false) => 0,
  }
}

/// Button polling task.
///
/// Runs at 50 Hz, computes the desired omega from the two buttons and
/// publishes it to `OMEGA_CHANNEL` whenever the value changes.
#[embassy_executor::task]
pub async fn button_task(buttons: ButtonInputs) {
  info!("Button task started (50 Hz polling)");

  let ButtonInputs { ccw, cw } = buttons;
  let mut current: i8 = 0;
  let mut last_sent: i8 = 0;

  // Publish the initial state so the fusion task starts from a known value.
  OMEGA_CHANNEL.send(0).await;

  loop {
    let ccw_pressed = ccw.is_low();
    let cw_pressed = cw.is_low();

    let target = target_from_buttons(ccw_pressed, cw_pressed);
    current = step_toward(current, target);

    if current != last_sent {
      trace!(
        "Omega update: target={}, current={}, ccw={}, cw={}",
        target, current, ccw_pressed, cw_pressed
      );
      OMEGA_CHANNEL.send(current).await;
      last_sent = current;
    }

    embassy_time::Timer::after(embassy_time::Duration::from_millis(POLL_INTERVAL_MS)).await;
  }
}

/// Mode-switch button task.
///
/// Detects a falling edge on Button A (with simple debouncing) and
/// toggles between [`mode::InputMode::Joystick`] and
/// [`mode::InputMode::Tilt`] on each press.
#[embassy_executor::task]
pub async fn mode_switch_task(button: Input<'static>) {
  info!("Mode-switch task started");

  let mut prev_pressed = button.is_low();

  loop {
    embassy_time::Timer::after(embassy_time::Duration::from_millis(MODE_POLL_INTERVAL_MS)).await;
    let pressed = button.is_low();

    // Falling edge: was released, now pressed.
    if !prev_pressed && pressed {
      // Cheap software debounce: confirm the press is still held after
      // the debounce window before we commit to a mode toggle.
      embassy_time::Timer::after(embassy_time::Duration::from_millis(MODE_DEBOUNCE_MS)).await;
      if button.is_low() {
        let new_mode = mode::toggle();
        info!("Mode switched to {:?}", new_mode);
      }
      prev_pressed = true;
    } else {
      prev_pressed = pressed;
    }
  }
}
