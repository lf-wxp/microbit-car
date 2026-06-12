#![no_std]
#![no_main]

mod button;
mod display;
mod joystick;
mod joystick_right;
mod mode;
mod radio;
mod signal;
mod tilt;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_time::{Duration, Timer};

use defmt::info;
use protocol::MotionPayload;

use crate::mode::{InputMode, MODE_CHANGED};

/// Saturating sum of the two omega contributors, clamped to the
/// protocol's `[-100, 100]` range.
///
/// Stick = continuous primary control, buttons = fine-grained trim.
/// Adding (rather than taking max) preserves directional intent: a
/// CCW button press still dampens a CW stick deflection.
fn merge_omega(stick: i8, button: i8) -> i8 {
  (stick as i16 + button as i16).clamp(-100, 100) as i8
}

/// Main entry point for the controller firmware
/// Runs on micro:bit v2 (nRF52833)
#[embassy_executor::main]
async fn main(spawner: Spawner) {
  let p = embassy_nrf::init(Default::default());

  info!("Controller firmware started");

  // Initialize the on-board 5x5 LED matrix and spawn the display task.
  // Pin assignments come from the official micro:bit v2 schematic:
  //   ROW1=P0.21, ROW2=P0.22, ROW3=P0.15, ROW4=P0.24, ROW5=P0.19
  //   COL1=P0.28, COL2=P0.11, COL3=P0.31, COL4=P1.05, COL5=P0.30
  let matrix = display::init(
    p.P0_21, p.P0_22, p.P0_15, p.P0_24, p.P0_19, p.P0_28, p.P0_11, p.P0_31, p.P1_05, p.P0_30,
  );
  spawner.spawn(display::display_task(matrix).unwrap());

  // Initialize radio and spawn radio task
  let radio = radio::init(p.RADIO);
  spawner.spawn(radio::radio_task(radio).unwrap());

  // Initialize joystick SAADC and spawn joystick task
  // micro:bit edge connector: P1 = P0.03 (AIN2), P2 = P0.04 (AIN3)
  let saadc = joystick::init(p.SAADC, p.P0_03, p.P0_04);
  spawner.spawn(joystick::joystick_task(saadc).unwrap());

  // Spawn the right-stick task. By default this runs against a mock
  // sample source (always centered) so the fusion pipeline is exercised
  // end-to-end even without the hardware. Enable the `right-stick-hw`
  // Cargo feature (and finish the constructor in
  // `joystick_right.rs` / wire the call below) when the second stick
  // is on the board.
  #[cfg(not(feature = "right-stick-hw"))]
  let right_stick = joystick_right::RightStickSource::new();
  #[cfg(feature = "right-stick-hw")]
  let right_stick = {
    use embassy_nrf::Peri;
    use embassy_nrf::peripherals;
    // TODO: replace these `todo!()`s with the two AIN-capable pins you
    // wire the second stick to, e.g. `p.P0_02` (AIN0) and `p.P0_05`
    // (AIN3). The dummy type annotations only exist so the call
    // type-checks; until real pins are picked, enabling this feature
    // will panic at runtime on first use.
    let pin_x: Peri<'static, peripherals::P0_02> = todo!("pick the X-axis pin for the right stick");
    let pin_y: Peri<'static, peripherals::P0_05> = todo!("pick the Y-axis pin for the right stick");
    joystick_right::RightStickSource::new(p.SAADC, pin_x, pin_y)
  };
  spawner.spawn(joystick_right::joystick_right_task(right_stick).unwrap());
  // Initialize on-board I2C (TWISPI0) for the LSM303AGR accelerometer
  // and spawn the tilt task. Internal I2C: SCL=P0.08, SDA=P0.16.
  let twim = tilt::init(p.TWISPI0, p.P0_16, p.P0_08);
  spawner.spawn(tilt::tilt_task(twim).unwrap());

  // Initialize C/D buttons (omega control) and spawn button task.
  // C button = P16 (P0.09) -> CCW, D button = P15 (P0.13) -> CW.
  let buttons = button::init(p.P0_09, p.P0_13);
  spawner.spawn(button::button_task(buttons).unwrap());

  // Initialize the extension-board A button (edge-connector P13 = P0.17)
  // as the input-mode toggle. B (P14/P0.01) is reserved for future use.
  let mode_button = button::init_mode_switch(p.P0_17);
  spawner.spawn(button::mode_switch_task(mode_button).unwrap());

  info!("All input tasks spawned, entering fusion loop (default mode = Joystick)");

  // Fusion loop: combine the active velocity source (joystick or tilt)
  // with the two rotational sources (right stick + C/D buttons) into a
  // single MotionPayload and forward it to the radio task. We re-emit
  // periodically so the car always has a recent command even when the
  // user holds still.
  let mut last_vx: i8 = 0;
  let mut last_vy: i8 = 0;
  let mut last_button_omega: i8 = 0;
  let mut last_stick_omega: i8 = 0;

  loop {
    // Outer arbitration: velocity sources + button omega + (mode/timer/right-stick).
    // The fourth arm carries the lower-rate / event-driven signals so
    // we keep the existing 4-way `select` shape and nest a small
    // `select` inside it for the additional channels.
    match select4(
      joystick::JOYSTICK_MOTION_CHANNEL.receive(),
      tilt::TILT_MOTION_CHANNEL.receive(),
      button::OMEGA_CHANNEL.receive(),
      select(
        joystick_right::STICK_OMEGA_CHANNEL.receive(),
        select(
          MODE_CHANGED.wait(),
          Timer::after(Duration::from_millis(200)),
        ),
      ),
    )
    .await
    {
      Either4::First(motion) => {
        last_vx = motion.vx;
        last_vy = motion.vy;
      }
      Either4::Second(motion) => {
        last_vx = motion.vx;
        last_vy = motion.vy;
      }
      Either4::Third(omega) => {
        last_button_omega = omega;
      }
      Either4::Fourth(inner) => match inner {
        Either::First(stick_omega) => {
          last_stick_omega = stick_omega;
        }
        Either::Second(mode_or_tick) => match mode_or_tick {
          Either::First(new_mode) => {
            // On mode switch, zero translational velocity immediately so a
            // stale tilt reading can't keep the car rolling.
            last_vx = 0;
            last_vy = 0;
            info!(
              "Fusion: mode -> {:?} (vx/vy zeroed, omega preserved)",
              new_mode
            );
            // Suppress unused-variable warning when defmt is filtered out.
            let _ = InputMode::Joystick;
          }
          Either::Second(_) => {
            // Periodic re-send tick falls through to broadcast below.
          }
        },
      },
    }

    let combined = MotionPayload {
      vx: last_vx,
      vy: last_vy,
      omega: merge_omega(last_stick_omega, last_button_omega),
    };
    radio::MOTION_TX_CHANNEL.send(combined).await;

    // Mirror the command to the LED matrix. `try_send` keeps the
    // fusion loop non-blocking even if the display task hasn't drained
    // the previous value yet (the channel has capacity 1, so the
    // freshest frame always wins).
    let _ = display::DISPLAY_CHANNEL.try_send(combined);

    // Tiny pacing delay keeps the loop from spinning when several
    // sources fire in the same tick.
    Timer::after(Duration::from_millis(1)).await;
  }
}
