#![no_std]
#![no_main]

mod diagnostic;
mod display;
mod light;
mod motor;
mod motorbit;
mod pca9685;
mod radio;
mod rgb;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_nrf::config::{Config, HfclkSource};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Instant, Timer};


use defmt::{info, warn};

/// Maximum time (ms) we tolerate without any inbound radio packet
/// before forcing all motors to stop.
///
/// This is a **secondary** failsafe that covers the case where the
/// motion channel is somehow wedged (e.g. the radio RX task crashes
/// or the channel saturates). The **primary** link-loss detection is
/// the per-motion expiry timer (`MOTION_EXPIRY_MS`) inside the main
/// loop, which fires within ~60 ms of the controller going away.
///
/// We keep this global watchdog at 200 ms as a belt-and-braces
/// measure — it should never fire during normal operation because
/// the faster motion-expiry path already caught the disconnection.
const FAILSAFE_TIMEOUT_MS: u32 = 200;

/// Failsafe watchdog tick. We can't simply `select` on the motion
/// channel forever because a wedged controller stops sending entirely;
/// instead we wake every `POLL_TICK_MS` to compare `now` against the
/// last RX timestamp.
const POLL_TICK_MS: u64 = 40;

/// Motion command expiry timeout (ms).
///
/// The controller streams motion packets at ~50 Hz (one every 20 ms).
/// If more than this many milliseconds pass without a fresh motion
/// command, we assume the controller has gone away and immediately
/// stop all motors.
///
/// 60 ms ≈ 3 missed motion frames, which is generous enough to
/// tolerate a single dropped packet but fast enough that the
/// worst-case "ghost roll" at full speed is only a few centimetres
/// — well below the ~250 ms perceptible drift of earlier versions.
///
/// This is the **primary** link-loss detection mechanism. It is much
/// faster than the global `FAILSAFE_TIMEOUT_MS` watchdog because it
/// is directly coupled to the motion stream cadence rather than the
/// slower heartbeat channel.
const MOTION_EXPIRY_MS: u64 = 60;

/// Main entry point for the car firmware
/// Runs on micro:bit v2 (nRF52833)
#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // The nRF52833 RADIO peripheral (BLE / IEEE 802.15.4) requires the
  // 32 MHz external crystal as its high-frequency clock; the internal
  // RC oscillator cannot drive the RF PLL. micro:bit v2 has the HFXO
  // populated, but `embassy-nrf` defaults to `Internal` to support
  // hobby boards without an external crystal, so we have to opt in
  // explicitly here. Without this, `try_send` happily reports success
  // while the modulator outputs garbage off-frequency, and `receive`
  // never sees a framestart.
  let mut config = Config::default();
  config.hfclk_source = HfclkSource::ExternalXtal;
  let p = embassy_nrf::init(config);

  // MotorBit V1/V2 wires its on-board passive buzzer to micro:bit P0
  // (= nRF52833 P0_02) via a slide switch. When the switch is ON, P0
  // electrically reaches the buzzer; an undriven (floating) pin would
  // pick up crosstalk from neighbouring traces and chirp continuously.
  // Holding it Low silences any accidental drive on this line.
  //
  // NOTE: The continuous "buzz" we used to hear at boot was *not*
  // coming from this pin — it was the H-bridge / motor coil resonating
  // at the PCA9685's old 50 Hz PWM carrier. That is now fixed in
  // `pca9685.rs` by raising the carrier to ~1.5 kHz. We still pin
  // P0_02 Low here as a belt-and-braces measure so the buzzer stays
  // quiet regardless of the slide-switch position.
  //
  // The binding *must* be kept alive (note the leading `_` keeps the
  // value, only suppressing the unused-variable warning); a bare `_`
  // would drop it immediately and free the pin.
  let _buzzer_silence = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);

  // Initialize the on-board 5x5 LED matrix and spawn the display task.
  // Pin assignments come from the official micro:bit v2 schematic:
  //   ROW1=P0.21, ROW2=P0.22, ROW3=P0.15, ROW4=P0.24, ROW5=P0.19
  //   COL1=P0.28, COL2=P0.11, COL3=P0.31, COL4=P1.05, COL5=P0.30
  let matrix = display::init(
    p.P0_21, p.P0_22, p.P0_15, p.P0_24, p.P0_19, p.P0_28, p.P0_11, p.P0_31, p.P1_05, p.P0_30,
  );
  spawner.spawn(display::display_task(matrix).unwrap());

  // Initialize the 4-LED WS2812 RGB strip on edge-connector P16 (P1_02)
  // using PWM0 + EasyDMA for reliable hardware-driven timing.
  // Start with everything off; the main loop will drive the colors
  // based on motion state below.
  let mut rgb_strip = rgb::RgbStrip::new(p.PWM0, p.P1_02);
  rgb_strip.clear();
  rgb_strip.show().await;

  info!("Car firmware started");

  // Sample the on-board A button (P0_14) *before* anything else so the
  // operator can opt into diagnostic mode at boot. Pressed = low.
  let diagnostic_requested = diagnostic::is_diagnostic_requested(p.P0_14).await;

  if diagnostic_requested {
    // Diagnostic path: skip radio init entirely so it can't interfere
    // with the controlled motor sweep. `diagnostic::run` is divergent
    // (`-> !`), so control never returns from this branch.
    let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;
    // Light up the matrix to indicate diagnostic mode is active.
    let _ = display::DISPLAY_CHANNEL.try_send(protocol::MotionPayload::forward(100));
    // Solid cyan on the RGB strip is an unmistakable "diagnostic mode"
    // marker that doesn't depend on the LED matrix being visible.
    rgb_strip.set_all(rgb::Color::CYAN);
    rgb_strip.show().await;
    diagnostic::run(&mut motor_driver).await;
  }

  // Initialize radio and spawn RX task
  let radio = radio::init(p.RADIO);
  spawner.spawn(radio::radio_rx_task(radio).unwrap());

  info!("Radio RX task spawned");

  // Initialize motor driver (PCA9685 via I2C)
  // micro:bit v2 edge connector: P19(SCL)=P0.26, P20(SDA)=P1.00
  let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;

  info!("Motor driver initialized, entering main loop");

  // Initialize the light

  let mut light = light::init(p.P0_17, p.P0_13);

  // Tracks whether failsafe has already forced a stop, so we don't
  // hammer the I2C bus restating "all motors off" every tick.
  // Reset to `false` on every successful motion command.
  let mut failsafe_engaged = false;

  // Whether we have received at least one motion command since boot.
  // Before the first motion the expiry timer should not fire (there
  // is no "last command" to expire).
  let mut motion_received = false;

  // Main loop: process motion commands from radio and drive motors.
  //
  // Two-layer link-loss protection:
  //   1. **Motion expiry** (fast, ~60 ms): every motion packet starts
  //      a short timer; if it expires without a new packet the motors
  //      are stopped immediately.
  //   2. **Global watchdog** (slower, ~200 ms): polls `last_rx_millis()`
  //      and catches cases where the motion channel itself is stuck.
  loop {
    // Build the expiry timer future. Before the first motion command
    // we use an "infinite" wait so the timer arm never wins the
    // `select3` — we don't want to stop motors that were never
    // started.
    let expiry_fut = if motion_received && !failsafe_engaged {
      Timer::after(Duration::from_millis(MOTION_EXPIRY_MS))
    } else {
      Timer::after(Duration::from_secs(3600))
    };
    let motion_fut = radio::MOTION_CHANNEL.receive();
    let tick_fut = Timer::after(Duration::from_millis(POLL_TICK_MS));

    // Three-way select: motion command, motion-expiry timeout, or
    // global watchdog tick.
    match select(select(motion_fut, expiry_fut), tick_fut).await {
      // --- Outer select: motion/expiry vs watchdog tick ---
      Either::First(inner) => match inner {
        // --- Motion command received ---
        Either::First(motion) => {
          // Got a fresh motion command from the controller -> exit
          // failsafe state and drive motors as requested.
          failsafe_engaged = false;
          motion_received = true;

          // Update the 5x5 LED matrix with the current motion direction.
          let _ = display::DISPLAY_CHANNEL.try_send(motion);

          // External lights: on when stopped, off when moving.
          if motion.vx != 0 || motion.vy != 0 || motion.omega != 0 {
            light.light_off();
          } else {
            light.light_on();
          }

          // Drive the 4 RGB LEDs from the motion vector so the operator
          // gets a quick visual confirmation of the current command:
          //   * stopped         -> dim white
          //   * forward / back  -> green / red
          //   * strafe L / R    -> blue / yellow
          //   * spin in place   -> magenta
          let rgb_color = motion_to_rgb(&motion);
          rgb_strip.set_all(rgb_color);
          rgb_strip.show().await;

          motor_driver.apply_motion(&motion).await;
        }
        // --- Motion expiry timeout ---
        Either::Second(_) => {
          // No fresh motion command arrived within MOTION_EXPIRY_MS.
          // The controller is almost certainly gone — stop immediately.
          if !failsafe_engaged {
            warn!(
              "Motion expired: no command for {} ms, stopping motors",
              MOTION_EXPIRY_MS
            );
            motor_driver.stop_all().await;
            // Show idle pattern on the LED matrix.
            let _ = display::DISPLAY_CHANNEL.try_send(protocol::MotionPayload::stop());
            // Signal failsafe via the RGB strip too (dim red).
            rgb_strip.set_all(rgb::Color::new(32, 0, 0));
            rgb_strip.show().await;
            failsafe_engaged = true;
          }
        }
      },
      // --- Global watchdog tick ---
      Either::Second(_) => {
        // Check whether the RX path has gone completely silent.
        // `radio::last_rx_millis()` returns 0 until the first packet
        // is parsed, which we treat as "never connected" => failsafe
        // (motors should remain stopped after boot anyway).
        let last_rx = radio::last_rx_millis();
        let now_ms = Instant::now().as_millis() as u32;
        let elapsed = now_ms.wrapping_sub(last_rx);
        let link_lost = last_rx == 0 || elapsed > FAILSAFE_TIMEOUT_MS;

        if link_lost && !failsafe_engaged {
          warn!(
            "Failsafe engaged: no RX for {} ms (last_rx={}), stopping motors",
            elapsed, last_rx
          );
          motor_driver.stop_all().await;
          // Show idle pattern on the LED matrix.
          let _ = display::DISPLAY_CHANNEL.try_send(protocol::MotionPayload::stop());
          // Signal link-loss failsafe on the RGB strip (dim red).
          rgb_strip.set_all(rgb::Color::new(32, 0, 0));
          rgb_strip.show().await;
          failsafe_engaged = true;
        }
      }
    }
  }
}

/// Map a [`MotionPayload`] to a single status color for the RGB strip.
///
/// The strip is purely informational, so we collapse the 3-axis vector
/// down to one of a small palette of easily distinguishable colors:
///
/// * idle              -> dim white
/// * forward dominant  -> green
/// * backward dominant -> red
/// * strafe right      -> yellow
/// * strafe left       -> blue
/// * pure rotation     -> magenta
///
/// Threshold of 10 matches the controller's joystick dead-zone so a
/// centred stick consistently shows the idle color.
fn motion_to_rgb(motion: &protocol::MotionPayload) -> rgb::Color {
  const DEADZONE: i8 = 10;
  let vx = motion.vx;
  let vy = motion.vy;
  let omega = motion.omega;

  let abs_vx = vx.unsigned_abs() as i16;
  let abs_vy = vy.unsigned_abs() as i16;
  let abs_omega = omega.unsigned_abs() as i16;

  // Idle: every axis is within the dead-zone.
  if abs_vx < DEADZONE as i16 && abs_vy < DEADZONE as i16 && abs_omega < DEADZONE as i16 {
    return rgb::Color::new(16, 16, 16);
  }

  // Pick the dominant axis. Translation wins ties against rotation so
  // that mostly-driving commands don't flicker to magenta on small yaw.
  let max_lin = abs_vx.max(abs_vy);
  if abs_omega > max_lin {
    return rgb::Color::MAGENTA;
  }

  if abs_vx >= abs_vy {
    if vx >= 0 {
      rgb::Color::GREEN
    } else {
      rgb::Color::RED
    }
  } else if vy >= 0 {
    rgb::Color::YELLOW
  } else {
    rgb::Color::BLUE
  }
}
