#![no_std]
#![no_main]

mod diagnostic;
mod motor;
mod motorbit;
mod pca9685;
mod radio;

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
/// The controller streams motion at ~50 Hz (every 20 ms) and a
/// heartbeat at ~5 Hz (every 200 ms), so any value above ~220 ms
/// is "definitely lost" — there is no legitimate quiet window that
/// long. We pick 250 ms to leave a tiny tolerance for one missed
/// heartbeat while keeping the worst-case "controller pulled →
/// motors stopped" latency well under a third of a second, which
/// is the practical limit before the chassis perceptibly drifts on
/// its last commanded velocity.
///
/// Earlier the timeout was 500 ms, which created a noticeable
/// "ghost roll": after the controller was switched off the car
/// happily executed the last motion command for another half
/// second, and at the same time the (no-longer-masked) 1.5 kHz
/// PWM carrier became audibly louder once the rolling/road noise
/// dropped away. Both symptoms vanish once the failsafe fires
/// promptly.
const FAILSAFE_TIMEOUT_MS: u32 = 250;

/// Failsafe watchdog tick. We can't simply `select` on the motion
/// channel forever because a wedged controller stops sending entirely;
/// instead we wake every `POLL_TICK_MS` to compare `now` against the
/// last RX timestamp. With a 250 ms timeout, polling every 50 ms keeps
/// the worst-case detection latency at ~300 ms while costing only a
/// few extra Timer wakeups per second.
const POLL_TICK_MS: u64 = 50;

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

  // LED row 1, col 1 on micro:bit v2 (top-left LED) - status indicator
  let mut led_col1 = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
  let mut _led_row1 = Output::new(p.P0_21, Level::High, OutputDrive::Standard);

  info!("Car firmware started");

  // Sample the on-board A button (P0_14) *before* anything else so the
  // operator can opt into diagnostic mode at boot. Pressed = low.
  let diagnostic_requested = diagnostic::is_diagnostic_requested(p.P0_14).await;

  if diagnostic_requested {
    // Diagnostic path: skip radio init entirely so it can't interfere
    // with the controlled motor sweep. `diagnostic::run` is divergent
    // (`-> !`), so control never returns from this branch.
    let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;
    led_col1.set_high();
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

  // Tracks whether failsafe has already forced a stop, so we don't
  // hammer the I2C bus restating "all motors off" every 100 ms.
  // Reset to `false` on every successful motion command.
  let mut failsafe_engaged = false;

  // Main loop: process motion commands from radio and drive motors,
  // while a 100 ms ticker enforces a link-loss failsafe stop.
  loop {
    let motion_fut = radio::MOTION_CHANNEL.receive();
    let tick_fut = Timer::after(Duration::from_millis(POLL_TICK_MS));

    match select(motion_fut, tick_fut).await {
      Either::First(motion) => {
        // Got a fresh motion command from the controller -> exit
        // failsafe state and drive motors as requested.
        failsafe_engaged = false;

        // LED indicates whether the chassis is currently commanded to move.
        if motion.vx != 0 || motion.vy != 0 || motion.omega != 0 {
          led_col1.set_high();
        } else {
          led_col1.set_low();
        }

        motor_driver.apply_motion(&motion).await;
      }
      Either::Second(_) => {
        // Watchdog tick: check whether the RX path has gone silent.
        // `radio::last_rx_millis()` returns 0 until the first packet
        // is parsed, which we treat as "never connected" => failsafe
        // (motors should remain stopped after boot anyway).
        let last_rx = radio::last_rx_millis();
        let now_ms = Instant::now().as_millis() as u32;
        // `wrapping_sub` makes the comparison robust across the
        // ~49-day u32-millis wrap; in practice we'll never hit it.
        let elapsed = now_ms.wrapping_sub(last_rx);
        let link_lost = last_rx == 0 || elapsed > FAILSAFE_TIMEOUT_MS;

        if link_lost && !failsafe_engaged {
          warn!(
            "Failsafe engaged: no RX for {} ms (last_rx={}), stopping motors",
            elapsed, last_rx
          );
          motor_driver.stop_all().await;
          led_col1.set_low();
          failsafe_engaged = true;
        }
      }
    }
  }
}
