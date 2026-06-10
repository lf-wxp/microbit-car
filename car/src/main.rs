#![no_std]
#![no_main]

mod radio;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};

use defmt::info;
use protocol::MotionPayload;

/// Main entry point for the car firmware
/// Runs on micro:bit v2 (nRF52833)
#[embassy_executor::main]
async fn main(spawner: Spawner) {
  let p = embassy_nrf::init(Default::default());

  // LED row 1, col 1 on micro:bit v2 (top-left LED) - status indicator
  let mut led_col1 = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
  let mut _led_row1 = Output::new(p.P0_21, Level::High, OutputDrive::Standard);

  info!("Car firmware started");

  // Initialize radio and spawn RX task
  let radio = radio::init(p.RADIO);
  spawner.spawn(radio::radio_rx_task(radio).unwrap());

  info!("Radio RX task spawned, entering main loop");

  // Main loop: process motion commands from radio
  loop {
    // Wait for a motion command from the radio task
    let motion = radio::MOTION_CHANNEL.receive().await;

    // Blink LED based on motion state
    if motion.vx != 0 || motion.vy != 0 || motion.omega != 0 {
      // Moving: LED on
      led_col1.set_high();
    } else {
      // Stopped: LED off
      led_col1.set_low();
    }

    // TODO: Apply motion to Mecanum wheel motors
    // The inverse kinematics formula (see protocol docs):
    //   motor_fl = vx - vy - k * omega
    //   motor_fr = vx + vy + k * omega
    //   motor_rl = vx + vy - k * omega
    //   motor_rr = vx - vy + k * omega
    handle_motion(&motion);
  }
}

/// Apply motion command to motors (placeholder for actual motor control)
fn handle_motion(motion: &MotionPayload) {
  if motion.vx == 0 && motion.vy == 0 && motion.omega == 0 {
    defmt::trace!("Motors: STOP");
    return;
  }

  // Mecanum inverse kinematics (k=1.0 for simplicity)
  let vx = motion.vx as i16;
  let vy = motion.vy as i16;
  let omega = motion.omega as i16;

  let motor_fl = vx - vy - omega;
  let motor_fr = vx + vy + omega;
  let motor_rl = vx + vy - omega;
  let motor_rr = vx - vy + omega;

  defmt::trace!(
    "Motors: FL={}, FR={}, RL={}, RR={}",
    motor_fl,
    motor_fr,
    motor_rl,
    motor_rr
  );
}
