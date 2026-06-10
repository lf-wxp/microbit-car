#![no_std]
#![no_main]

mod radio;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};

use defmt::info;
use protocol::MotionPayload;

/// Main entry point for the controller firmware
/// Runs on micro:bit v2 (nRF52833)
#[embassy_executor::main]
async fn main(spawner: Spawner) {
  let p = embassy_nrf::init(Default::default());

  // LED row 1, col 1 on micro:bit v2 (top-left LED) - connection indicator
  let mut led_col1 = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
  let mut _led_row1 = Output::new(p.P0_21, Level::High, OutputDrive::Standard);

  info!("Controller firmware started");

  // Initialize radio and spawn radio task
  let radio = radio::init(p.RADIO);
  spawner.spawn(radio::radio_task(radio).unwrap());

  info!("Radio task spawned, entering main loop");

  // Main loop: read joystick input and send motion commands
  // For now, send a test pattern to verify radio communication
  let mut phase: u8 = 0;

  loop {
    // Generate test motion commands (will be replaced by joystick input)
    let motion = match phase % 5 {
      0 => MotionPayload::forward(50),  // Forward
      1 => MotionPayload::strafe(50),   // Strafe right
      2 => MotionPayload::forward(-50), // Backward
      3 => MotionPayload::strafe(-50),  // Strafe left
      4 => MotionPayload::rotate(30),   // Spin
      _ => MotionPayload::stop(),
    };

    // Send motion command to radio task
    radio::MOTION_TX_CHANNEL.send(motion).await;

    // Blink LED to indicate activity
    led_col1.set_high();
    Timer::after(Duration::from_millis(50)).await;
    led_col1.set_low();

    // Wait before next command
    Timer::after(Duration::from_millis(950)).await;

    phase = phase.wrapping_add(1);
  }
}
