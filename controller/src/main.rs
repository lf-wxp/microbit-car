#![no_std]
#![no_main]

mod joystick;
mod radio;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};

use defmt::info;

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

  // Initialize joystick SAADC and spawn joystick task
  // micro:bit edge connector: P1 = P0.03 (AIN2), P2 = P0.04 (AIN3)
  let saadc = joystick::init(p.SAADC, p.P0_03, p.P0_04);
  spawner.spawn(joystick::joystick_task(saadc).unwrap());

  info!("Radio and joystick tasks spawned, entering main loop");

  // Main loop: forward joystick motion commands to radio TX channel
  loop {
    // Receive motion from joystick task
    let motion = joystick::JOYSTICK_MOTION_CHANNEL.receive().await;

    // Forward to radio for transmission to car
    radio::MOTION_TX_CHANNEL.send(motion).await;

    // Blink LED to indicate activity
    led_col1.set_high();
    Timer::after(Duration::from_millis(20)).await;
    led_col1.set_low();
  }
}
