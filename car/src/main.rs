#![no_std]
#![no_main]

mod motor;
mod motorbit;
mod pca9685;
mod radio;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};

use defmt::info;

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

  info!("Radio RX task spawned");

  // Initialize motor driver (PCA9685 via I2C)
  // micro:bit v2 edge connector: P19(SCL)=P0.26, P20(SDA)=P1.00
  let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;

  info!("Motor driver initialized, entering main loop");

  // Main loop: process motion commands from radio and drive motors
  loop {
    // Wait for a motion command from the radio task
    let motion = radio::MOTION_CHANNEL.receive().await;

    // Blink LED based on motion state
    if motion.vx != 0 || motion.vy != 0 || motion.omega != 0 {
      led_col1.set_high();
    } else {
      led_col1.set_low();
    }

    // Apply motion to Mecanum wheel motors via PCA9685
    motor_driver.apply_motion(&motion).await;
  }
}
