#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;

use defmt::info;

/// Main entry point for the car firmware
/// Runs on micro:bit v2 (nRF52833)
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
  let p = embassy_nrf::init(Default::default());

  // LED row 1, col 1 on micro:bit v2 (top-left LED)
  let mut led_col1 = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
  let mut _led_row1 = Output::new(p.P0_21, Level::High, OutputDrive::Standard);

  info!("Car firmware started");

  loop {
    led_col1.set_high();
    Timer::after_millis(500).await;
    led_col1.set_low();
    Timer::after_millis(500).await;
  }
}
