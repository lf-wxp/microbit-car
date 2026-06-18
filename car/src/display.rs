//! 5x5 LED matrix display driver (micro:bit v2 car side).
//!
//! Similar to the controller side, this module renders the current motion
//! state as a directional indicator pattern on the car's on-board 5x5 LED
//! matrix. Pattern rendering logic comes from the shared module
//! [`protocol::display`].
//!
//! # Hardware Connections
//!
//! micro:bit v2 LED matrix pins (same as controller):
//! - ROW1-5: P0.21, P0.22, P0.15, P0.24, P0.19 (anode, active-high)
//! - COL1-5: P0.28, P0.11, P0.31, P1.05, P0.30 (cathode, active-low)
//!
//! # Usage
//!
//! Call [`init`] in `main` to initialize pins, then spawn [`display_task`].
//! The main loop sends `MotionPayload` updates via [`DISPLAY_CHANNEL`].

use embassy_futures::select::{Either, select};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use defmt::{info, trace};
use protocol::MotionPayload;
use protocol::display::{self, CENTRE_DOT, Frame};

/// Per-row scan dwell time. 5 rows × 400 µs = 2 ms full frame, i.e. ~500 Hz refresh.
const SCAN_INTERVAL_US: u64 = 400;

/// Channel for receiving the latest motion command from the main loop.
/// Capacity of 1 combined with `try_send` semantics on the producer side
/// ensures the buffer always holds the most recent value.
pub static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 1> = Channel::new();

/// GPIO pin ownership. `rows` are anodes (active-high), `cols` are cathodes (active-low).
pub struct MatrixPins {
  pub rows: [Output<'static>; 5],
  pub cols: [Output<'static>; 5],
}

/// Configure all 10 matrix pins as push-pull outputs, initially "off"
/// (rows low, cols high).
#[allow(clippy::too_many_arguments)]
pub fn init(
  row1: Peri<'static, peripherals::P0_21>,
  row2: Peri<'static, peripherals::P0_22>,
  row3: Peri<'static, peripherals::P0_15>,
  row4: Peri<'static, peripherals::P0_24>,
  row5: Peri<'static, peripherals::P0_19>,
  col1: Peri<'static, peripherals::P0_28>,
  col2: Peri<'static, peripherals::P0_11>,
  col3: Peri<'static, peripherals::P0_31>,
  col4: Peri<'static, peripherals::P1_05>,
  col5: Peri<'static, peripherals::P0_30>,
) -> MatrixPins {
  let rows = [
    Output::new(row1, Level::Low, OutputDrive::Standard),
    Output::new(row2, Level::Low, OutputDrive::Standard),
    Output::new(row3, Level::Low, OutputDrive::Standard),
    Output::new(row4, Level::Low, OutputDrive::Standard),
    Output::new(row5, Level::Low, OutputDrive::Standard),
  ];
  let cols = [
    Output::new(col1, Level::High, OutputDrive::Standard),
    Output::new(col2, Level::High, OutputDrive::Standard),
    Output::new(col3, Level::High, OutputDrive::Standard),
    Output::new(col4, Level::High, OutputDrive::Standard),
    Output::new(col5, Level::High, OutputDrive::Standard),
  ];
  info!("Car LED matrix initialized (5x5)");
  MatrixPins { rows, cols }
}

/// Display task: continuously multiplexes `current_frame` while listening
/// for new motion updates on [`DISPLAY_CHANNEL`].
#[embassy_executor::task]
pub async fn display_task(pins: MatrixPins) {
  info!("Car display task started");
  let MatrixPins { mut rows, mut cols } = pins;

  let mut current: Frame = CENTRE_DOT;
  let mut row_idx: usize = 0;
  let scan_tick = Duration::from_micros(SCAN_INTERVAL_US);

  loop {
    // Light the current row, then wait for the next scan tick or a new motion update.
    light_row(&mut rows, &mut cols, row_idx, &current[row_idx]);

    match select(Timer::after(scan_tick), DISPLAY_CHANNEL.receive()).await {
      Either::First(()) => {
        row_idx = (row_idx + 1) % 5;
      }
      Either::Second(payload) => {
        // Blank the current row before switching patterns to avoid ghosting.
        blank_all(&mut rows, &mut cols);
        let next = display::motion_to_frame(&payload);
        if next != current {
          trace!(
            "Car display: vx={}, vy={}, omega={} -> new frame",
            payload.vx, payload.vy, payload.omega
          );
          current = next;
        }
        // Reset scan position so the new pattern starts from the top.
        row_idx = 0;
      }
    }
  }
}

/// Drive a single row of the frame buffer.
fn light_row(
  rows: &mut [Output<'static>; 5],
  cols: &mut [Output<'static>; 5],
  row_idx: usize,
  row: &[bool; 5],
) {
  for (i, pin) in rows.iter_mut().enumerate() {
    if i == row_idx {
      pin.set_high();
    } else {
      pin.set_low();
    }
  }
  for (i, pin) in cols.iter_mut().enumerate() {
    if row[i] {
      pin.set_low();
    } else {
      pin.set_high();
    }
  }
}

/// Blank all LEDs — prevents ghosting during frame transitions.
fn blank_all(rows: &mut [Output<'static>; 5], cols: &mut [Output<'static>; 5]) {
  for pin in rows.iter_mut() {
    pin.set_low();
  }
  for pin in cols.iter_mut() {
    pin.set_high();
  }
}
