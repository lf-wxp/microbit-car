//! 5x5 LED matrix display for the controller (micro:bit v2).
//!
//! Renders the *current* `MotionPayload` as a directional glyph on the
//! on-board 5x5 LED matrix so the operator gets immediate visual
//! feedback about what the car was last told to do.
//!
//! # Glyph mapping
//!
//! Given `MotionPayload { vx, vy, omega }` (each in `[-100, 100]`):
//!
//! * If `|vx|`, `|vy|`, `|omega|` are all below [`MOVE_THRESHOLD`],
//!   show a single centre dot ("idle").
//! * Otherwise, if `|omega| > max(|vx|, |vy|) * ROTATION_BIAS / 10`,
//!   render a clockwise (`omega > 0`) or counter-clockwise (`omega < 0`)
//!   spinner glyph.
//! * Otherwise, pick one of eight compass arrows based on the direction
//!   of `(vy, vx)`. Convention: `+vx = forward = arrow up`,
//!   `+vy = right = arrow right`, mirroring the body-frame the car uses.
//!
//! # Multiplexing
//!
//! The matrix is wired as 5 row-anodes and 5 column-cathodes, so to
//! light pixel `(r, c)` we drive `ROW[r]` high, `COL[c]` low and
//! everyone else off. We therefore time-multiplex one row at a time at
//! ~200 Hz (`SCAN_INTERVAL_US` per row, 5 rows -> ~1 kHz frame rate
//! at the GPIO level, well above the human flicker threshold).
//!
//! The frame buffer is updated whenever a new [`MotionPayload`] arrives
//! on [`DISPLAY_CHANNEL`]; rendering and scanning are decoupled so a
//! burst of motion updates never starves the multiplexer.

use embassy_futures::select::{Either, select};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use defmt::{info, trace};
use protocol::MotionPayload;

/// Below this absolute value a velocity component is considered "zero".
/// Tuned to roughly match the joystick dead-zone so a centred stick
/// renders as the idle dot rather than a flickering arrow.
const MOVE_THRESHOLD: i8 = 15;

/// Rotation-vs-translation bias, expressed as tenths.
///
/// We treat the payload as "rotation dominant" iff
/// `|omega| * 10 > max(|vx|, |vy|) * ROTATION_BIAS`. A value of `15`
/// means omega has to be ~1.5x the translation magnitude to win, which
/// stops a tiny stick wiggle from suppressing a clear spin command.
const ROTATION_BIAS: i16 = 15;

/// Per-row dwell time during multiplexing. 5 rows * 400 µs = 2 ms full
/// frame, i.e. ~500 Hz refresh — invisible flicker, low CPU.
const SCAN_INTERVAL_US: u64 = 400;

/// Channel carrying the most recent motion command from the fusion
/// loop. Capacity 1 plus `try_send` semantics in the producer keeps
/// the buffer always-fresh without blocking the radio path.
pub static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 1> = Channel::new();

/// A 5x5 frame buffer. `frame[row][col] == true` means the pixel is lit.
pub type Frame = [[bool; 5]; 5];

/// All-off frame, useful as a safe initial value.
pub const BLANK: Frame = [[false; 5]; 5];

/// Pixel at the geometric centre of the matrix (idle indicator).
const CENTRE_DOT: Frame = pattern([
  "     ", //
  "     ", //
  "  X  ", //
  "     ", //
  "     ", //
]);

/// Convert a 5-row ASCII art block into a [`Frame`]. Any non-space
/// character lights the pixel; this keeps the glyph table below
/// readable as drawings rather than bit-twiddled hex literals.
const fn pattern(rows: [&str; 5]) -> Frame {
  let mut out = BLANK;
  let mut r = 0;
  while r < 5 {
    let bytes = rows[r].as_bytes();
    let mut c = 0;
    while c < 5 && c < bytes.len() {
      out[r][c] = bytes[c] != b' ';
      c += 1;
    }
    r += 1;
  }
  out
}

// --- 8-way translation arrows ---------------------------------------
//
// `+vx` is "forward" (arrow up), `+vy` is "right" (arrow right).
// Indexing convention used by [`arrow_for`]:
//   0 = N (+vx)        1 = NE (+vx,+vy)   2 = E (+vy)
//   3 = SE (-vx,+vy)   4 = S (-vx)        5 = SW (-vx,-vy)
//   6 = W (-vy)        7 = NW (+vx,-vy)

const ARROW_N: Frame = pattern([
  "  X  ", //
  " XXX ", //
  "X X X", //
  "  X  ", //
  "  X  ", //
]);

const ARROW_NE: Frame = pattern([
  "  XXX", //
  "   XX", //
  "  X X", //
  " X   ", //
  "X    ", //
]);

const ARROW_E: Frame = pattern([
  "  X  ", //
  "   X ", //
  "XXXXX", //
  "   X ", //
  "  X  ", //
]);

const ARROW_SE: Frame = pattern([
  "X    ", //
  " X   ", //
  "  X X", //
  "   XX", //
  "  XXX", //
]);

const ARROW_S: Frame = pattern([
  "  X  ", //
  "  X  ", //
  "X X X", //
  " XXX ", //
  "  X  ", //
]);

const ARROW_SW: Frame = pattern([
  "    X", //
  "   X ", //
  "X X  ", //
  "XX   ", //
  "XXX  ", //
]);

const ARROW_W: Frame = pattern([
  "  X  ", //
  " X   ", //
  "XXXXX", //
  " X   ", //
  "  X  ", //
]);

const ARROW_NW: Frame = pattern([
  "XXX  ", //
  "XX   ", //
  "X X  ", //
  "   X ", //
  "    X", //
]);

const ARROWS: [Frame; 8] = [
  ARROW_N, ARROW_NE, ARROW_E, ARROW_SE, ARROW_S, ARROW_SW, ARROW_W, ARROW_NW,
];

// --- Rotation glyphs (asymmetric ring so direction is unambiguous) --

const SPIN_CW: Frame = pattern([
  " XXX ", //
  "X   X", //
  "X   X", //
  "X    ", //
  " XX  ", //
]);

const SPIN_CCW: Frame = pattern([
  " XXX ", //
  "X   X", //
  "X   X", //
  "    X", //
  "  XX ", //
]);

/// Pick the compass arrow whose direction best matches `(vx, vy)`.
///
/// Uses an octant lookup based on the magnitude ratio so we don't drag
/// in `libm` for `atan2`. This keeps the function `const`-friendly and
/// dependency-free.
fn arrow_for(vx: i8, vy: i8) -> Frame {
  // Treat sub-threshold components as exactly zero so we don't pick a
  // diagonal when one axis is basically idle.
  let vx = if vx.abs() < MOVE_THRESHOLD { 0 } else { vx };
  let vy = if vy.abs() < MOVE_THRESHOLD { 0 } else { vy };
  let ax = (vx as i16).abs();
  let ay = (vy as i16).abs();

  // A component counts as "diagonal" when neither axis dominates the
  // other by more than 2x. Using integer ratios avoids floats.
  let diagonal = ax > 0 && ay > 0 && ax * 2 >= ay && ay * 2 >= ax;

  let idx = match (vx.signum(), vy.signum(), diagonal) {
    (1, 0, _) | (1, _, false) if vx.unsigned_abs() as i16 >= ay => 0, // N
    (1, 1, true) => 1,                                                // NE
    (0, 1, _) | (_, 1, false) => 2,                                   // E
    (-1, 1, true) => 3,                                               // SE
    (-1, 0, _) | (-1, _, false) => 4,                                 // S
    (-1, -1, true) => 5,                                              // SW
    (0, -1, _) | (_, -1, false) => 6,                                 // W
    (1, -1, true) => 7,                                               // NW
    _ => return CENTRE_DOT,
  };
  ARROWS[idx]
}

/// Convert a [`MotionPayload`] into the glyph that best represents it.
pub fn motion_to_frame(payload: &MotionPayload) -> Frame {
  let MotionPayload { vx, vy, omega } = *payload;
  let max_lin = (vx.abs() as i16).max(vy.abs() as i16);
  let abs_omega = omega.abs() as i16;

  let translating = max_lin >= MOVE_THRESHOLD as i16;
  let rotating = abs_omega >= MOVE_THRESHOLD as i16;

  if !translating && !rotating {
    return CENTRE_DOT;
  }

  // Rotation wins when omega is meaningfully larger than the linear
  // magnitude (or when there's no linear motion at all).
  if rotating && (!translating || abs_omega * 10 > max_lin * ROTATION_BIAS) {
    return if omega > 0 { SPIN_CW } else { SPIN_CCW };
  }

  arrow_for(vx, vy)
}

/// GPIO ownership for the 5x5 matrix. `rows` are anodes (active high),
/// `cols` are cathodes (active low).
pub struct MatrixPins {
  pub rows: [Output<'static>; 5],
  pub cols: [Output<'static>; 5],
}

/// Configure all ten matrix pins as push-pull outputs in the "off"
/// state (rows low, cols high).
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
  info!("LED matrix initialized (5x5)");
  MatrixPins { rows, cols }
}

/// Display task: continuously multiplex `current_frame` while listening
/// for new motion updates on [`DISPLAY_CHANNEL`].
#[embassy_executor::task]
pub async fn display_task(pins: MatrixPins) {
  info!("Display task started");
  let MatrixPins { mut rows, mut cols } = pins;

  let mut current: Frame = CENTRE_DOT;
  let mut row_idx: usize = 0;
  let scan_tick = Duration::from_micros(SCAN_INTERVAL_US);

  loop {
    // Light the active row, then wait for either the next scan tick or
    // a new motion update — whichever happens first.
    light_row(&mut rows, &mut cols, row_idx, &current[row_idx]);

    match select(Timer::after(scan_tick), DISPLAY_CHANNEL.receive()).await {
      Either::First(()) => {
        row_idx = (row_idx + 1) % 5;
      }
      Either::Second(payload) => {
        // Always blank the currently lit row before redrawing so a
        // glyph change can't briefly latch a pixel "on" outside its
        // intended dwell slot.
        blank_all(&mut rows, &mut cols);
        let next = motion_to_frame(&payload);
        if next != current {
          trace!(
            "Display: vx={}, vy={}, omega={} -> new frame",
            payload.vx, payload.vy, payload.omega
          );
          current = next;
        }
        // Reset the scan position so the new glyph starts cleanly.
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
  // Park every other row low so only `row_idx` sources current.
  for (i, pin) in rows.iter_mut().enumerate() {
    if i == row_idx {
      pin.set_high();
    } else {
      pin.set_low();
    }
  }
  // Active-low columns: pull the lit pixels' columns to ground, leave
  // the rest high so they sink no current.
  for (i, pin) in cols.iter_mut().enumerate() {
    if row[i] {
      pin.set_low();
    } else {
      pin.set_high();
    }
  }
}

/// Drive every row low and every column high — useful between frame
/// changes to avoid ghosting.
fn blank_all(rows: &mut [Output<'static>; 5], cols: &mut [Output<'static>; 5]) {
  for pin in rows.iter_mut() {
    pin.set_low();
  }
  for pin in cols.iter_mut() {
    pin.set_high();
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn motion(vx: i8, vy: i8, omega: i8) -> MotionPayload {
    MotionPayload { vx, vy, omega }
  }

  #[test]
  fn idle_renders_centre_dot() {
    assert_eq!(motion_to_frame(&motion(0, 0, 0)), CENTRE_DOT);
    assert_eq!(motion_to_frame(&motion(5, -5, 3)), CENTRE_DOT);
  }

  #[test]
  fn rotation_dominates_when_much_larger() {
    assert_eq!(motion_to_frame(&motion(20, 0, 100)), SPIN_CW);
    assert_eq!(motion_to_frame(&motion(0, 20, -100)), SPIN_CCW);
  }

  #[test]
  fn cardinal_arrows() {
    assert_eq!(motion_to_frame(&motion(80, 0, 0)), ARROW_N);
    assert_eq!(motion_to_frame(&motion(-80, 0, 0)), ARROW_S);
    assert_eq!(motion_to_frame(&motion(0, 80, 0)), ARROW_E);
    assert_eq!(motion_to_frame(&motion(0, -80, 0)), ARROW_W);
  }

  #[test]
  fn diagonal_arrows() {
    assert_eq!(motion_to_frame(&motion(60, 60, 0)), ARROW_NE);
    assert_eq!(motion_to_frame(&motion(-60, 60, 0)), ARROW_SE);
    assert_eq!(motion_to_frame(&motion(-60, -60, 0)), ARROW_SW);
    assert_eq!(motion_to_frame(&motion(60, -60, 0)), ARROW_NW);
  }
}
