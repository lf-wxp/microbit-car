//! Shared logic for the 5x5 LED matrix display.
//!
//! This module provides pure logic for converting a [`MotionPayload`] into a
//! 5x5 LED frame buffer, without any hardware driver code. Both the controller
//! and car can use this module to render directional indicator patterns on their
//! respective LED matrices.
//!
//! # Glyph Mapping Rules
//!
//! Given `MotionPayload { vx, vy, omega }` (each in `[-100, 100]`):
//!
//! * If `|vx|`, `|vy|`, and `|omega|` are all below [`MOVE_THRESHOLD`],
//!   display a centre dot ("idle").
//! * Otherwise, if `|omega| > max(|vx|, |vy|) * ROTATION_BIAS / 10`,
//!   render a clockwise (`omega > 0`) or counter-clockwise (`omega < 0`)
//!   rotation pattern.
//! * Otherwise, select one of eight directional arrows based on `(vy, vx)`.
//!   Convention: `+vx = forward = arrow up`, `+vy = right = arrow right`.

use crate::MotionPayload;

/// Speed components below this absolute value are treated as "zero".
/// Roughly matches the joystick dead zone so that a centred stick renders
/// as an idle dot rather than a flickering arrow.
pub const MOVE_THRESHOLD: i8 = 15;

/// Rotation-vs-translation bias factor, in tenths.
///
/// When `|omega| * 10 > max(|vx|, |vy|) * ROTATION_BIAS`, the motion is
/// considered "rotation-dominant". A value of `15` means omega must be
/// roughly 1.5× the translation magnitude to win.
pub const ROTATION_BIAS: i16 = 15;

/// 5x5 frame buffer. `frame[row][col] == true` means the pixel is lit.
pub type Frame = [[bool; 5]; 5];

/// All-off frame, usable as a safe initial value.
pub const BLANK: Frame = [[false; 5]; 5];

/// Single dot at the matrix centre (idle indicator).
pub const CENTRE_DOT: Frame = pattern([
  "     ", //
  "     ", //
  "  X  ", //
  "     ", //
  "     ", //
]);

/// Convert 5 rows of ASCII art into a [`Frame`].
/// Any non-space character lights the corresponding pixel.
pub const fn pattern(rows: [&str; 5]) -> Frame {
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

// --- Eight-direction translation arrows ---
//
// `+vx` is "forward" (arrow up), `+vy` is "right" (arrow right).
// Index convention used by [`arrow_for`]:
//   0 = N (+vx)        1 = NE (+vx,+vy)   2 = E (+vy)
//   3 = SE (-vx,+vy)   4 = S (-vx)        5 = SW (-vx,-vy)
//   6 = W (-vy)        7 = NW (+vx,-vy)

pub const ARROW_N: Frame = pattern([
  "  X  ", //
  " XXX ", //
  "X X X", //
  "  X  ", //
  "  X  ", //
]);

pub const ARROW_NE: Frame = pattern([
  "  XXX", //
  "   XX", //
  "  X X", //
  " X   ", //
  "X    ", //
]);

pub const ARROW_E: Frame = pattern([
  "  X  ", //
  "   X ", //
  "XXXXX", //
  "   X ", //
  "  X  ", //
]);

pub const ARROW_SE: Frame = pattern([
  "X    ", //
  " X   ", //
  "  X X", //
  "   XX", //
  "  XXX", //
]);

pub const ARROW_S: Frame = pattern([
  "  X  ", //
  "  X  ", //
  "X X X", //
  " XXX ", //
  "  X  ", //
]);

pub const ARROW_SW: Frame = pattern([
  "    X", //
  "   X ", //
  "X X  ", //
  "XX   ", //
  "XXX  ", //
]);

pub const ARROW_W: Frame = pattern([
  "  X  ", //
  " X   ", //
  "XXXXX", //
  " X   ", //
  "  X  ", //
]);

pub const ARROW_NW: Frame = pattern([
  "XXX  ", //
  "XX   ", //
  "X X  ", //
  "   X ", //
  "    X", //
]);

/// Eight-direction arrow array, indexed by compass direction.
pub const ARROWS: [Frame; 8] = [
  ARROW_N, ARROW_NE, ARROW_E, ARROW_SE, ARROW_S, ARROW_SW, ARROW_W, ARROW_NW,
];

// --- Rotation patterns (asymmetric ring, direction is obvious at a glance) ---

pub const SPIN_CW: Frame = pattern([
  " XXX ", //
  "X   X", //
  "X   X", //
  "X    ", //
  " XX  ", //
]);

pub const SPIN_CCW: Frame = pattern([
  " XXX ", //
  "X   X", //
  "X   X", //
  "    X", //
  "  XX ", //
]);

/// Select the best-matching compass arrow for the given `(vx, vy)` direction.
///
/// Uses octant lookup (based on magnitude ratio), no `libm` `atan2` needed.
pub fn arrow_for(vx: i8, vy: i8) -> Frame {
  // Components below threshold are treated as zero to avoid selecting
  // a diagonal when one axis is essentially idle.
  let vx = if vx.abs() < MOVE_THRESHOLD { 0 } else { vx };
  let vy = if vy.abs() < MOVE_THRESHOLD { 0 } else { vy };
  let ax = (vx as i16).abs();
  let ay = (vy as i16).abs();

  // When both axes differ by no more than 2× we consider it "diagonal".
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

/// Convert a [`MotionPayload`] into the frame that best represents its
/// motion state.
pub fn motion_to_frame(payload: &MotionPayload) -> Frame {
  let MotionPayload { vx, vy, omega } = *payload;
  let max_lin = (vx.abs() as i16).max(vy.abs() as i16);
  let abs_omega = omega.abs() as i16;

  let translating = max_lin >= MOVE_THRESHOLD as i16;
  let rotating = abs_omega >= MOVE_THRESHOLD as i16;

  if !translating && !rotating {
    return CENTRE_DOT;
  }

  // When omega is significantly larger than linear magnitude (or there
  // is no linear motion), rotation takes priority.
  if rotating && (!translating || abs_omega * 10 > max_lin * ROTATION_BIAS) {
    return if omega > 0 { SPIN_CW } else { SPIN_CCW };
  }

  arrow_for(vx, vy)
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
