//! Input-mode arbitration for the controller.
//!
//! The controller supports two velocity input sources:
//!
//! - `Joystick`  — analog thumb-stick on edge-connector P1/P2 (default).
//! - `Tilt`      — on-board LSM303AGR accelerometer used as a tilt sensor.
//!
//! The active mode is toggled by pressing the on-board A button.
//! Modules that produce `(vx, vy)` data (joystick / tilt) only emit a value
//! when their mode is active, while `omega` (C/D buttons) is always live.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Available input modes.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum InputMode {
  Joystick = 0,
  Tilt = 1,
}

/// Globally observable current mode. Stored as `u8` for atomic access from
/// any task. Use [`current()`] / [`set()`] to interact.
static MODE: AtomicU8 = AtomicU8::new(InputMode::Joystick as u8);

/// Mode-change notifier. Producers that need to react to a mode switch
/// (e.g. clear a stale velocity buffer) await this signal.
pub static MODE_CHANGED: Signal<CriticalSectionRawMutex, InputMode> = Signal::new();

/// Returns the currently active input mode.
#[inline]
pub fn current() -> InputMode {
  match MODE.load(Ordering::Relaxed) {
    1 => InputMode::Tilt,
    _ => InputMode::Joystick,
  }
}

/// Set a specific mode and notify listeners.
pub fn set(mode: InputMode) {
  MODE.store(mode as u8, Ordering::Relaxed);
  MODE_CHANGED.signal(mode);
}

/// Toggle between Joystick and Tilt and return the new mode.
pub fn toggle() -> InputMode {
  let next = match current() {
    InputMode::Joystick => InputMode::Tilt,
    InputMode::Tilt => InputMode::Joystick,
  };
  set(next);
  next
}
