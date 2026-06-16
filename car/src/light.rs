//! Car status lights module.
//!
//! Provides a simple interface to control the left and right status LEDs
//! on the micro:bit-car chassis. The lights are used as a visual indicator
//! of the car's motion state:
//!
//! - **Lights ON** → car is stopped (no motion command)
//! - **Lights OFF** → car is moving
//!
//! # Hardware
//!
//! - Left light: P0_17 (active-low, Level::Low = ON)
//! - Right light: P0_13 (active-low, Level::Low = ON)
//!
//! # Example
//!
//! ```ignore
//! let mut light = light::init(p.P0_17, p.P0_13);
//! light.light_on();   // turn on when stopped
//! light.light_off();  // turn off when moving
//! ```

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{Peri, peripherals};

use defmt::info;

/// Car status lights (left and right).
///
/// Controls two LEDs that indicate whether the car is currently
/// moving or stopped. The LEDs are active-low (Level::Low = ON).
///
/// Use [`init`] to create an instance.
pub struct Light {
  left: Output<'static>,
  right: Output<'static>,
}

impl Light {
  /// Turn on both status lights.
  ///
  /// Sets both left and right GPIO pins to low level (active-low LED).
  /// Typically called when the car is stopped to indicate idle state.
  ///
  /// # Example
  ///
  /// ```ignore
  /// let mut light = light::init(p.P0_17, p.P0_13);
  /// light.light_on();  // show car is stopped
  /// ```
  pub(crate) fn light_on(&mut self) {
    self.left.set_low();
    self.right.set_low();
  }

  /// Turn off both status lights.
  ///
  /// Sets both left and right GPIO pins to high level (active-low LED).
  /// Typically called when the car is moving to indicate active state.
  ///
  /// # Example
  ///
  /// ```ignore
  /// let mut light = light::init(p.P0_17, p.P0_13);
  /// light.light_off();  // turn off lights while moving
  /// ```
  pub(crate) fn light_off(&mut self) {
    self.left.set_high();
    self.right.set_high();
  }
}

/// Initialize the status lights.
///
/// Takes ownership of the two GPIO pins and configures them as outputs
/// with active-low logic (Level::Low = LED ON).
///
/// # Arguments
///
/// * `left` - The left LED pin (typically P0_17)
/// * `right` - The right LED pin (typically P0_13)
///
/// # Returns
///
/// A [`Light`] instance ready to control the status lights.
///
/// # Example
///
/// ```ignore
/// let light = light::init(p.P0_17, p.P0_13);
/// ```
pub fn init(
  left: Peri<'static, peripherals::P0_17>,
  right: Peri<'static, peripherals::P0_13>,
) -> Light {
  let light = Light {
    left: Output::new(left, Level::Low, OutputDrive::Standard),
    right: Output::new(right, Level::Low, OutputDrive::Standard),
  };
  info!("Light initialized");
  light
}
