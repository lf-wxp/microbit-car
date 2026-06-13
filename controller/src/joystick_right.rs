//! Right-hand joystick driver — produces the rotational `omega`
//! component of the motion command.
//!
//! ## Status
//!
//! The hardware is **not wired yet**. To unblock the firmware-side
//! integration we ship two interchangeable sample sources:
//!
//! - **Default (mock):** the source returns a centered reading at every
//!   tick, i.e. `omega = 0`. The full pipeline (channel, fusion, radio)
//!   still runs end-to-end so we can develop and test it without the
//!   physical stick.
//! - **`right-stick-hw` feature:** a real SAADC-backed sampler placeholder
//!   you can finish wiring up once the second stick is on the board. The
//!   pin choice is left as `todo!()` deliberately — pick whichever AIN
//!   channel ends up free on your shield and update the constructor
//!   here plus the call site in `main.rs`. No other module changes.
//!
//! ## Mapping
//!
//! Only the X axis of the right stick is consumed: deflecting right
//! produces a positive `omega` (CW), left produces a negative one
//! (CCW). The Y axis is reserved for a future "throttle / speed gain"
//! feature; we sample it for symmetry but currently discard the value.
//!
//! ## Coexistence with the C/D buttons
//!
//! The button driver continues to publish to its own
//! [`crate::button::OMEGA_CHANNEL`]. The fusion loop in `main` performs
//! a saturating sum of *stick omega* and *button omega* so the buttons
//! act as fine-grained trim on top of the stick.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use defmt::{info, trace};

use crate::signal::{self, DEFAULT_DEAD_ZONE, EmaFilter};

/// Channel that publishes the right-stick-derived `omega` value to
/// the fusion loop. Capacity 4 absorbs short bursts.
pub static STICK_OMEGA_CHANNEL: Channel<CriticalSectionRawMutex, i8, 4> = Channel::new();

/// Sampling interval — matches the left joystick / button cadence so
/// every input source updates at the same rhythm.
const SAMPLE_INTERVAL_MS: u64 = 20;

// =====================================================================
// Sample source: real (feature = "right-stick-hw") vs mock (default).
//
// Both expose the same surface:
//   - `RightStickSource::new(...)` — built in `main.rs` from whatever
//     peripherals the chosen variant needs.
//   - `async fn sample(&mut self) -> (i16 /* x */, i16 /* y */)` —
//     returns the most recent raw 10-bit ADC readings (or mock values
//     in the same units), already clamped to `[0, ADC_MAX]`.
//
// Because the two variants are gated on a Cargo feature the unused
// branch is *compiled out entirely*, so there is no runtime cost.
// =====================================================================

#[cfg(not(feature = "right-stick-hw"))]
mod source {
  //! Mock implementation used when the second stick has not been wired
  //! up yet. Always returns a centered reading so the rest of the
  //! firmware behaves as if the stick exists and is at rest.

  use super::signal::ADC_MID;

  /// Empty stand-in for the real driver. Owns no peripherals so it can
  /// be constructed from `main` without touching the SAADC.
  pub struct RightStickSource;

  impl RightStickSource {
    pub const fn new() -> Self {
      Self
    }

    /// Always returns `(center, center)` → both axes idle.
    pub async fn sample(&mut self) -> (i16, i16) {
      (ADC_MID, ADC_MID)
    }
  }
}

#[cfg(feature = "right-stick-hw")]
mod source {
  //! Real SAADC-backed implementation. Finalize the pin assignment when
  //! the second stick lands on the board and remove the `todo!()`s.

  use embassy_nrf::saadc::{self, ChannelConfig, Config, Resolution, Saadc};
  use embassy_nrf::{Peri, peripherals};

  use super::signal::ADC_MAX;

  // The SAADC interrupt is bound exactly once by the left-stick module
  // (`crate::joystick::Irqs`). Re-binding it here would emit a second
  // `SAADC` interrupt symbol and break linking, so we share the same
  // `Irqs` struct.
  use crate::joystick::Irqs;

  /// SAADC-backed right-stick sampler. The actual pin types depend on
  /// where the stick gets wired; we leave them as generic peripheral
  /// references so `main.rs` controls the choice in one place.
  pub struct RightStickSource {
    saadc: Saadc<'static, 2>,
  }

  impl RightStickSource {
    /// Build the SAADC for the right stick. Pick two unused AIN-capable
    /// pins; the obvious candidate today is `P0` (AIN0). The second
    /// pin must avoid the LED-matrix shared pins (P3/P4/P10) unless
    /// you are willing to disable the LED display.
    ///
    /// The pins are taken by `impl Input + 'static` because in
    /// `embassy-nrf` it is the `Peri<'static, peripherals::Pxx>` value
    /// itself (not the bare pin marker type) that implements the
    /// `saadc::Input` trait. A typical call site looks like:
    ///
    /// ```ignore
    /// RightStickSource::new(p.SAADC, p.P0_00, p.P0_01)
    /// ```
    pub fn new(
      saadc_periph: Peri<'static, peripherals::SAADC>,
      pin_x: impl saadc::Input + 'static,
      pin_y: impl saadc::Input + 'static,
    ) -> Self {
      // The body below is the production wiring — it compiles today and
      // only needs the right pins to be picked at the call site. We
      // gate it behind `todo!()` until the hardware lands so anyone
      // enabling the feature is forced to make a deliberate choice.
      #[allow(unreachable_code)]
      {
        let _: Saadc<'static, 2> = {
          // Match the left-stick configuration: pin SAADC at 10-bit so
          // the shared `signal::process_axis` pipeline (which assumes
          // 0..=1023) sees raw samples in the expected range.
          let mut config = Config::default();
          config.resolution = Resolution::_10BIT;
          let cfg_x = ChannelConfig::single_ended(pin_x);
          let cfg_y = ChannelConfig::single_ended(pin_y);
          Saadc::new(saadc_periph, Irqs, config, [cfg_x, cfg_y])
        };
        todo!("Wire the right-stick SAADC channels once the hardware lands");
      }
    }

    pub async fn sample(&mut self) -> (i16, i16) {
      let mut buf = [0i16; 2];
      self.saadc.sample(&mut buf).await;
      let x = buf[0].clamp(0, ADC_MAX);
      let y = buf[1].clamp(0, ADC_MAX);
      (x, y)
    }
  }
}

pub use source::RightStickSource;

/// Internal smoothing state for the right stick. We only forward the
/// X axis to the channel today; the Y filter is kept warm so a future
/// "speed gain" feature can drop in without behavioural surprises.
struct RightStickState {
  filter_x: EmaFilter,
  filter_y: EmaFilter,
}

impl RightStickState {
  const fn new() -> Self {
    Self {
      filter_x: EmaFilter::new(),
      filter_y: EmaFilter::new(),
    }
  }

  fn update(&mut self, raw_x: i16, raw_y: i16) -> i8 {
    let omega = signal::process_axis(raw_x, &mut self.filter_x, DEFAULT_DEAD_ZONE);
    // Drain Y through the filter to keep state warm without affecting output.
    let _ = signal::process_axis(raw_y, &mut self.filter_y, DEFAULT_DEAD_ZONE);
    omega
  }
}

/// Right-stick reading task. Mirrors the left-stick task in cadence
/// and shape so behaviour stays predictable across both inputs.
///
/// Unlike the left stick, the right stick is **always live**: it is
/// not gated by [`crate::mode::InputMode`] because rotation should
/// remain available regardless of whether translation comes from the
/// joystick or the tilt sensor.
#[embassy_executor::task]
pub async fn joystick_right_task(mut sampler: RightStickSource) {
  info!("Right-stick task started (50 Hz sampling)");

  let mut state = RightStickState::new();
  let mut last_sent: i8 = 0;

  // Publish a known starting value so the fusion loop has a definite
  // initial right-stick contribution.
  STICK_OMEGA_CHANNEL.send(0).await;

  loop {
    let (raw_x, raw_y) = sampler.sample().await;
    trace!("Right-stick raw: x={}, y={}", raw_x, raw_y);

    let omega = state.update(raw_x, raw_y);
    if omega != last_sent {
      trace!("Right-stick omega: {}", omega);
      STICK_OMEGA_CHANNEL.send(omega).await;
      last_sent = omega;
    }

    embassy_time::Timer::after(embassy_time::Duration::from_millis(SAMPLE_INTERVAL_MS)).await;
  }
}
