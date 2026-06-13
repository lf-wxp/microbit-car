//! Joystick input module for the controller firmware.
//!
//! Reads analog joystick values from two axes using the nRF52833 SAADC:
//! - Y-axis: P1 (AIN2) — forward/backward (up=1023, down=0)
//! - X-axis: P2 (AIN3) — left/right (right=1023, left=0)
//!
//! The DSP pipeline (dead zone + scale + EMA smoothing) lives in
//! [`crate::signal`] so the right-stick driver can reuse the same maths.

use embassy_nrf::bind_interrupts;
use embassy_nrf::saadc::{ChannelConfig, Config, InterruptHandler, Resolution, Saadc};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use defmt::{info, trace};
use protocol::MotionPayload;

use crate::mode::{self, InputMode};
use crate::signal::{self, ADC_MAX, DEFAULT_DEAD_ZONE, EmaFilter};

// Bind SAADC interrupt to the handler. Made `pub(crate)` so the
// right-stick driver can reuse the same binding instead of declaring
// a second one (which would clash on the SAADC symbol).
bind_interrupts!(pub(crate) struct Irqs {
  SAADC => InterruptHandler;
});

/// Channel for sending joystick-derived motion commands to the radio task
pub static JOYSTICK_MOTION_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> =
  Channel::new();

/// Joystick state with one EMA filter per axis.
struct JoystickState {
  filter_x: EmaFilter,
  filter_y: EmaFilter,
}

impl JoystickState {
  const fn new() -> Self {
    Self {
      filter_x: EmaFilter::new(),
      filter_y: EmaFilter::new(),
    }
  }

  /// Run a fresh ADC sample through the shared DSP pipeline and assemble
  /// the resulting `MotionPayload`.
  ///
  /// Axis convention: right stick X (right = positive) maps to `vy`,
  /// stick Y (forward = positive) maps to `vx`. `omega` is left at 0;
  /// rotation is provided by the buttons / right stick instead.
  fn update(&mut self, raw_x: i16, raw_y: i16) -> MotionPayload {
    let smoothed_x = signal::process_axis(raw_x, &mut self.filter_x, DEFAULT_DEAD_ZONE);
    let smoothed_y = signal::process_axis(raw_y, &mut self.filter_y, DEFAULT_DEAD_ZONE);

    MotionPayload {
      vx: smoothed_y,
      vy: smoothed_x,
      omega: 0,
    }
  }

  /// Drop the smoothing memory back to zero so the first sample after
  /// reactivation isn't tainted by stale state.
  fn reset(&mut self) {
    self.filter_x.reset();
    self.filter_y.reset();
  }
}

/// Initialize the SAADC peripheral for joystick reading.
///
/// Configures two channels:
/// - Channel 0: P1 (AIN2) for Y-axis
/// - Channel 1: P2 (AIN3) for X-axis
///
/// Returns a configured Saadc instance ready for sampling.
pub fn init<'d>(
  saadc_periph: Peri<'d, peripherals::SAADC>,
  pin_y: Peri<'d, peripherals::P0_03>, // P1 = AIN2 on micro:bit edge connector
  pin_x: Peri<'d, peripherals::P0_04>, // P2 = AIN3 on micro:bit edge connector
) -> Saadc<'d, 2> {
  // The shared DSP pipeline (`signal::process_axis`) assumes 10-bit
  // samples (0..=1023). embassy-nrf defaults to 12-bit, which would make
  // the idle reading land near 1877 and `clamp(0, ADC_MAX=1023)` would
  // saturate the entire right/up half-range. Pin the resolution so the
  // hardware matches the DSP assumption.
  let mut config = Config::default();
  config.resolution = Resolution::_10BIT;

  // Channel 0: Y-axis (P1 / AIN2)
  let channel_y = ChannelConfig::single_ended(pin_y);
  // Channel 1: X-axis (P2 / AIN3)
  let channel_x = ChannelConfig::single_ended(pin_x);

  let saadc = Saadc::new(saadc_periph, Irqs, config, [channel_y, channel_x]);

  info!("Joystick SAADC initialized (Y=P1/AIN2, X=P2/AIN3, 10-bit)");
  saadc
}

/// Joystick reading task.
///
/// Continuously samples the joystick axes and sends MotionPayload
/// to the JOYSTICK_MOTION_CHANNEL for the radio task to transmit.
///
/// Sampling rate: ~50 Hz (20ms interval) for responsive control.
#[embassy_executor::task]
pub async fn joystick_task(mut saadc: Saadc<'static, 2>) {
  info!("Joystick task started (50 Hz sampling)");

  let mut state = JoystickState::new();
  let mut buf = [0i16; 2];
  let mut prev_motion = MotionPayload::stop();

  loop {
    // Sample both channels simultaneously
    saadc.sample(&mut buf).await;

    // SAADC returns signed values; for single-ended config on nRF52833
    // the range is 0..+VDD (mapped to 0..1023 at 10-bit resolution).
    // Clamp negative noise to 0.
    let raw_y = buf[0].clamp(0, ADC_MAX);
    let raw_x = buf[1].clamp(0, ADC_MAX);

    trace!("Joystick raw: x={}, y={}", raw_x, raw_y);

    // Process through dead zone + smoothing filter
    let motion = state.update(raw_x, raw_y);

    // Only publish samples while the joystick is the active velocity
    // source. When inactive we reset the smoothing state so the next
    // "first sample" after reactivation is honest, and we don't push
    // anything so the tilt source isn't overridden.
    if mode::current() == InputMode::Joystick {
      if motion != prev_motion {
        trace!("Joystick motion: vx={}, vy={}", motion.vx, motion.vy);
        JOYSTICK_MOTION_CHANNEL.send(motion).await;
        prev_motion = motion;
      }
    } else {
      state.reset();
      // Sentinel that can never equal a real `MotionPayload`, ensuring
      // the very next sample after re-activation is forwarded.
      prev_motion = MotionPayload {
        vx: i8::MIN,
        vy: i8::MIN,
        omega: 0,
      };
    }

    // 50 Hz sampling interval
    embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
  }
}
