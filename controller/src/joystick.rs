//! Joystick input module for the controller firmware.
//!
//! Reads analog joystick values from two axes using the nRF52833 SAADC:
//! - Y-axis: P1 (AIN2) — forward/backward (up=1023, down=0)
//! - X-axis: P2 (AIN3) — left/right (right=1023, left=0)
//!
//! Applies dead zone filtering and exponential moving average (EMA) smoothing
//! to produce stable MotionPayload values for Mecanum wheel control.

use embassy_nrf::bind_interrupts;
use embassy_nrf::saadc::{ChannelConfig, Config, InterruptHandler, Saadc};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use defmt::{info, trace};
use protocol::MotionPayload;

// Bind SAADC interrupt to the handler
bind_interrupts!(struct Irqs {
  SAADC => InterruptHandler;
});

// --- Configuration Constants ---

/// ADC resolution: 10-bit (0..1023)
const ADC_MAX: i16 = 1023;

/// ADC midpoint (joystick center position)
const ADC_MID: i16 = ADC_MAX / 2; // 511

/// Dead zone threshold (±DEAD_ZONE around center is treated as zero)
/// ~5% of half-range to filter out joystick drift and noise
const DEAD_ZONE: i16 = 30;

/// EMA smoothing factor numerator (alpha = SMOOTH_NUM / SMOOTH_DEN)
/// Higher value = less smoothing, faster response
/// Lower value = more smoothing, slower response
const SMOOTH_NUM: i32 = 3;

/// EMA smoothing factor denominator
const SMOOTH_DEN: i32 = 8;

/// Maximum output magnitude for MotionPayload fields
const OUTPUT_MAX: i32 = 100;

/// Half-range of ADC (from center to max/min)
const HALF_RANGE: i16 = ADC_MID;

/// Channel for sending joystick-derived motion commands to the radio task
pub static JOYSTICK_MOTION_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> =
  Channel::new();

/// Joystick state with smoothing filters
struct JoystickState {
  /// Smoothed X-axis value (scaled to -100..+100 range)
  smooth_x: i32,
  /// Smoothed Y-axis value (scaled to -100..+100 range)
  smooth_y: i32,
}

impl JoystickState {
  const fn new() -> Self {
    Self {
      smooth_x: 0,
      smooth_y: 0,
    }
  }

  /// Apply EMA smoothing: output = alpha * new_value + (1 - alpha) * old_output
  /// Uses integer arithmetic to avoid floating point on embedded target.
  fn update(&mut self, raw_x: i16, raw_y: i16) -> MotionPayload {
    // Convert raw ADC (0..1023) to centered value (-511..+512)
    // X-axis: right=1023 -> +, left=0 -> -
    let centered_x = raw_x - ADC_MID;
    // Y-axis: up=1023 -> +, down=0 -> -
    let centered_y = raw_y - ADC_MID;

    // Apply dead zone
    let dx = apply_dead_zone(centered_x);
    let dy = apply_dead_zone(centered_y);

    // Scale to -100..+100 range
    let scaled_x = scale_to_output(dx);
    let scaled_y = scale_to_output(dy);

    // Apply EMA smoothing (fixed-point: multiply by SMOOTH_DEN to keep precision)
    self.smooth_x =
      (SMOOTH_NUM * scaled_x as i32 + (SMOOTH_DEN - SMOOTH_NUM) * self.smooth_x) / SMOOTH_DEN;
    self.smooth_y =
      (SMOOTH_NUM * scaled_y as i32 + (SMOOTH_DEN - SMOOTH_NUM) * self.smooth_y) / SMOOTH_DEN;

    // Clamp final output to valid i8 range
    let vx = self.smooth_y.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i8;
    let vy = self.smooth_x.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i8;

    MotionPayload { vx, vy, omega: 0 }
  }
}

/// Apply dead zone: values within ±DEAD_ZONE of center are treated as zero.
/// Values outside the dead zone are remapped to fill the full range smoothly.
#[inline]
fn apply_dead_zone(value: i16) -> i16 {
  if value.abs() <= DEAD_ZONE {
    return 0;
  }
  // Remap: remove dead zone gap so output starts from 0 at the edge of dead zone
  if value > 0 {
    value - DEAD_ZONE
  } else {
    value + DEAD_ZONE
  }
}

/// Scale a dead-zone-adjusted value to the -100..+100 output range.
/// Input range after dead zone removal: -(HALF_RANGE - DEAD_ZONE)..+(HALF_RANGE - DEAD_ZONE)
#[inline]
fn scale_to_output(value: i16) -> i16 {
  let effective_range = (HALF_RANGE - DEAD_ZONE) as i32;
  if effective_range == 0 {
    return 0;
  }
  let scaled = (value as i32 * OUTPUT_MAX) / effective_range;
  scaled.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i16
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
  let config = Config::default();

  // Channel 0: Y-axis (P1 / AIN2)
  let channel_y = ChannelConfig::single_ended(pin_y);
  // Channel 1: X-axis (P2 / AIN3)
  let channel_x = ChannelConfig::single_ended(pin_x);

  let saadc = Saadc::new(saadc_periph, Irqs, config, [channel_y, channel_x]);

  info!("Joystick SAADC initialized (Y=P1/AIN2, X=P2/AIN3)");
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

    // Only send if the motion has changed (reduces radio traffic)
    if motion != prev_motion {
      trace!("Joystick motion: vx={}, vy={}", motion.vx, motion.vy);
      JOYSTICK_MOTION_CHANNEL.send(motion).await;
      prev_motion = motion;
    }

    // 50 Hz sampling interval
    embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
  }
}
