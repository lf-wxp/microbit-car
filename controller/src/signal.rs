//! Shared signal-processing primitives for analog stick inputs.
//!
//! All controllers in this firmware share the same DSP pipeline:
//!
//! 1. **Center** the raw 10-bit ADC sample around 0 (`raw - ADC_MID`).
//! 2. **Dead zone** small magnitudes around the center to ignore drift.
//! 3. **Scale** the remaining range to a portable `-100..=100` output so
//!    the radio protocol stays hardware-agnostic.
//! 4. **Exponential moving average** smoothing to filter out ADC jitter
//!    without introducing visible latency.
//!
//! Keeping this in one place avoids duplication between
//! [`crate::joystick`] (left stick → vx/vy) and
//! [`crate::joystick_right`] (right stick → omega).

/// 10-bit ADC full-scale value.
pub const ADC_MAX: i16 = 1023;

/// 10-bit ADC mid-point (idle joystick reading).
pub const ADC_MID: i16 = ADC_MAX / 2;

/// Half-range from the center to either extreme.
pub const HALF_RANGE: i16 = ADC_MID;

/// Final magnitude exposed to the protocol layer (`MotionPayload` fields).
pub const OUTPUT_MAX: i32 = 100;

/// Default dead zone (~5% of half-range). Filters out resting-stick drift
/// and ADC noise. Individual call sites may override per axis if needed.
pub const DEFAULT_DEAD_ZONE: i16 = 30;

/// Default EMA numerator. `alpha = SMOOTH_NUM / SMOOTH_DEN = 3/8`.
pub const DEFAULT_SMOOTH_NUM: i32 = 3;

/// Default EMA denominator (also the integer-arithmetic precision).
pub const DEFAULT_SMOOTH_DEN: i32 = 8;

/// Apply dead zone: values within `±dead_zone` of zero collapse to `0`,
/// values outside it are shifted toward zero so the output starts at 0
/// at the edge of the dead zone (no step discontinuity).
#[inline]
pub fn apply_dead_zone(value: i16, dead_zone: i16) -> i16 {
  if value.abs() <= dead_zone {
    return 0;
  }
  if value > 0 {
    value - dead_zone
  } else {
    value + dead_zone
  }
}

/// Linearly scale a dead-zone-adjusted value to the `[-OUTPUT_MAX, OUTPUT_MAX]`
/// range, taking the dead zone into account so the full output range is
/// reachable just past the dead-zone edge.
#[inline]
pub fn scale_to_output(value: i16, dead_zone: i16) -> i16 {
  let effective_range = (HALF_RANGE - dead_zone) as i32;
  if effective_range <= 0 {
    return 0;
  }
  let scaled = (value as i32 * OUTPUT_MAX) / effective_range;
  scaled.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i16
}

/// Single-axis EMA filter using fixed-point integer math (no FPU usage).
///
/// `state` carries the current smoothed value in the same units as the
/// input samples (post-scale: `-100..=100`). `update` returns the new
/// smoothed value.
#[derive(Debug, Clone, Copy)]
pub struct EmaFilter {
  state: i32,
}

impl EmaFilter {
  /// New filter starting at 0 (idle).
  pub const fn new() -> Self {
    Self { state: 0 }
  }

  /// Reset the smoothed state back to zero — useful when the source
  /// becomes inactive (e.g. mode switched away) so the next "first
  /// sample" after reactivation is honest.
  #[inline]
  pub fn reset(&mut self) {
    self.state = 0;
  }

  /// EMA update step: `state = (num * sample + (den - num) * state) / den`.
  ///
  /// When `sample == 0` (joystick centred) the EMA state decays
  /// exponentially but integer division causes it to linger at small
  /// non-zero values like `±3`, `±2`, `±1` before finally reaching
  /// zero.  Each of those intermediate values produces a tiny motor
  /// command that the car hears as audible coil whine or causes a
  /// brief creep after the operator releases the stick.
  ///
  /// The snap-to-zero threshold below is chosen so that any residual
  /// state whose magnitude would produce a PWM duty cycle below the
  /// motor's stiction threshold is eliminated immediately.  With
  /// `SPEED_TO_PWM_SCALE = 40` (motor.rs), a raw value of `±3` maps
  /// to `±120` out of `4095` (~3%), well below any visible motion
  /// but still loud enough to be audible.  We snap anything whose
  /// absolute value is ≤ `SNAP_THRESHOLD` to zero as soon as the
  /// input goes idle.
  const SNAP_THRESHOLD: i32 = 5;

  #[inline]
  pub fn update(&mut self, sample: i32, num: i32, den: i32) -> i32 {
    // Fast path: already idle and no new input → stay at zero.
    if sample == 0 && self.state == 0 {
      return 0;
    }
    // Snap-to-zero: when the input is idle and the residual state
    // is small enough that it represents only noise / decay tail,
    // jump to zero immediately rather than letting it slowly decay
    // through ±5 → ±3 → ±2 → ±1 → 0.
    if sample == 0 && self.state.abs() <= Self::SNAP_THRESHOLD {
      self.state = 0;
      return 0;
    }
    self.state = (num * sample + (den - num) * self.state) / den;
    self.state
  }
}

impl Default for EmaFilter {
  fn default() -> Self {
    Self::new()
  }
}

/// End-to-end single-axis pipeline: raw 10-bit ADC sample → smoothed,
/// clamped `i8` in `[-100, 100]`. Returns the new value; the filter
/// state is updated in place.
#[inline]
pub fn process_axis(raw: i16, filter: &mut EmaFilter, dead_zone: i16) -> i8 {
  let centered = raw.clamp(0, ADC_MAX) - ADC_MID;
  let after_dz = apply_dead_zone(centered, dead_zone);
  let scaled = scale_to_output(after_dz, dead_zone) as i32;
  let smoothed = filter.update(scaled, DEFAULT_SMOOTH_NUM, DEFAULT_SMOOTH_DEN);
  smoothed.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i8
}
