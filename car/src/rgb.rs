//! WS2812 / NeoPixel RGB LED strip driver (PWM + EasyDMA on P16).
//!
//! Drives a chain of 4 WS2812-compatible RGB LEDs connected to the
//! micro:bit v2 edge connector pin P16 (= nRF52833 `P1_02`).
//!
//! Uses the nRF52833's PWM peripheral with EasyDMA to generate the
//! strict 800 kHz WS2812 timing entirely in hardware — no CPU
//! bit-banging required. This approach is immune to compiler
//! optimization levels, interrupt latency, and flash wait states.
//!
//! # Timing
//!
//! With `prescaler = Div1` the PWM base clock is 16 MHz. We pick
//! `max_duty = 20` so each PWM period is `20 / 16 MHz = 1.25 µs`,
//! exactly one WS2812 bit slot. Inside that slot:
//!
//! * `T0H` = 5 ticks ≈ 0.31 µs (datasheet window 0.20–0.50 µs)
//! * `T1H` = 13 ticks ≈ 0.81 µs (datasheet window 0.65–0.95 µs)
//!
//! # Wire format
//!
//! Each LED consumes 24 bits in `G R B` order, MSB first. After all
//! data has been clocked out the line must stay low for at least 50 µs
//! to latch the new frame.
//!
//! # Hardware notes
//!
//! - The micro:bit edge connector is 3.3 V, while WS2812 chips
//!   typically expect a 5 V VDD and 0.7·VDD logic high (≈ 3.5 V).
//!   In practice 3.3 V usually works for short chains, but a level
//!   shifter or a WS2812B variant powered at 3.3 V is recommended for
//!   reliable operation.
//! - With only 4 LEDs the total transmission takes
//!   `4 × 24 × 1.25 µs ≈ 120 µs`, which is negligible.
//!
//! # Example
//!
//! ```ignore
//! let mut strip = rgb::RgbStrip::new(p.PWM0, p.P1_02);
//! strip.set_all(rgb::Color::BLUE);
//! strip.show().await;
//! ```

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Level, OutputDrive};
use embassy_nrf::peripherals;
use embassy_nrf::pwm::{
  Config as PwmConfig, Prescaler, SequenceConfig, SequenceLoad, SequencePwm, SingleSequenceMode,
  SingleSequencer,
};
use embassy_time::{Duration, Timer};

use defmt::info;

/// Number of LEDs in the chain. Hard-coded to match the on-car wiring.
pub const LED_COUNT: usize = 4;

/// 24-bit RGB color.
///
/// Stored in the natural `r, g, b` order; the driver re-orders the
/// bytes into `G R B` when serializing on the wire.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Color {
  pub r: u8,
  pub g: u8,
  pub b: u8,
}

impl Color {
  /// Black / off.
  pub const BLACK: Color = Color::new(0, 0, 0);
  /// Pure red at full brightness.
  pub const RED: Color = Color::new(255, 0, 0);
  /// Pure green at full brightness.
  pub const GREEN: Color = Color::new(0, 255, 0);
  /// Pure blue at full brightness.
  pub const BLUE: Color = Color::new(0, 0, 255);
  /// White (all channels full).
  #[allow(dead_code)] // Part of the public color palette; kept for callers.
  pub const WHITE: Color = Color::new(255, 255, 255);
  /// Yellow (R + G).
  pub const YELLOW: Color = Color::new(255, 255, 0);
  /// Cyan (G + B).
  pub const CYAN: Color = Color::new(0, 255, 255);
  /// Magenta (R + B).
  pub const MAGENTA: Color = Color::new(255, 0, 255);

  /// Build a [`Color`] from raw 8-bit channels.
  pub const fn new(r: u8, g: u8, b: u8) -> Self {
    Self { r, g, b }
  }
}

// --- PWM timing constants -------------------------------------------

/// PWM TOP value: 20 ticks at 16 MHz = 1.25 µs per bit slot.
const TOP_TICKS: u16 = 20;
/// Duty ticks for a logical `0` bit (T0H ≈ 0.31 µs).
const T0H_TICKS: u16 = 5;
/// Duty ticks for a logical `1` bit (T1H ≈ 0.81 µs).
const T1H_TICKS: u16 = 13;

/// PWM duty word for a logical `0` bit.
///
/// Bit 15 is the polarity flag: setting it makes the line high while
/// `counter < duty`, which is the polarity WS2812 wants (a short high
/// pulse at the start of every bit slot).
const WORD_T0: u16 = 0x8000 | T0H_TICKS;
/// PWM duty word for a logical `1` bit.
const WORD_T1: u16 = 0x8000 | T1H_TICKS;

/// 24 bits per LED (G, R, B).
const BITS_PER_LED: usize = 24;
/// Total PWM word count for one full frame.
const FRAME_WORDS: usize = LED_COUNT * BITS_PER_LED;

/// WS2812 latch / reset window, datasheet says ≥ 50 µs. We use 80 µs
/// for generous margin.
const RESET_US: u64 = 80;
/// Approximate time per bit in µs (rounded up for safety).
const BIT_PERIOD_US: u64 = 2;
/// Total frame busy time: data + latch.
const FRAME_BUSY_US: u64 = (FRAME_WORDS as u64) * BIT_PERIOD_US + RESET_US;

// --- Driver ---------------------------------------------------------

/// 4-LED WS2812 RGB strip driver using PWM + EasyDMA.
///
/// Buffers the requested color for each LED and only flushes the
/// chain when [`RgbStrip::show`] is called, allowing the caller to
/// stage multi-LED updates without intermediate flicker.
pub struct RgbStrip {
  pwm: SequencePwm<'static>,
  buf: [u16; FRAME_WORDS],
  colors: [Color; LED_COUNT],
}

impl RgbStrip {
  /// Create a new RGB strip driver on PWM0 / P1_02.
  ///
  /// Configures the PWM peripheral for WS2812 timing and starts with
  /// all LEDs off.
  pub fn new(
    pwm: Peri<'static, peripherals::PWM0>,
    pin: Peri<'static, peripherals::P1_02>,
  ) -> Self {
    let mut config = PwmConfig::default();
    config.prescaler = Prescaler::Div1;
    config.max_duty = TOP_TICKS;
    config.sequence_load = SequenceLoad::Common;
    // High-drive output keeps the WS2812 logic threshold solidly met
    // even with a few cm of jumper cable.
    config.ch0_drive = OutputDrive::HighDrive;
    config.ch0_idle_level = Level::Low;

    let pwm = SequencePwm::new_1ch(pwm, pin, config)
      .expect("SequencePwm::new_1ch failed (P1_02 / PWM0)");

    info!("RGB strip initialized on P16 (P1_02), {} LEDs via PWM0", LED_COUNT);

    Self {
      pwm,
      buf: [WORD_T0; FRAME_WORDS],
      colors: [Color::BLACK; LED_COUNT],
    }
  }

  /// Set one LED's color in the buffer. Out-of-range indices are
  /// silently ignored so the caller doesn't have to bounds-check.
  #[allow(dead_code)] // Per-LED API; main currently only calls `set_all`.
  pub fn set(&mut self, index: usize, color: Color) {
    if index >= LED_COUNT {
      return;
    }
    self.colors[index] = color;
  }

  /// Fill every LED with the same color.
  pub fn set_all(&mut self, color: Color) {
    for c in self.colors.iter_mut() {
      *c = color;
    }
  }

  /// Convenience: turn every LED off (does not auto-flush).
  pub fn clear(&mut self) {
    self.set_all(Color::BLACK);
  }

  /// Push the buffered colors out to the chain via DMA.
  ///
  /// This encodes the color buffer into PWM duty words and kicks off
  /// a hardware DMA transfer. The function awaits until the transfer
  /// completes and the WS2812 latch window has elapsed.
  pub async fn show(&mut self) {
    // Encode colors into PWM buffer.
    for (led_idx, color) in self.colors.iter().enumerate() {
      let base = led_idx * BITS_PER_LED;
      write_byte(&mut self.buf[base..base + 8], color.g);
      write_byte(&mut self.buf[base + 8..base + 16], color.r);
      write_byte(&mut self.buf[base + 16..base + 24], color.b);
    }

    // Start DMA transfer.
    let cfg = SequenceConfig::default();
    let sequencer = SingleSequencer::new(&mut self.pwm, &self.buf, cfg);
    // Ignore errors — if PWM fails there's nothing we can do.
    let _ = sequencer.start(SingleSequenceMode::Times(1));

    // Wait for the frame to finish plus the WS2812 latch window.
    Timer::after(Duration::from_micros(FRAME_BUSY_US)).await;
  }
}

/// Convert one colour byte to 8 PWM duty words, MSB first.
fn write_byte(slot: &mut [u16], byte: u8) {
  for (i, word) in slot.iter_mut().enumerate().take(8) {
    let bit_high = (byte >> (7 - i)) & 1 != 0;
    *word = if bit_high { WORD_T1 } else { WORD_T0 };
  }
}
