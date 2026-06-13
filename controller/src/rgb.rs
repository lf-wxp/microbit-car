//! WS2812 ("NeoPixel") status indicator on the controller's extension
//! board.
//!
//! Four daisy-chained WS2812 LEDs share a single data line on
//! edge-connector **P8** (= `P0_10`). They are driven by the nRF52833's
//! PWM peripheral with EasyDMA so the strict 800 kHz timing is met
//! without CPU bit-banging.
//!
//! # Current responsibility
//!
//! Only **LED0** is actively rendered at the moment, and it shows the
//! radio link state:
//!
//! | State          | Colour          |
//! |----------------|-----------------|
//! | `Connecting`   | breathing amber |
//! | `Connected`    | dim green       |
//! | `Disconnected` | dim red         |
//!
//! LEDs 1–3 are kept blank but reserved for future use (input mode,
//! omega direction, motion magnitude, …). The driver always shifts out
//! all four LEDs because WS2812 is a daisy-chained protocol — there is
//! no way to address only the first pixel — so adding a new indicator
//! later is a pure render-side change.
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
//! The high bit (`0x8000`) of each duty word is set so the PWM line is
//! driven high for the configured count and then pulled low for the
//! rest of the period — matching the WS2812 pulse polarity.
//!
//! After every frame we hold the line low for ≥ 60 µs (`RESET_US`) to
//! latch the data into the LEDs.

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Level, OutputDrive};
use embassy_nrf::peripherals;
use embassy_nrf::pwm::{
  Config as PwmConfig, Prescaler, SequenceConfig, SequenceLoad, SequencePwm, SingleSequenceMode,
  SingleSequencer,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use defmt::{info, warn};

/// Radio-link health as observed by the controller.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum ConnectionState {
  /// We have started the radio but haven't yet seen a heartbeat reply
  /// from the car.
  Connecting,
  /// At least one heartbeat reply has been received within
  /// `radio::HEARTBEAT_TIMEOUT_MS`.
  Connected,
  /// No heartbeat reply within the timeout window.
  Disconnected,
}

/// Aggregate state consumed by the renderer. Today only `connection`
/// drives a pixel; the other fields are placeholders that other
/// subsystems can populate later without touching the driver.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RgbState {
  /// Radio link state, rendered on LED0.
  pub connection: ConnectionState,
  // Reserved: input mode, omega direction, motion magnitude, …
  // Add fields here and extend `render` when new indicators are wired.
}

impl RgbState {
  /// Initial state used before any signal arrives.
  pub const INITIAL: Self = Self {
    connection: ConnectionState::Connecting,
  };
}

/// Latest-value channel for RGB state. `Signal` collapses bursts so
/// the renderer only ever sees the freshest value; producers never
/// block.
pub static RGB_STATE: Signal<CriticalSectionRawMutex, RgbState> = Signal::new();

/// 24-bit GRB colour (the order WS2812 actually expects on the wire).
#[derive(Clone, Copy, Debug)]
struct Rgb {
  r: u8,
  g: u8,
  b: u8,
}

impl Rgb {
  const OFF: Self = Self::new(0, 0, 0);

  /// Dim green: comfortable as a "good" indicator at very close range.
  const GREEN_DIM: Self = Self::new(0, 12, 0);

  /// Dim red: comfortable as a "bad" indicator at very close range.
  const RED_DIM: Self = Self::new(12, 0, 0);

  const fn new(r: u8, g: u8, b: u8) -> Self {
    Self { r, g, b }
  }

  /// Amber tone with a configurable brightness, used for the
  /// `Connecting` breathing animation.
  const fn amber(brightness: u8) -> Self {
    // Slightly biased towards red so the colour reads as warm amber
    // rather than yellow-green.
    Self::new(brightness, brightness / 2, 0)
  }
}

// --- Bitstream layout -----------------------------------------------

/// Number of LEDs in the daisy chain on the extension board.
const NUM_LEDS: usize = 4;
/// 24 bits per LED (G, R, B).
const BITS_PER_LED: usize = 24;
/// 1.25 µs per bit at 16 MHz / Div1 / TOP=20.
const BIT_PERIOD_US: u32 = 2; // round up to be safe
/// WS2812 latch / reset window, datasheet says ≥ 50 µs.
const RESET_US: u64 = 80;

const TOP_TICKS: u16 = 20;
const T0H_TICKS: u16 = 5;
const T1H_TICKS: u16 = 13;

/// PWM duty word for a logical `0` bit.
///
/// Bit 15 is the polarity flag: setting it makes the line high while
/// `counter < duty`, which is the polarity WS2812 wants (a short high
/// pulse at the start of every bit slot).
const WORD_T0: u16 = 0x8000 | T0H_TICKS;
/// PWM duty word for a logical `1` bit.
const WORD_T1: u16 = 0x8000 | T1H_TICKS;

/// Total PWM word count for one full frame.
const FRAME_WORDS: usize = NUM_LEDS * BITS_PER_LED;

/// Approximate time it takes to shift one frame out, plus the WS2812
/// latch window. Used as a "DMA done" wait so the next frame doesn't
/// trample an in-flight transfer.
const FRAME_BUSY_US: u64 = (FRAME_WORDS as u64) * (BIT_PERIOD_US as u64) + RESET_US;

/// Owns the PWM peripheral and the DMA-visible bit buffer.
///
/// The buffer lives inside the struct so its lifetime tracks the
/// driver's; `Ws2812::write` borrows it mutably for the duration of
/// each transfer.
pub struct Ws2812 {
  pwm: SequencePwm<'static>,
  buf: [u16; FRAME_WORDS],
}

impl Ws2812 {
  /// Configure `PWM0` to drive `data_pin` as a single-channel WS2812
  /// stream.
  pub fn new(
    pwm: Peri<'static, peripherals::PWM0>,
    data_pin: Peri<'static, peripherals::P0_10>,
  ) -> Self {
    let mut config = PwmConfig::default();
    config.prescaler = Prescaler::Div1;
    config.max_duty = TOP_TICKS;
    config.sequence_load = SequenceLoad::Common;
    // High-drive output keeps the WS2812 logic threshold solidly met
    // even with a few cm of jumper cable.
    config.ch0_drive = OutputDrive::HighDrive;
    config.ch0_idle_level = Level::Low;

    let pwm = SequencePwm::new_1ch(pwm, data_pin, config)
      .expect("SequencePwm::new_1ch failed (P0_10 / PWM0)");

    Self {
      pwm,
      buf: [WORD_T0; FRAME_WORDS],
    }
  }

  /// Encode the four pixel colours into the bit buffer and kick off a
  /// DMA-driven transmission.
  ///
  /// The function returns once the line has been idle long enough for
  /// the WS2812 chain to latch the new values, so callers can reuse
  /// `self` immediately afterwards.
  ///
  /// Module-private because `Rgb` is internal and the only legitimate
  /// caller is `rgb_task` below.
  async fn write(&mut self, pixels: &[Rgb; NUM_LEDS]) {
    encode_pixels(pixels, &mut self.buf);

    let cfg = SequenceConfig::default();
    let sequencer = SingleSequencer::new(&mut self.pwm, &self.buf, cfg);
    if let Err(e) = sequencer.start(SingleSequenceMode::Times(1)) {
      warn!("WS2812 PWM start failed: {:?}", defmt::Debug2Format(&e));
      return;
    }

    // DMA runs autonomously; just wait long enough for the frame plus
    // the WS2812 latch window, then drop the sequencer so the buffer
    // can be reused.
    Timer::after(Duration::from_micros(FRAME_BUSY_US)).await;
  }
}

/// Serialise the four colours into the PWM word buffer in WS2812
/// wire order (G, R, B; MSB first per byte).
fn encode_pixels(pixels: &[Rgb; NUM_LEDS], buf: &mut [u16; FRAME_WORDS]) {
  for (led_idx, px) in pixels.iter().enumerate() {
    let base = led_idx * BITS_PER_LED;
    write_byte(&mut buf[base..base + 8], px.g);
    write_byte(&mut buf[base + 8..base + 16], px.r);
    write_byte(&mut buf[base + 16..base + 24], px.b);
  }
}

/// Convert one colour byte to 8 PWM duty words, MSB first.
fn write_byte(slot: &mut [u16], byte: u8) {
  for i in 0..8 {
    let bit_high = (byte >> (7 - i)) & 1 != 0;
    slot[i] = if bit_high { WORD_T1 } else { WORD_T0 };
  }
}

// --- Rendering ------------------------------------------------------

/// Translate a high-level [`RgbState`] into the four pixel colours we
/// actually push down the wire.
///
/// `phase` is a free-running counter incremented on every animation
/// tick; it drives the breathing animation for the `Connecting` state.
/// The other LEDs stay [`Rgb::OFF`] until new indicators are added.
fn render(state: &RgbState, phase: u8) -> [Rgb; NUM_LEDS] {
  let led0 = match state.connection {
    ConnectionState::Connected => Rgb::GREEN_DIM,
    ConnectionState::Disconnected => Rgb::RED_DIM,
    ConnectionState::Connecting => Rgb::amber(breathing_brightness(phase)),
  };
  [led0, Rgb::OFF, Rgb::OFF, Rgb::OFF]
}

/// Triangular brightness ramp in `[2, 18]`. Avoids float math and
/// keeps the LED comfortably visible without being painful to look
/// at.
fn breathing_brightness(phase: u8) -> u8 {
  let p = phase % 32;
  let level = if p < 16 { p } else { 31 - p };
  // level ∈ [0, 15] -> map to [2, 18] linearly.
  2 + level
}

// --- Task -----------------------------------------------------------

/// Animation tick interval. Slow enough that the breathing LED stays
/// gentle, fast enough that a connection state flip feels immediate.
const ANIM_TICK: Duration = Duration::from_millis(60);

/// Driver task: blocks on either a state change or the next animation
/// tick, then re-renders all four LEDs.
#[embassy_executor::task]
pub async fn rgb_task(mut driver: Ws2812) {
  info!("RGB task started (4x WS2812 on P0_10)");

  let mut state = RgbState::INITIAL;
  let mut phase: u8 = 0;

  // Push an initial frame so the LEDs reflect `Connecting` immediately
  // after boot rather than whatever they powered up showing.
  driver.write(&render(&state, phase)).await;

  let mut next_tick = Instant::now() + ANIM_TICK;
  loop {
    // Wait for whichever happens first: a fresh state, or the animation
    // deadline. Using `select` keeps the task responsive without busy
    // polling.
    match embassy_futures::select::select(
      RGB_STATE.wait(),
      Timer::at(next_tick),
    )
    .await
    {
      embassy_futures::select::Either::First(new_state) => {
        state = new_state;
      }
      embassy_futures::select::Either::Second(()) => {
        next_tick = Instant::now() + ANIM_TICK;
        phase = phase.wrapping_add(1);
      }
    }

    driver.write(&render(&state, phase)).await;
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn breathing_ramps_up_and_down() {
    // Should never exceed 18 or drop below 2 across a full cycle.
    for p in 0..64u8 {
      let b = breathing_brightness(p);
      assert!(b >= 2);
      assert!(b <= 18);
    }
    // Symmetric around the midpoint.
    assert_eq!(breathing_brightness(0), breathing_brightness(31));
    assert_eq!(breathing_brightness(15), breathing_brightness(16));
  }

  #[test]
  fn connected_renders_green_only() {
    let frame = render(
      &RgbState {
        connection: ConnectionState::Connected,
      },
      0,
    );
    assert_eq!(frame[0].g, 12);
    assert_eq!(frame[0].r, 0);
    for px in &frame[1..] {
      assert_eq!(px.r, 0);
      assert_eq!(px.g, 0);
      assert_eq!(px.b, 0);
    }
  }

  #[test]
  fn disconnected_renders_red_only() {
    let frame = render(
      &RgbState {
        connection: ConnectionState::Disconnected,
      },
      0,
    );
    assert_eq!(frame[0].r, 12);
    assert_eq!(frame[0].g, 0);
  }

  #[test]
  fn encode_first_led_g_msb() {
    // G byte's MSB on LED0 is buf[0]: green = 0x80 -> first word T1.
    let pixels = [
      Rgb::new(0, 0x80, 0),
      Rgb::OFF,
      Rgb::OFF,
      Rgb::OFF,
    ];
    let mut buf = [0u16; FRAME_WORDS];
    encode_pixels(&pixels, &mut buf);
    assert_eq!(buf[0], WORD_T1);
    assert_eq!(buf[1], WORD_T0);
  }
}
