//! WS2812 / NeoPixel RGB LED strip driver (bit-bang on P16).
//!
//! Drives a chain of 4 WS2812-compatible RGB LEDs connected to the
//! micro:bit v2 edge connector pin P16 (= nRF52833 `P1_02`). The
//! protocol is single-wire, return-to-zero, with strict ~1.25 µs bit
//! cells; we generate the waveform by toggling the GPIO inside a
//! critical section and busy-waiting via inline assembly so that
//! interrupt latency cannot stretch a bit slot.
//!
//! # Wire format
//!
//! Each LED consumes 24 bits in `G R B` order, MSB first. A logical
//! `1` is encoded as ~0.8 µs high + ~0.45 µs low; a logical `0` as
//! ~0.4 µs high + ~0.85 µs low. After all data has been clocked out
//! the line must stay low for at least 50 µs to latch the new frame.
//!
//! # Hardware notes
//!
//! - The micro:bit edge connector is 3.3 V, while WS2812 chips
//!   typically expect a 5 V VDD and 0.7·VDD logic high (≈ 3.5 V).
//!   In practice 3.3 V usually works for short chains, but a level
//!   shifter or a WS2812B variant powered at 3.3 V is recommended for
//!   reliable operation.
//! - With only 4 LEDs the total transmission takes
//!   `4 × 24 × 1.25 µs ≈ 120 µs`, which is short enough to run inside
//!   a critical section without disturbing radio / motor tasks.
//!
//! # Example
//!
//! ```ignore
//! let mut strip = rgb::init(p.P1_02);
//! strip.set_all(rgb::Color::BLUE);
//! strip.show();
//! ```

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{Peri, peripherals};

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

/// 4-LED WS2812 RGB strip driver.
///
/// Buffers the requested color for each LED and only flushes the
/// chain when [`RgbStrip::show`] is called, allowing the caller to
/// stage multi-LED updates without intermediate flicker.
pub struct RgbStrip {
  pin: Output<'static>,
  buffer: [Color; LED_COUNT],
}

impl RgbStrip {
  /// Set one LED's color in the buffer. Out-of-range indices are
  /// silently ignored so the caller doesn't have to bounds-check.
  #[allow(dead_code)] // Per-LED API; main currently only calls `set_all`.
  pub fn set(&mut self, index: usize, color: Color) {
    if index >= LED_COUNT {
      return;
    }
    self.buffer[index] = color;
  }

  /// Fill every LED with the same color.
  pub fn set_all(&mut self, color: Color) {
    for c in self.buffer.iter_mut() {
      *c = color;
    }
  }

  /// Convenience: turn every LED off (does not auto-flush).
  pub fn clear(&mut self) {
    self.set_all(Color::BLACK);
  }

  /// Push the buffered colors out to the chain.
  ///
  /// Runs inside a critical section to keep bit timing within spec
  /// (interrupt latency on this MCU can easily exceed a single 0.4 µs
  /// bit half-period). After the data burst we hold the line low for
  /// > 50 µs so the LEDs latch the new frame.
  pub fn show(&mut self) {
    // Disable all interrupts via PRIMASK (CPSID i) to guarantee
    // uninterrupted bit timing. `cortex_m::interrupt::free` does
    // exactly this on thumbv7em targets.
    cortex_m::interrupt::free(|_| {
      for color in self.buffer.iter() {
        // WS2812 expects G, R, B in that order, MSB first.
        write_byte(&mut self.pin, color.g);
        write_byte(&mut self.pin, color.r);
        write_byte(&mut self.pin, color.b);
      }
    });
    // Reset / latch: line must stay low for >= 50 µs. We use 80 µs to
    // leave generous margin against clock jitter.
    self.pin.set_low();
    delay_ns(80_000);
  }
}

/// Initialize the RGB strip driver on the given GPIO pin.
///
/// The pin is configured as a high-drive push-pull output starting
/// low so the LEDs see a stable idle level before the first frame.
pub fn init(pin: Peri<'static, peripherals::P1_02>) -> RgbStrip {
  // `HighDrive0Standard1` would be subtly wrong for WS2812 (which
  // wants strong drive in *both* directions to fight line capacitance);
  // `Standard` is the safe default and works well for 4 LEDs at 30 cm.
  let pin = Output::new(pin, Level::Low, OutputDrive::Standard);
  let strip = RgbStrip {
    pin,
    buffer: [Color::BLACK; LED_COUNT],
  };
  info!("RGB strip initialized on P16 (P1_02), {} LEDs", LED_COUNT);
  strip
}

// ---------------------------------------------------------------------
// Low-level bit-bang helpers.
//
// The nRF52833 runs at 64 MHz, so one CPU cycle is ~15.625 ns.
// We use a custom inline assembly delay loop (`subs + bne`) that
// gives exact cycle-level control regardless of compiler optimization
// level. This is critical because:
//   1. A naive `for _ in 0..n { nop() }` loop can be reordered or
//      partially unrolled by LLVM at opt-level ≥ 2.
//   2. `cortex_m::asm::delay` has a conservative 1.5× overhead on M4
//      (it divides by 2 targeting M7's 2-cycle loop, but M4 actually
//      takes 2 cycles per iteration too, plus the +1 bias).
//
// The exact timing values below were tuned against the published
// WS2812B timing spec (T0H 0.20–0.50 µs, T1H 0.55–0.85 µs,
// total cell ≥ 1.25 µs) and include the GPIO toggle latency
// (~2–3 cycles for a peripheral store on Cortex-M4).
// ---------------------------------------------------------------------

/// CPU frequency in MHz.
const CPU_MHZ: u32 = 64;

/// Busy-wait for approximately `ns` nanoseconds using inline assembly.
///
/// On Cortex-M4 (nRF52833 @ 64 MHz) the inner loop is:
///   ```asm
///   1: subs r0, #1    // 1 cycle
///      bne  1b        // 1 cycle (taken) or 1 cycle (not-taken on exit)
///   ```
/// Each taken iteration costs exactly **2 cycles** on Cortex-M4 (no
/// pipeline refill penalty for short backward branches on M4). The
/// final not-taken iteration also costs 1+1 = 2 cycles.
///
/// So `iterations` loop iterations ≈ `iterations × 2` CPU cycles.
/// We compute: iterations = target_cycles / 2 (rounded up).
///
/// Unlike `cortex_m::asm::delay` which conservatively divides by 2
/// *and* adds 1 (designed for superscalar M7), we control the exact
/// factor here for M4-accurate timing.
#[inline(always)]
fn delay_ns(ns: u32) {
  // target_cycles = ns × 64 / 1000
  let target_cycles = (ns * CPU_MHZ).div_ceil(1000);
  // Each loop iteration = 2 cycles on Cortex-M4.
  let iterations = target_cycles / 2;
  if iterations > 0 {
    unsafe {
      core::arch::asm!(
        "1:",
        "subs {0}, #1",
        "bne 1b",
        inout(reg) iterations => _,
        options(nomem, nostack),
      );
    }
  }
}

/// Clock one byte out, MSB first.
#[inline(always)]
fn write_byte(pin: &mut Output<'static>, byte: u8) {
  // Manual unroll keeps the per-bit critical path tight; a `for` loop
  // with a runtime mask noticeably stretches T0H past the 0.5 µs limit
  // on debug builds.
  write_bit(pin, byte & 0x80 != 0);
  write_bit(pin, byte & 0x40 != 0);
  write_bit(pin, byte & 0x20 != 0);
  write_bit(pin, byte & 0x10 != 0);
  write_bit(pin, byte & 0x08 != 0);
  write_bit(pin, byte & 0x04 != 0);
  write_bit(pin, byte & 0x02 != 0);
  write_bit(pin, byte & 0x01 != 0);
}

/// Clock one bit out using the WS2812 NRZ encoding.
///
/// WS2812B datasheet timing (typical, ±150 ns tolerance):
///   - T1H: 0.80 µs high  →  T1L: 0.45 µs low
///   - T0H: 0.40 µs high  →  T0L: 0.85 µs low
///   - Total bit cell ≈ 1.25 µs
///
/// The GPIO peripheral write on nRF52833 takes ~2–3 cycles (~47 ns).
/// The `delay_ns` values below are trimmed to account for this
/// overhead so the *total* pulse width lands within spec.
#[inline(always)]
fn write_bit(pin: &mut Output<'static>, bit: bool) {
  if bit {
    // T1H ≈ 0.80 µs high, T1L ≈ 0.45 µs low.
    pin.set_high();
    delay_ns(700);
    pin.set_low();
    delay_ns(350);
  } else {
    // T0H ≈ 0.40 µs high, T0L ≈ 0.85 µs low.
    pin.set_high();
    delay_ns(300);
    pin.set_low();
    delay_ns(750);
  }
}
