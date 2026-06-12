//! Tilt-to-motion input using the on-board LSM303AGR accelerometer.
//!
//! micro:bit v2 has the accelerometer wired to the internal I2C bus
//! (`SCL = P0_08`, `SDA = P0_16`). Holding the board with the
//! micro:bit logo facing up:
//!
//! - Tilt forward (logo tips away from you) -> car moves forward (+vx)
//! - Tilt backward                          -> car moves backward (-vx)
//! - Tilt right                             -> car strafes right (+vy)
//! - Tilt left                              -> car strafes left  (-vy)
//!
//! Output goes to [`TILT_MOTION_CHANNEL`] only while the active input
//! mode is [`mode::InputMode::Tilt`], so it never fights the joystick.
//! Smoothing and dead zone behave the same as the joystick path so the
//! car feels consistent across input modes.

use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use defmt::{error, info, trace};
use protocol::MotionPayload;

use crate::mode::{self, InputMode};

bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// --- LSM303AGR accelerometer ---

/// 7-bit I2C address of the accelerometer block inside LSM303AGR.
const ACC_ADDR: u8 = 0x19;

/// `WHO_AM_I_A` register, expected to read back `0x33`.
const REG_WHO_AM_I: u8 = 0x0F;
const WHO_AM_I_VALUE: u8 = 0x33;

/// Control register 1: ODR + low-power + axis enables.
/// `0x57` = 100 Hz ODR, normal-mode, X/Y/Z enabled.
const REG_CTRL_REG1: u8 = 0x20;
const CTRL_REG1_VALUE: u8 = 0x57;

/// Control register 4: scale + high-resolution.
/// `0x00` = ±2 g full scale, 10-bit normal mode (BDU off).
const REG_CTRL_REG4: u8 = 0x23;
const CTRL_REG4_VALUE: u8 = 0x00;

/// First byte of the X/Y/Z output block (`OUT_X_L_A`).
/// We OR `0x80` into the register address to enable auto-increment so
/// a single multi-byte read returns all six axis bytes.
const REG_OUT_X_L: u8 = 0x28 | 0x80;

// --- Tilt-to-velocity mapping ---

/// Output velocity range matches the rest of the protocol (-100..+100).
const OUTPUT_MAX: i32 = 100;

/// Raw accelerometer counts (10-bit left-justified into i16) considered
/// to be "still". Empirically about ±0.05 g of jitter at rest.
const DEAD_ZONE: i16 = 800;

/// Raw counts that map to full scale output. Roughly ±0.5 g of tilt
/// (≈ 30°) is treated as "full speed", which feels comfortable to hold
/// without the user having to invert the board.
const FULL_TILT: i16 = 8000;

/// EMA smoothing factor numerator/denominator (alpha = 3 / 8).
const SMOOTH_NUM: i32 = 3;
const SMOOTH_DEN: i32 = 8;

/// Channel for tilt-derived motion samples consumed by the fusion loop.
pub static TILT_MOTION_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> = Channel::new();

/// Initialize TWIM0 for the on-board I2C bus.
///
/// embassy-nrf 0.10 requires a caller-supplied DMA buffer placed in RAM
/// because EasyDMA cannot read directly from flash; we allocate one
/// statically here so it lives for `'static`.
pub fn init(
  twim_periph: Peri<'static, peripherals::TWISPI0>,
  pin_sda: Peri<'static, peripherals::P0_16>,
  pin_scl: Peri<'static, peripherals::P0_08>,
) -> Twim<'static> {
  // 16 bytes is more than enough for our largest transfer (6-byte XYZ
  // read with a 1-byte register address prefix).
  static RAM_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let ram_buf = RAM_BUF.init([0u8; 16]);

  let mut config = twim::Config::default();
  config.frequency = twim::Frequency::K100;
  Twim::new(twim_periph, Irqs, pin_sda, pin_scl, config, ram_buf)
}

/// Configure the accelerometer: verify WHO_AM_I, then set ODR/scale.
async fn setup_accelerometer(twim: &mut Twim<'static>) -> bool {
  // Read WHO_AM_I first so we can fail fast if the bus is misconfigured.
  let mut who = [0u8; 1];
  if let Err(e) = twim.write_read(ACC_ADDR, &[REG_WHO_AM_I], &mut who).await {
    error!("LSM303AGR WHO_AM_I read failed: {:?}", e);
    return false;
  }
  if who[0] != WHO_AM_I_VALUE {
    error!(
      "LSM303AGR WHO_AM_I mismatch: expected {=u8:#x}, got {=u8:#x}",
      WHO_AM_I_VALUE, who[0]
    );
    return false;
  }

  if let Err(e) = twim
    .write(ACC_ADDR, &[REG_CTRL_REG1, CTRL_REG1_VALUE])
    .await
  {
    error!("LSM303AGR CTRL_REG1 write failed: {:?}", e);
    return false;
  }
  if let Err(e) = twim
    .write(ACC_ADDR, &[REG_CTRL_REG4, CTRL_REG4_VALUE])
    .await
  {
    error!("LSM303AGR CTRL_REG4 write failed: {:?}", e);
    return false;
  }

  info!("LSM303AGR accelerometer configured (100 Hz, +/-2 g)");
  true
}

/// Read X/Y axes (we ignore Z). Each axis is 16-bit little-endian
/// left-justified; the meaningful precision is 10 bits.
async fn read_xy(twim: &mut Twim<'static>) -> Option<(i16, i16)> {
  let mut buf = [0u8; 6];
  if let Err(e) = twim.write_read(ACC_ADDR, &[REG_OUT_X_L], &mut buf).await {
    error!("LSM303AGR XYZ read failed: {:?}", e);
    return None;
  }
  let raw_x = i16::from_le_bytes([buf[0], buf[1]]);
  let raw_y = i16::from_le_bytes([buf[2], buf[3]]);
  Some((raw_x, raw_y))
}

/// Apply dead zone + clamp + linear map to -100..+100.
#[inline]
fn map_axis(raw: i16) -> i32 {
  let abs = raw.unsigned_abs() as i32;
  if abs <= DEAD_ZONE as i32 {
    return 0;
  }
  let effective = (FULL_TILT - DEAD_ZONE) as i32;
  if effective <= 0 {
    return 0;
  }
  let magnitude = ((abs - DEAD_ZONE as i32) * OUTPUT_MAX / effective).min(OUTPUT_MAX);
  if raw < 0 { -magnitude } else { magnitude }
}

/// Tilt sampling task. Always runs at 50 Hz, but only publishes new
/// motion samples while the controller is in [`InputMode::Tilt`].
#[embassy_executor::task]
pub async fn tilt_task(mut twim: Twim<'static>) {
  info!("Tilt task started (50 Hz sampling)");

  if !setup_accelerometer(&mut twim).await {
    error!("Tilt task aborting: accelerometer not responding");
    return;
  }

  let mut smooth_x: i32 = 0;
  let mut smooth_y: i32 = 0;
  let mut last_sent = MotionPayload::stop();

  loop {
    Timer::after(Duration::from_millis(20)).await;

    // When the user switches away from Tilt, drop any residual motion
    // so the joystick (or the next switch back) starts cleanly.
    if mode::current() != InputMode::Tilt {
      if last_sent != MotionPayload::stop() {
        smooth_x = 0;
        smooth_y = 0;
        last_sent = MotionPayload::stop();
      }
      continue;
    }

    let Some((raw_ax, raw_ay)) = read_xy(&mut twim).await else {
      continue;
    };

    // Accelerometer axis convention on the micro:bit (logo up):
    //   tilt right     -> +X
    //   tilt forward   -> -Y  (we want +vx for forward, so invert Y)
    let scaled_vx = map_axis(-raw_ay);
    let scaled_vy = map_axis(raw_ax);

    smooth_x = (SMOOTH_NUM * scaled_vy + (SMOOTH_DEN - SMOOTH_NUM) * smooth_x) / SMOOTH_DEN;
    smooth_y = (SMOOTH_NUM * scaled_vx + (SMOOTH_DEN - SMOOTH_NUM) * smooth_y) / SMOOTH_DEN;

    let motion = MotionPayload {
      vx: smooth_y.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i8,
      vy: smooth_x.clamp(-OUTPUT_MAX, OUTPUT_MAX) as i8,
      omega: 0,
    };

    if motion != last_sent {
      trace!(
        "Tilt motion: raw=({}, {}) -> vx={}, vy={}",
        raw_ax, raw_ay, motion.vx, motion.vy
      );
      TILT_MOTION_CHANNEL.send(motion).await;
      last_sent = motion;
    }
  }
}
