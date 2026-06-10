#![no_std]

//! Shared radio communication core for micro:bit car project.
//!
//! Provides common radio configuration, initialization, and low-level
//! packet send/receive utilities used by both the car and controller firmware.

use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::{Peri, bind_interrupts, peripherals, radio};
use embassy_time::{Duration, Timer};

use defmt::{info, trace, warn};
use protocol::RadioPacket;

// ─── Radio Configuration Constants ───────────────────────────────────────────

/// IEEE 802.15.4 channel (11-26), channel 15 avoids common WiFi interference
pub const RADIO_CHANNEL: u8 = 15;

/// Transmission power in dBm
pub const TX_POWER: i8 = 4;

/// Maximum retry attempts for sending a normal packet
pub const MAX_TX_RETRIES: u8 = 3;

/// Maximum retry attempts for sending an emergency stop packet
pub const MAX_EMERGENCY_RETRIES: u8 = 5;

/// Retry delay between TX attempts in microseconds
pub const TX_RETRY_DELAY_US: u64 = 200;

/// Heartbeat interval in milliseconds (controller -> car)
pub const HEARTBEAT_INTERVAL_MS: u64 = 200;

/// Heartbeat timeout: if no command received within this duration, stop the car
pub const HEARTBEAT_TIMEOUT_MS: u64 = 500;

// ─── Interrupt Binding ───────────────────────────────────────────────────────

bind_interrupts!(struct Irqs {
  RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

// ─── Radio Initialization ────────────────────────────────────────────────────

/// Initialize the IEEE 802.15.4 radio peripheral with shared configuration.
///
/// Both car and controller use identical radio settings to ensure
/// they can communicate on the same channel.
pub fn init(radio_periph: Peri<'static, peripherals::RADIO>, role: &str) -> Radio<'static> {
  let mut radio = Radio::new(radio_periph, Irqs);
  radio.set_channel(RADIO_CHANNEL);
  radio.set_transmission_power(TX_POWER);
  info!(
    "{} radio initialized: channel={}, tx_power={}dBm",
    role, RADIO_CHANNEL, TX_POWER
  );
  radio
}

// ─── Low-level Packet Transmission ──────────────────────────────────────────

/// Send a radio packet with retry logic.
///
/// Attempts to transmit the packet up to `max_retries` times with a brief
/// delay between attempts. Uses CCA (Clear Channel Assessment) before each
/// transmission to avoid collisions.
pub async fn send_packet_with_retries(
  radio: &mut Radio<'static>,
  packet: &RadioPacket,
  max_retries: u8,
) -> bool {
  let (buf, len) = packet.to_bytes();

  let mut tx_packet = Packet::new();
  tx_packet.copy_from_slice(&buf[..len]);

  for attempt in 0..max_retries {
    match radio.try_send(&mut tx_packet).await {
      Ok(()) => {
        trace!("TX success (attempt {})", attempt + 1);
        return true;
      }
      Err(e) => {
        trace!("TX failed (attempt {}): {:?}", attempt + 1, e);
        Timer::after(Duration::from_micros(TX_RETRY_DELAY_US)).await;
      }
    }
  }
  warn!("TX failed after {} retries", max_retries);
  false
}

/// Send a radio packet with default retry count (MAX_TX_RETRIES).
pub async fn send_packet(radio: &mut Radio<'static>, packet: &RadioPacket) {
  send_packet_with_retries(radio, packet, MAX_TX_RETRIES).await;
}

// ─── Sequence Number Helper ─────────────────────────────────────────────────

/// Advance a sequence number with wrapping.
#[inline]
pub fn next_seq(seq: &mut u8) -> u8 {
  let current = *seq;
  *seq = seq.wrapping_add(1);
  current
}
