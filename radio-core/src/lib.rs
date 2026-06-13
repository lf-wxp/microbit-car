#![no_std]

//! Shared radio communication core for micro:bit car project.
//!
//! Provides common radio configuration, initialization, and low-level
//! packet send/receive utilities used by both the car and controller firmware.

use embassy_futures::select::{Either, select};
use embassy_nrf::radio::ieee802154::{Cca, Packet, Radio};
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

/// Hard timeout for a single `try_send` call.
///
/// `embassy-nrf`'s IEEE 802.15.4 driver can leave the RADIO state
/// machine in a state where `try_send` never resolves (neither
/// `phyend` nor `ccabusy` ever fires). To stop a stuck transmit from
/// hanging the whole radio task, we race the call against a short
/// timer and treat the timeout as a transient failure that triggers
/// a retry.
pub const TX_ATTEMPT_TIMEOUT_MS: u64 = 5;

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
  // Effectively disable CCA for this private point-to-point link.
  //
  // The default `Cca::CarrierSense` mode aborts transmission whenever ANY
  // 2.4 GHz signal is sensed (BLE advertisements, WiFi, etc.), causing
  // `try_send` to return `Err(ChannelInUse)` indefinitely in noisy RF
  // environments. Since this project uses a dedicated PHY-only link with
  // no 802.15.4 coexistence requirements, we set the energy detection
  // threshold to its maximum (0xFF) so the channel is always considered
  // clear and packets are always transmitted.
  radio.set_cca(Cca::EnergyDetection { ed_threshold: 0xFF });
  info!(
    "{} radio initialized: channel={}, tx_power={}dBm, cca=disabled",
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

  let mut succeeded = false;

  for attempt in 0..max_retries {
    // Race the radio's `try_send` future against a short timeout so a
    // wedged peripheral state machine cannot block the caller forever.
    let send_fut = radio.try_send(&mut tx_packet);
    let timeout_fut = Timer::after(Duration::from_millis(TX_ATTEMPT_TIMEOUT_MS));
    match select(send_fut, timeout_fut).await {
      Either::First(Ok(())) => {
        trace!("TX success (attempt {})", attempt + 1);
        succeeded = true;
        break;
      }
      Either::First(Err(e)) => {
        // Diagnostic-level logging: surface CCA / channel issues so that
        // link-layer problems are visible without enabling the trace level.
        warn!("TX failed (attempt {}): {:?}", attempt + 1, e);
        Timer::after(Duration::from_micros(TX_RETRY_DELAY_US)).await;
      }
      Either::Second(_) => {
        // try_send never resolved within the budget. Most often this
        // means the RADIO peripheral is stuck mid state-transition;
        // `select` cancels the future on drop which lets embassy-nrf
        // run its `OnDrop` cleanup and reset the state machine.
        warn!(
          "TX timeout (attempt {}, {}ms)",
          attempt + 1,
          TX_ATTEMPT_TIMEOUT_MS
        );
        Timer::after(Duration::from_micros(TX_RETRY_DELAY_US)).await;
      }
    }
  }

  if !succeeded {
    warn!("TX failed after {} retries", max_retries);
  }
  succeeded
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
