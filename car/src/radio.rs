//! Radio communication module for the car firmware.
//!
//! Uses IEEE 802.15.4 (2.4 GHz) radio on nRF52833 to receive motion commands
//! from the controller and send back response/telemetry data.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

use defmt::{info, trace, warn};
use protocol::{
  CarStatus, MessageType, MotionPayload, RadioPacket, TelemetryPayload, create_heartbeat_packet,
  create_response_packet, create_telemetry_packet,
};
use radio_core::{self, next_seq, send_packet};

/// Per-iteration timeout on `radio.receive()`. We bound the await
/// so a wedged RX state machine cannot silently hang the task
/// forever; on timeout the future is dropped, embassy-nrf's
/// `OnDrop` cleanup resets the RADIO peripheral, and the next loop
/// iteration starts a fresh receive attempt.
const RX_ATTEMPT_TIMEOUT_MS: u64 = 1000;

/// Channel for passing received motion commands from radio task to main logic
pub static MOTION_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> = Channel::new();

/// Channel for sending telemetry data from main logic to radio task
#[allow(dead_code)]
pub static TELEMETRY_CHANNEL: Channel<CriticalSectionRawMutex, TelemetryPayload, 2> =
  Channel::new();

/// Channel to signal emergency stop from radio task to main logic
pub static EMERGENCY_STOP: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Millis-since-boot of the most recent successfully-parsed inbound
/// packet (any `MessageType`). 0 means "never received anything yet".
///
/// Read by the link-watchdog in `main.rs` to detect when the
/// controller has gone away. `u32` millis is good for ~49 days, far
/// beyond any realistic operating session.
static LAST_RX_MS: AtomicU32 = AtomicU32::new(0);

/// Returns the millis-since-boot value at which the last inbound
/// packet was received, or `0` if no packet has ever been seen.
pub fn last_rx_millis() -> u32 {
  LAST_RX_MS.load(Ordering::Relaxed)
}

/// Refresh the last-RX timestamp to "now". Called for every packet we
/// successfully parse, regardless of `msg_type`.
fn touch_last_rx() {
  // Saturating cast: `as u32` already truncates `u64` millis, which is
  // fine because we only ever compare with `wrapping_sub` semantics in
  // the watchdog.
  let now_ms = Instant::now().as_millis() as u32;
  LAST_RX_MS.store(now_ms, Ordering::Relaxed);
}

/// Initialize the radio peripheral for the car
pub fn init(radio_periph: Peri<'static, peripherals::RADIO>) -> Radio<'static> {
  radio_core::init(radio_periph, "Car")
}

/// Radio receiver task: continuously listens for packets from the controller
/// and dispatches them to the appropriate channels.
///
/// Each receive call is bounded by [`RX_ATTEMPT_TIMEOUT_MS`] so the
/// task never silently wedges if the underlying RADIO peripheral
/// stops firing completion events.
#[embassy_executor::task]
pub async fn radio_rx_task(mut radio: Radio<'static>) {
  info!("Car radio RX task started");

  let mut seq: u8 = 0;
  let mut rx_packet = Packet::new();

  loop {
    let recv_fut = radio.receive(&mut rx_packet);
    let timeout_fut = Timer::after(Duration::from_millis(RX_ATTEMPT_TIMEOUT_MS));
    match select(recv_fut, timeout_fut).await {
      Either::First(Ok(())) => {
        let data: &[u8] = &rx_packet;
        if data.is_empty() {
          trace!("Received empty packet, ignoring");
        } else {
          match RadioPacket::from_bytes(data) {
            Some(packet) => {
              touch_last_rx();
              handle_received_packet(&packet, &mut radio, &mut seq).await;
            }
            None => {
              warn!("Failed to parse radio packet (len={})", data.len());
            }
          }
        }
      }
      Either::First(Err(e)) => {
        trace!("RX error: {:?}", e);
      }
      Either::Second(_) => {
        // `recv_fut` is dropped here; embassy-nrf's OnDrop cleanup
        // resets the RADIO peripheral, so the next iteration starts
        // a fresh receive attempt.
        trace!("RX timeout, restarting receive");
      }
    }
  }
}

/// Handle a successfully parsed radio packet
async fn handle_received_packet(packet: &RadioPacket, radio: &mut Radio<'static>, seq: &mut u8) {
  match packet.header.msg_type {
    MessageType::Motion => {
      if let Some(motion) = MotionPayload::from_bytes(packet.get_payload()) {
        trace!(
          "RX motion: vx={}, vy={}, omega={}",
          motion.vx, motion.vy, motion.omega
        );
        if MOTION_CHANNEL.try_send(motion).is_err() {
          warn!("Motion channel full, dropping command");
        }
        // Intentionally NO ACK here.
        //
        // The controller streams motion at ~50 Hz; ACKing every packet
        // turned the radio into a 50 Hz TX burst source, which the
        // micro:bit's on-board buck regulator and the MotorBit's
        // power filtering inductors picked up as a faint but audible
        // low-frequency hum (electromagnetic-induced acoustic noise).
        //
        // Link liveness is already covered by the Heartbeat reply
        // path (~5 Hz) below — that's what `controller::radio` uses
        // to drive its `ConnectionState` LED — so dropping motion
        // ACKs is functionally invisible to the user while cutting
        // TX activity by an order of magnitude.
      }
    }
    MessageType::EmergencyStop => {
      info!("Emergency stop received!");
      let _ = EMERGENCY_STOP.try_send(());
      let _ = MOTION_CHANNEL.try_send(MotionPayload::stop());
      // Keep ACK for E-stop: it's a one-shot critical event, the
      // controller may want to retry until acknowledged, and the
      // resulting single TX has no impact on steady-state noise.
      let response = create_response_packet(next_seq(seq), CarStatus::Stopped, 0);
      send_packet(radio, &response).await;
    }
    MessageType::Heartbeat => {
      trace!("Heartbeat received, replying");
      // Heartbeat reply at ~5 Hz is required for the controller's
      // link-state machine; 5 Hz is well below the audible-resonance
      // band and barely contributes to acoustic noise compared to
      // the previous 50 Hz motion ACK rate.
      let hb = create_heartbeat_packet(next_seq(seq));
      send_packet(radio, &hb).await;
    }
    MessageType::Telemetry | MessageType::Response => {
      // Car should not receive these types, ignore
      trace!("Unexpected message type received, ignoring");
    }
  }
}

/// Send a telemetry packet
#[allow(dead_code)]
pub async fn send_telemetry(radio: &mut Radio<'static>, seq: &mut u8, telemetry: TelemetryPayload) {
  let pkt = create_telemetry_packet(next_seq(seq), telemetry);
  send_packet(radio, &pkt).await;
}
