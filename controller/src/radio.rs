//! Radio communication module for the controller firmware.
//!
//! Uses IEEE 802.15.4 (2.4 GHz) radio on nRF52833 to send motion commands
//! to the car and receive response/telemetry data back.

use embassy_futures::select::{Either4, select4};
use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};

use defmt::{error, info, trace};
use protocol::{
  MessageType, MotionPayload, RadioPacket, ResponsePayload, TelemetryPayload,
  create_heartbeat_packet, create_motion_packet,
};
use radio_core::{
  self, HEARTBEAT_INTERVAL_MS, MAX_EMERGENCY_RETRIES, next_seq, send_packet,
  send_packet_with_retries,
};

use crate::rgb::{ConnectionState, RGB_STATE, RgbState};

/// Time without a heartbeat reply after which we declare the link
/// down. Sized to a small multiple of `HEARTBEAT_INTERVAL_MS` so a
/// single missed packet doesn't flicker the indicator red.
pub const HEARTBEAT_TIMEOUT_MS: u64 = HEARTBEAT_INTERVAL_MS * 3;

/// How often to re-evaluate the link state from `last_heartbeat`.
/// Faster than the heartbeat interval so a missed reply transitions
/// the LED with sub-second latency.
const LINK_EVAL_INTERVAL_MS: u64 = 200;

/// Channel for sending motion commands from main logic to radio TX task
pub static MOTION_TX_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> = Channel::new();

/// Channel for receiving response status from car
pub static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, ResponsePayload, 2> = Channel::new();

/// Channel for receiving telemetry data from car
pub static TELEMETRY_RX_CHANNEL: Channel<CriticalSectionRawMutex, TelemetryPayload, 2> =
  Channel::new();

/// Channel carrying every received heartbeat response from the car.
/// `radio_task` consumes this internally to refresh the link timer
/// and publish a [`ConnectionState`] update on [`RGB_STATE`].
static HEARTBEAT_RX: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();

/// Initialize the radio peripheral for the controller
pub fn init(radio_periph: Peri<'static, peripherals::RADIO>) -> Radio<'static> {
  radio_core::init(radio_periph, "Controller")
}

/// Radio task: handles both TX (sending commands) and RX (receiving responses).
///
/// Steady state is `radio.receive()` — we want the controller to
/// look exactly like the car from the link's perspective, since
/// the car responds within 1–3 ms of receiving a motion frame and
/// the previous design (alternating short TX bursts with a 10 ms
/// RX window) systematically missed those replies.
///
/// `select4` lets four asynchronous events preempt the receive
/// future:
///   * a queued outbound motion packet,
///   * the periodic heartbeat tick,
///   * the link-state evaluation tick,
///   * a completed inbound packet.
///
/// Whenever we leave the `radio.receive()` arm to do anything
/// else, the future is dropped and embassy-nrf's `OnDrop` cleanup
/// resets the RADIO state machine, so the next loop iteration
/// re-enters RX cleanly.
#[embassy_executor::task]
pub async fn radio_task(mut radio: Radio<'static>) {
  info!("Controller radio task started");

  let mut seq: u8 = 0;
  let mut rx_packet = Packet::new();

  // Link-state bookkeeping. We publish `Connecting` upfront so the LED
  // doesn't sit in its power-on state while we wait for the first
  // heartbeat round-trip.
  let mut link_state = ConnectionState::Connecting;
  let mut last_heartbeat: Option<Instant> = None;
  publish_link_state(link_state);

  let mut heartbeat_ticker = Ticker::every(Duration::from_millis(HEARTBEAT_INTERVAL_MS));
  let mut link_eval_ticker = Ticker::every(Duration::from_millis(LINK_EVAL_INTERVAL_MS));

  loop {
    // Default await: every branch except the receive completion
    // requires us to drop `recv_fut` and run a short TX path or
    // bookkeeping step before returning to RX.
    let recv_fut = radio.receive(&mut rx_packet);
    let motion_fut = MOTION_TX_CHANNEL.receive();
    let hb_fut = heartbeat_ticker.next();
    let link_fut = link_eval_ticker.next();

    match select4(recv_fut, motion_fut, hb_fut, link_fut).await {
      Either4::First(result) => match result {
        Ok(()) => {
          let data: &[u8] = &rx_packet;
          if !data.is_empty() {
            if let Some(packet) = RadioPacket::from_bytes(data) {
              trace!(
                "RX packet: type={:?}, len={}",
                packet.header.msg_type,
                data.len()
              );
              handle_received_packet(&packet);
            } else {
              trace!("RX parse failed (len={})", data.len());
            }
          }
        }
        Err(e) => {
          trace!("RX error: {:?}", e);
        }
      },
      Either4::Second(motion) => {
        let packet = create_motion_packet(next_seq(&mut seq), motion.vx, motion.vy, motion.omega);
        send_packet(&mut radio, &packet).await;
      }
      Either4::Third(_) => {
        let hb = create_heartbeat_packet(next_seq(&mut seq));
        send_packet(&mut radio, &hb).await;
      }
      Either4::Fourth(_) => {
        // Drain any heartbeat-reply notifications produced by the
        // RX path so a fresh `last_heartbeat` lands before we
        // recompute the link state.
        while HEARTBEAT_RX.try_receive().is_ok() {
          last_heartbeat = Some(Instant::now());
        }
        let next_state = derive_link_state(last_heartbeat);
        if next_state != link_state {
          info!("Radio link: {:?} -> {:?}", link_state, next_state);
          link_state = next_state;
          publish_link_state(link_state);
        }
      }
    }
  }
}

/// Decide the current link state from the timestamp of the most
/// recent heartbeat reply.
fn derive_link_state(last_heartbeat: Option<Instant>) -> ConnectionState {
  let Some(last) = last_heartbeat else {
    return ConnectionState::Connecting;
  };
  if last.elapsed() <= Duration::from_millis(HEARTBEAT_TIMEOUT_MS) {
    ConnectionState::Connected
  } else {
    ConnectionState::Disconnected
  }
}

/// Push the latest link state into the shared RGB signal so the LED
/// driver re-renders.
fn publish_link_state(connection: ConnectionState) {
  RGB_STATE.signal(RgbState { connection });
}

/// Handle a received packet from the car
fn handle_received_packet(packet: &RadioPacket) {
  match packet.header.msg_type {
    MessageType::Response => {
      if let Some(response) = ResponsePayload::from_bytes(packet.get_payload()) {
        trace!(
          "RX response: status={:?}, info={}",
          response.status, response.info
        );
        let _ = RESPONSE_CHANNEL.try_send(response);
      }
    }
    MessageType::Telemetry => {
      if let Some(telemetry) = TelemetryPayload::from_bytes(packet.get_payload()) {
        trace!(
          "RX telemetry: battery={}, speed={}, heading={}",
          telemetry.battery, telemetry.speed, telemetry.heading
        );
        let _ = TELEMETRY_RX_CHANNEL.try_send(telemetry);
      }
    }
    MessageType::Heartbeat => {
      trace!("Heartbeat response received");
      // Notify the radio task; it owns the link-state machine.
      let _ = HEARTBEAT_RX.try_send(());
    }
    MessageType::Motion | MessageType::EmergencyStop => {
      // Controller should not receive these types, ignore
      trace!("Unexpected message type received, ignoring");
    }
  }
}

/// Send an emergency stop command to the car (higher retry count)
#[allow(dead_code)]
pub async fn send_emergency_stop(radio: &mut Radio<'static>, seq: &mut u8) {
  let packet = RadioPacket::new(MessageType::EmergencyStop, next_seq(seq));
  let success = send_packet_with_retries(radio, &packet, MAX_EMERGENCY_RETRIES).await;
  if success {
    info!("Emergency stop sent successfully");
  } else {
    error!("Failed to send emergency stop!");
  }
}
