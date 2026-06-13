//! Radio communication module for the controller firmware.
//!
//! Uses IEEE 802.15.4 (2.4 GHz) radio on nRF52833 to send motion commands
//! to the car and receive response/telemetry data back.

use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

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
/// This task alternates between sending pending commands and listening for
/// responses from the car. It also sends periodic heartbeats and keeps
/// the [`RGB_STATE`] link indicator in sync with how recently the car
/// last replied.
#[embassy_executor::task]
pub async fn radio_task(mut radio: Radio<'static>) {
  info!("Controller radio task started");

  let mut seq: u8 = 0;
  let mut heartbeat_timer = 0u64;
  let mut rx_packet = Packet::new();

  // Link-state bookkeeping. We publish `Connecting` upfront so the LED
  // doesn't sit in its power-on state while we wait for the first
  // heartbeat round-trip.
  let mut link_state = ConnectionState::Connecting;
  let mut last_heartbeat: Option<Instant> = None;
  publish_link_state(link_state);

  loop {
    // Check if there's a motion command to send
    if let Ok(motion) = MOTION_TX_CHANNEL.try_receive() {
      let packet = create_motion_packet(next_seq(&mut seq), motion.vx, motion.vy, motion.omega);
      send_packet(&mut radio, &packet).await;
    }

    // Send periodic heartbeat
    heartbeat_timer += 10;
    if heartbeat_timer >= HEARTBEAT_INTERVAL_MS {
      heartbeat_timer = 0;
      let hb = create_heartbeat_packet(next_seq(&mut seq));
      send_packet(&mut radio, &hb).await;
    }

    // Drain any heartbeat-reply notifications produced by the RX path.
    // Each one bumps `last_heartbeat` forward.
    while HEARTBEAT_RX.try_receive().is_ok() {
      last_heartbeat = Some(Instant::now());
    }

    // Re-evaluate link state every loop tick.
    let next_state = derive_link_state(last_heartbeat);
    if next_state != link_state {
      info!("Radio link: {:?} -> {:?}", link_state, next_state);
      link_state = next_state;
      publish_link_state(link_state);
    }

    // Try to receive a response (with short timeout)
    match embassy_futures::select::select(
      radio.receive(&mut rx_packet),
      Timer::after(Duration::from_millis(10)),
    )
    .await
    {
      embassy_futures::select::Either::First(result) => match result {
        Ok(()) => {
          let data: &[u8] = &rx_packet;
          if !data.is_empty()
            && let Some(packet) = RadioPacket::from_bytes(data)
          {
            handle_received_packet(&packet);
          }
        }
        Err(e) => {
          trace!("RX error: {:?}", e);
        }
      },
      embassy_futures::select::Either::Second(_) => {
        // Timeout, no packet received - continue loop
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
