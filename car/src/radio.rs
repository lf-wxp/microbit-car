//! Radio communication module for the car firmware.
//!
//! Uses IEEE 802.15.4 (2.4 GHz) radio on nRF52833 to receive motion commands
//! from the controller and send back response/telemetry data.

use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use defmt::{info, trace, warn};
use protocol::{
  CarStatus, MessageType, MotionPayload, RadioPacket, TelemetryPayload, create_heartbeat_packet,
  create_response_packet, create_telemetry_packet,
};
use radio_core::{self, next_seq, send_packet};

/// Channel for passing received motion commands from radio task to main logic
pub static MOTION_CHANNEL: Channel<CriticalSectionRawMutex, MotionPayload, 4> = Channel::new();

/// Channel for sending telemetry data from main logic to radio task
#[allow(dead_code)]
pub static TELEMETRY_CHANNEL: Channel<CriticalSectionRawMutex, TelemetryPayload, 2> =
  Channel::new();

/// Channel to signal emergency stop from radio task to main logic
pub static EMERGENCY_STOP: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Initialize the radio peripheral for the car
pub fn init(radio_periph: Peri<'static, peripherals::RADIO>) -> Radio<'static> {
  radio_core::init(radio_periph, "Car")
}

/// Radio receiver task: continuously listens for packets from the controller
/// and dispatches them to the appropriate channels.
#[embassy_executor::task]
pub async fn radio_rx_task(mut radio: Radio<'static>) {
  info!("Car radio RX task started");

  let mut seq: u8 = 0;
  let mut rx_packet = Packet::new();

  loop {
    // Wait for incoming packet
    match radio.receive(&mut rx_packet).await {
      Ok(()) => {
        let data: &[u8] = &rx_packet;
        if data.is_empty() {
          trace!("Received empty packet, ignoring");
          continue;
        }

        match RadioPacket::from_bytes(data) {
          Some(packet) => {
            handle_received_packet(&packet, &mut radio, &mut seq).await;
          }
          None => {
            warn!("Failed to parse radio packet");
          }
        }
      }
      Err(e) => {
        trace!("RX error: {:?}", e);
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
        // Send ACK response
        let response = create_response_packet(next_seq(seq), CarStatus::Moving, 0);
        send_packet(radio, &response).await;
      }
    }
    MessageType::EmergencyStop => {
      info!("Emergency stop received!");
      let _ = EMERGENCY_STOP.try_send(());
      let _ = MOTION_CHANNEL.try_send(MotionPayload::stop());
      // Send ACK
      let response = create_response_packet(next_seq(seq), CarStatus::Stopped, 0);
      send_packet(radio, &response).await;
    }
    MessageType::Heartbeat => {
      trace!("Heartbeat received");
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
