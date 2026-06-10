#![no_std]

// Radio protocol for micro:bit communication
// Uses 2.4GHz radio for wireless communication between car and controller
// Designed for Mecanum wheel omnidirectional car

// Maximum radio packet size for micro:bit (32 bytes payload)
pub const MAX_PACKET_SIZE: usize = 32;
// Maximum payload size (header 4 bytes + checksum 1 byte = 5 bytes overhead)
pub const MAX_PAYLOAD_SIZE: usize = MAX_PACKET_SIZE - RadioHeader::SIZE - 1;
// Protocol version for compatibility checking
pub const PROTOCOL_VERSION: u8 = 2;

/// Message types for radio communication
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum MessageType {
  // Heartbeat to check connection status
  Heartbeat = 0,
  // Motion command from controller to car (vector control)
  Motion = 1,
  // Response from car to controller
  Response = 2,
  // Telemetry data from car
  Telemetry = 3,
  // Emergency stop command (no payload needed)
  EmergencyStop = 4,
}

/// Car response status
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum CarStatus {
  // Car is ready and idle
  Ready = 0,
  // Car is moving
  Moving = 1,
  // Car is stopped
  Stopped = 2,
  // Car encountered an error
  Error = 3,
  // Battery low warning
  BatteryLow = 4,
}

/// Radio packet header (4 bytes)
/// Layout: [version: 1][msg_type: 1][seq: 1][payload_len: 1]
#[derive(Clone, Copy, defmt::Format)]
pub struct RadioHeader {
  // Protocol version for compatibility
  pub version: u8,
  // Message type
  pub msg_type: MessageType,
  // Sequence number for packet ordering (wraps around at 255)
  pub seq: u8,
  // Payload length (0..=MAX_PAYLOAD_SIZE)
  pub payload_len: u8,
}

impl RadioHeader {
  // Size of header in bytes
  pub const SIZE: usize = 4;

  /// Serialize header to bytes
  pub fn to_bytes(&self) -> [u8; Self::SIZE] {
    [
      self.version,
      self.msg_type as u8,
      self.seq,
      self.payload_len,
    ]
  }

  /// Deserialize header from bytes
  pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
    if bytes.len() < Self::SIZE {
      return None;
    }
    let payload_len = bytes[3];
    // Validate payload_len is within allowed range
    if payload_len as usize > MAX_PAYLOAD_SIZE {
      return None;
    }
    Some(Self {
      version: bytes[0],
      msg_type: match bytes[1] {
        0 => MessageType::Heartbeat,
        1 => MessageType::Motion,
        2 => MessageType::Response,
        3 => MessageType::Telemetry,
        4 => MessageType::EmergencyStop,
        _ => return None,
      },
      seq: bytes[2],
      payload_len,
    })
  }
}

/// Mecanum wheel motion command payload (3 bytes)
/// Layout: [vx: 1][vy: 1][omega: 1]
///
/// Uses signed integers (i8) to represent velocity vectors:
/// - vx: forward/backward speed (-100 to +100), positive = forward
/// - vy: lateral speed (-100 to +100), positive = right strafe
/// - omega: rotation speed (-100 to +100), positive = clockwise
///
/// Motion examples for Mecanum wheels:
///   vx=100,  vy=0,    omega=0   -> move forward
///   vx=-100, vy=0,    omega=0   -> move backward
///   vx=0,    vy=100,  omega=0   -> strafe right
///   vx=0,    vy=-100, omega=0   -> strafe left
///   vx=0,    vy=0,    omega=100 -> spin clockwise
///   vx=0,    vy=0,    omega=-100-> spin counter-clockwise
///   vx=70,   vy=70,   omega=0   -> move diagonally (front-right)
///   vx=100,  vy=0,    omega=50  -> forward with clockwise arc
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MotionPayload {
  // Forward/backward velocity: -100 (full backward) to +100 (full forward)
  pub vx: i8,
  // Lateral velocity: -100 (full left) to +100 (full right)
  pub vy: i8,
  // Rotational velocity: -100 (full CCW) to +100 (full CW)
  pub omega: i8,
}

impl MotionPayload {
  // Size of motion payload in bytes
  pub const SIZE: usize = 3;

  /// Create a stop motion (all zeros)
  pub fn stop() -> Self {
    Self { vx: 0, vy: 0, omega: 0 }
  }

  /// Create a forward motion
  pub fn forward(speed: i8) -> Self {
    Self { vx: speed, vy: 0, omega: 0 }
  }

  /// Create a lateral (strafe) motion
  pub fn strafe(speed: i8) -> Self {
    Self { vx: 0, vy: speed, omega: 0 }
  }

  /// Create a rotation-only motion
  pub fn rotate(speed: i8) -> Self {
    Self { vx: 0, vy: 0, omega: speed }
  }

  /// Serialize motion to bytes
  pub fn to_bytes(&self) -> [u8; Self::SIZE] {
    [self.vx as u8, self.vy as u8, self.omega as u8]
  }

  /// Deserialize motion from bytes
  pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
    if bytes.len() < Self::SIZE {
      return None;
    }
    Some(Self {
      vx: bytes[0] as i8,
      vy: bytes[1] as i8,
      omega: bytes[2] as i8,
    })
  }

  /// Clamp all values to valid range [-100, 100]
  pub fn clamped(self) -> Self {
    Self {
      vx: self.vx.max(-100).min(100),
      vy: self.vy.max(-100).min(100),
      omega: self.omega.max(-100).min(100),
    }
  }
}

/// Response payload (2 bytes)
/// Layout: [status: 1][info: 1]
#[derive(Clone, Copy, defmt::Format)]
pub struct ResponsePayload {
  // Car status
  pub status: CarStatus,
  // Additional info (e.g., battery level 0-100)
  pub info: u8,
}

impl ResponsePayload {
  // Size of response payload in bytes
  pub const SIZE: usize = 2;

  /// Serialize response to bytes
  pub fn to_bytes(&self) -> [u8; Self::SIZE] {
    [self.status as u8, self.info]
  }

  /// Deserialize response from bytes
  pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
    if bytes.len() < Self::SIZE {
      return None;
    }
    let status = match bytes[0] {
      0 => CarStatus::Ready,
      1 => CarStatus::Moving,
      2 => CarStatus::Stopped,
      3 => CarStatus::Error,
      4 => CarStatus::BatteryLow,
      _ => return None,
    };
    Some(Self {
      status,
      info: bytes[1],
    })
  }
}

/// Telemetry payload (6 bytes)
/// Layout: [battery: 1][speed: 1][heading: 1][vx: 1][vy: 1][omega: 1]
///
/// Reports the current state of the Mecanum car.
#[derive(Clone, Copy, defmt::Format)]
pub struct TelemetryPayload {
  // Battery level (0-100%)
  pub battery: u8,
  // Current overall speed magnitude (0-100)
  pub speed: u8,
  // Current heading (0-255, mapped to 0-360 degrees: heading_deg = heading * 360 / 256)
  pub heading: u8,
  // Current forward/backward velocity (-100 to +100)
  pub vx: i8,
  // Current lateral velocity (-100 to +100)
  pub vy: i8,
  // Current rotational velocity (-100 to +100)
  pub omega: i8,
}

impl TelemetryPayload {
  // Size of telemetry payload in bytes
  pub const SIZE: usize = 6;

  /// Serialize telemetry to bytes
  pub fn to_bytes(&self) -> [u8; Self::SIZE] {
    [
      self.battery,
      self.speed,
      self.heading,
      self.vx as u8,
      self.vy as u8,
      self.omega as u8,
    ]
  }

  /// Deserialize telemetry from bytes
  pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
    if bytes.len() < Self::SIZE {
      return None;
    }
    Some(Self {
      battery: bytes[0],
      speed: bytes[1],
      heading: bytes[2],
      vx: bytes[3] as i8,
      vy: bytes[4] as i8,
      omega: bytes[5] as i8,
    })
  }
}

/// Compute XOR checksum over a byte slice
/// Simple but effective for detecting single-bit errors in short packets
fn compute_checksum(data: &[u8]) -> u8 {
  let mut checksum: u8 = 0;
  for &byte in data {
    checksum ^= byte;
  }
  checksum
}

/// Complete radio packet (max 32 bytes)
/// Layout: [header: 4][payload: 0-27][checksum: 1]
///
/// The checksum is computed over header + payload bytes using XOR.
/// Note: nRF52833 hardware CRC can also be enabled at the HAL layer
/// for additional protection; this software checksum provides an
/// extra application-level integrity check.
#[derive(defmt::Format)]
pub struct RadioPacket {
  // Packet header
  pub header: RadioHeader,
  // Packet payload (variable length, max MAX_PAYLOAD_SIZE bytes)
  pub payload: [u8; MAX_PAYLOAD_SIZE],
  // Actual payload length (0..=MAX_PAYLOAD_SIZE)
  payload_len: u8,
}

impl RadioPacket {
  /// Create a new radio packet
  pub fn new(msg_type: MessageType, seq: u8) -> Self {
    Self {
      header: RadioHeader {
        version: PROTOCOL_VERSION,
        msg_type,
        seq,
        payload_len: 0,
      },
      payload: [0; MAX_PAYLOAD_SIZE],
      payload_len: 0,
    }
  }

  /// Set payload data
  pub fn set_payload(&mut self, data: &[u8]) -> Result<(), ()> {
    if data.len() > MAX_PAYLOAD_SIZE {
      return Err(());
    }
    self.payload[..data.len()].copy_from_slice(data);
    self.payload_len = data.len() as u8;
    self.header.payload_len = data.len() as u8;
    Ok(())
  }

  /// Get payload data slice
  pub fn get_payload(&self) -> &[u8] {
    &self.payload[..self.payload_len as usize]
  }

  /// Get the actual total size of the packet on the wire
  /// (header + payload + checksum)
  pub fn actual_size(&self) -> usize {
    RadioHeader::SIZE + self.payload_len as usize + 1
  }

  /// Serialize packet to bytes for radio transmission.
  /// Returns the buffer and the actual number of valid bytes.
  /// Only the first `actual_size()` bytes should be transmitted.
  pub fn to_bytes(&self) -> ([u8; MAX_PACKET_SIZE], usize) {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let header_bytes = self.header.to_bytes();
    buf[..RadioHeader::SIZE].copy_from_slice(&header_bytes);
    let payload_end = RadioHeader::SIZE + self.payload_len as usize;
    buf[RadioHeader::SIZE..payload_end]
      .copy_from_slice(&self.payload[..self.payload_len as usize]);
    // Append XOR checksum after payload
    let checksum = compute_checksum(&buf[..payload_end]);
    buf[payload_end] = checksum;
    let total_len = payload_end + 1;
    (buf, total_len)
  }

  /// Deserialize packet from bytes received via radio.
  /// Validates version, payload length, and checksum.
  pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
    if bytes.len() < RadioHeader::SIZE + 1 {
      // Minimum: header + checksum (no payload)
      return None;
    }
    let header = RadioHeader::from_bytes(bytes)?;
    if header.version != PROTOCOL_VERSION {
      return None;
    }
    let payload_len = header.payload_len as usize;
    let expected_total = RadioHeader::SIZE + payload_len + 1;
    if bytes.len() < expected_total {
      return None;
    }
    // Verify checksum
    let data_end = RadioHeader::SIZE + payload_len;
    let expected_checksum = compute_checksum(&bytes[..data_end]);
    let received_checksum = bytes[data_end];
    if expected_checksum != received_checksum {
      return None;
    }
    let mut packet = Self::new(header.msg_type, header.seq);
    packet
      .set_payload(&bytes[RadioHeader::SIZE..data_end])
      .ok()?;
    Some(packet)
  }
}

// --- Helper functions for creating packets ---

/// Create a motion command packet for Mecanum wheel control
pub fn create_motion_packet(seq: u8, vx: i8, vy: i8, omega: i8) -> RadioPacket {
  let mut packet = RadioPacket::new(MessageType::Motion, seq);
  let motion = MotionPayload { vx, vy, omega }.clamped();
  packet.set_payload(&motion.to_bytes()).ok();
  packet
}

/// Create an emergency stop packet (no payload, highest priority)
pub fn create_emergency_stop_packet(seq: u8) -> RadioPacket {
  RadioPacket::new(MessageType::EmergencyStop, seq)
}

/// Create a response packet
pub fn create_response_packet(seq: u8, status: CarStatus, info: u8) -> RadioPacket {
  let mut packet = RadioPacket::new(MessageType::Response, seq);
  let resp = ResponsePayload { status, info };
  packet.set_payload(&resp.to_bytes()).ok();
  packet
}

/// Create a heartbeat packet
pub fn create_heartbeat_packet(seq: u8) -> RadioPacket {
  RadioPacket::new(MessageType::Heartbeat, seq)
}

/// Create a telemetry packet
pub fn create_telemetry_packet(seq: u8, telemetry: TelemetryPayload) -> RadioPacket {
  let mut packet = RadioPacket::new(MessageType::Telemetry, seq);
  packet.set_payload(&telemetry.to_bytes()).ok();
  packet
}
