# microbit-car

A wireless **Mecanum-wheel omnidirectional car** built around two
[BBC micro:bit v2] boards (nRF52833) and written in **`#![no_std]` async Rust**
on top of the [Embassy] runtime.

One micro:bit acts as the **car** (driving four DC motors through a MotorBit
shield + PCA9685 PWM driver), the other acts as the **controller**
(reading a thumb-stick, the on-board accelerometer and four shield buttons,
fusing them into a single motion vector and sending it over the air).

The two boards talk to each other directly on **2.4 GHz IEEE 802.15.4** with a
small custom application-layer protocol — no Wi-Fi, no BLE, no companion
phone needed.

[BBC micro:bit v2]: https://tech.microbit.org/hardware/2-1-revision/
[Embassy]: https://embassy.dev/

---

## Table of contents

- [microbit-car](#microbit-car)
  - [Table of contents](#table-of-contents)
  - [Hardware](#hardware)
  - [Repository layout](#repository-layout)
  - [System architecture](#system-architecture)
  - [Wire protocol](#wire-protocol)
    - [Packet frame](#packet-frame)
    - [Message types](#message-types)
    - [`MotionPayload` semantics](#motionpayload-semantics)
    - [Radio configuration](#radio-configuration)
    - [Hex examples](#hex-examples)
  - [Controller design](#controller-design)
    - [Input pipelines](#input-pipelines)
    - [Fusion loop](#fusion-loop)
  - [Car design](#car-design)
  - [Pin map (cheat-sheet)](#pin-map-cheat-sheet)
    - [Controller (micro:bit v2)](#controller-microbit-v2)
    - [Car (micro:bit v2 + MotorBit shield)](#car-microbit-v2--motorbit-shield)
  - [Building \& flashing](#building--flashing)
    - [Prerequisites](#prerequisites)
    - [One-shot build](#one-shot-build)
    - [Flash](#flash)
    - [Useful extras](#useful-extras)
  - [Driving the car](#driving-the-car)
  - [Logging \& debugging](#logging--debugging)
  - [Troubleshooting](#troubleshooting)
  - [Extending the project](#extending-the-project)
    - [1. Add a new input source on the controller](#1-add-a-new-input-source-on-the-controller)
    - [2. Add a new output / actuator on the car](#2-add-a-new-output--actuator-on-the-car)
    - [3. Add a new message type to the protocol](#3-add-a-new-message-type-to-the-protocol)
    - [4. Tune the driving feel](#4-tune-the-driving-feel)
    - [5. Port to a different chassis](#5-port-to-a-different-chassis)
    - [6. Port to a different MCU](#6-port-to-a-different-mcu)
  - [Contributing](#contributing)
    - [Workflow](#workflow)
    - [Coding conventions](#coding-conventions)
    - [Commit message format](#commit-message-format)
    - [Pull-request template](#pull-request-template)
    - [Reporting bugs / requesting features](#reporting-bugs--requesting-features)
  - [Acknowledgements](#acknowledgements)
  - [License](#license)

---

## Hardware

| Role        | Board / Module                                            | Notes                                            |
|-------------|-----------------------------------------------------------|--------------------------------------------------|
| MCU (×2)    | BBC micro:bit v2 (nRF52833, Cortex-M4F, 2.4 GHz radio)    | One for the car, one for the controller         |
| Motor shield| **MotorBit** expansion board (PCA9685 over I²C @ `0x40`)  | 4× DC motors (M1–M4) + 8× servo headers (S1–S8) |
| Drivetrain  | 4× Mecanum wheels + 4× brushed DC motors                  | Controlled in pairs of PWM channels             |
| Input       | XY thumb-stick (10-bit analog), on-board LSM303AGR accel. | Tilt mode is digital (I²C), joystick is analog  |
| Buttons     | A / B / C / D on the controller shield (P13 / P14 / P16 / P15) | Internal pull-ups, active-low                   |
| Power       | LiPo / battery pack on the MotorBit board                 | The micro:bit is powered through the shield     |

> Only the on-board **micro:bit v2** peripherals are required. The MotorBit
> shield is only used on the **car** side; the **controller** uses the
> shield buttons (P13–P16) but does not need the motor section.

---

## Repository layout

```text
microbit-car/
├── Cargo.toml          # Cargo workspace
├── Embed.toml          # probe-rs default configuration (nRF52833_xxAA)
├── Makefile.toml       # cargo-make tasks (build / flash / size / clippy / fmt)
├── memory.x (per crate) # Linker memory layout
│
├── protocol/           # #![no_std] wire-format crate (shared by both sides)
│   └── src/lib.rs      # MessageType, RadioPacket, MotionPayload, ...
│
├── radio-core/         # #![no_std] shared radio config + TX/RX helpers
│   └── src/lib.rs      # init(), send_packet*, retry policy, channel #, ...
│
├── controller/         # Controller firmware (binary crate)
│   └── src/
│       ├── main.rs     # Embassy `main`, spawns input/radio tasks, fusion loop
│       ├── radio.rs    # 802.15.4 TX/RX task, heartbeats, response handling
│       ├── joystick.rs # SAADC sampling, dead-zone + EMA smoothing
│       ├── tilt.rs     # LSM303AGR accelerometer driver, tilt → vx/vy
│       ├── button.rs   # C/D omega buttons + A mode-toggle button
│       └── mode.rs     # InputMode arbitration (Joystick ⇄ Tilt) via atomic + Signal
│
└── car/                # Car firmware (binary crate)
    └── src/
        ├── main.rs     # Embassy `main`, motor driver + radio RX glue
        ├── radio.rs    # 802.15.4 RX task, dispatches Motion / E-Stop / HB
        ├── motor.rs    # MotorDriver: Mecanum inverse kinematics + I²C init
        ├── motorbit.rs # MotorBit shield abstraction (4 DC motors + 8 servos)
        └── pca9685.rs  # Low-level PCA9685 PWM driver
```

---

## System architecture

```text
┌──────────────────────── controller (micro:bit v2) ──────────────────────────┐
│                                                                              │
│   joystick_task ─┐                                                           │
│   tilt_task     ─┤── (vx, vy) ─┐                                             │
│                  │             ├──► fusion loop (main.rs) ──► MOTION_TX ─►   │
│   button_task ──── omega ─────┘                                       │     │
│                                                                       ▼     │
│                                                              radio_task ───┐│
│   mode_switch_task ──► AtomicU8 + Signal (InputMode) ◄── (gates joystick/tilt)│
└───────────────────────────────────────────────────────────────────┬──────┬──┘
                                                                    │      │
                                                            802.15.4│ ch15 │ACK / heartbeat
                                                                    ▼      ▲
┌────────────────────────────── car (micro:bit v2) ────────────────────────────┐
│                                                                              │
│   radio_rx_task ──► MOTION_CHANNEL ──► main loop ──► MotorDriver.apply_motion│
│                                                          │                   │
│                                                          ▼                   │
│                                          ┌── inverse kinematics ─┐           │
│                                          │  (vx,vy,ω) → 4 motors │           │
│                                          └────────┬──────────────┘           │
│                                                   ▼                          │
│                                  MotorBit ──► PCA9685 ──► I²C ──► motors     │
└──────────────────────────────────────────────────────────────────────────────┘
```

Key design properties:

- **All inter-task communication is `embassy_sync::channel::Channel` or
  `Signal`.** No global mutable state, no `Mutex<RefCell<...>>` indirection.
- The **controller fusion loop** is the single source of truth for the
  outgoing `MotionPayload`: it samples whichever input source is currently
  active (`Joystick` or `Tilt`) for `(vx, vy)` and always merges in the
  latest `omega` from the C/D buttons.
- The **mode toggle** is stored in an `AtomicU8` so any task can read it
  cheaply, and a `Signal<InputMode>` lets the fusion loop zero stale
  velocities the instant the mode flips.
- The **car** runs its motor driver in the *main task* and only treats
  the radio task as a producer of `MotionPayload`s on a bounded channel.
  This keeps the I²C-heavy code on a single executor task and avoids
  contention on the TWIM peripheral.

---

## Wire protocol

Defined in [`protocol/src/lib.rs`](protocol/src/lib.rs). One header, a few
fixed-size payloads, and an XOR checksum — small enough to fit comfortably
in a single 32-byte 802.15.4 frame.

### Packet frame

```text
 0       1         2     3           4 .. 4+N-1     4+N
┌───────┬─────────┬─────┬───────────┬──────────────┬────────┐
│ ver   │ msgtype │ seq │ payload_n │   payload    │ XOR    │
└───────┴─────────┴─────┴───────────┴──────────────┴────────┘
   1B       1B       1B      1B          N bytes      1B
```

- `ver`         — `PROTOCOL_VERSION` (currently `2`); receiver drops mismatched versions.
- `msgtype`     — see [`MessageType`](protocol/src/lib.rs).
- `seq`         — wraps at 255, used purely for tracing / dedup.
- `payload_n`   — `0..=MAX_PAYLOAD_SIZE` (= `32 - 4 - 1` = 27 bytes max).
- `XOR`         — software checksum over `header + payload`.
  802.15.4 hardware CRC is also active at the HAL layer, so this is an
  extra application-level integrity check.

### Message types

| Type            | Direction        | Payload                | Purpose                                |
|-----------------|------------------|------------------------|----------------------------------------|
| `Heartbeat`     | both ways        | none                   | Liveness / connection indicator        |
| `Motion`        | controller → car | `MotionPayload` (3 B)  | `(vx, vy, omega)` velocity vector      |
| `EmergencyStop` | controller → car | none                   | Force the car to stop, retried 5×      |
| `Response`      | car → controller | `ResponsePayload` (2B) | `CarStatus` + extra info byte          |
| `Telemetry`     | car → controller | `TelemetryPayload`(6B) | Battery / speed / heading / vx/vy/ω    |

### `MotionPayload` semantics

```rust
pub struct MotionPayload {
  pub vx: i8,    // forward(+) / backward(-),   range -100..=100
  pub vy: i8,    // strafe right(+) / left(-),  range -100..=100
  pub omega: i8, // CW(+) / CCW(-) rotation,    range -100..=100
}
```

Mecanum inverse kinematics on the car side
(see `car/src/motor.rs::apply_motion`):

```text
motor_FL (M1) = vx - vy - k * omega
motor_FR (M2) = vx + vy + k * omega
motor_RL (M3) = vx + vy - k * omega
motor_RR (M4) = vx - vy + k * omega
```

After the four raw values are computed they are normalised to fit
`[-100, 100]`, then mapped to the PCA9685 range `[-4095, 4095]`. `k` is a
geometry factor exposed as `GEOMETRY_FACTOR_K` in `motor.rs` (defaults
to `1`); tweak it to taste for your chassis.

### Radio configuration

Defined in [`radio-core/src/lib.rs`](radio-core/src/lib.rs):

| Parameter                | Value          | Why                                    |
|--------------------------|----------------|----------------------------------------|
| `RADIO_CHANNEL`          | `15`           | Sits between common Wi-Fi channels     |
| `TX_POWER`               | `+4 dBm`       | Plenty of range indoors                |
| `MAX_TX_RETRIES`         | `3`            | Best-effort retry for normal commands  |
| `MAX_EMERGENCY_RETRIES`  | `5`            | Higher reliability for E-Stop          |
| `HEARTBEAT_INTERVAL_MS`  | `200`          | Controller → car liveness ping         |
| `HEARTBEAT_TIMEOUT_MS`   | `500`          | Car-side fail-safe (stop on silence)   |

### Hex examples

All examples below are produced by the actual `RadioPacket::to_bytes()`
implementation (XOR over `header || payload`). They're useful as golden
test vectors when you write a sniffer or a third-party emulator.

```text
Heartbeat       seq=0x05
  bytes (5):    02 00 05 00 07
                ^^ ^^ ^^ ^^ ^^
                |  |  |  |  └── XOR(02,00,05,00) = 0x07
                |  |  |  └───── payload_len = 0
                |  |  └──────── seq        = 0x05
                |  └─────────── msg_type   = 0 (Heartbeat)
                └────────────── version    = 2
```

```text
Motion (full forward)            seq=0x2A  vx=+100  vy=0  omega=0
  bytes (8):    02 01 2A 03 64 00 00 4E
                            ^^ ^^ ^^
                            |  |  └── omega = 0
                            |  └───── vy    = 0
                            └──────── vx    = 0x64 = +100
  XOR check:    02^01^2A^03^64^00^00 = 0x4E ✓
```

```text
Motion (back + strafe right + CCW)   seq=0x2B  vx=-100  vy=+50  omega=-25
  bytes (8):    02 01 2B 03 9C 32 E7 62
                            ^^ ^^ ^^
                            |  |  └── omega = 0xE7 = -25 (i8)
                            |  └───── vy    = 0x32 = +50
                            └──────── vx    = 0x9C = -100 (i8)
  XOR check:    02^01^2B^03^9C^32^E7 = 0x62 ✓
```

```text
EmergencyStop   seq=0xFF
  bytes (5):    02 04 FF 00 F9
                ^^ ^^ ^^ ^^ ^^
                |  |  |  |  └── XOR = 0xF9
                |  |  |  └───── payload_len = 0
                |  |  └──────── seq         = 0xFF
                |  └─────────── msg_type    = 4 (EmergencyStop)
                └────────────── version     = 2
```

```text
Response        seq=0x10  status=Moving(1)  info=85 (battery %)
  bytes (7):    02 02 10 02 01 55 46
                            ^^ ^^
                            |  └── info   = 0x55 = 85
                            └───── status = 1 (Moving)
  XOR check:    02^02^10^02^01^55 = 0x46 ✓
```

```text
Telemetry       seq=0x42
                battery=85  speed=70  heading=128  vx=100  vy=0  omega=0
  bytes (11):   02 03 42 06 55 46 80 64 00 00 B2
                            ^^ ^^ ^^ ^^ ^^ ^^
                            |  |  |  |  |  └── omega   = 0
                            |  |  |  |  └───── vy      = 0
                            |  |  |  └──────── vx      = +100
                            |  |  └─────────── heading = 128 (≈180°)
                            |  └────────────── speed   = 70
                            └───────────────── battery = 85 %
  XOR check:    02^03^42^06^55^46^80^64^00^00 = 0xB2 ✓
```

> **Sanity-check snippet** (run with `cargo test -p protocol` after adding
> the test): roundtrip every example through `RadioPacket::from_bytes(...)`
> and assert `to_bytes()` reproduces the same buffer — this catches
> endianness or padding regressions in two lines of code.

---

## Controller design

The controller spawns five Embassy tasks plus the main fusion loop:

| Task              | File           | Output                                         |
|-------------------|----------------|------------------------------------------------|
| `joystick_task`   | `joystick.rs`  | `JOYSTICK_MOTION_CHANNEL` (vx, vy)             |
| `tilt_task`       | `tilt.rs`      | `TILT_MOTION_CHANNEL`     (vx, vy)             |
| `button_task`     | `button.rs`    | `OMEGA_CHANNEL`            (omega)             |
| `mode_switch_task`| `button.rs`    | `mode::set()` + `MODE_CHANGED` signal          |
| `radio_task`      | `radio.rs`     | `MOTION_TX_CHANNEL` consumer; sends packets    |

### Input pipelines

- **Joystick (default).** SAADC samples both axes at ~50 Hz, applies a ±30
  count dead-zone, remaps the remaining ±481 counts linearly to ±100, then
  passes them through an EMA filter (`α = 3/8`). When the active mode is
  *not* `Joystick`, samples are still consumed (to keep the filter warm)
  but **not** published to the channel.

- **Tilt.** LSM303AGR accelerometer at 100 Hz / ±2 g; we read X/Y, ignore Z.
  Holding the board logo-up: tilting forward maps to `+vx`, tilting right
  maps to `+vy`. Same dead-zone + EMA pipeline as the joystick. The X axis
  raw counts are inverted vs. the joystick so the user-perceived directions
  agree across both modes.

- **Buttons.** C → omega CCW (`-100`), D → omega CW (`+100`). Holding both
  cancels out (anti-spin brake). To avoid jerks, omega ramps toward the
  target by `OMEGA_STEP = 20` per 20 ms tick (so it takes ~100 ms to reach
  full rotation).

- **Mode switch.** A button (P13) toggles between `Joystick` and `Tilt`
  with a 50 ms software debounce. On every toggle the fusion loop also
  zeroes `vx` / `vy` immediately so a stale tilt sample can’t keep the
  car rolling after you switch modes.

### Fusion loop

`main.rs` simply `select4`s over the three input channels plus a 200 ms
heartbeat timer / mode-changed signal, and re-emits the latest combined
`MotionPayload` to the radio task. This guarantees the car always
receives a fresh command (or `0,0,0` once you let go), never a stale one.

---

## Car design

`car/src/main.rs` keeps things deliberately small:

```rust
let radio = radio::init(p.RADIO);
spawner.spawn(radio::radio_rx_task(radio).unwrap());

let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;

loop {
    let motion = radio::MOTION_CHANNEL.receive().await;
    motor_driver.apply_motion(&motion).await;
}
```

- **`radio::radio_rx_task`** validates the version, dispatches by
  `MessageType`, ACKs `Motion` / `EmergencyStop`, mirrors `Heartbeat`s
  back, and pushes successfully parsed `MotionPayload`s onto
  `MOTION_CHANNEL`.
- **`MotorDriver::apply_motion`** runs the inverse kinematics, normalises,
  scales `[-100..100]` to `[-4095..4095]`, then drives M1–M4 via the
  `MotorBit` → `PCA9685` stack.
- The `MotorBit` abstraction also exposes `set_servo_angle` /
  `set_servo_duty` for S1–S8, so adding a pan/tilt camera or a gripper is
  just a matter of grabbing `motor_driver.twim_mut()` and instantiating
  a `Pca9685::resume(...)` + `MotorBit::new(...)` pair.

---

## Pin map (cheat-sheet)

### Controller (micro:bit v2)

| Function                       | Edge connector | nRF52833 GPIO | Direction / config  |
|--------------------------------|----------------|---------------|---------------------|
| Joystick Y axis (up = 1023)    | **P1**         | `P0.03` (AIN2)| SAADC single-ended  |
| Joystick X axis (right = 1023) | **P2**         | `P0.04` (AIN3)| SAADC single-ended  |
| Right joystick X (omega)       | _TBD_          | `_TBD_` (AINx)| SAADC, feature `right-stick-hw` |
| Right joystick Y (reserved)    | _TBD_          | `_TBD_` (AINx)| SAADC, feature `right-stick-hw` |
| Shield button **A** (mode tgl) | **P13**        | `P0.17`       | Input, `Pull::Up`   |
| Shield button **B** (reserved) | **P14**        | `P0.01`       | (unused)            |
| Shield button **C** (omega CCW)| **P16**        | `P0.09`       | Input, `Pull::Up`   |
| Shield button **D** (omega CW )| **P15**        | `P0.13`       | Input, `Pull::Up`   |
| Accelerometer SCL              | internal       | `P0.08`       | TWIM @ 100 kHz      |
| Accelerometer SDA              | internal       | `P0.16`       | TWIM @ 100 kHz      |
| Status LED (top-left of grid)  | row1/col1      | `P0.21`/`P0.28`| GPIO output        |

### Car (micro:bit v2 + MotorBit shield)

| Function                       | Edge connector | nRF52833 GPIO | Notes                       |
|--------------------------------|----------------|---------------|-----------------------------|
| I²C SCL → PCA9685              | **P19**        | `P0.26`       | TWIM0                       |
| I²C SDA → PCA9685              | **P20**        | `P1.00`       | TWIM0                       |
| PCA9685 channels CH0/CH1       | (on shield)    | —             | Motor M1 (FL) +/- terminals |
| PCA9685 channels CH2/CH3       | (on shield)    | —             | Motor M2 (FR) +/- terminals |
| PCA9685 channels CH4/CH5       | (on shield)    | —             | Motor M3 (RL) +/- terminals |
| PCA9685 channels CH6/CH7       | (on shield)    | —             | Motor M4 (RR) +/- terminals |
| PCA9685 channels CH8..CH15     | (on shield)    | —             | Servo headers S1..S8        |
| Status LED                     | row1/col1      | `P0.21`/`P0.28`| Lit while a motion ≠ 0      |

> **Wheel layout (top view, motor → corner mapping):**
> `M1 = front-left`, `M2 = front-right`, `M3 = rear-left`, `M4 = rear-right`.
> If your chassis spins or strafes the wrong way, the easiest fix is to
> swap the two terminals on the offending motor — no code change required.

---

## Building & flashing

### Prerequisites

```bash
rustup target add thumbv7em-none-eabihf
cargo install cargo-make           # task runner used by Makefile.toml
cargo install probe-rs --features cli   # flasher + RTT viewer
# optional, for `cargo make size-*`
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

A debug probe is required to flash the firmware. The on-board **DAPLink**
of any micro:bit v2 works out of the box (just plug the USB cable);
external CMSIS-DAP / J-Link probes work too.

### One-shot build

```bash
cargo make build-all          # builds protocol + radio-core + car + controller
```

Or per-crate:

```bash
cargo make build-car          # cargo build --release -p car
cargo make build-controller   # cargo build --release -p controller
```

### Flash

Plug in the **car**'s micro:bit, then:

```bash
cargo make flash-car
```

Unplug it, plug in the **controller**'s micro:bit, then:

```bash
cargo make flash-controller
```

Both tasks invoke `probe-rs run --chip nRF52833_xxAA …`, which also
attaches RTT and streams `defmt` logs to your terminal.

### Useful extras

```bash
cargo make check              # cargo check --workspace
cargo make clippy             # cargo clippy --workspace -- -D warnings
cargo make fmt                # cargo fmt --all
cargo make fmt-check          # CI-friendly fmt check
cargo make size-car           # cargo size -p car --release -- -A
cargo make size-controller    # cargo size -p controller --release -- -A
cargo make clean              # cargo clean
```

---

## Driving the car

1. Power up the **car** (battery on the MotorBit shield).
2. Power up the **controller** (USB power bank or coin-cell pack).
3. Wait ~1 second for both radios to come up — the top-left LED on each
   board lights up once it sees activity.
4. Default mode is **Joystick**:
   - Push the stick **forward / back** → car drives forward / backward.
   - Push **left / right** → car strafes sideways.
   - **Diagonal** push → diagonal Mecanum motion.
5. Press the shield's **A button** to toggle to **Tilt** mode:
   - Hold the controller logo-up and tilt to drive.
   - Press A again to switch back to the joystick.
6. **C / D buttons** add rotation on top of either translation source:
   - C alone → spin counter-clockwise.
   - D alone → spin clockwise.
   - C + D → snap rotation back to zero.
7. **Right joystick (planned, optional).** A second analog stick can be
   wired up later to provide continuous `omega` control. The firmware
   already runs the full pipeline against a mock source by default;
   enable Cargo feature `right-stick-hw` and finish the constructor in
   `controller/src/joystick_right.rs` once the hardware is in place.
   The right stick and the C/D buttons are then **summed and clamped
   to ±100**, so the buttons act as fine trim on top of the stick.
8. Letting go of all inputs immediately sends `(0, 0, 0)`; the car halts.
9. The car also halts on its own if it stops hearing from the controller
   for `HEARTBEAT_TIMEOUT_MS` (500 ms by default — set in
   `radio-core`).

---

## Logging & debugging

- All logs use [`defmt`] over RTT. Default level is `debug` (set in
  `Makefile.toml` via `DEFMT_LOG=debug`).
- `cargo make flash-*` automatically attaches RTT, so any `info!` /
  `trace!` / `error!` from the firmware appears in the same terminal.
- To get more or less verbose output for a single run, override the
  variable: `DEFMT_LOG=trace cargo make flash-controller`.
- For deeper inspection use `probe-rs attach --chip nRF52833_xxAA` to
  reattach to a running board without re-flashing.

[`defmt`]: https://defmt.ferrous-systems.com/

---

## Troubleshooting

| Symptom                                                                | Likely cause                                                                                  | Fix                                                                                                                                  |
|------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| `probe-rs` says **"no probe was found"**                               | DAPLink not enumerated; another tool is holding the port; cable is power-only                 | Replug the USB cable, close `pyOCD` / Arduino IDE / micro:bit web flasher, try a data cable                                          |
| Flashing succeeds but **no logs appear**                               | RTT not attached, or `DEFMT_LOG` filtered everything out                                      | Run `DEFMT_LOG=debug cargo make flash-controller`, or `probe-rs attach` after flashing                                               |
| Car ignores commands, controller LED still blinks                      | Different `RADIO_CHANNEL` / `PROTOCOL_VERSION` flashed on each side                           | Re-flash both boards from the **same** workspace checkout                                                                            |
| Car drives but **wrong direction** (e.g. "forward" goes left)          | Motor terminals on one wheel are swapped, or M1–M4 don't match FL/FR/RL/RR                    | Swap the two wires on the offending motor; do **not** rewire in code                                                                 |
| Spinning instead of strafing when pushing the stick sideways           | Two diagonally-opposite wheels are reversed                                                   | Identify the pair from the `defmt` motor logs and flip both                                                                          |
| Car keeps creeping after you let go of the stick                       | Joystick centre is offset (manufacturing tolerance) and exceeds `DEAD_ZONE`                   | Increase `DEAD_ZONE` in `controller/src/joystick.rs` (try `40`)                                                                      |
| Tilt mode feels too sensitive / not sensitive enough                   | `FULL_TILT` doesn't match how far you actually tilt                                           | Tweak `FULL_TILT` and `DEAD_ZONE` in `controller/src/tilt.rs`                                                                        |
| Car halts every ~500 ms even with the stick held                       | Heartbeat is missing — radio range / interference                                             | Move the boards closer, change `RADIO_CHANNEL`, or raise `TX_POWER` in `radio-core/src/lib.rs`                                       |
| `cargo build` fails with **"target 'thumbv7em-none-eabihf' not found"** | Cross-compile target missing                                                                  | `rustup target add thumbv7em-none-eabihf`                                                                                            |
| `cargo install probe-rs` fails on macOS                                | `libusb` / `libudev` headers missing                                                          | `brew install libusb pkg-config`                                                                                                     |
| Controller works but tilt mode never engages                           | Shield button A wiring or `Pull::Up` mismatch                                                 | Check P13 with a multimeter; the line should be high at idle, low when pressed                                                       |

---

## Extending the project

The codebase is intentionally split so each axis of extension lives in
exactly one place.

### 1. Add a new input source on the controller

Suppose you want to add an IR remote or a second joystick.

1. Create `controller/src/ir.rs` with:
   - An `init(...)` function that takes the peripherals you need and
     returns a configured driver.
   - A public `pub static IR_MOTION_CHANNEL: Channel<...> = Channel::new();`.
   - A `#[embassy_executor::task] pub async fn ir_task(...)` that samples
     and pushes into the channel **only while its mode is active**
     (look at `joystick.rs` for the `mode::current() == InputMode::…`
     guard pattern).
2. Add a new variant to `mode::InputMode` (e.g. `Ir = 2`) and update
   `mode::toggle()` to cycle through the new state.
3. In `controller/src/main.rs`:
   - Spawn `ir_task` in `main` next to the existing `joystick_task` /
     `tilt_task`.
   - Extend the `select4` (or upgrade to `select` with more arms) so the
     fusion loop also reacts to `IR_MOTION_CHANNEL`.
4. Done — the radio path doesn’t need any change, because the fusion loop
   still emits a single `MotionPayload`.

### 2. Add a new output / actuator on the car

For example a servo-driven gripper or pan-tilt camera.

1. The `MotorBit` abstraction already exposes `set_servo_angle` and
   `set_servo_duty` for channels S1–S8. Add a small wrapper module
   (e.g. `car/src/gripper.rs`) that takes
   `&mut Twim<'static>` from `MotorDriver::twim_mut()` and drives the
   relevant servo.
2. If you need a *new* command from the controller, add a payload type
   in `protocol/src/lib.rs` (see “Add a new message type” below) and
   plumb it through `car/src/radio.rs`'s `handle_received_packet`.

### 3. Add a new message type to the protocol

1. In `protocol/src/lib.rs`:
   - Append a new variant to `MessageType` (give it the next free `u8`).
   - Add the matching arm in `RadioHeader::from_bytes` (otherwise old
     payloads will be silently dropped).
   - Define a `FooPayload` struct with `SIZE`, `to_bytes`, `from_bytes`.
   - Add a `create_foo_packet(...)` helper to mirror the existing ones.
2. Bump `PROTOCOL_VERSION` so old firmware on either side rejects the
   new layout cleanly instead of mis-parsing it.
3. In `controller/src/radio.rs` and/or `car/src/radio.rs`, extend
   `handle_received_packet` to match on the new `MessageType` and
   forward the parsed payload to its consumer over a new
   `embassy_sync` channel.

### 4. Tune the driving feel

| Knob                                    | File                          | Effect                                  |
|-----------------------------------------|-------------------------------|-----------------------------------------|
| `GEOMETRY_FACTOR_K`                     | `car/src/motor.rs`            | More/less rotational authority          |
| `SPEED_TO_PWM_SCALE`                    | `car/src/motor.rs`            | Top speed (max PWM duty)                |
| `DEAD_ZONE` / `SMOOTH_NUM` / `SMOOTH_DEN` (joystick) | `controller/src/joystick.rs` | Stick null zone & responsiveness  |
| `DEAD_ZONE` / `FULL_TILT`               | `controller/src/tilt.rs`      | Tilt sensitivity & full-scale angle     |
| `OMEGA_MAX` / `OMEGA_STEP`              | `controller/src/button.rs`    | Rotation strength & ramp time           |
| `RADIO_CHANNEL` / `TX_POWER`            | `radio-core/src/lib.rs`       | RF coexistence & range                  |
| `HEARTBEAT_TIMEOUT_MS`                  | `radio-core/src/lib.rs`       | How long the car coasts after RF loss   |

### 5. Port to a different chassis

The whole car-side abstraction stack is `MotorDriver → MotorBit → PCA9685`.
- For a **non-Mecanum** drivetrain (differential / Ackermann), keep
  the protocol and `MotorBit`/`PCA9685` layers and rewrite only
  `MotorDriver::apply_motion` with the new mixing equations.
- For a **non-PCA9685** motor driver (e.g. TB6612 wired straight to the
  micro:bit), replace `motor.rs` + `motorbit.rs` + `pca9685.rs` and keep
  the rest as-is. As long as the new driver consumes `MotionPayload`,
  the controller and the radio stack don’t need to change at all.

### 6. Port to a different MCU

`embassy-nrf` is the only nRF-specific dependency on the firmware side,
and it’s isolated to `radio-core`, `tilt.rs`, `joystick.rs`,
`button.rs`, `motor.rs`. To target a different Embassy-supported MCU:

1. Swap `embassy-nrf` for the relevant `embassy-*` HAL in `Cargo.toml`.
2. Re-implement the four hardware modules above against the new HAL
   (each one is small — under ~250 LOC).
3. `protocol` and `radio-core`'s public API can stay the same provided
   the new MCU also has an 802.15.4-capable radio; otherwise replace
   `radio-core` with a transport of your choice (BLE, ESP-NOW, RFM69,
   …) — the application code only depends on its `init` / `send_packet`
   surface.

---

## Contributing

Contributions are welcome — bug reports, hardware variants, new input
sources, alternate transports, and documentation improvements all land
through the same lightweight workflow.

### Workflow

1. **Fork** this repository and create a feature branch off `main`:
   ```bash
   git checkout -b feat/short-description
   ```
2. **Make your change.** Keep each PR focused on one logical concern;
   if you find yourself touching `protocol/`, `controller/` and `car/`
   in the same commit, that's a hint to split it up.
3. **Run the local quality gate** before pushing:
   ```bash
   cargo make fmt-check
   cargo make clippy
   cargo make build-all
   ```
   All three must pass with **zero** warnings — `clippy` is invoked with
   `-D warnings`, so any new lint will fail CI.
4. **Test on real hardware** when the change is functional (motor
   mixing, radio behaviour, input handling). Capture a short clip and
   drop it into `docs/media/` so reviewers don't have to re-flash to
   see what you mean.
5. **Open a PR** with the template below and request a review.

### Coding conventions

- **Rust edition / toolchain:** `2021` edition, stable toolchain pinned
  by `rust-toolchain.toml` (if present). No nightly-only features unless
  they're already in use.
- **`#![no_std]` everywhere on the firmware side.** Use
  `heapless`, `static_cell`, and stack buffers — never `alloc`.
- **Indentation:** 2 spaces, matching `rustfmt.toml` and the existing
  files. Run `cargo make fmt` before committing.
- **Naming:** modules are `snake_case`, types are `UpperCamelCase`,
  constants are `SCREAMING_SNAKE_CASE`. Public items get a doc comment;
  inner helpers get a one-line `//` comment when their purpose isn't
  obvious.
- **Logging:** `defmt` only. Prefer `info!` / `debug!` / `trace!` /
  `warn!` / `error!` according to severity, and **never** include
  user-controlled data in a log message that runs in a hot loop.
- **Comments and identifiers are written in English.** This matches the
  rest of the codebase and keeps the diff readable for non-Chinese
  contributors.
- **No silent breaking changes** to `protocol/`. If a PR changes the
  wire format it **must** bump `PROTOCOL_VERSION` in
  [`protocol/src/lib.rs`](protocol/src/lib.rs); reviewers will reject
  PRs that don't.
- **Channels & signals over shared mutable state.** New tasks should
  communicate through `embassy_sync::channel::Channel` /
  `Signal` — see `controller/src/joystick.rs` for the canonical pattern.

### Commit message format

We follow a relaxed [Conventional Commits] style. The first line is
`type(scope): summary`, and the body (optional) explains _why_, not
_what_:

```text
feat(controller): add IR remote as a third input source
fix(car): prevent motor stutter when payload arrives twice in 1 ms
perf(radio): shrink heartbeat to 4 bytes by dropping seq field
docs(readme): add tilt-mode demo GIF
refactor(motor): collapse mixing matrix into a const lookup table
test(protocol): roundtrip every documented hex example
```

Valid `type` values: `feat`, `fix`, `perf`, `refactor`, `docs`, `test`,
`chore`, `ci`. Valid `scope`s mirror the crate / module: `protocol`,
`radio-core`, `controller`, `car`, `radio`, `motor`, `tilt`,
`joystick`, `button`, `mode`, `readme`.

[Conventional Commits]: https://www.conventionalcommits.org/en/v1.0.0/

### Pull-request template

Copy this into the PR description and fill in the blanks:

```markdown
### Summary
<!-- One paragraph: what changed and why. -->

### Hardware tested on
- [ ] micro:bit v2 (controller)
- [ ] micro:bit v2 (car)
- [ ] MotorBit shield v__
- [ ] N/A — protocol / docs only

### Verification
- [ ] `cargo make fmt-check` passes
- [ ] `cargo make clippy` passes (no new warnings)
- [ ] `cargo make build-all` passes
- [ ] Drove the car for ≥ 60 s without anomalies
- [ ] (If protocol changed) bumped `PROTOCOL_VERSION` and updated
      [Hex examples](#hex-examples)

### Screenshots / clips
<!-- Drop into docs/media/ and link them here. -->

### Notes for reviewers
<!-- Trade-offs, follow-ups, anything you punted on. -->
```

### Reporting bugs / requesting features

Open an issue with:

- **Hardware:** which micro:bit revision, which MotorBit revision,
  which battery / motors.
- **Firmware commit:** output of `git rev-parse --short HEAD` on both
  boards.
- **Reproduction:** smallest sequence of inputs that triggers the
  problem.
- **Logs:** the relevant `defmt` excerpt (`DEFMT_LOG=trace` is fine for
  bug reports; redact anything personal).

---

## Acknowledgements

- The amazing folks behind [Embassy] for making `async` embedded Rust
  pleasant to write.
- [`probe-rs`](https://probe.rs/) for one-command flashing + RTT.
- [`defmt`](https://defmt.ferrous-systems.com/) for cheap, structured
  logging on tiny MCUs.
- The [BBC micro:bit] educational foundation for an inexpensive,
  hackable platform with a built-in radio.
- Yahboom / ELECFREAKS-style **MotorBit** vendors for the PCA9685-based
  shield that makes Mecanum-wheel projects approachable.

[BBC micro:bit]: https://microbit.org/

---

## License

Distributed under the terms of the [LICENSE](LICENSE) file in this repository.
