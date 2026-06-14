# microbit-car

[English Version](README.md) | [中文版本](README_zh.md)

基于两块 [BBC micro:bit v2] 开发板（nRF52833）构建的无线**麦克纳姆轮全向移动小车**，使用 **`#![no_std]` 异步 Rust** 编写，运行在 [Embassy] 运行时之上。

一块 micro:bit 作为**小车**（通过 MotorBit 扩展板和 PCA9685 PWM 驱动器控制四个直流电机），另一块作为**控制器**（读取摇杆、板载加速度计和四个扩展板按钮，融合为单一运动向量并通过无线发送）。

两块开发板之间通过 **2.4 GHz IEEE 802.15.4** 直接通信，使用自定义的应用层协议——无需 Wi-Fi、无需 BLE、无需手机配合。

[BBC micro:bit v2]: https://tech.microbit.org/hardware/2-1-revision/
[Embassy]: https://embassy.dev/

---

## 目录

- [microbit-car](#microbit-car)
  - [目录](#目录)
  - [硬件清单](#硬件清单)
  - [仓库结构](#仓库结构)
  - [系统架构](#系统架构)
  - [通信协议](#通信协议)
    - [数据包帧格式](#数据包帧格式)
    - [消息类型](#消息类型)
    - [MotionPayload 语义](#motionpayload-语义)
    - [无线电配置](#无线电配置)
    - [十六进制示例](#十六进制示例)
  - [控制器设计](#控制器设计)
    - [输入管线](#输入管线)
    - [融合循环](#融合循环)
    - [状态指示器（RGB LED）](#状态指示器rgb-led)
  - [小车设计](#小车设计)
  - [诊断模式](#诊断模式)
  - [故障安全行为](#故障安全行为)
  - [硬件注意事项（蜂鸣器与 PWM 载波）](#硬件注意事项蜂鸣器与-pwm-载波)
    - [1. 启动时 P0\_02 保持低电平（蜂鸣器静音）](#1-启动时-p0_02-保持低电平蜂鸣器静音)
    - [2. PCA9685 载波为 1.5 kHz，而不是 50 Hz](#2-pca9685-载波为-15-khz而不是-50-hz)
  - [引脚映射（速查表）](#引脚映射速查表)
    - [控制器（micro:bit v2）](#控制器microbit-v2)
    - [小车（micro:bit v2 + MotorBit 扩展板）](#小车microbit-v2--motorbit-扩展板)
  - [构建与烧录](#构建与烧录)
    - [前置条件](#前置条件)
    - [一键构建](#一键构建)
    - [烧录](#烧录)
    - [实用额外命令](#实用额外命令)
  - [驾驶小车](#驾驶小车)
  - [日志与调试](#日志与调试)
  - [故障排除](#故障排除)
  - [扩展项目](#扩展项目)
    - [1. 在控制器上添加新的输入源](#1-在控制器上添加新的输入源)
    - [2. 在小车上添加新的输出/执行器](#2-在小车上添加新的输出执行器)
    - [3. 在协议中添加新的消息类型](#3-在协议中添加新的消息类型)
    - [4. 添加新的 RGB 指示器](#4-添加新的-rgb-指示器)
    - [5. 调整驾驶手感](#5-调整驾驶手感)
    - [6. 移植到不同的底盘](#6-移植到不同的底盘)
    - [7. 移植到不同的 MCU](#7-移植到不同的-mcu)
  - [贡献指南](#贡献指南)
    - [工作流程](#工作流程)
    - [编码规范](#编码规范)
    - [提交消息格式](#提交消息格式)
    - [拉取请求模板](#拉取请求模板)
    - [报告 Bug / 请求新功能](#报告-bug--请求新功能)
  - [致谢](#致谢)
  - [许可证](#许可证)

---

## 硬件清单

| 角色        | 板卡 / 模块                                            | 说明                                            |
|-------------|-----------------------------------------------------------|--------------------------------------------------|
| MCU（×2）    | BBC micro:bit v2（nRF52833，Cortex-M4F，2.4 GHz 无线电）    | 一块用于小车，一块用于控制器         |
| 电机扩展板| **MotorBit** 扩展板（PCA9685 over I²C @ `0x40`）  | 4× 直流电机（M1–M4）+ 8× 伺服接头（S1–S8） |
| 传动系统  | 4× 麦克纳姆轮 + 4× 有刷直流电机                  | 成对控制 PWM 通道             |
| 输入       | XY 拇指摇杆（10 位模拟），板载 LSM303AGR 加速度计 | 倾斜模式为数字式（I²C），摇杆为模拟式  |
| 按钮     | 扩展板上的 A / B / C / D 按钮（P13 / P14 / P16 / P15） | 内部上拉，低电平有效                   |
| 电源       | MotorBit 板上的 LiPo / 电池组                 | micro:bit 通过扩展板供电     |

> 只需要板载的 **micro:bit v2** 外设。MotorBit 扩展板仅用于**小车**侧；**控制器**使用扩展板按钮（P13–P16），但不需要电机部分。

---

## 仓库结构

```text
microbit-car/
├── Cargo.toml          # Cargo 工作区
├── Embed.toml          # probe-rs 默认配置（nRF52833_xxAA）
├── Makefile.toml       # cargo-make 任务（构建 / 烧录 / 大小 / clippy / fmt）
├── memory.x（每个 crate） # 链接器内存布局
│
├── protocol/           # #![no_std] 线格式 crate（两侧共享）
│   └── src/lib.rs      # MessageType, RadioPacket, MotionPayload, ...
│
├── radio-core/         # #![no_std] 共享无线电配置 + TX/RX 辅助函数
│   └── src/lib.rs      # init(), send_packet*, 重试策略, 通道号 #, ...
│
├── controller/         # 控制器固件（二进制 crate）
│   └── src/
│       ├── main.rs     # Embassy `main`，生成输入/无线电任务，融合循环
│       ├── radio.rs    # 802.15.4 TX/RX 任务，心跳，链路状态机
│       ├── joystick.rs # SAADC 采样，死区 + EMA 平滑
│       ├── tilt.rs     # LSM303AGR 加速度计驱动，倾斜 → vx/vy
│       ├── button.rs   # C/D 欧米伽按钮 + A 模式切换按钮
│       ├── mode.rs     # 输入模式仲裁（Joystick ⇄ Tilt）通过 atomic + Signal
│       ├── display.rs  # 5×5 LED 矩阵指示器（运动方向）
│       └── rgb.rs      # 扩展板上 4× WS2812 状态 LED（P8，LED0 显示链路状态）
│
├── car/                # 小车固件（二进制 crate）
    └── src/
        ├── main.rs        # Embassy `main`，电机驱动 + 无线电 RX 胶水代码 + RX 故障安全
        ├── radio.rs       # 802.15.4 RX 任务，分发 Motion / E-Stop / HB
        ├── diagnostic.rs  # 启动时 A 按钮电机接线扫描（M1→M2→M3→M4）
        ├── motor.rs       # MotorDriver：麦克纳姆逆运动学 + I²C 初始化
        ├── motorbit.rs    # MotorBit 扩展板抽象（4 直流电机 + 8 伺服）
        └── pca9685.rs     # 底层 PCA9685 PWM 驱动（载波 @ 1.5 kHz）
```

---

## 系统架构

```text
┌──────────────────────── 控制器（micro:bit v2）──────────────────────────┐
│                                                                              │
│   joystick_task ─┐                                                           │
│   tilt_task     ─┤── (vx, vy) ─┐                                             │
│                  │             ├──► 融合循环（main.rs）──► MOTION_TX ─►   │
│   button_task ──── omega ─────┘                                       │     │
│                                                                       ▼     │
│                                                              radio_task ───┐│
│   mode_switch_task ──► AtomicU8 + Signal（InputMode）◄──（门控摇杆/倾斜）│
└───────────────────────────────────────────────────────────────────┬──────┬──┘
                                                                    │      │
                                                            802.15.4│ ch15 │ACK / 心跳
                                                                    ▼      ▲
┌────────────────────────────── 小车（micro:bit v2）───────────────────────────┐
│                                                                              │
│   radio_rx_task ──► MOTION_CHANNEL ──► 主循环 ──► MotorDriver.apply_motion│
│                                                          │                   │
│                                                          ▼                   │
│                                          ┌── 逆运动学 ─┐           │
│                                          │  (vx,vy,ω) → 4 电机 │           │
│                                          └────────┬──────────────┘           │
│                                                   ▼                          │
│                                  MotorBit ──► PCA9685 ──► I²C ──► 电机     │
└──────────────────────────────────────────────────────────────────────────────┘
```

关键设计属性：

- **所有任务间通信都使用 `embassy_sync::channel::Channel` 或 `Signal`**。无全局可变状态，无 `Mutex<RefCell<...>>` 间接引用。
- **控制器融合循环**是传出 `MotionPayload` 的唯一事实来源：它采样当前活动的输入源（`Joystick` 或 `Tilt`）获取 `(vx, vy)`，并始终合并来自 C/D 按钮的最新 `omega`。
- **模式切换**存储在 `AtomicU8` 中，因此任何任务都可以廉价读取，并且 `Signal<InputMode>` 允许融合循环在模式翻转的瞬间立即清零过期速度。
- **小车**在主任务中运行其电机驱动程序，仅将无线电任务视为 `MotionPayload` 的有界通道的生产者。这将 I²C 密集型代码保持在单个执行器任务上，并避免 TWIM 外设上的竞争。

---

## 通信协议

定义在 [`protocol/src/lib.rs`](protocol/src/lib.rs)。一个头部，几个固定大小的负载，和一个 XOR 校验和——足够小以适配单个 32 字节的 802.15.4 帧。

### 数据包帧格式

```text
 0       1         2     3           4 .. 4+N-1     4+N
┌───────┬─────────┬─────┬───────────┬──────────────┬────────┐
│ ver   │ msgtype │ seq │ payload_n │   payload    │ XOR    │
└───────┴─────────┴─────┴───────────┴──────────────┴────────┘
   1B       1B       1B      1B          N 字节        1B
```

- `ver`         — `PROTOCOL_VERSION`（当前为 `2`）；接收方丢弃版本不匹配的数据包。
- `msgtype`     — 参见 [`MessageType`](protocol/src/lib.rs)。
- `seq`         — 在 255 处回绕，仅用于跟踪 / 去重。
- `payload_n`   — `0..=MAX_PAYLOAD_SIZE`（= `32 - 4 - 1` = 最大 27 字节）。
- `XOR`         — 覆盖 `header + payload` 的软件校验和。802.15.4 硬件 CRC 在 HAL 层也是激活的，因此这是额外的应用级完整性检查。

### 消息类型

| 类型            | 方向        | 负载                | 用途                                |
|-----------------|------------------|------------------------|----------------------------------------|
| `Heartbeat`     | 双向        | 无                   | 存活 / 连接指示器（5 Hz） |
| `Motion`        | 控制器 → 小车 | `MotionPayload`（3 字节）  | `(vx, vy, omega)` 速度向量      |
| `EmergencyStop` | 控制器 → 小车 | 无                   | 强制小车停止，重试 5 次      |
| `Response`      | 小车 → 控制器 | `ResponsePayload`（2 字节） | `CarStatus` + 额外信息字节          |
| `Telemetry`     | 小车 → 控制器 | `TelemetryPayload`（6 字节） | 电池 / 速度 / 航向 / vx/vy/ω    |

> **ACK 策略：**小车**不会 ACK 每个 `Motion` 数据包**。控制器以 ~50 Hz 的频率流式传输运动数据；对每个数据包进行 ACK 会将无线电变成 50 Hz TX 突发源，micro:bit 的板载降压调节器和 MotorBit 的电源滤波电感会将其作为微弱但可听见的低频嗡嗡声（电磁感应声学噪声 / 线圈啸叫）拾取。链路活跃度完全由 `Heartbeat` 回复路径承载（~5 Hz），控制器上的 RGB 指示器用它来驱动其 `ConnectionState`。`EmergencyStop` 仍然单独 ACK，因为它是一个一次性关键事件。

### MotionPayload 语义

```rust
pub struct MotionPayload {
  pub vx: i8,    // 前进(+) / 后退(-)，   范围 -100..=100
  pub vy: i8,    // 向右横向移动(+) / 向左(-)，  范围 -100..=100
  pub omega: i8, // 顺时针(+) / 逆时针(-) 旋转，   范围 -100..=100
}
```

小车侧的麦克纳姆逆运动学（参见 `car/src/motor.rs::apply_motion`）：

```text
motor_FL（M1）= vx + vy - k * omega
motor_FR（M2）= vx - vy + k * omega
motor_RL（M3）= vx - vy - k * omega
motor_RR（M4）= vx + vy + k * omega
```

符号约定：`vx>0 = 前进`，`vy>0 = 向右横向移动`，`omega>0 = 顺时针`（从上方看）。

计算出四个原始值后，将其归一化以适配 `[-100, 100]`，然后映射到 PCA9685 范围 `[-4095, 4095]`。`k` 是几何因子，在 `motor.rs` 中作为 `GEOMETRY_FACTOR_K` 公开（默认为 `1`）；根据您的底盘需要自行调整。

### 无线电配置

定义在 [`radio-core/src/lib.rs`](radio-core/src/lib.rs)：

| 参数                | 值          | 原因                                    |
|--------------------------|----------------|----------------------------------------|
| `RADIO_CHANNEL`          | `15`           | 位于常用 Wi-Fi 通道之间     |
| `TX_POWER`               | `+4 dBm`       | 室内范围充足                |
| `MAX_TX_RETRIES`         | `3`            | 普通命令的最佳努力重试  |
| `MAX_EMERGENCY_RETRIES`  | `5`            | 紧急停止的更高可靠性  |
| `HEARTBEAT_INTERVAL_MS`  | `200`          | 控制器 → 小车存活 ping         |
| `HEARTBEAT_TIMEOUT_MS`   | `600`          | 3× 心跳间隔；双向故障安全：小车在此无线电静默时间后停止，控制器的 RGB 指示器变红 |

### 十六进制示例

以下所有示例均由实际的 `RadioPacket::to_bytes()` 实现生成（XOR 覆盖 `header || payload`）。当您编写嗅探器或第三方模拟器时，它们可用作黄金测试向量。

```text
Heartbeat       seq=0x05
  字节（5）：    02 00 05 00 07
                ^^ ^^ ^^ ^^ ^^
                |  |  |  |  └── XOR(02,00,05,00) = 0x07
                |  |  |  └───── payload_len = 0
                |  |  └──────── seq        = 0x05
                |  └─────────── msg_type   = 0（Heartbeat）
                └────────────── version    = 2
```

```text
Motion（全速前进）            seq=0x2A  vx=+100  vy=0  omega=0
  字节（8）：    02 01 2A 03 64 00 00 4E
                            ^^ ^^ ^^
                            |  |  └── omega = 0
                            |  └───── vy    = 0
                            └──────── vx    = 0x64 = +100
  XOR 检查：    02^01^2A^03^64^00^00 = 0x4E ✓
```

```text
Motion（后退 + 向右横向移动 + 逆时针）   seq=0x2B  vx=-100  vy=+50  omega=-25
  字节（8）：    02 01 2B 03 9C 32 E7 62
                            ^^ ^^ ^^
                            |  |  └── omega = 0xE7 = -25（i8）
                            |  └───── vy    = 0x32 = +50
                            └──────── vx    = 0x9C = -100（i8）
  XOR 检查：    02^01^2B^03^9C^32^E7 = 0x62 ✓
```

```text
EmergencyStop   seq=0xFF
  字节（5）：    02 04 FF 00 F9
                ^^ ^^ ^^ ^^ ^^
                |  |  |  |  └── XOR = 0xF9
                |  |  |  └───── payload_len = 0
                |  |  └──────── seq         = 0xFF
                |  └─────────── msg_type    = 4（EmergencyStop）
                └────────────── version     = 2
```

```text
Response        seq=0x10  status=Moving(1)  info=85（电池 %）
  字节（7）：    02 02 10 02 01 55 46
                            ^^ ^^
                            |  └── info   = 0x55 = 85
                            └───── status = 1（Moving）
  XOR 检查：    02^02^10^02^01^55 = 0x46 ✓
```

```text
Telemetry       seq=0x42
                电池=85  速度=70  航向=128  vx=100  vy=0  omega=0
  字节（11）：   02 03 42 06 55 46 80 64 00 00 B2
                            ^^ ^^ ^^ ^^ ^^ ^^
                            |  |  |  |  |  └── omega   = 0
                            |  |  |  |  └───── vy      = 0
                            |  |  |  └──────── vx      = +100
                            |  |  └─────────── 航向 = 128（≈180°）
                            |  └────────────── 速度   = 70
                            └───────────────── 电池 = 85 %
  XOR 检查：    02^03^42^06^55^46^80^64^00^00 = 0xB2 ✓
```

> **健全性检查片段**（在添加测试后使用 `cargo test -p protocol` 运行）：将每个示例通过 `RadioPacket::from_bytes(...)` 往返测试，并断言 `to_bytes()` 重现相同的缓冲区——这可以在两行代码中捕获字节序或填充回归。

---

## 控制器设计

控制器生成六个 Embassy 任务以及主融合循环：

| 任务              | 文件           | 输出                                         |
|-------------------|----------------|------------------------------------------------|
| `joystick_task`   | `joystick.rs`  | `JOYSTICK_MOTION_CHANNEL`（vx, vy）             |
| `tilt_task`       | `tilt.rs`      | `TILT_MOTION_CHANNEL`     （vx, vy）             |
| `button_task`     | `button.rs`    | `OMEGA_CHANNEL`            （omega）             |
| `mode_switch_task`| `button.rs`    | `mode::set()` + `MODE_CHANGED` 信号          |
| `radio_task`      | `radio.rs`     | `MOTION_TX_CHANNEL` 消费者；发送数据包，还在 `RGB_STATE` 上发布链路状态 |
| `rgb_task`        | `rgb.rs`       | 从最新的 `RGB_STATE` 渲染四个 WS2812 LED |

### 输入管线

- **摇杆（默认）。** SAADC 以 ~50 Hz 的频率采样两个轴，应用 ±30 计数死区，将剩余的 ±481 计数线性映射到 ±100，然后通过 EMA 滤波器（`α = 3/8`）。当活动模式*不是* `Joystick` 时，仍然消耗样本（以保持滤波器预热），但**不**发布到通道。

- **倾斜。** LSM303AGR 加速度计以 100 Hz / ±2 g 运行；我们读取 X/Y，忽略 Z。将开发板标志朝上：向前倾斜映射为 `+vx`，向右倾斜映射为 `+vy`。与摇杆相同的死区 + EMA 管线。X 轴原始计数与摇杆相比反转，因此两种模式之间的用户感知方向一致。

- **按钮。** C → omega 逆时针（`-100`），D → omega 顺时针（`+100`）。同时按住两者会抵消（防旋转制动）。为避免抖动，omega 在每个 20 ms 滴答声中向目标斜坡变化 `OMEGA_STEP = 20`（因此达到全旋转大约需要 ~100 ms）。

- **模式切换。** A 按钮（P13）在 `Joystick` 和 `Tilt` 之间切换，带有 50 ms 软件去抖。每次切换时，融合循环也会立即将 `vx` / `vy` 清零，以便在切换模式后，过期的倾斜样本不会让小车继续滚动。

### 融合循环

`main.rs` 只需在三个输入通道加上 200 ms 心跳定时器 / 模式更改信号上进行 `select4`，并重新发出最新的组合 `MotionPayload` 到无线电任务。这保证了小车始终接收到新的命令（或者松开时收到 `0,0,0`），永远不会收到过时的命令。

### 状态指示器（RGB LED）

控制器的扩展板承载**四个菊花链连接的 WS2812 LED**，连接到边缘连接器 **P8**（= `P0_10`）上的单个数据线。它们由 nRF52833 的 `PWM0` 外设通过 EasyDMA 驱动，因此满足严格的 800 kHz / ±150 ns WS2812 时序，而无需 CPU 位操作：

- PWM 基时钟 16 MHz，`max_duty = 20` ⇒ 一个周期恰好是 1.25 µs（= 一个 WS2812 位时隙）。
- `T0H = 5 / 20`（≈ 0.31 µs），`T1H = 13 / 20`（≈ 0.81 µs）——两者都在数据手册窗口内。
- 每个帧后，线路保持低电平至少 80 µs 以锁存。

**目前仅渲染 LED0**，反映无线电链路状态：

| 状态          | LED0 颜色      | 含义                                                       |
|----------------|------------------|---------------------------------------------------------------|
| `Connecting`   | 呼吸琥珀色  | 无线电已启动，但小车尚未回复                    |
| `Connected`    | 绿色      | 在上次 `HEARTBEAT_TIMEOUT_MS` 内收到了心跳回复 |
| `Disconnected` | 红色          | 在 `HEARTBEAT_TIMEOUT_MS`（= 600 ms）内没有心跳回复      |

LED 1–3 保持空白，但驱动程序始终移位出所有四个像素（WS2812 是单线菊花链——无法仅寻址第一个）。因此，添加新的指示器是纯粹的渲染端更改：扩展 [`RgbState`](controller/src/rgb.rs) 和 `render` 函数，驱动程序本身不需要知道。

---

## 小车设计

`car/src/main.rs` 刻意保持精简：

```rust
// 启动时将引脚 P0_02 拉低以静音板载蜂鸣器线路
// 无论 MotorBit 滑动开关位置如何（参见
// "硬件注意事项" 下方）。
let _buzzer_silence = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);

// 可选：在上电后前 2 秒内点击 A 按钮（P0_14）
// 进入电机接线诊断模式，而不是正常的无线电循环。
if diagnostic::is_diagnostic_requested(p.P0_14).await {
    let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;
    diagnostic::run(&mut motor_driver).await; // -> !
}

let radio = radio::init(p.RADIO);
spawner.spawn(radio::radio_rx_task(radio).unwrap());
let mut motor_driver = motor::MotorDriver::new(p.TWISPI0, p.P0_26, p.P1_00).await;

loop {
    // 在新鲜运动数据包和 100 ms 看门狗滴答之间进行 `select`，
    // 以强制执行链路丢失故障安全（参见 "故障安全行为"）。
    match select(radio::MOTION_CHANNEL.receive(), Timer::after_millis(100)).await {
        Either::First(motion) => motor_driver.apply_motion(&motion).await,
        Either::Second(_)     => check_failsafe_and_maybe_stop(&mut motor_driver).await,
    }
}
```

- **`radio::radio_rx_task`** 验证版本，按 `MessageType` 分发，ACK `Heartbeat` 和 `EmergencyStop`，并将成功解析的 `MotionPayload` 推送到 `MOTION_CHANNEL`。**它不会 ACK `Motion`**——参见[消息类型](#消息类型)下的 "ACK 策略" 说明。
- **`MotorDriver::apply_motion`** 运行逆运动学，归一化，将 `[-100..100]` 缩放到 `[-4095..4095]`，然后通过 `MotorBit` → `PCA9685` 堆栈驱动 M1–M4。
- `MotorBit` 抽象还公开了 `set_servo_angle` / `set_servo_duty`（用于 S1–S8），因此添加云台/倾斜摄像头或机械爪只需要获取 `motor_driver.twim_mut()` 并实例化 `Pca9685::resume(...)` + `MotorBit::new(...)` 对。

---

## 诊断模式

当您接线新的底盘时，第一个问题总是*"M1 是否实际上是左前轮，其 `+` 方向是否实际上是前进？"* 小车固件附带内置的电机接线扫描，可以准确回答该问题，**无需涉及无线电链路或控制器**。

**如何进入：** 打开小车电源并在前 **2 秒**内**点击板载 A 按钮**（micro:bit 上的左侧 tactile 按钮，GPIO `P0_14`）。短按即可——无需按住或与重置键协调。

**它做什么：** 固件完全跳过无线电初始化，并运行一个无限循环，以 ~50% PWM 占空比驱动**一次一个电机**：

```text
对于每个循环：
  M1 左前  ：前进 1 秒，停止，后退 1 秒，停止
  M2 右前  ：前进 1 秒，停止，后退 1 秒，停止
  M3 左后   ：前进 1 秒，停止，后退 1 秒，停止
  M4 右后 ：前进 1 秒，停止，后退 1 秒，停止
  暂停 2 秒，重复
```

每个阶段都通过 `defmt` 公布，因此您可以将日志与物理旋转的轮子关联起来。**此处的 "前进" 是 PCA9685 报告为正的方向**——如果日志显示 `FORWARD` 时轮子向底盘后方滚动，请交换其两个电机线（**不要**更改代码）。

断电或按重置键以退出诊断模式。

参见 [`car/src/diagnostic.rs`](car/src/diagnostic.rs) 获取源代码。

---

## 故障安全行为

小车的主循环对与控制器失去联系的情况非常警惕。两个独立的超时协同工作：

| 层       | 位置                          | 触发                                                 | 操作                                  |
|-------------|--------------------------------|---------------------------------------------------------|-----------------------------------------|
| 应用 | `car/src/main.rs`              | 在 `FAILSAFE_TIMEOUT_MS`（= 500 ms）内没有任何类型的数据包传入 | `MotorDriver::stop_all()`，状态 LED 熄灭 |
| 链路        | `controller/src/radio.rs`      | 在 `HEARTBEAT_TIMEOUT_MS`（= 600 ms）内没有 `Heartbeat` 回复 | 控制器上的 RGB LED0 变为红色   |

因为心跳以 ~5 Hz（每 200 ms）到达，而运动以 ~50 Hz 到达，500 ms 的静默是一个舒适的余量：单个丢失的帧不会被注意到，三个丢失的心跳会停止小车。看门狗使用 `Instant::now()` 对抗 `radio::last_rx_millis()`，后者在*每个*成功解析的数据包上更新——Motion、Heartbeat 或 E-Stop——因此即使控制器正在发送没有运动的心跳，链路仍然被认为是活动的。

故障安全状态**保持粘性直到清除**：一旦启动，小车保持停止状态（我们不会每 100 ms 敲 I²C 总线重新声明 "所有电机关闭"）直到新数据包到达，此时循环正常恢复。启动也算作故障安全（`last_rx == 0`），因此在上电无线电链路之前，轮子永远不会旋转。

---

## 硬件注意事项（蜂鸣器与 PWM 载波）

固件中有两个小而重要的缓解措施，专门用于消除 MotorBit 扩展板否则会产生的噪声。

### 1. 启动时 P0_02 保持低电平（蜂鸣器静音）

MotorBit V1.0 / V2.0 通过滑动开关将其板载无源蜂鸣器连接到 micro:bit `P0`（= nRF52833 `P0_02`）。当开关**打开**时，`P0_02` 与蜂鸣器电气连接；未驱动（浮动）的引脚会拾取相邻走线的串扰，并导致蜂鸣器持续啁啾。

因此，小车固件将 `P0_02` 分配为 GPIO 输出并保持**低电平** 用于程序的整个生命周期，因此无论滑动开关位置如何，蜂鸣器都保持安静：

```rust
let _buzzer_silence = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);
```

前导下划线保持绑定处于活动状态（裸 `_` 会立即删除它并释放引脚）。如果您实际上*想要*将蜂鸣器用于音调 / 警报，请删除此行并从 PWM 外设驱动 `P0_02`。

### 2. PCA9685 载波为 1.5 kHz，而不是 50 Hz

标准 PCA9685 默认载波为 **50 Hz**（设计用于业余伺服）。在 50 Hz 下，有刷直流电机和 MotorBit 上的 H 桥滤波电感会产生 audible 共振——每当任何电机通电时，您都会听到低*嗡嗡声*或*哼声*，即使在静止时也是如此。

[`car/src/pca9685.rs`](car/src/pca9685.rs) 因此将载波提高到 `DEFAULT_PWM_FREQ_HZ = 1500`，这远高于人声音频带，在商用直流电机上是听不见的。此更改对堆栈的其余部分不可见：PWM 占空比仍表示为 0..=4095 / 周期，S1–S8 上的伺服（假定 50 Hz）未被默认固件使用。**如果您接线伺服，** 请降低频率（或将伺服放在单独的驱动器上）——大多数业余伺服无法容忍 1.5 kHz 脉冲速率。

---

## 引脚映射（速查表）

### 控制器（micro:bit v2）

| 功能                       | 边缘连接器 | nRF52833 GPIO | 方向 / 配置  |
|--------------------------------|----------------|---------------|---------------------|
| 摇杆 Y 轴（向上 = 1023）    | **P1**         | `P0.03`（AIN2)| SAADC 单端  |
| 摇杆 X 轴（向右 = 1023） | **P2**         | `P0.04`（AIN3)| SAADC 单端  |
| 右摇杆 X（omega）       | _待定_          | `_TBD_`（AINx)| SAADC，功能 `right-stick-hw` |
| 右摇杆 Y（保留）    | _待定_          | `_TBD_`（AINx)| SAADC，功能 `right-stick-hw` |
| 扩展板按钮 **A**（模式切换） | **P13**        | `P0.17`       | 输入，`Pull::Up`   |
| 扩展板按钮 **B**（保留） | **P14**        | `P0.01`       | （未使用）            |
| 扩展板按钮 **C**（omega 逆时针）| **P16**        | `P1.02`       | 输入，`Pull::Up`   |
| 扩展板按钮 **D**（omega 顺时针）| **P15**        | `P0.13`       | 输入，`Pull::Up`   |
| 扩展板 RGB LED 链（×4）      | **P8**         | `P0.10`       | PWM0 / EasyDMA，WS2812 800 kHz |
| 加速度计 SCL              | 内部       | `P0.08`       | TWIM @ 100 kHz      |
| 加速度计 SDA              | 内部       | `P0.16`       | TWIM @ 100 kHz      |
| 状态 LED（网格左上角）  | row1/col1      | `P0.21`/`P0.28`| GPIO 输出        |

### 小车（micro:bit v2 + MotorBit 扩展板）

| 功能                       | 边缘连接器 | nRF52833 GPIO | 说明                                          |
|--------------------------------|----------------|---------------|------------------------------------------------|
| I²C SCL → PCA9685              | **P19**        | `P0.26`       | TWIM0                                          |
| I²C SDA → PCA9685              | **P20**        | `P1.00`       | TWIM0                                          |
| **A 按钮**（诊断模式） | 板载       | `P0.14`       | 低电平有效，内部 `Pull::Up`；在启动后 2 秒内点击 |
| **蜂鸣器线路**（保持低电平）     | **P0**         | `P0.02`       | GPIO 输出低；静音 MotorBit 蜂鸣器      |
| PCA9685 通道 CH0/CH1       | （在扩展板上）    | —             | 电机 M1（FL）+/- 端子                    |
| PCA9685 通道 CH2/CH3       | （在扩展板上）    | —             | 电机 M2（FR）+/- 端子                    |
| PCA9685 通道 CH4/CH5       | （在扩展板上）    | —             | 电机 M3（RL）+/- 端子                    |
| PCA9685 通道 CH6/CH7       | （在扩展板上）    | —             | 电机 M4（RR）+/- 端子                    |
| PCA9685 通道 CH8..CH15     | （在扩展板上）    | —             | 伺服接头 S1..S8（50 Hz **未**激活——参见硬件注意事项） |
| 状态 LED                     | row1/col1      | `P0.21`/`P0.28`| 当运动 ≠ 0 时点亮，或在诊断模式下常亮 |

> **轮子布局（顶视图，电机 → 角落映射）：** `M1 = 左前`，`M2 = 右前`，`M3 = 左后`，`M4 = 右后`。如果您的底盘旋转或横向移动方向错误，最简单的修复方法是交换有问题电机的两个端子——无需更改代码。

---

## 构建与烧录

### 前置条件

```bash
rustup target add thumbv7em-none-eabihf
cargo install cargo-make           # 任务运行器，Makefile.toml 使用
cargo install probe-rs --features cli   # 烧录器 + RTT 查看器
# 可选，用于 `cargo make size-*`
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

需要调试探针来烧录固件。任何 micro:bit v2 的板载 **DAPLink** 开箱即用（只需插入 USB 线缆）；外部 CMSIS-DAP / J-Link 探针也可以。

### 一键构建

```bash
cargo make build-all          # 构建 protocol + radio-core + car + controller
```

或按 crate：

```bash
cargo make build-car          # cargo build --release -p car
cargo make build-controller   # cargo build --release -p controller
```

### 烧录

插入**小车**的 micro:bit，然后：

```bash
cargo make flash-car
```

拔下它，插入**控制器**的 micro:bit，然后：

```bash
cargo make flash-controller
```

这两个任务都调用 `probe-rs run --chip nRF52833_xxAA …`，它还附加 RTT 并将 `defmt` 日志流式传输到您的终端。

### 实用额外命令

```bash
cargo make check              # cargo check --workspace
cargo make clippy             # cargo clippy --workspace -- -D warnings
cargo make fmt                # cargo fmt --all
cargo make fmt-check          # CI 友好的 fmt 检查
cargo make size-car           # cargo size -p car --release -- -A
cargo make size-controller    # cargo size -p controller --release -- -A
cargo make clean              # cargo clean
```

---

## 驾驶小车

0. *（可选，新底盘第一次使用时）*打开小车电源并在 2 秒内点击板载 **A 按钮**以运行[诊断模式](#诊断模式)接线扫描。验证每个轮子以正确方向响应后，断电以退出诊断模式。
1. 打开**小车**电源（MotorBit 扩展板上的电池）。
2. 打开**控制器**电源（USB 充电宝或纽扣电池组）。
3. 观察控制器扩展板上**第一个 RGB LED**（LED0，边缘连接器 P8）。它启动时显示**呼吸琥珀色** "连接中" 脉冲，一旦小车回复第一个心跳（通常在 ~200 ms 内）就变为**绿色**，如果链路断开超过 600 ms 则变为**红色**。
4. 默认模式为**摇杆**：
   - 向前 / 向后推摇杆 → 小车向前 / 向后行驶。
   - 向左 / 向右推 → 小车横向移动。
   - **对角**推 → 对角麦克纳姆运动。
5. 按扩展板的 **A 按钮**切换到**倾斜**模式：
   - 将控制器标志朝上握住并倾斜以驾驶。
   - 再次按 A 切换回摇杆。
6. **C / D 按钮**在任何平移源之上添加旋转：
   - 仅 C → 逆时针旋转。
   - 仅 D → 顺时针旋转。
   - C + D → 将旋转立即归零。
7. **右摇杆（计划中，可选）。** 稍后可以连接第二个模拟摇杆以提供连续的 `omega` 控制。固件已经针对模拟源完整运行管道；启用 Cargo 功能 `right-stick-hw` 并在硬件就绪后完成 `controller/src/joystick_right.rs` 中的构造函数。然后右摇杆和 C/D 按钮**求和并钳位到 ±100**，因此按钮充当摇杆之上的精细微调。
8. 松开所有输入立即发送 `(0, 0, 0)`；小车停止。
9. 如果小车在 `FAILSAFE_TIMEOUT_MS`（500 ms，参见[故障安全行为](#故障安全行为)）内停止从控制器接收信号，它也会自行停止。控制器反过来在 `HEARTBEAT_TIMEOUT_MS`（600 ms）后将其 RGB 指示器翻转为红色，因此您在 wondering 为什么小车停止响应之前获得控制器上的视觉提示，即链路已断开。

---

## 日志与调试

- 所有日志使用 RTT 上的 [`defmt`]。默认级别为 `debug`（在 `Makefile.toml` 中通过 `DEFMT_LOG=debug` 设置）。
- `cargo make flash-*` 自动附加 RTT，因此固件中的任何 `info!` / `trace!` / `error!` 都出现在同一终端中。
- 要为单次运行获取更多或更少详细输出，请覆盖变量：`DEFMT_LOG=trace cargo make flash-controller`。
- 对于更深入的检查，使用 `probe-rs attach --chip nRF52833_xxAA` 在烧录后重新附加到正在运行的开发板。

[`defmt`]: https://defmt.ferrous-systems.com/

---

## 故障排除

| 症状                                                                | 可能原因                                                                                  | 修复                                                                                                                                  |
|------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| `probe-rs` 显示 **"未找到探针"**                               | DAPLink 未枚举；另一个工具正在占用端口；线缆仅供电                 | 重新插入 USB 线缆，关闭 `pyOCD` / Arduino IDE / micro:bit 网页烧录工具，尝试数据线                                          |
| 烧录成功但**没有日志出现**                               | RTT 未附加，或 `DEFMT_LOG` 过滤掉了所有内容                                      | 运行 `DEFMT_LOG=debug cargo make flash-controller`，或在烧录后运行 `probe-rs attach`                                               |
| 小车忽略命令，控制器 LED 仍然闪烁                      | 每侧刷新了不同的 `RADIO_CHANNEL` / `PROTOCOL_VERSION`                           | 从**同一**工作区检出重新烧录两块开发板                                                                            |
| 小车行驶但**方向错误**（例如 "前进" 向左走）          | 一个轮子的电机端子交换，或 M1–M4 与 FL/FR/RL/RR 不匹配                    | 交换有问题电机的两个电线；**不要**在代码中重新接线                                                                 |
| 侧向推动摇杆时旋转而不是横向移动           | 两个对角相对的轮子反转                                                   | 从 `defmt` 电机日志中识别该对并翻转两者                                                                          |
| 松开摇杆后小车继续蠕动                       | 摇杆中心偏移（制造公差）并超过 `DEAD_ZONE`                   | 增加 `controller/src/joystick.rs` 中的 `DEAD_ZONE`（尝试 `40`）                                                                      |
| 倾斜模式感觉太敏感 / 不够敏感                   | `FULL_TILT` 与您实际倾斜的程度不匹配                                           | 调整 `controller/src/tilt.rs` 中的 `FULL_TILT` 和 `DEAD_ZONE`                                                                        |
| 即使按住摇杆，小车也每隔 ~500 ms 停止一次                       | 故障安全触发——控制器→小车链路正在丢包                                     | 将开发板移近，更改 `RADIO_CHANNEL`，提高 `radio-core/src/lib.rs` 中的 `TX_POWER`，或放宽 `car/src/main.rs` 中的 `FAILSAFE_TIMEOUT_MS` |
| 小车旋转 / 横向移动错误，但您无法判断哪个轮子有故障  | 在摇杆控制下很难诊断                                                       | 断电并在 2 秒内点击**A 按钮**以进入[诊断模式](#诊断模式)，然后观察哪个轮子转动      |
| 即使静止时底盘也有连续的低嗡嗡声 / 哼声                | PCA9685 载波恢复为 50 Hz，或蜂鸣器开关打开时 `P0_02` 保持浮动         | 验证 `car/src/pca9685.rs` 中的 `DEFAULT_PWM_FREQ_HZ = 1500`，以及 `main.rs` 中仍然有 `Output::new(p.P0_02, Level::Low, ...)`（参见[硬件注意事项](#硬件注意事项蜂鸣器与-pwm-载波)） |
| `cargo build` 失败，显示 **"未找到目标 'thumbv7em-none-eabihf'"** | 缺少交叉编译目标                                                                  | `rustup target add thumbv7em-none-eabihf`                                                                                            |
| `cargo install probe-rs` 在 macOS 上失败                                | 缺少 `libusb` / `libudev` 头文件                                                          | `brew install libusb pkg-config`                                                                                                     |
| 控制器工作但倾斜模式从未启动                           | 扩展板 A 接线或 `Pull::Up` 不匹配                                                 | 用万用表检查 P13；该线路在空闲时应为高电平，按下时为低电平                                                       |
| 控制器上的 RGB LED 永远不会点亮                              | P8 未绑定到 `P0.10`，或 LED 由 3.3 V 供电而灯带需要 5 V                | 验证扩展板的丝印显示 P8 = 数据；如果是，请给灯带提供干净的 5 V（或 ≥ 4.5 V）电源和共地             |
| RGB LED 亮起但**颜色看起来不对**（例如 "绿色" 显示红色）       | 克隆的 WS2812 芯片具有非标准字节顺序（RGB / BGR 而不是 GRB）                    | 在 [`controller/src/rgb.rs`](controller/src/rgb.rs) 中交换 `encode_pixels` 中的字节顺序                                            |
| 控制器的 LED 保持**琥珀色 forever**                               | 小车固件未运行，在不同的 `RADIO_CHANNEL` 上，或超出范围                     | 重新烧录小车，验证两块开发板来自同一检出，将它们移近                                                  |

---

## 扩展项目

代码库有意拆分，因此每个扩展轴都位于确切的一个位置。

### 1. 在控制器上添加新的输入源

假设您想添加红外遥控器或第二个摇杆。

1. 创建 `controller/src/ir.rs`，包含：
   - 一个 `init(...)` 函数，接受您需要的外设并返回配置的驱动程序。
   - 一个公共 `pub static IR_MOTION_CHANNEL: Channel<...> = Channel::new();`。
   - 一个 `#[embassy_executor::task] pub async fn ir_task(...)`，采样并将数据推送到通道**仅当其模式处于活动状态时**（查看 `joystick.rs` 中的 `mode::current() == InputMode::…` 保护模式）。
2. 向 `mode::InputMode` 添加新变体（例如 `Ir = 2`）并更新 `mode::toggle()` 以循环遍历新状态。
3. 在 `controller/src/main.rs` 中：
   - 在现有的 `joystick_task` / `tilt_task` 旁边生成 `ir_task`。
   - 扩展 `select4`（或升级为带有更多臂的 `select`），以便融合循环也对 `IR_MOTION_CHANNEL` 做出反应。
4. 完成——无线电路径不需要任何更改，因为融合循环仍然发出单个 `MotionPayload`。

### 2. 在小车上添加新的输出/执行器

例如伺服驱动的机械爪或云台摄像头。

1. `MotorBit` 抽象已经为通道 S1–S8 公开了 `set_servo_angle` 和 `set_servo_duty`。添加一个小包装模块（例如 `car/src/gripper.rs`），从 `MotorDriver::twim_mut()` 获取 `&mut Twim<'static>` 并驱动相关伺服。
2. 如果您需要来自控制器的*新*命令，请在 `protocol/src/lib.rs` 中添加负载类型（参见下面的 "添加新的消息类型"）并通过新的 `embassy_sync` 通道将其引入 `car/src/radio.rs` 的 `handle_received_packet`。

### 3. 在协议中添加新的消息类型

1. 在 `protocol/src/lib.rs` 中：
   - 向 `MessageType` 附加新变体（给它下一个空闲的 `u8`）。
   - 在 `RadioHeader::from_bytes` 中添加匹配臂（否则旧负载将被静默丢弃）。
   - 定义 `FooPayload` 结构体，包含 `SIZE`、`to_bytes`、`from_bytes`。
   - 添加 `create_foo_packet(...)` 辅助函数以镜像现有的。
2. 提升 `PROTOCOL_VERSION`，以便任一侧的旧固件干净地拒绝新布局，而不是错误解析它。
3. 在 `controller/src/radio.rs` 和/或 `car/src/radio.rs` 中，扩展 `handle_received_packet` 以匹配新的 `MessageType` 并将解析的负载转发到其消费者。

### 4. 添加新的 RGB 指示器

P8 上的四 LED 链已接线，但目前仅使用 LED0（用于无线电链路状态）。要重新利用其中一个备用 LED——例如显示活动输入模式、omega 的符号或电池警告：

1. 打开 [`controller/src/rgb.rs`](controller/src/rgb.rs) 并向 `RgbState` 添加新字段（例如 `pub mode: InputMode`）。相应地更新 `INITIAL` 常量。
2. 扩展 `render` 函数，使新字段映射到 `frame[1]`、`frame[2]` 或 `frame[3]` 之一。保持现有的 `frame[0]` 映射用于链路状态不变。
3. 从拥有新信息片段的任何任务中，通过 `RGB_STATE.signal(...)` 发布刷新的 `RgbState`。`controller/src/radio.rs::publish_link_state` 中的模式是规范示例——每个字段保留一个任务作为写入器，以便更新保持序列化。
4. 不需要在 `main.rs`、协议或小车中进行更改：驱动程序任务自动获取最新的 `RgbState` 并在 ~60 ms 内重新渲染。

因为 WS2812 是单线菊花链，驱动程序始终发出所有四个像素，而不管您实际驱动了多少个——没有每 LED 的 "开/关" 成本需要担心，只需保持亮度低（目标 `value ≤ 20 / 255`），以便指示器从近距离观看时保持舒适。

### 5. 调整驾驶手感

| 旋钮                                    | 文件                          | 效果                                  |
|-----------------------------------------|-------------------------------|-----------------------------------------|
| `GEOMETRY_FACTOR_K`                     | `car/src/motor.rs`            | 更多/更少旋转权限          |
| `SPEED_TO_PWM_SCALE`                    | `car/src/motor.rs`            | 最高速度（最大 PWM 占空比）                |
| `DEAD_ZONE` / `SMOOTH_NUM` / `SMOOTH_DEN`（摇杆） | `controller/src/joystick.rs` | 摇杆死区和响应度  |
| `DEAD_ZONE` / `FULL_TILT`               | `controller/src/tilt.rs`      | 倾斜灵敏度和全尺寸角度     |
| `OMEGA_MAX` / `OMEGA_STEP`              | `controller/src/button.rs`    | 旋转强度和斜坡时间           |
| `RADIO_CHANNEL` / `TX_POWER`            | `radio-core/src/lib.rs`       | 射频共存和范围                  |
| `HEARTBEAT_TIMEOUT_MS`                  | `radio-core/src/lib.rs`       | 射频丢失后小车滑行的时间   |

### 6. 移植到不同的底盘

整个小车侧抽象堆栈是 `MotorDriver → MotorBit → PCA9685`。
- 对于**非麦克纳姆**传动系统（差速 / 阿克曼），保留协议和 `MotorBit`/`PCA9685` 层，仅用新的混合方程重写 `MotorDriver::apply_motion`。
- 对于**非 PCA9685** 电机驱动程序（例如 TB6612 直接连接到 micro:bit），替换 `motor.rs` + `motorbit.rs` + `pca9685.rs` 并保持其余部分不变。只要新驱动程序使用 `MotionPayload`，控制器和无线电堆栈根本不需要更改。

### 7. 移植到不同的 MCU

`embassy-nrf` 是固件侧唯一特定于 nRF 的依赖项，它隔离在 `radio-core`、`tilt.rs`、`joystick.rs`、`button.rs`、`motor.rs` 中。要定位不同的 Embassy 支持的 MCU：

1. 在 `Cargo.toml` 中将 `embassy-nrf` 替换为相关的 `embassy-*` HAL。
2. 针对新 HAL 重新实现上述四个硬件模块（每个都很小——低于 ~250 LOC）。
3. 只要新 MCU 也具有支持 802.15.4 的无线电，`protocol` 和 `radio-core` 的公共 API 可以保持不变；否则用您选择的传输（BLE、ESP-NOW、RFM69、…）替换 `radio-core`——应用程序代码仅依赖于其 `init` / `send_packet` 表面。

---

## 贡献指南

欢迎贡献——错误报告、硬件变体、新输入源、替代传输和文档改进都通过相同的工作流程进入。

### 工作流程

1. **Fork** 此仓库并从 `main` 创建一个功能分支：
   ```bash
   git checkout -b feat/short-description
   ```
2. **进行更改。** 保持每个 PR 专注于一个逻辑问题；如果您发现自己在同一个提交中触及 `protocol/`、`controller/` 和 `car/`，这是拆分的提示。
3. **在推送之前运行本地质量门**：
   ```bash
   cargo make fmt-check
   cargo make clippy
   cargo make build-all
   ```
   三者都必须**零**警告通过——`clippy` 使用 `-D warnings` 调用，因此任何新的 lint 都将使 CI 失败。
4. **在真实硬件上测试**当更改是功能性的（电机混合、无线电行为、输入处理）。拍摄短片并将其放入 `docs/media/` 中，以便审阅者无需重新烧录即可看到您的意思。
5. **打开 PR** 并使用下面的模板并请求审阅。

### 编码规范

- **Rust 版本 / 工具链：** `2021` 版本，由 `rust-toolchain.toml` 固定的稳定工具链（如果存在）。没有 nightly-only 功能，除非它们已在使用中。
- **固件侧随处都是 `#![no_std]`。** 使用 `heapless`、`static_cell` 和堆栈缓冲区——永远不要使用 `alloc`。
- **缩进：** 2 个空格，匹配 `rustfmt.toml` 和现有文件。在提交之前运行 `cargo make fmt`。
- **命名：** 模块是 `snake_case`，类型是 `UpperCamelCase`，常量是 `SCREAMING_SNAKE_CASE`。公共项获得文档注释；内部辅助函数在目的不明显时获得一行 `//` 注释。
- **日志：** 仅 `defmt`。根据严重程度首选 `info!` / `debug!` / `trace!` / `warn!` / `error!`，并且**永远不要**在热循环中运行的日志消息中包含用户控制的数据。
- **注释和标识符以英语编写。** 这与代码库的其余部分匹配，并使非中文贡献者的差异保持可读性。
- **`protocol/` 上没有静默的重大更改。** 如果 PR 更改线格式，它**必须**提升 [`protocol/src/lib.rs`](protocol/src/lib.rs) 中的 `PROTOCOL_VERSION`；审阅者将拒绝没有这样做的 PR。
- **通道和信号优于共享可变状态。** 新任务应通过 `embassy_sync::channel::Channel` / `Signal` 进行通信——参见 `controller/src/joystick.rs` 以获取规范模式。

### 提交消息格式

我们遵循宽松的[约定提交]样式。第一行是 `type(scope): summary`，正文（可选）解释 _why_，而不是 _what_：

```text
feat(controller): 添加红外遥控器作为第三输入源
fix(car): 防止数据包在 1 ms 内到达两次时电机抖动
perf(radio): 通过丢弃 seq 字段将心跳缩小到 4 字节
docs(readme): 添加倾斜模式演示 GIF
refactor(motor): 将混合矩阵折叠为 const 查找表
test(protocol): 往返每个文档化的十六进制示例
```

有效的 `type` 值：`feat`、`fix`、`perf`、`refactor`、`docs`、`test`、`chore`、`ci`。有效的 `scope` 镜像 crate / 模块：`protocol`、`radio-core`、`controller`、`car`、`radio`、`motor`、`tilt`、`joystick`、`button`、`mode`、`readme`。

[约定提交]: https://www.conventionalcommits.org/en/v1.0.0/

### 拉取请求模板

将其复制到 PR 描述中并填写空白：

```markdown
### 摘要
<!-- 一段话：更改了什么以及为什么。 -->

### 测试的硬件
- [ ] micro:bit v2（控制器）
- [ ] micro:bit v2（小车）
- [ ] MotorBit 扩展板 v__
- [ ] 不适用——仅协议 / 文档

### 验证
- [ ] `cargo make fmt-check` 通过
- [ ] `cargo make clippy` 通过（无新警告）
- [ ] `cargo make build-all` 通过
- [ ] 驾驶小车 ≥ 60 秒无异常
- [ ] （如果协议已更改）提升了 `PROTOCOL_VERSION` 并更新了[十六进制示例](#十六进制示例)

### 截图 / 剪辑
<!-- 放入 docs/media/ 并在此链接它们。 -->

### 给审阅者的注意事项
<!-- 权衡、后续行动、您推迟的任何事情。 -->
```

### 报告 Bug / 请求新功能

打开一个问题，包含：

- **硬件：** 哪个 micro:bit 修订版，哪个 MotorBit 修订版，哪个电池 / 电机。
- **固件提交：** 两块开发板上 `git rev-parse --short HEAD` 的输出。
- **复现：** 触发问题的最小输入序列。
- **日志：** 相关的 `defmt` 摘录（`DEFMT_LOG=trace` 对于错误报告来说没问题；隐瞒任何个人信息）。

---

## 致谢

- [Embassy] 背后的出色团队，让 `async` 嵌入式 Rust 写起来很愉快。
- [`probe-rs`](https://probe.rs/) 用于一键烧录 + RTT。
- [`defmt`](https://defmt.ferrous-systems.com/) 用于在微型 MCU 上进行廉价、结构化日志。
- [BBC micro:bit] 教育基金会，提供了一个廉价、可破解的平台，并带有板载无线电。
- Yahboom / ELECFREAKS 风格的 **MotorBit** 供应商，提供基于 PCA9685 的扩展板，使麦克纳姆轮项目易于上手。
- **WorldSemi WS2812** 用于无处不在的单线 RGB 像素，将任何备用 GPIO 变成状态显示器。

[BBC micro:bit]: https://microbit.org/

---

## 许可证

根据实际存储库中的 [LICENSE](LICENSE) 文件条款分发。
