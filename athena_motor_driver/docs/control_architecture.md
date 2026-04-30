# Athena Drive Control Architecture

This document describes the control pipeline that translates a high-level
operator intent (joystick / teleop) into low-level torque commands on the four
Unitree A1 motors that drive Athena's tracks.

The pipeline is split across two physical processors:

| Stage | Where it runs | Loop rate |
|---|---|---|
| Twist → wheel velocity setpoint | ROS 2 driver (`athena_motor_driver`, host PC) | 50 Hz |
| Wheel velocity → motor torque | Teensy 4.0 firmware (`athena_motor_firmware`) | 1000 Hz |
| FOC torque tracking | Unitree A1 motor MCU | internal |

The host PC and the Teensy talk over USB serial using the
[Crosstalk](https://github.com/StefanFabian/crosstalk) framing protocol. The
Teensy talks to the four motors over two redundant RS-485 buses (front bus and
rear bus), each carrying one left motor (`motor_id = 0`) and one right motor
(`motor_id = 1`).

## 1. End-to-end overview

```mermaid
flowchart LR
    JOY["Joystick / teleop node<br/>(upstream, outside this repo)"] -->|"geometry_msgs/Twist<br/>on /cmd_vel"| DRV
    NAV["Autonomy / nav2"] -->|"geometry_msgs/Twist<br/>on /cmd_vel"| DRV

    subgraph HOST["Host PC — athena_motor_driver (50 Hz)"]
        DRV["AthenaMotorDriver node"] --> DDC["DiffDriveController<br/>(forward kinematics)"]
        DDC -->|"left_w, right_w<br/>[rad/s]"| TX["MotorCommand<br/>(VELOCITY mode)"]
    end

    TX -->|"Crosstalk over USB serial<br/>(1.152 Mbaud)"| FW

    subgraph TEENSY["Teensy 4.0 — athena_motor_firmware (1 kHz)"]
        FW["main.cpp<br/>host_comm dispatch"] --> MC["MotorController<br/>(velocity ramp, safety)"]
        MC --> MSL["MotorSideController<br/>LEFT"]
        MC --> MSR["MotorSideController<br/>RIGHT"]
        MSL --> PIDL["PID + Feed-Forward"]
        MSR --> PIDR["PID + Feed-Forward"]
        PIDL -->|"torque_left [Nm]"| BUS_F
        PIDR -->|"torque_right [Nm]"| BUS_F
        PIDL -->|"torque_left [Nm]"| BUS_R
        PIDR -->|"torque_right [Nm]"| BUS_R
        BUS_F["MotorComm<br/>front RS-485 bus"]
        BUS_R["MotorComm<br/>rear RS-485 bus"]
    end

    BUS_F --> M_FL["Front-left A1 motor"]
    BUS_F --> M_FR["Front-right A1 motor"]
    BUS_R --> M_RL["Rear-left A1 motor"]
    BUS_R --> M_RR["Rear-right A1 motor"]

    M_FL -->|"Status: pos, vel, torque"| BUS_F
    M_FR -->|"Status: pos, vel, torque"| BUS_F
    M_RL -->|"Status: pos, vel, torque"| BUS_R
    M_RR -->|"Status: pos, vel, torque"| BUS_R

    MC -->|"FullMotorStatus"| FB["Crosstalk reply"]
    FB --> DRV
    DRV -->|"athena_motor_interface/FullMotorStatus<br/>on motor_status"| HIGHER["Diagnostics / odometry"]
```

Key idea: the ROS driver only does kinematics (Twist → per-side wheel rate). All
real-time control (velocity ramp, PID, feed-forward, position hold, FOC torque
generation) lives on the Teensy because it runs deterministically at 1 kHz.

## 2. Host-side: from `/cmd_vel` to wheel velocity setpoint

The ROS node `AthenaMotorDriver` (`athena_motor_driver/src/athena_motor_driver.cpp`)
runs a 20 ms timer (`update()`). On each tick it picks the most recent input,
applies safety limits, computes per-side wheel velocities, and serializes a
`MotorCommand` over Crosstalk.

```mermaid
flowchart TD
    A["/cmd_vel<br/>geometry_msgs/Twist"] --> B{"torque_mode_<br/>parameter?"}
    A2["~/forward_torque<br/>TorqueCommand"] --> B
    B -->|"false (default)"| C["last_motor_command_received_ ← now()<br/>twist_msg_ stored"]
    B -->|"true"| C2["last_motor_command_ = Torque(left, right)<br/>(passes through unchanged)"]

    subgraph TIMER["Timer: every 20 ms — update()"]
        D{"now − last_command_received_<br/> > 250 ms?"}
        D -->|yes| E["Send zero command<br/>Velocity(0,0) or Torque(0,0)"]
        D -->|no, twist| F["1. dir_sign = invert_forward_direction ? −1 : 1"]
        F --> G{"abs(angular.z) ><br/>angular_velocity_limit_?"}
        G -->|yes| H["Clip angular,<br/>scale linear to keep curvature"]
        G -->|no| J["DiffDriveController.computeMotorCommand(<br/>dir_sign · linear, angular)"]
        H --> J
        J --> K["MotorCommand(VELOCITY,<br/>left_w, right_w) [rad/s]"]
        D -->|"no, torque"| L["Forward last_motor_command_<br/>unchanged"]
        K --> M["sendObject(...) via cross_talker_"]
        L --> M
        E --> M
    end

    M -->|"USB serial frame"| TEENSY["Teensy"]
```

### 2.1 Differential-drive forward kinematics

Implemented in `src/controllers/diff_drive_controller.cpp`. With

- `L` = `wheel_separation` (m)
- `r` = `wheel_radius` (m)
- `α` = `rotational_amplification`

the per-side wheel angular velocities are

\[
\omega_{\text{left}} = \frac{v_x - \alpha \cdot \omega_z \cdot L/2}{r},
\qquad
\omega_{\text{right}} = \frac{v_x + \alpha \cdot \omega_z \cdot L/2}{r}.
\]

`rotational_amplification > 1` pre-amplifies in-place yaw commands so the
operator gets responsive turning despite high track friction. The angular
velocity is also clamped to `angular_velocity_limit_` *before* this
computation, with `linear` scaled proportionally so the commanded curvature is
preserved (see `update()` in `athena_motor_driver.cpp`).

### 2.2 Safety / timeouts on the host

- **250 ms command timeout** (`last_motor_command_received_ + 250ms < now()`):
  if no upstream command (Twist or Torque) arrives, the driver injects a
  zero command. This protects against an upstream node freezing.
- **Mode gating**: when `torque_mode_` is true, the `/cmd_vel` callback is a
  no-op (and vice versa) so the two modes can never fight.
- **Velocity sanity** is enforced one layer down, in the firmware
  (`MAX_PLAUSIBLE_VELOCITY_COMMAND = 30 rad/s`).

### 2.3 Other host responsibilities

- **PID/feed-forward gains**: declared as ROS parameters; whenever any of them
  is reconfigured, `pid_updated_` is set and on the next `update()` tick a
  `ChangePIDGainsCommand` is sent to the Teensy.
- **`UpdateSettings`**: enables/disables debug data, and the
  `disable_acceleration_limiting` flag (PID-tuning only — bypasses the velocity
  ramp on the firmware).
- **`~/reboot` service**: forwards a `TeensyRebootCommand` over Crosstalk and
  closes the serial port; the firmware reboots, which causes USB to re-enumerate.
- **Status fan-out**: `FullMotorStatus` packets coming back from the Teensy are
  republished on `motor_status`. `MotorDebugData` is republished on `~/debug_data`
  when `enable_debug` is true.

## 3. Firmware-side: from wheel velocity to motor torque

The firmware (`athena_motor_firmware/src/main.cpp`) has two concurrent
contexts:

- **`loop()`** — non-time-critical: handles host serial (Crosstalk dispatch),
  publishes `FullMotorStatus`, errors, and debug data.
- **`motorControlLoop()`** — runs from an `IntervalTimer` ISR every
  `MAIN_LOOP_PERIOD_US = 1000 µs` (1 kHz). This is the real-time control loop.

```mermaid
flowchart TD
    subgraph LOOP["loop() — non-realtime"]
        L1["host_comm.processSerialData()"] --> L2{"Object?"}
        L2 -->|MotorCommand| L3["plausibility check<br/>(±30 rad/s, ±60 Nm)"]
        L3 --> L4["motor_controller.setCommand()<br/>time_since_last_command = 0"]
        L2 -->|ChangePIDGainsCommand| L5["MotorController.set*Gains(...)"]
        L2 -->|UpdateSettings| L6["enable_debug, disable_accel_limit"]
        L2 -->|TeensyRebootCommand| L7["ack + SCB_AIRCR reset"]
        L8["host_comm.sendObject(full_motor_status)"]
        L9["if enable_debug: send MotorDebugData"]
    end

    subgraph ISR["motorControlLoop() — 1 kHz ISR"]
        I1{"time_since_last_command<br/>≥ 200 ms?"}
        I1 -->|yes| I2["motor_controller.stop()<br/>BRAKE, target_velocity = 0"]
        I1 -->|no| I4["motor_controller.update()"]
        I2 --> I4
        I4 --> I5["full_motor_status updated"]
    end

    L4 -.->|"shared command_"| I4
    I5 -.->|"shared full_motor_status"| L8
```

Note: the host-driver timeout is 250 ms; the firmware-side timeout is 200 ms.
This means the firmware will brake first if the host is silent — even if the
host driver is alive but its upstream is silent, the firmware also catches
that case independently because the host then sends `Velocity(0,0)`.

### 3.1 `MotorController::update()`

Called every 1 ms. The dt passed to PIDs is capped at 20 ms to keep the
observer/integrator stable after pauses (e.g. after an E-stop or a USB drop).

```mermaid
flowchart TD
    U0["dt = min(time_since_last_command, 20 ms)"] --> U1["computeMotorCommands(dt)"]
    U1 --> U2["sendReceiveBothBuses(front_working, rear_working)"]
    U2 --> U3["tryInitializePosition()<br/>(set hold_position once both buses report)"]
    U3 --> U4["addMeasurements()<br/>(feed position & velocity filters)"]
    U4 --> U5["assembleMotorStatus()"]
    U5 --> U6["collectDebugData()"]
    U6 --> U7["return motor_status_"]
```

`sendReceiveBothBuses` interleaves TX/RX on the two RS-485 buses so the
front and rear motors can process their commands in parallel. If a bus has
been dead for too many cycles (`MAX_RESET_SKIP_COUNT = 10`), the firmware
resets that bus and skips it for one cycle to recover.

### 3.2 `computeMotorCommands` — torque selection

```mermaid
flowchart TD
    A["computeMotorCommands(dt)"] --> B{"both sides<br/>isWorking()?"}
    B -->|no| C["BRAKE both<br/>velocity = 0<br/>resetPIDControllers()<br/>initialized_position_ = false"]
    B -->|yes| D["torque = computeTorque(dt)"]
    D --> E["setCommandFromTorque(left)<br/>setCommandFromTorque(right)"]
    E --> F{"torque magnitude > MIN_TORQUE_FOR_FOC<br/>(0.5 Nm)?"}
    F -->|yes| G["mode = FOC<br/>torque clamped to ±MOTOR_TORQUE_LIMIT (30 Nm)"]
    F -->|no| H["mode = BRAKE<br/>torque = 0"]
```

Two important behaviors:

- If a side has no working motor in `MOTOR_STATUS_TIMEOUT_MS = 20 ms`, both
  sides are forcibly braked and the position controller is invalidated (so it
  cannot snap back to a stale hold-position when comms recover).
- If torque magnitude is below `MIN_TORQUE_FOR_FOC = 0.5 Nm`, the firmware
  drops to `BRAKE` mode for that motor instead of sending a tiny FOC command.
  This avoids parasitic noise / heating at zero output.

### 3.3 `MotorController::computeTorque` — velocity ramp + per-side PID

This is the core kinematic-to-torque conversion for the **velocity** path.
(The torque path skips most of this — it just slew-rate-limits the host's
torque request.)

```mermaid
flowchart TD
    A["computeTorque(dt)"] --> Z{"initialized_position_?"}
    Z -->|no| ZX["return (0, 0)"]
    Z -->|yes| B{"command_.mode?"}
    B -->|TORQUE| BT["slew-limit each side at MAX_TORQUE_CHANGE = 150 Nm/s<br/>(no PID, no feed-forward)"]
    BT --> BTX["return (torque_left, torque_right)"]
    B -->|BRAKE| BB["return (0, 0)"]
    B -->|VELOCITY| C["target_velocity = (cmd.left, −cmd.right)<br/>(right side sign flipped: opposite mounting)"]
    C --> D{"isAccelerating?"}
    D -->|yes| E["accel = MAX_ACCELERATION (6 rad/s²)"]
    D -->|no| F["accel = MAX_DECELERATION (16 rad/s²)<br/>(decel faster than accel)"]
    E --> G["velocity ← limit step to accel·dt"]
    F --> G
    G --> H{"disable_accel_limiting<br/>(PID-tuning only)?"}
    I["left_torque = MotorSideController(left).computeTorque(velocity.left, dt)"]
    H -->|yes| HX["velocity = target_velocity (skip ramp)"]
    H -->|no| I
    HX --> I
    I --> J["right_torque = MotorSideController(right).computeTorque(velocity.right, dt)"]
    J --> K{"is_rotating?<br/>(opposite signs or one near zero)"}
    N["slew-limit each torque at MAX_TORQUE_CHANGE"]
    K -->|yes| L["add ±k_s_rotational<br/>(extra static-friction kick<br/>only when turning in place)"]
    K -->|no| N
    L --> N
    N --> O["return (left_torque, right_torque)"]
```

The asymmetric accel/decel limits are intentional: starting up gently keeps
chains/tracks happy, but stopping needs to be authoritative. The rotational
feed-forward compensates for the strong static friction that fights yaw on a
tracked vehicle — it is added *on top of* the per-side velocity feed-forward
inside the PID, but only when the wheels are actually turning in opposite
directions or scrubbing.

### 3.4 `MotorSideController::computeTorque` — position hold ↔ velocity tracking

Each side automatically chooses between a **position-hold PID** (when commanded
velocity is below `VELOCITY_DEAD_ZONE = 0.1 rad/s`) and a **velocity-tracking
PID** otherwise. This means a stationary command does not let the robot creep
on a slope — the position PID actively resists external pushes.

```mermaid
stateDiagram-v2
    [*] --> POSITION
    POSITION: POSITION mode<br/>holds latest filtered position
    VELOCITY: VELOCITY mode<br/>tracks target_velocity

    POSITION --> VELOCITY: abs(target_velocity) >= 0.1 rad/s, velocity_pid_.reset()
    VELOCITY --> POSITION: abs(target_velocity) < 0.1 rad/s, position_filter_ and position_PID reset, hold_position_ = current
```

Position and velocity are estimated by fusing the front and rear motor
readings (each side has two motors) through `PositionMeasurementFilter` and
`VelocityMeasurementFilter`. This gives redundancy: if the front motor on the
left side drops out, the rear-left reading still drives the controller.

### 3.5 PID + feed-forward kernel

`PIDController::computeTorque(goal, current, dt)` implements

\[
u_\text{pid} = k_p \cdot e + k_i \cdot \!\!\int e\, dt + k_d \cdot \dot e,
\]

where \(e = \text{goal} - \text{current}\). When \(|\text{goal}|\) is above
`FEED_FORWARD_DEAD_ZONE = 0.1` (rad/s for velocity, rad for position) a
feed-forward term is added:

\[
u_\text{ff} = k_v \cdot \text{goal} + \operatorname{sign}(\text{goal}) \cdot k_s.
\]

`k_v` is a velocity-proportional inverse-plant model term, `k_s` covers
static friction so the PID does not have to integrate up from zero.

Output limiting is two-stage:

1. Slew-rate clamp: \(|u_t - u_{t-1}| \le \text{MAX\_TORQUE\_CHANGE} \cdot dt\).
2. Saturation clamp: \([-\text{MOTOR\_TORQUE\_LIMIT}, +\text{MOTOR\_TORQUE\_LIMIT}]\).

```mermaid
flowchart LR
    G["goal"] --> SUM["−"]
    C["current"] --> SUM
    SUM -->|"e"| KP["k_p · e"]
    SUM -->|"e"| INT["∫ e dt → k_i · ∫"]
    SUM -->|"e"| DER["d/dt e → k_d · ė"]
    KP --> ADD["Σ"]
    INT --> ADD
    DER --> ADD
    G -->|"goal outside dead zone"| FF["k_v · goal + sign(goal) · k_s"]
    FF --> ADD
    ADD --> SLEW["slew-rate limit<br/>(MAX_TORQUE_CHANGE · dt)"]
    SLEW --> SAT["saturate<br/>±MOTOR_TORQUE_LIMIT"]
    SAT --> OUT["torque output"]
```

Same structure is reused for both the velocity loop and the position loop on
each side; only the gains and the meaning of `goal`/`current` differ.

## 4. Wire-level: motor command framing

`MotorComm` packages the 16-bit fixed-point payload required by the Unitree A1
motor:

- **TX**: 34-byte command frame containing `motor_id`, `mode`, `torque`, plus
  unused `velocity`/`position`/`k_p`/`k_w` fields and a CRC32.
- **RX**: 78-byte status frame with measured `torque`, two velocity bands,
  `position`, `temperature`, and an `error_code`, also CRC32-checked.

Even though the A1 motor supports a built-in stiffness controller (`k_p`,
`k_w`), Athena drives the motor in **pure FOC torque mode** — i.e. it always
sends `mode = FOC` with `k_p = k_w = 0` and the torque it computed. All
control intelligence stays on the Teensy.

The Teensy interleaves the two buses (front / rear) so that:

1. Send LEFT command on front bus *and* on rear bus (parallel).
2. Read LEFT status from front bus and rear bus.
3. Send RIGHT command on front bus and rear bus.
4. Read RIGHT status from both.

This roughly halves the per-cycle bus blocking time vs. a strictly serial
implementation.

## 5. Layered safety summary

```mermaid
flowchart LR
    subgraph U["Upstream"]
        UP["Joystick / nav / autonomy"]
    end

    subgraph H["Host driver (50 Hz)"]
        H1["torque_mode gating"]
        H2["angular_velocity_limit clamp<br/>(curvature-preserving)"]
        H3["250 ms upstream-silence ⇒ zero command"]
    end

    subgraph T["Teensy (1 kHz)"]
        T1["plausibility: ±30 rad/s, ±60 Nm"]
        T2["200 ms host-silence ⇒ stop()"]
        T3["accel ramp: +6 / −16 rad/s²"]
        T4["per-side PID + FF<br/>output ±30 Nm, slew 150 Nm/s"]
        T5["20 ms motor-status timeout ⇒ BRAKE"]
        T6["bus reset / skip on comm failure"]
        T7["min-torque dead-zone ⇒ BRAKE not FOC"]
    end

    subgraph M["Motor"]
        M1["A1 firmware FOC limits"]
    end

    UP --> H1
    H1 --> H2 --> H3 --> T1 --> T2 --> T3 --> T4 --> T5 --> T6 --> T7 --> M1
```

Every layer has its own independent fallback to **stop the tracks**, and the
fallbacks bracket each other (host timeout > firmware timeout > motor-status
timeout). This is what lets a USB cable yank, a `ros2 launch` kill, or a single
RS-485 bus failure all fail safe instead of leaving the motors driving.

## 6. Where to look in the code

| Concern | File |
|---|---|
| ROS node, topic plumbing, parameters | `athena_motor_driver/src/athena_motor_driver.cpp` |
| Twist → wheel velocity (forward kinematics) | `athena_motor_driver/src/controllers/diff_drive_controller.cpp` |
| Crosstalk / serial wrapper | `athena_motor_driver/src/crosstalk_lib_serial_wrapper.hpp` |
| Wire types (`MotorCommand`, `FullMotorStatus`, ...) | `athena_motor_interface/include/athena_motor_interface/athena_motor_interfaces.h` |
| Teensy main loop + Crosstalk dispatch | `athena_motor_firmware/src/main.cpp` |
| Velocity ramp, dual-bus orchestration, mode selection | `athena_motor_firmware/src/motor_controller.cpp` |
| Per-side position/velocity mode switch + filtering | `athena_motor_firmware/src/motor_side_controller.cpp` |
| PID + feed-forward kernel | `athena_motor_firmware/src/pid_controller.cpp` |
| All tunable constants (timeouts, limits, gains) | `athena_motor_firmware/include/config.h` |
| Default ROS-side gains and kinematics | `athena_motor_driver/config/params.yaml` |
