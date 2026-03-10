#pragma once

// Athena Motor Driver Firmware Configuration
//
// Architecture: Teensy 4.0 controls 4 Unitree motors over 2 RS485 buses.
// Host (Jetson) communicates via USB/crosstalk protocol.
//
// Control hierarchy:
//   main.cpp (host comms + timer ISR)
//     -> MotorController (velocity ramp, torque safety)
//       -> MotorSideController x2 (position/velocity PID + measurement fusion)
//         -> PIDController (PID + feed-forward)
//
// Motor comm: MotorComm handles serial protocol (34B TX, 78B RX, CRC32).
// Two buses: Serial1 (front), Serial2 (rear), each with left (ID=0) and right (ID=1) motors.

// ============================================================
// Timing
// ============================================================
static constexpr int MAIN_LOOP_PERIOD_US = 1000;   // 1000 Hz control loop
static constexpr int COMMAND_TIMEOUT_MS = 200;     // Stop motors if no host command
static constexpr int MOTOR_STATUS_TIMEOUT_MS = 20; // Motor considered dead
static constexpr int STARTUP_DELAY_MS = 20;
static constexpr int REBOOT_DELAY_MS = 10;

// ============================================================
// Torque limits
// ============================================================
static constexpr float MAX_PLAUSIBLE_TORQUE_COMMAND = 60.0f; // Nm
static constexpr float MOTOR_TORQUE_LIMIT = 30.0f;           // Nm
static constexpr float MAX_TORQUE_CHANGE = 150.0f;           // Nm/s
static constexpr float MIN_TORQUE_FOR_FOC = 0.5f;            // Below this -> BRAKE mode

// ============================================================
// Velocity / acceleration limits
// ============================================================
static constexpr float MAX_PLAUSIBLE_VELOCITY_COMMAND = 30.0f; // rad/s
static constexpr float MAX_ACCELERATION = 6.0f;                // rad/s^2
static constexpr float MAX_DECELERATION = 16.0f;               // rad/s^2
static constexpr float VELOCITY_DEAD_ZONE = 0.1f;              // rad/s, below this -> position hold
static constexpr float FEED_FORWARD_DEAD_ZONE = 0.1f;          // Goal threshold for feed-forward

// ============================================================
// Communication recovery
// ============================================================
static constexpr int MAX_RESET_SKIP_COUNT = 10; // Cycles to skip after comm failure

// ============================================================
// Position filter
// ============================================================
// Range comes from 14-bit encoder (16384 ticks) mapped through gear ratio.
// This was recorded, as I'm unsure about how the motor limits are computed.
// It seems to be based on a range of 512 ticks, but it's not based on 9-bit as it
// goes above 255. Maybe -256 to +256. Then it would be +/-176,75774051
static constexpr float POSITION_LOWER_END = -176.756729f;
static constexpr float POSITION_UPPER_END = 176.752930f;

// ============================================================
// Velocity filter
// ============================================================
static constexpr int VELOCITY_FILTER_WINDOW_SIZE = 50;        // ~100ms averaging window at 500 Hz
static constexpr float MAX_PLAUSIBLE_POSITION_CHANGE = 0.35f; // rad (~20 deg), sanity check
static constexpr int MAX_MEASUREMENT_AGE_US = 6000;           // Discard stale measurements

// ============================================================
// Status LED
// ============================================================
static constexpr int LED_SLOW_BLINK_MS = 1000;
static constexpr int LED_FAST_BLINK_MS = 200;

// ============================================================
// Monitoring
// ============================================================
static constexpr int STATUS_AGE_BUFFER_SIZE = 50;
static constexpr int VALID_FILTER_SIZE = 50;
static constexpr int LOOP_TIME_FILTER_SIZE = 10;
