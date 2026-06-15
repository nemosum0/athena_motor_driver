# athena_motor_driver

ROS 2 driver for the Athena robot motor controller. This node communicates with the Teensy-based firmware via serial to control the motors (velocity or torque control) and retrieve status information.

## Overview

The `athena_motor_driver` node provides an interface between the ROS 2 navigation stack (or other controllers) and the low-level motor hardware. It supports differential drive kinematics.

## Nodes

### `athena_motor_driver`

The main driver node.

#### Subscribed Topics

| Topic              | Type                                       | Description                                                                  |
| ------------------ | ------------------------------------------ | ---------------------------------------------------------------------------- |
| `cmd_vel`          | `geometry_msgs/msg/Twist`                  | Target velocity command for the robot. Ignored if torque mode is active.     |
| `~/forward_torque` | `athena_motor_interface/msg/TorqueCommand` | Direct torque command for left and right motors. Only active in torque mode. |

#### Published Topics

| Topic          | Type                                         | Description                                                                   |
| -------------- | -------------------------------------------- | ----------------------------------------------------------------------------- |
| `motor_status` | `athena_motor_interface/msg/FullMotorStatus` | Comprehensive status of the motors including position, velocity, and current. |
| `~/debug_data` | `athena_motor_interface/msg/DebugData`       | PID internals and other debug info. Published only if `enable_debug` is true. |

#### Services

| Service    | Type                   | Description                                           |
| ---------- | ---------------------- | ----------------------------------------------------- |
| `~/reboot` | `std_srvs/srv/Trigger` | Reboots the microcontroller via the serial interface. |

#### Parameters

The node can be configured via `config/params.yaml`.

| Parameter                  | Type   | Default                             | Description                                                             |
| -------------------------- | ------ | ----------------------------------- | ----------------------------------------------------------------------- |
| `port_name`                | string | `"/dev/tty_drive_motor_controller"` | Serial port device path.                                                |
| `baud_rate`                | int    | -                                   | Serial baud rate (auto-configured to 1152000 in code but customizable). |
| `controller`               | string | `"diff_drive"`                      | Controller type. Currently only supports "diff_drive".                  |
| `enable_debug`             | bool   | `true`                              | Whether to publish extra debug data.                                    |
| `wheel_separation`         | double | `1.248`                             | Distance between wheels.                                                |
| `wheel_radius`             | double | `0.075`                             | Radius of the wheels in meters.                                         |
| `rotational_amplification` | double | `6.4`                               | Helper factor for turn-in-place tuning.                                 |

**PID & Startup Parameters:**

Sets of parameters `k_p`, `k_i`, `k_d` for `left_velocity_pid`, `right_velocity_pid`, `left_position_pid`, `right_position_pid`.
Sets of parameters `gain`, `offset` for `left_velocity_startup`, `right_velocity_startup`.

## Launch

To launch the driver:

```bash
ros2 launch athena_motor_driver athena_motor_driver_launch.yml
```
