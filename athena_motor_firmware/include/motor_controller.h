#pragma once

#include <Arduino.h>

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "math/ring_buffer.h"
#include "motor_side_controller.h"
#include <memory>

class MotorComm;

class MotorController
{
public:
  // Min torque to issue movement command rather than brake
  static constexpr float MIN_TORQUE = 0.5f;

  MotorController();

  ~MotorController();

  void init( std::shared_ptr<MotorComm> front_comm, std::shared_ptr<MotorComm> rear_comm );

  void setCommand( const MotorCommand &command );

  void setPositionPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setVelocityPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setVelocityFeedForwardGains( float left_k_v, float left_k_s, float right_k_v,
                                    float right_k_s );

  void setPositionFeedForwardGains( float left_k_v, float left_k_s, float right_k_v,
                                    float right_k_s );

  void setDisableAccelerationLimiting( bool disable ) { disable_acceleration_limiting_ = disable; }

  void stop();

  const FullMotorStatus &update();

  MotorComm &frontComm() { return *front_motor_comm_; }

  MotorComm &rearComm() { return *rear_motor_comm_; }

  MotorError::Error getError() const
  {
    return static_cast<MotorError::Error>( debug_data_.error );
  }

  const MotorDebugData &debugData() const { return debug_data_; }

private:
  // In m/s^2. To reach 1U/s=2*pi rad/s in 0.5 seconds, acceleration would be 2m/s^2
  static constexpr float MAX_ACCELERATION = 8.0f;
  static constexpr float MAX_DECELERATION = 24.0f;

  struct Torque {
    float left = 0;
    float right = 0;

    Torque( float left, float right ) : left( left ), right( right ) { }

    Torque() = default;
  };

  Torque computeTorque();

  struct Velocity {
    float left = 0;
    float right = 0;
  };

  MotorCommand command_;
  Velocity target_velocity_;
  Velocity velocity_;
  elapsedMicros time_since_last_command_ = 0;

  RingBuffer<elapsedMillis, 50> status_ages_;
  MotorDebugData debug_data_;

  MotorSideController left_;
  MotorSideController right_;
  FullMotorStatus motor_status_;

  std::shared_ptr<MotorComm> front_motor_comm_;
  std::shared_ptr<MotorComm> rear_motor_comm_;
  int reset_skip_count_front_ = 0; // If motor comm fails try to skip communication for a few times
  int reset_skip_count_rear_ = 0;
  bool disable_acceleration_limiting_ = false; // For tuning PID controller
  bool initialized_position_ = false;
};
