#pragma once

#include <Arduino.h>

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include "math/ring_buffer.h"
#include "motor_comm.h"
#include "motor_side_controller.h"
#include <memory>

class MotorController
{
public:
  static constexpr float MIN_TORQUE = MIN_TORQUE_FOR_FOC;

  MotorController();

  ~MotorController();

  void init( std::shared_ptr<MotorComm> front_comm, std::shared_ptr<MotorComm> rear_comm );

  void setCommand( const MotorCommand &command );

  void setPositionPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setVelocityPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setVelocityFeedForwardGains( float left_k_v, float left_k_s, float right_k_v, float right_k_s );

  void setPositionFeedForwardGains( float left_k_v, float left_k_s, float right_k_v, float right_k_s );

  void setRotationalFeedForwardGains( float left_k_s, float right_k_s );

  void setDisableAccelerationLimiting( bool disable ) { disable_acceleration_limiting_ = disable; }

  void setVelocityRampLimits( float max_accel_rad_s2, float max_decel_rad_s2 );

  void stop();

  const FullMotorStatus &update();

  MotorComm &frontComm() { return *front_motor_comm_; }

  MotorComm &rearComm() { return *rear_motor_comm_; }

  MotorError::Error getError() const { return static_cast<MotorError::Error>( debug_data_.error ); }

  const MotorDebugData &debugData() const { return debug_data_; }

private:
  struct Torque {
    float left = 0;
    float right = 0;

    Torque( float left, float right ) : left( left ), right( right ) { }

    Torque() = default;
  };

  Torque computeTorque( float dt );
  void computeMotorCommands( float dt );
  void sendReceiveBothBuses( bool front_working, bool rear_working );
  void tryInitializePosition();
  void assembleMotorStatus();
  void collectDebugData();

  struct Velocity {
    float left = 0;
    float right = 0;
  };

  MotorCommand command_;
  MotorCommCommand left_command_;
  MotorCommCommand right_command_;
  Velocity target_velocity_;
  Torque torque_;
  Velocity velocity_;
  elapsedMicros time_since_last_command_ = 0;

  RingBuffer<elapsedMillis, STATUS_AGE_BUFFER_SIZE> status_ages_;
  MotorDebugData debug_data_;

  MotorSideController left_;
  MotorSideController right_;
  FullMotorStatus motor_status_;

  std::shared_ptr<MotorComm> front_motor_comm_;
  std::shared_ptr<MotorComm> rear_motor_comm_;
  int reset_skip_count_front_ = 0; // If motor comm fails try to skip communication for a few times
  int reset_skip_count_rear_ = 0;
  bool disable_acceleration_limiting_ = false; // For tuning PID controller
  float max_acceleration_rad_s2_ = MAX_ACCELERATION;
  float max_deceleration_rad_s2_ = MAX_DECELERATION;
  bool initialized_position_ = false;
  float rotational_feed_forward_k_s_left_ = 0.0f;
  float rotational_feed_forward_k_s_right_ = 0.0f;
};
