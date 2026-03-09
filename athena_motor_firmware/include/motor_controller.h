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

  void setLadrcGains( const LadrcGains &left_gains, const LadrcGains &right_gains );

  void setDisableAccelerationLimiting( bool disable ) { disable_acceleration_limiting_ = disable; }

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

  Torque computeTorque();
  void computeMotorCommands( MotorCommCommand &left_command, MotorCommCommand &right_command );
  void sendReceiveBus( std::shared_ptr<MotorComm> &comm, int &reset_skip_count,
                       const MotorCommCommand &left_command, const MotorCommCommand &right_command,
                       bool bus_working, bool is_front );
  void tryInitializePosition();
  void assembleMotorStatus( const MotorCommCommand &left_command,
                            const MotorCommCommand &right_command );
  void collectDebugData();

  struct Velocity {
    float left = 0;
    float right = 0;
  };

  MotorCommand command_;
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
  bool initialized_position_ = false;
  float rotational_feed_forward_k_s_left_ = 0.0f;
  float rotational_feed_forward_k_s_right_ = 0.0f;

  MotorCommCommand left_command_;
  MotorCommCommand right_command_;
};
