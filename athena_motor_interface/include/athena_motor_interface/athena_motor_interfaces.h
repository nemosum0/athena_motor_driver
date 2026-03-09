#ifndef ATHENA_MOTOR_INTERFACES_H
#define ATHENA_MOTOR_INTERFACES_H

#include "./crosstalk.hpp"

const int BAUD_RATE = 115200;

enum class CommandType : uint8_t {
  INVALID = 0,
  MOTOR_COMMAND = 1,
  MOTOR_ERROR = 2,
  CHANGE_LADRC_GAINS = 3,
  MOTOR_STATUS = 4,
  FULL_MOTOR_STATUS = 5,
  UPDATE_SETTINGS = 6,
  TEENSY_REBOOT = 8,
};

struct AckCommand {
  CommandType type;
};

REFL_AUTO( type( AckCommand, crosstalk::id( 0 ) ), field( type ) )

struct MotorCommand {
  enum class MotorMode : uint8_t { BRAKE, VELOCITY, TORQUE };
  MotorMode mode = MotorMode::BRAKE;
  float left = 0.f;
  float right = 0.f;

  MotorCommand() = default;

  static MotorCommand Velocity( float left, float right )
  {
    MotorCommand command;
    command.mode = MotorMode::VELOCITY;
    command.left = left;
    command.right = right;
    return command;
  }

  static MotorCommand Torque( float left, float right )
  {
    MotorCommand command;
    command.mode = MotorMode::TORQUE;
    command.left = left;
    command.right = right;
    return command;
  }
};

REFL_AUTO( type( MotorCommand, crosstalk::id( 1 ) ), field( mode ), field( left ), field( right ) )

struct MotorError {
  enum class Error : uint8_t { NO_ERROR = 0, STATUS_TIMEOUT = 1 };

  Error error = Error::NO_ERROR;

  MotorError() = default;

  MotorError( Error error ) : error( error ) { }
};

REFL_AUTO( type( MotorError, crosstalk::id( 2 ) ), field( error ) )

struct LadrcGains {
  float b0 = 1.0f;
  float omega_c = 40.0f;
  float omega_o = 150.0f;
  float kp_pos = 1600.0f;
  float f_c = 0.5f;
  float f_s = 1.0f;
  float slip_torque_threshold = 5.0f;
  float slip_vel_threshold = 1.0f;

  LadrcGains() = default;
};

REFL_AUTO( type( LadrcGains ), field( b0 ), field( omega_c ), field( omega_o ), field( kp_pos ),
           field( f_c ), field( f_s ), field( slip_torque_threshold ), field( slip_vel_threshold ) )

struct ChangeLadrcGainsCommand {
  LadrcGains left_gains;
  LadrcGains right_gains;

  ChangeLadrcGainsCommand() = default;

  ChangeLadrcGainsCommand( const LadrcGains &left, const LadrcGains &right )
      : left_gains( left ), right_gains( right )
  {
  }
};

REFL_AUTO( type( ChangeLadrcGainsCommand, crosstalk::id( 3 ) ), field( left_gains ),
           field( right_gains ) )

struct MotorStatus {
  enum class Error : uint8_t {
    NO_ERROR = 0, // TODO: Find out other error codes
  };

  enum class Mode : uint8_t { INVALID, BRAKE, FOC, CALIBRATE };

  bool valid = false;
  Mode mode = Mode::INVALID;
  int8_t temperature = 0;
  Error error = Error::NO_ERROR;

  float target_velocity = 0;
  float target_torque = 0;
  float torque = 0;
  //! The motor speed (high speed and low speed, no clue what that means, ask Unitree)
  float velocity_high = 0;
  float velocity_low = 0;
  float position = 0;
  float acceleration = 0;
  uint32_t age_ms = UINT32_MAX;
};

REFL_AUTO( type( MotorStatus, crosstalk::id( 4 ) ), field( valid ), field( mode ),
           field( temperature ), field( error ), field( target_velocity ), field( target_torque ),
           field( torque ), field( velocity_high ), field( velocity_low ), field( position ),
           field( acceleration ), field( age_ms ) )

struct FullMotorStatus {
  MotorStatus front_left;
  MotorStatus front_right;
  MotorStatus rear_left;
  MotorStatus rear_right;
  float velocity_left = 0;
  float velocity_right = 0;
};

REFL_AUTO( type( FullMotorStatus, crosstalk::id( 5 ) ), field( front_left ), field( front_right ),
           field( rear_left ), field( rear_right ), field( velocity_left ), field( velocity_right ) )

struct UpdateSettings {
  bool enable_debug = false;
  bool disable_acceleration_limiting = false;
};

REFL_AUTO( type( UpdateSettings, crosstalk::id( 6 ) ), field( enable_debug ),
           field( disable_acceleration_limiting ) )

struct PIDDebugData {
  float goal = std::numeric_limits<float>::quiet_NaN();
  float current = std::numeric_limits<float>::quiet_NaN();
  float dt = std::numeric_limits<float>::quiet_NaN();
  float error = std::numeric_limits<float>::quiet_NaN();
  float derivative = std::numeric_limits<float>::quiet_NaN();
  float integral = std::numeric_limits<float>::quiet_NaN();
  float raw_output = std::numeric_limits<float>::quiet_NaN();
  float output = std::numeric_limits<float>::quiet_NaN();
};

REFL_AUTO( type( PIDDebugData ), field( goal ), field( current ), field( dt ), field( error ),
           field( derivative ), field( integral ), field( raw_output ), field( output ) )

struct MotorStatusDebugData {
  float freq_front_left = 0;
  float freq_front_right = 0;
  float freq_rear_left = 0;
  float freq_rear_right = 0;
};

REFL_AUTO( type( MotorStatusDebugData ), field( freq_front_left ), field( freq_front_right ),
           field( freq_rear_left ), field( freq_rear_right ) )

struct MotorDebugData {
  PIDDebugData left_velocity_pid;
  PIDDebugData right_velocity_pid;
  PIDDebugData left_position_pid;
  PIDDebugData right_position_pid;
  MotorStatusDebugData status;
  enum class Error { NO_ERROR, NO_MOTOR_STATUS };
  Error error;
  uint16_t average_loop_time_us = 0;
};

REFL_AUTO( type( MotorDebugData, crosstalk::id( 7 ) ), field( left_velocity_pid ),
           field( right_velocity_pid ), field( left_position_pid ), field( right_position_pid ),
           field( status ), field( error ), field( average_loop_time_us ) )

struct TeensyRebootCommand {
  uint8_t magic = 0xDE;
  uint8_t command = 0xAD;
};

REFL_AUTO( type( TeensyRebootCommand, crosstalk::id( 8 ) ), field( magic ), field( command ) )

#endif // ATHENA_MOTOR_INTERFACES_H
