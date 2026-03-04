#include "motor_controller.h"
#include "config.h"
#include "motor_comm.h"
#include "throttle_printer.hpp"

#include <algorithm>

MotorController::MotorController() { }

MotorController::~MotorController() = default;

void MotorController::init( std::shared_ptr<MotorComm> front_comm,
                            std::shared_ptr<MotorComm> rear_comm )
{
  front_motor_comm_ = front_comm;
  rear_motor_comm_ = rear_comm;
}

void MotorController::setCommand( const MotorCommand &command ) { command_ = command; }

void MotorController::setPositionPIDGains( const PIDGains &left_pid_gains,
                                           const PIDGains &right_pid_gains )
{
  left_.setPositionPIDGains( left_pid_gains.k_p, left_pid_gains.k_i, left_pid_gains.k_d );
  right_.setPositionPIDGains( right_pid_gains.k_p, right_pid_gains.k_i, right_pid_gains.k_d );
}

void MotorController::setVelocityPIDGains( const PIDGains &left_pid_gains,
                                           const PIDGains &right_pid_gains )
{
  left_.setVelocityPIDGains( left_pid_gains.k_p, left_pid_gains.k_i, left_pid_gains.k_d );
  right_.setVelocityPIDGains( right_pid_gains.k_p, right_pid_gains.k_i, right_pid_gains.k_d );
}

void MotorController::setVelocityFeedForwardGains( float left_k_v, float left_k_s, float right_k_v,
                                                   float right_k_s )
{
  left_.setVelocityFeedForwardGains( left_k_v, left_k_s );
  right_.setVelocityFeedForwardGains( right_k_v, right_k_s );
}

void MotorController::setPositionFeedForwardGains( float left_k_v, float left_k_s, float right_k_v,
                                                   float right_k_s )
{
  left_.setPositionFeedForwardGains( left_k_v, left_k_s );
  right_.setPositionFeedForwardGains( right_k_v, right_k_s );
}

void MotorController::stop()
{
  command_ = MotorCommand();
  target_velocity_.left = 0;
  target_velocity_.right = 0;
}

namespace
{
bool isAccelerating( float target_velocity, float current_velocity )
{
  return std::signbit( target_velocity ) == std::signbit( current_velocity ) &&
         std::abs( target_velocity ) > std::abs( current_velocity );
}

float limitVelocityChange( float target_velocity, float current_velocity, float max_velocity_change )
{
  if ( std::abs( target_velocity - current_velocity ) <= max_velocity_change ) {
    return target_velocity;
  }
  return current_velocity + std::copysign( max_velocity_change, target_velocity - current_velocity );
}
} // namespace

MotorController::Torque MotorController::computeTorque()
{
  if ( !initialized_position_ ) {
    return { 0, 0 }; // Do not issue any torque commands until position is initialized
  }
  if ( command_.mode == MotorCommand::MotorMode::TORQUE ) {
    return { command_.left, -command_.right };
  } else if ( command_.mode == MotorCommand::MotorMode::BRAKE ) {
    return { 0, 0 };
  }
  target_velocity_.left = command_.left;
  target_velocity_.right = -command_.right;
  // Limit acceleration
  // Really simple ramp up and faster ramp down for breaking
  float acceleration = MAX_DECELERATION;
  if ( isAccelerating( target_velocity_.left, velocity_.left ) ||
       isAccelerating( target_velocity_.right, velocity_.right ) ) {
    acceleration = MAX_ACCELERATION;
  }

  // Cap elapsed time to 30 ms to avoid large jumps after long delays
  long elapsed_micros = std::min<long>( time_since_last_command_, 30'000 );
  const float max_velocity_change = acceleration * elapsed_micros / 1E6f;

  velocity_.left = limitVelocityChange( target_velocity_.left, velocity_.left, max_velocity_change );
  velocity_.right =
      limitVelocityChange( target_velocity_.right, velocity_.right, max_velocity_change );

  if ( disable_acceleration_limiting_ ) {
    // If acceleration limits are disabled, we just set the target velocity directly
    // This is useful for tuning the PID controller but should not be used in normal operation
    velocity_.left = target_velocity_.left;
    velocity_.right = target_velocity_.right;
  }

  float left_torque = left_.computeTorque( velocity_.left );
  float right_torque = right_.computeTorque( velocity_.right );

  if ( !std::isfinite( left_torque ) || !std::isfinite( right_torque ) ) {
    static ThrottlePrinter printer( 1000 );
    printer.print( "Invalid torque values detected. This is a bug!" );
    left_torque = 0;
    right_torque = 0;
  }
  return { left_torque, right_torque };
}

static void setCommandFromTorque( MotorCommCommand &command, float torque )
{
  if ( std::abs( torque ) > MotorController::MIN_TORQUE ) {
    command.mode = MotorMode::FOC;
    command.torque = constrain( torque, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT );
  } else {
    command.mode = MotorMode::BRAKE;
    command.torque = 0;
  }
}

void MotorController::computeMotorCommands( MotorCommCommand &left_command,
                                            MotorCommCommand &right_command )
{
  left_command.motor_id = 0;
  right_command.motor_id = 1;
  const bool left_working = left_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  const bool right_working = right_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  if ( !left_working || !right_working ) {
    // At least one motor on each side needs to be working, otherwise we stop
    velocity_.left = 0;
    velocity_.right = 0;
    left_command.mode = MotorMode::BRAKE;
    right_command.mode = MotorMode::BRAKE;
    // Reset all PID controllers so it will not try to jump back to a position when power is restored
    left_.resetPIDControllers();
    right_.resetPIDControllers();
    initialized_position_ = false;
    debug_data_.error = MotorDebugData::Error::NO_MOTOR_STATUS;
  } else {
    Torque torque = computeTorque();
    setCommandFromTorque( left_command, initialized_position_ ? torque.left : 0 );
    setCommandFromTorque( right_command, initialized_position_ ? torque.right : 0 );
  }
  time_since_last_command_ = 0;
}

void MotorController::sendReceiveBus( std::shared_ptr<MotorComm> &comm, int &reset_skip_count,
                                      const MotorCommCommand &left_command,
                                      const MotorCommCommand &right_command, bool bus_working,
                                      bool is_front )
{
  MotorCommStatus left_status;
  MotorCommStatus right_status;
  if ( bus_working || ++reset_skip_count > MAX_RESET_SKIP_COUNT ) {
    // When communication fails, skip commands for a few cycles so if motor comm is
    // misaligned it has time to recover
    reset_skip_count = 0;
    comm->sendReceive( left_command, right_command, left_status, right_status );
  } else {
    comm->resetComm();
  }

  if ( is_front ) {
    left_.updateFrontStatus( left_status, 0 );
    right_.updateFrontStatus( right_status, 1 );
  } else {
    left_.updateRearStatus( left_status, 0 );
    right_.updateRearStatus( right_status, 1 );
  }
}

void MotorController::tryInitializePosition()
{
  if ( initialized_position_ )
    return;

  left_.resetPositionFilter();
  right_.resetPositionFilter();
  // If at least one motor on each bus is valid, we can initialize the position
  const bool front_has_valid = left_.frontStatus().valid || right_.frontStatus().valid;
  const bool rear_has_valid = left_.rearStatus().valid || right_.rearStatus().valid;
  if ( front_has_valid && rear_has_valid ) {
    initialized_position_ = true;
    left_.initializePosition();
    right_.initializePosition();
  }
}

void MotorController::assembleMotorStatus( const MotorCommCommand &left_command,
                                           const MotorCommCommand &right_command )
{
  motor_status_.front_left = left_.frontStatus();
  motor_status_.front_right = right_.frontStatus();
  motor_status_.rear_left = left_.rearStatus();
  motor_status_.rear_right = right_.rearStatus();
  motor_status_.velocity_left = left_.filteredVelocity();
  motor_status_.velocity_right = right_.filteredVelocity();

  const float left_target_torque = left_command.mode == MotorMode::FOC ? left_command.torque : 0;
  motor_status_.front_left.target_torque = left_target_torque;
  motor_status_.rear_left.target_torque = left_target_torque;
  motor_status_.front_left.target_velocity = velocity_.left;
  motor_status_.rear_left.target_velocity = velocity_.left;

  const float right_target_torque = right_command.mode == MotorMode::FOC ? right_command.torque : 0;
  motor_status_.front_right.target_torque = right_target_torque;
  motor_status_.rear_right.target_torque = right_target_torque;
  motor_status_.front_right.target_velocity = velocity_.right;
  motor_status_.rear_right.target_velocity = velocity_.right;

  motor_status_.front_left.age_ms = left_.frontAgeMs();
  motor_status_.front_right.age_ms = right_.frontAgeMs();
  motor_status_.rear_left.age_ms = left_.rearAgeMs();
  motor_status_.rear_right.age_ms = right_.rearAgeMs();
}

void MotorController::collectDebugData()
{
  status_ages_.push( elapsedMillis() );
  long status_age_ms = std::max<long>( 1, status_ages_.front() ); // Avoid 0ms from first call.
  debug_data_.status.freq_front_left = left_.validFrontFreq( status_age_ms );
  debug_data_.status.freq_front_right = right_.validFrontFreq( status_age_ms );
  debug_data_.status.freq_rear_left = left_.validRearFreq( status_age_ms );
  debug_data_.status.freq_rear_right = right_.validRearFreq( status_age_ms );
  debug_data_.left_velocity_pid = left_.velocityPIDDebugData();
  debug_data_.right_velocity_pid = right_.velocityPIDDebugData();
  debug_data_.left_position_pid = left_.positionPIDDebugData();
  debug_data_.right_position_pid = right_.positionPIDDebugData();
}

const FullMotorStatus &MotorController::update()
{
  debug_data_.error = MotorDebugData::Error::NO_ERROR;

  MotorCommCommand left_command;
  MotorCommCommand right_command;
  computeMotorCommands( left_command, right_command );

  const bool front_working =
      left_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS;
  const bool rear_working =
      left_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS;

  sendReceiveBus( front_motor_comm_, reset_skip_count_front_, left_command, right_command,
                  front_working, true );
  sendReceiveBus( rear_motor_comm_, reset_skip_count_rear_, left_command, right_command,
                  rear_working, false );

  tryInitializePosition();
  left_.addMeasurements();
  right_.addMeasurements();

  assembleMotorStatus( left_command, right_command );
  collectDebugData();

  return motor_status_;
}
