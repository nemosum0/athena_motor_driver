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

void MotorController::setVelocityStartupParams( float left_gain, float left_offset,
                                                float right_gain, float right_offset )
{
  left_.setVelocityStartupParams( left_gain, left_offset );
  right_.setVelocityStartupParams( right_gain, right_offset );
}

void MotorController::setVelocityRampLimits( float max_accel_rad_s2, float max_decel_rad_s2 )
{
  constexpr float k_min = 0.1f;
  constexpr float k_max = 200.0f;
  max_acceleration_rad_s2_ = std::clamp( max_accel_rad_s2, k_min, k_max );
  max_deceleration_rad_s2_ = std::clamp( max_decel_rad_s2, k_min, k_max );
}

void MotorController::setVelocityReferenceJerkLimit( float max_jerk_rad_s3 )
{
  constexpr float k_max_jerk = 1.0e6f;
  max_track_jerk_rad_s3_ = std::clamp( max_jerk_rad_s3, 0.f, k_max_jerk );
}

void MotorController::setDerivativeFilterCutoff( float cutoff_hz, float sample_hz )
{
  left_.setDerivativeFilterCutoff( cutoff_hz, sample_hz );
  right_.setDerivativeFilterCutoff( cutoff_hz, sample_hz );
}

void MotorController::stop()
{
  command_.mode = MotorCommand::MotorMode::BRAKE;
  command_.left = 0;
  command_.right = 0;
  target_velocity_.left = 0;
  target_velocity_.right = 0;
  velocity_reference_accel_left_rad_s2_ = 0.f;
  velocity_reference_accel_right_rad_s2_ = 0.f;
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

float limitTorqueChange( float target_torque, float current_torque, float max_torque_change )
{
  if ( std::abs( target_torque - current_torque ) <= max_torque_change ) {
    return target_torque;
  }
  return current_torque + std::copysign( max_torque_change, target_torque - current_torque );
}

float slewScalarToward( float current, float goal, float max_step )
{
  if ( goal > current + max_step )
    return current + max_step;
  if ( goal < current - max_step )
    return current - max_step;
  return goal;
}
} // namespace

void MotorController::updateVelocityReferenceWithLimits( float commanded_velocity_rad_s,
                                                         float &reference_velocity_rad_s,
                                                         float &reference_signed_accel_rad_s2,
                                                         float dt )
{
  const float error = commanded_velocity_rad_s - reference_velocity_rad_s;
  float accel_mag = max_deceleration_rad_s2_;
  if ( isAccelerating( commanded_velocity_rad_s, reference_velocity_rad_s ) ) {
    accel_mag = max_acceleration_rad_s2_;
  }
  constexpr float k_err_eps = 1e-5f;
  const float desired_signed_accel =
      ( std::fabs( error ) < k_err_eps ) ? 0.f : std::copysign( accel_mag, error );

  if ( max_track_jerk_rad_s3_ <= 0.f ) {
    reference_signed_accel_rad_s2 = desired_signed_accel;
  } else {
    const float max_da = max_track_jerk_rad_s3_ * dt;
    reference_signed_accel_rad_s2 =
        slewScalarToward( reference_signed_accel_rad_s2, desired_signed_accel, max_da );
  }

  const float max_dv = std::fabs( reference_signed_accel_rad_s2 ) * dt;
  reference_velocity_rad_s =
      limitVelocityChange( commanded_velocity_rad_s, reference_velocity_rad_s, max_dv );
}

MotorController::Torque MotorController::computeTorque( float dt )
{
  if ( !initialized_position_ ) {
    return { 0, 0 }; // Do not issue any torque commands until position is initialized
  }
  if ( command_.mode == MotorCommand::MotorMode::TORQUE ) {
    float max_torque_change = MAX_TORQUE_CHANGE * dt;
    float torque_left = limitTorqueChange( command_.left, torque_.left, max_torque_change );
    float torque_right = limitTorqueChange( -command_.right, torque_.right, max_torque_change );
    torque_.left = torque_left;
    torque_.right = torque_right;
    return { torque_left, torque_right };
  } else if ( command_.mode == MotorCommand::MotorMode::BRAKE ) {
    return { 0, 0 };
  }
  target_velocity_.left = command_.left;
  target_velocity_.right = -command_.right;

  updateVelocityReferenceWithLimits( target_velocity_.left, velocity_.left,
                                     velocity_reference_accel_left_rad_s2_, dt );
  updateVelocityReferenceWithLimits( target_velocity_.right, velocity_.right,
                                     velocity_reference_accel_right_rad_s2_, dt );

  if ( disable_acceleration_limiting_ ) {
    // If acceleration limits are disabled, we just set the target velocity directly
    // This is useful for tuning the PID controller but should not be used in normal operation
    velocity_.left = target_velocity_.left;
    velocity_.right = target_velocity_.right;
    velocity_reference_accel_left_rad_s2_ = 0.f;
    velocity_reference_accel_right_rad_s2_ = 0.f;
  }

  float left_torque = left_.computeTorque( velocity_.left, dt );
  float right_torque = right_.computeTorque( velocity_.right, dt );

  if ( !std::isfinite( left_torque ) || !std::isfinite( right_torque ) ) {
    static ThrottlePrinter printer( 1000 );
    printer.print( "Invalid torque values detected. This is a bug!" );
    left_torque = 0;
    right_torque = 0;
  }

  const float max_torque_change = MAX_TORQUE_CHANGE * dt;
  left_torque = limitTorqueChange( left_torque, torque_.left, max_torque_change );
  right_torque = limitTorqueChange( right_torque, torque_.right, max_torque_change );
  torque_.left = left_torque;
  torque_.right = right_torque;

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

void MotorController::computeMotorCommands( float dt )
{
  left_command_.motor_id = 0;
  right_command_.motor_id = 1;
  const bool left_working = left_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  const bool right_working = right_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  if ( !left_working || !right_working ) {
    // At least one motor on each side needs to be working, otherwise we stop
    velocity_.left = 0;
    velocity_.right = 0;
    velocity_reference_accel_left_rad_s2_ = 0.f;
    velocity_reference_accel_right_rad_s2_ = 0.f;
    left_command_.mode = MotorMode::BRAKE;
    right_command_.mode = MotorMode::BRAKE;
    // Reset all PID controllers so it will not try to jump back to a position when power is restored
    left_.resetPIDControllers();
    right_.resetPIDControllers();
    initialized_position_ = false;
    debug_data_.error = MotorDebugData::Error::NO_MOTOR_STATUS;
  } else {
    Torque torque = computeTorque( dt );
    setCommandFromTorque( left_command_, initialized_position_ ? torque.left : 0 );
    setCommandFromTorque( right_command_, initialized_position_ ? torque.right : 0 );
  }
}

void MotorController::sendReceiveBothBuses( bool front_working, bool rear_working )
{
  bool front_active = front_working || ++reset_skip_count_front_ > MAX_RESET_SKIP_COUNT;
  bool rear_active = rear_working || ++reset_skip_count_rear_ > MAX_RESET_SKIP_COUNT;

  if ( front_active )
    reset_skip_count_front_ = 0;
  else
    front_motor_comm_->resetComm();

  if ( rear_active )
    reset_skip_count_rear_ = 0;
  else
    rear_motor_comm_->resetComm();

  MotorCommStatus front_left_status, front_right_status;
  MotorCommStatus rear_left_status, rear_right_status;

  // Interleaved communication: overlap motor processing time across buses.
  // Both buses use independent serial lines, so TX/RX can overlap.

  // Phase 1: Send left motor command on both buses
  if ( front_active )
    front_motor_comm_->sendCommand( left_command_ );
  if ( rear_active )
    rear_motor_comm_->sendCommand( left_command_ );

  // Phase 2: Read left motor responses (motor was processing during the other bus's TX)
  if ( front_active )
    front_left_status = front_motor_comm_->receiveStatus();
  if ( rear_active )
    rear_left_status = rear_motor_comm_->receiveStatus();

  // Phase 3: Send right motor command on both buses
  if ( front_active )
    front_motor_comm_->sendCommand( right_command_ );
  if ( rear_active )
    rear_motor_comm_->sendCommand( right_command_ );

  // Phase 4: Read right motor responses
  if ( front_active )
    front_right_status = front_motor_comm_->receiveStatus();
  if ( rear_active )
    rear_right_status = rear_motor_comm_->receiveStatus();

  left_.updateFrontStatus( front_left_status, 0 );
  right_.updateFrontStatus( front_right_status, 1 );
  left_.updateRearStatus( rear_left_status, 0 );
  right_.updateRearStatus( rear_right_status, 1 );
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

void MotorController::assembleMotorStatus()
{
  motor_status_.front_left = left_.frontStatus();
  motor_status_.front_right = right_.frontStatus();
  motor_status_.rear_left = left_.rearStatus();
  motor_status_.rear_right = right_.rearStatus();
  motor_status_.velocity_left = left_.filteredVelocity();
  motor_status_.velocity_right = right_.filteredVelocity();

  const float left_target_torque = left_command_.mode == MotorMode::FOC ? left_command_.torque : 0;
  motor_status_.front_left.target_torque = left_target_torque;
  motor_status_.rear_left.target_torque = left_target_torque;
  motor_status_.front_left.target_velocity = velocity_.left;
  motor_status_.rear_left.target_velocity = velocity_.left;

  const float right_target_torque = right_command_.mode == MotorMode::FOC ? right_command_.torque : 0;
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

  // Cap dt to 20 ms to prevent observer divergence after loop pauses (e.g. E-stop)
  const float dt = std::min<float>( float( time_since_last_command_ ) / 1E6f, 0.020f );
  time_since_last_command_ = 0;
  computeMotorCommands( dt );

  const bool front_working =
      left_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS;
  const bool rear_working =
      left_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS;

  sendReceiveBothBuses( front_working, rear_working );

  tryInitializePosition();
  left_.addMeasurements();
  right_.addMeasurements();

  assembleMotorStatus();
  collectDebugData();

  return motor_status_;
}
