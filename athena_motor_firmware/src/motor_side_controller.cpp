#include "motor_side_controller.h"
#include "config.h"
#include "motor_comm.h"

static MotorStatus toMotorStatus( const MotorCommStatus &status )
{
  MotorStatus result;
  result.valid = status.valid;
  switch ( status.mode ) {
  case MotorMode::BRAKE:
    result.mode = MotorStatus::Mode::BRAKE;
    break;
  case MotorMode::FOC:
    result.mode = MotorStatus::Mode::FOC;
    break;
  case MotorMode::CALIBRATE:
    result.mode = MotorStatus::Mode::CALIBRATE;
    break;
  default:
    result.mode = MotorStatus::Mode::INVALID;
  }
  result.temperature = status.temperature;
  result.error = MotorStatus::Error( status.error_code );
  result.torque = status.torque;
  result.velocity_high = status.velocity_high;
  result.velocity_low = status.velocity_low;
  result.position = status.position;
  result.acceleration = status.acceleration;
  return result;
}

MotorSideController::MotorSideController()
    : velocity_pid_( 0, 0, 0, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT, MAX_TORQUE_CHANGE ),
      position_pid_( 0, 0, 0, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT, MAX_TORQUE_CHANGE )
{
}

void MotorSideController::updateStatus( const MotorCommStatus &status, uint8_t expected_motor_id,
                                        MotorStatus &out_status,
                                        MeanFilter<uint8_t, VALID_FILTER_SIZE> &valid_filter,
                                        elapsedMillis &age )
{
  out_status = toMotorStatus( status );
  out_status.valid &= status.motor_id == expected_motor_id;
  valid_filter.addValue( out_status.valid ? 1 : 0 );
  if ( out_status.valid )
    age = 0;
}

void MotorSideController::updateFrontStatus( const MotorCommStatus &status, uint8_t expected_motor_id )
{
  updateStatus( status, expected_motor_id, front_status_, front_valid_, front_age_ );
}

void MotorSideController::updateRearStatus( const MotorCommStatus &status, uint8_t expected_motor_id )
{
  updateStatus( status, expected_motor_id, rear_status_, rear_valid_, rear_age_ );
}

void MotorSideController::addMeasurements()
{
  position_filter_.addMeasurements( front_status_, rear_status_ );
  velocity_filter_.addMeasurements( front_status_, rear_status_ );
}

bool MotorSideController::isWorking( int timeout_ms ) const
{
  return front_age_ < static_cast<unsigned long>( timeout_ms ) ||
         rear_age_ < static_cast<unsigned long>( timeout_ms );
}

void MotorSideController::resetPositionFilter() { position_filter_.reset(); }

void MotorSideController::initializePosition() { hold_position_ = position_filter_.getFiltered(); }

float MotorSideController::computeTorque( float target_velocity, float dt )
{
  if ( std::abs( target_velocity ) < VELOCITY_DEAD_ZONE ) {
    if ( control_mode_ != ControlMode::POSITION ) {
      control_mode_ = ControlMode::POSITION;
      position_filter_.reset();
      position_pid_.reset();
      hold_position_ = position_filter_.getFiltered();
    }
    const float measured_position = position_filter_.getFiltered();
    return position_pid_.computeTorque( hold_position_, measured_position, dt );
  } else {
    if ( control_mode_ != ControlMode::VELOCITY ) {
      control_mode_ = ControlMode::VELOCITY;
      velocity_pid_.reset();
    }
    const float measured_velocity = velocity_filter_.getFiltered();
    return velocity_pid_.computeTorque( target_velocity, measured_velocity, dt );
  }
}

void MotorSideController::resetPIDControllers()
{
  velocity_pid_.reset();
  position_pid_.reset();
}

void MotorSideController::setPositionPIDGains( float kp, float ki, float kd )
{
  position_pid_.setGains( kp, ki, kd );
}

void MotorSideController::setVelocityPIDGains( float kp, float ki, float kd )
{
  velocity_pid_.setGains( kp, ki, kd );
}

void MotorSideController::setVelocityFeedForwardParams( float gain, float offset )
{
  velocity_pid_.setFeedForwardParams( gain, offset );
}

void MotorSideController::setPositionFeedForwardParams( float gain, float offset )
{
  position_pid_.setFeedForwardParams( gain, offset );
}

void MotorSideController::setDerivativeFilterCutoff( float cutoff_hz, float sample_hz )
{
  velocity_pid_.setDerivativeFilterCutoff( cutoff_hz, sample_hz );
  position_pid_.setDerivativeFilterCutoff( cutoff_hz, sample_hz );
}

float MotorSideController::validFrontFreq( long age_ms ) const
{
  if ( age_ms == 0 )
    return 0;
  return front_valid_.getSum() * 1000.0f / age_ms;
}

float MotorSideController::validRearFreq( long age_ms ) const
{
  if ( age_ms == 0 )
    return 0;
  return rear_valid_.getSum() * 1000.0f / age_ms;
}
