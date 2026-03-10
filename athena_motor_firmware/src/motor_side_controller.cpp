#include "motor_side_controller.h"
#include "config.h"
#include "motor_comm.h"

#include <cmath>

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

MotorSideController::MotorSideController() { }

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
}

bool MotorSideController::isWorking( int timeout_ms ) const
{
  return front_age_ < static_cast<unsigned long>( timeout_ms ) ||
         rear_age_ < static_cast<unsigned long>( timeout_ms );
}

void MotorSideController::resetPositionFilter() { position_filter_.reset(); }

void MotorSideController::initializePosition() { ladrc_.reset(); }

void MotorSideController::updateObserver( float dt )
{
  // Re-center position near zero to prevent float32 precision loss
  // during prolonged rotation. Shifts both filter and observer by the
  // same offset so all relative differences are preserved.
  static constexpr float RECENTER_THRESHOLD =
      POSITION_UPPER_END - POSITION_LOWER_END; // ~one full encoder range
  const float pos = position_filter_.getFiltered();
  if ( std::abs( pos ) > RECENTER_THRESHOLD ) {
    position_filter_.recenter( pos );
    ladrc_.recenterPosition( pos );
  }

  const float measured_position = position_filter_.getFiltered();
  float measured_torque = 0.0f;
  int valid_count = 0;
  if ( front_status_.valid ) {
    measured_torque += front_status_.torque;
    valid_count++;
  }
  if ( rear_status_.valid ) {
    measured_torque += rear_status_.torque;
    valid_count++;
  }
  if ( valid_count > 0 ) {
    measured_torque /= valid_count;
  }
  ladrc_.updateObserver( measured_position, measured_torque, dt );
}

float MotorSideController::computeTorque( float target_velocity, float dt )
{
  return static_cast<float>( ladrc_.computeControlLaw( target_velocity, dt ) );
}

void MotorSideController::setAppliedTorque( float torque ) { ladrc_.setAppliedTorque( torque ); }

void MotorSideController::resetControllers() { ladrc_.reset(); }

void MotorSideController::setLadrcConfig( const LadrcController::Config &config )
{
  ladrc_.setConfig( config );
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
