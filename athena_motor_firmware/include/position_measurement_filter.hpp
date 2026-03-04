#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"

#include <Arduino.h>

class PositionMeasurementFilter
{
public:
  void addMeasurements( const MotorStatus &front, const MotorStatus &rear );

  float getFiltered() const;

  void reset()
  {
    position_ = 0;
    front_offset_ = 0;
    rear_offset_ = 0;
    front_initialized_ = false;
    rear_initialized_ = false;
  }

private:
  float computeNormalizedPosition( float position, float offset ) const;

  float position_ = 0;
  float front_offset_ = 0;
  float rear_offset_ = 0;
  bool front_initialized_ = false;
  bool rear_initialized_ = false;
  static constexpr float LOWER_END = POSITION_LOWER_END;
  static constexpr float UPPER_END = POSITION_UPPER_END;
  static constexpr float POSITION_RANGE = UPPER_END - LOWER_END;
};

inline float PositionMeasurementFilter::computeNormalizedPosition( float position, float offset ) const
{
  // Normalize position to be within the range [LOWER_END, UPPER_END]
  float normalized_position = position - offset;
  if ( normalized_position < LOWER_END ) {
    normalized_position += POSITION_RANGE;
  } else if ( normalized_position > UPPER_END ) {
    normalized_position -= POSITION_RANGE;
  }
  return normalized_position;
}

inline void PositionMeasurementFilter::addMeasurements( const MotorStatus &front,
                                                        const MotorStatus &rear )
{
  if ( !front.valid && !rear.valid ) {
    return;
  }

  // Compute position base
  float new_position = 0;
  if ( front.valid && front_initialized_ ) {
    if ( front.position < LOWER_END || front.position > UPPER_END ) {
      Serial.printf( "New extreme front position: %f\n", front.position );
      return;
    }
    new_position = computeNormalizedPosition( front.position, front_offset_ );
  }
  if ( rear.valid && rear_initialized_ ) {
    if ( rear.position < LOWER_END || rear.position > UPPER_END ) {
      Serial.printf( "New extreme rear position: %f\n", rear.position );
      return;
    }
    new_position += computeNormalizedPosition( rear.position, rear_offset_ );
  }
  if ( front.valid && front_initialized_ && rear.valid && rear_initialized_ ) {
    new_position /= 2; // Average both positions
  }
  position_ = new_position;

  // Initialize positions if necessary
  if ( front.valid && !front_initialized_ ) {
    front_offset_ = front.position - position_;
    front_initialized_ = true;
  }
  if ( rear.valid && !rear_initialized_ ) {
    rear_offset_ = rear.position - position_;
    rear_initialized_ = true;
  }
}

inline float PositionMeasurementFilter::getFiltered() const { return position_; }
