#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "math/mean_filter.h"
#include <cmath>
#include <elapsedMillis.h>

class VelocityMeasurementFilter
{
  static constexpr int WINDOW_SIZE = 50; // Number of measurements to consider for filtering
  static constexpr float MAX_PLAUSIBLE_POSITION_CHANGE = 0.35f; // In rad, corresponds to 20 degrees
public:
  void addMeasurements( const MotorStatus &front, const MotorStatus &rear );

  float getFiltered() const;

  void reset() { avg_sum = 0; }

private:
  struct Measurement {
    elapsedMicros timestamp;
    float position;
  };

  Measurement last_front_measurement_;
  Measurement last_rear_measurement_;
  float avg_sum = 0.0f;
};

inline void VelocityMeasurementFilter::addMeasurements( const MotorStatus &front,
                                                        const MotorStatus &rear )
{
  if ( !front.valid && !rear.valid ) {
    return;
  }

  bool front_updated = false;
  bool rear_updated = false;
  float velocity = 0;
  // Compute velocity based on position
  if ( front.valid ) {
    const float d_pos = front.position - last_front_measurement_.position;
    if ( last_front_measurement_.timestamp < 6000 &&
         std::abs( d_pos ) < MAX_PLAUSIBLE_POSITION_CHANGE ) {
      front_updated = true;
      velocity += d_pos / ( last_front_measurement_.timestamp / 1E6f );
    }
    last_front_measurement_ = { 0, front.position };
  }
  if ( rear.valid ) {
    const float d_pos = rear.position - last_rear_measurement_.position;
    if ( last_rear_measurement_.timestamp < 6000 &&
         std::abs( d_pos ) < MAX_PLAUSIBLE_POSITION_CHANGE ) {
      rear_updated = true;
      velocity += d_pos / ( last_rear_measurement_.timestamp / 1E6f );
    }
    last_rear_measurement_ = { 0, rear.position };
  }

  if ( !front_updated && !rear_updated )
    return;
  if ( front_updated && rear_updated )
    velocity /= 2;

  avg_sum -= avg_sum / WINDOW_SIZE;
  avg_sum += velocity;
}

inline float VelocityMeasurementFilter::getFiltered() const { return avg_sum / WINDOW_SIZE; }
