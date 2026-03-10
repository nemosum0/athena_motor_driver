#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"

#include <cmath>

class PositionMeasurementFilter
{
public:
  void addMeasurements( const MotorStatus &front, const MotorStatus &rear );

  void recenter( float offset );

  float getFiltered() const { return position_; }

  void reset()
  {
    position_ = 0;
    front_ = {};
    rear_ = {};
  }

private:
  struct MotorUnwrapState {
    float last_raw = 0.0f;
    float unwrapped = 0.0f;
    bool initialized = false;
    int glitch_count = 0;
  };

  float unwrapAndUpdate( MotorUnwrapState &state, float raw_position );

  float position_ = 0;
  MotorUnwrapState front_;
  MotorUnwrapState rear_;

  static constexpr float POSITION_RANGE = POSITION_UPPER_END - POSITION_LOWER_END;
  static constexpr float HALF_RANGE = POSITION_RANGE / 2.0f;
  static constexpr int MAX_CONSECUTIVE_GLITCHES = 5;
};

inline float PositionMeasurementFilter::unwrapAndUpdate( MotorUnwrapState &state, float raw_position )
{
  float delta = raw_position - state.last_raw;

  // Detect encoder wrap: a genuine wrap is ~353 rad, real motion is <0.35 rad/tick
  if ( delta > HALF_RANGE ) {
    delta -= POSITION_RANGE;
  } else if ( delta < -HALF_RANGE ) {
    delta += POSITION_RANGE;
  }

  // Reject implausible changes (sensor glitch or motor reconnect at new position)
  if ( std::abs( delta ) > MAX_PLAUSIBLE_POSITION_CHANGE ) {
    state.glitch_count++;
    if ( state.glitch_count >= MAX_CONSECUTIVE_GLITCHES ) {
      // Persistent mismatch — accept new raw position without updating unwrapped
      state.last_raw = raw_position;
      state.glitch_count = 0;
    }
    return state.unwrapped;
  }

  state.glitch_count = 0;
  state.last_raw = raw_position;
  state.unwrapped += delta;
  return state.unwrapped;
}

inline void PositionMeasurementFilter::addMeasurements( const MotorStatus &front,
                                                        const MotorStatus &rear )
{
  if ( !front.valid && !rear.valid ) {
    return;
  }

  float sum = 0;
  int count = 0;

  if ( front.valid && front_.initialized ) {
    sum += unwrapAndUpdate( front_, front.position );
    count++;
  }

  if ( rear.valid && rear_.initialized ) {
    sum += unwrapAndUpdate( rear_, rear.position );
    count++;
  }

  if ( count > 0 ) {
    position_ = sum / count;
  }

  // Late initialization: align new motor to current fused position
  if ( front.valid && !front_.initialized ) {
    front_.last_raw = front.position;
    front_.unwrapped = position_;
    front_.initialized = true;
  }
  if ( rear.valid && !rear_.initialized ) {
    rear_.last_raw = rear.position;
    rear_.unwrapped = position_;
    rear_.initialized = true;
  }
}

/// Subtract offset from all position state. Call together with matching
/// shifts in the observer (x1_hat, last_pos_meas, p_hold) to prevent
/// float32 precision loss during prolonged rotation.
inline void PositionMeasurementFilter::recenter( float offset )
{
  position_ -= offset;
  if ( front_.initialized )
    front_.unwrapped -= offset;
  if ( rear_.initialized )
    rear_.unwrapped -= offset;
}
