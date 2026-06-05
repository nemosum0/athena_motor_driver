#include "pid_controller.h"
#include "config.h"
#include <algorithm>
#include <cmath>

PIDController::PIDController( float kp, float ki, float kd, float min_output, float max_output,
                              float max_output_change )
    : kp_( kp ), ki_( ki ), kd_( kd ), max_output_( max_output ), min_output_( min_output ),
      max_output_change_( max_output_change ), first_compute_( true )
{
}

void PIDController::setGains( float kp, float ki, float kd )
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  integral_ = 0; // Reset integral to avoid sudden jumps when changing gains
}

void PIDController::setOutputLimits( float min_output, float max_output )
{
  min_output_ = min_output;
  max_output_ = max_output;
}

void PIDController::setFeedForwardParams( float gain, float static_offset )
{
  feed_forward_gain_ = gain;
  feed_forward_offset_ = static_offset;
}

void PIDController::setDerivativeFilterCutoff( float cutoff_hz, float sample_hz )
{
  if ( cutoff_hz <= 0.0f || sample_hz <= 0.0f ) {
    derivative_filter_coeff_ = 0.0f; // No filtering — raw derivative
    return;
  }

  // Bilinear-transform (Tustin) discretisation of a first-order low-pass.
  // This maps the analogue pole at ωc = 2π·cutoff_hz into the z-domain
  // more accurately than the naive Euler approximation, especially when
  // cutoff_hz is a non-trivial fraction of sample_hz.
  //
  //   H(s) = ωc / (s + ωc)
  //
  //   α = (1 - ωc·T/2) / (1 + ωc·T/2)   where T = 1/sample_hz
  //
  // α = 0   → no filtering  (cutoff  >> sample rate)
  // α → 1   → heavy filtering (cutoff << sample rate)

  const float wc = 2.0f * M_PI * cutoff_hz;
  const float T = 1.0f / sample_hz;
  derivative_filter_coeff_ = ( 1.0f - wc * T * 0.5f ) / ( 1.0f + wc * T * 0.5f );

  // Clamp: negative alpha would invert the filter (cutoff > Nyquist).
  // In that case just bypass filtering entirely.
  if ( derivative_filter_coeff_ < 0.0f ) {
    derivative_filter_coeff_ = 0.0f;
  }
}

void PIDController::reset()
{
  last_input_ = 0;
  integral_ = 0;
  last_error_ = 0;
  first_compute_ = true;
  feed_forward_term_ = 0.0f;
  feed_forward_active_ = false;
  last_output_ = 0.0f;
  filtered_derivative_ = 0.0f;
}

float PIDController::computeTorque( float goal, float current, float dt )
{
  if ( first_compute_ ) {
    last_input_ = current;
    last_error_ = goal - current;
    first_compute_ = false;
  }

  if ( dt <= 0.0f ) {
    return last_output_;
  }

  const float error = goal - current;

  const float raw_derivative = -( current - last_input_ ) / dt;
  // Low-pass filter on derivative measurement.
  // derivative_filter_coeff_ = 0.0 means no filtering (raw derivative passes straight through)
  // derivative_filter_coeff_ approaches 1.0 as cutoff frequency approaches 0 (infinite filtering)
  filtered_derivative_ = derivative_filter_coeff_ * filtered_derivative_ +
                         ( 1.0f - derivative_filter_coeff_ ) * raw_derivative;
  float output = 0.0f;

  const float max_output_change = max_output_change_ * dt;
  const float upper_limit = std::min( max_output_, last_output_ + max_output_change );
  const float lower_limit = std::max( min_output_, last_output_ - max_output_change );

  // Feed-forward control logic
  bool is_stationary = std::abs( current ) < FEED_FORWARD_DEAD_ZONE;
  bool should_move = std::abs( goal ) > FEED_FORWARD_DEAD_ZONE && feed_forward_gain_ > 0.0f;
  bool is_moving = !is_stationary && ( current * goal > 0 );

  bool direction_reversed = ( goal * current < 0.0f ) && should_move;
  bool enter_feed_forward =
      should_move && ( is_stationary || direction_reversed ) && !feed_forward_active_;
  bool exit_feed_forward = feed_forward_active_ && ( !should_move || is_moving );

  if ( enter_feed_forward ) {
    feed_forward_term_ = last_output_ + std::copysign( feed_forward_offset_, goal );
    feed_forward_active_ = true;
  } else if ( exit_feed_forward ) {
    feed_forward_active_ = false;

    if ( ki_ != 0.0f ) {
      integral_ = ( last_output_ - ( kp_ * error + kd_ * filtered_derivative_ ) ) / ki_;
      integral_ = std::clamp( integral_, min_output_ / ki_, max_output_ / ki_ );
    }
  }

  if ( feed_forward_active_ ) {
    // Feed-forward control
    feed_forward_term_ += std::copysign( feed_forward_gain_, goal ) * dt;
    feed_forward_term_ = std::clamp( feed_forward_term_, lower_limit, upper_limit );

    output = feed_forward_term_;
    debug_data_.raw_output = output;
  } else {
    // PID control
    float p_term = kp_ * error;
    float d_term = kd_ * filtered_derivative_;

    // Conditional integration anti-windup
    float predicted_integral = integral_ + error * dt;
    float predicted_output = p_term + ki_ * predicted_integral + d_term;

    bool hitting_upper_limit = predicted_output > max_output_ && error > 0;
    bool hitting_lower_limit = predicted_output < min_output_ && error < 0;

    if ( !hitting_upper_limit && !hitting_lower_limit ) {
      integral_ = predicted_integral;
    }

    output = p_term + ki_ * integral_ + d_term;
    debug_data_.raw_output = output;
  }

  output = std::clamp( output, lower_limit, upper_limit );

  // Update states
  last_input_ = current;
  last_error_ = error;
  last_output_ = output;

  // Populate debug data
  debug_data_.goal = goal;
  debug_data_.current = current;
  debug_data_.dt = dt;
  debug_data_.error = error;
  debug_data_.derivative = filtered_derivative_;
  debug_data_.integral = integral_;
  debug_data_.output = output;

  return output;
}