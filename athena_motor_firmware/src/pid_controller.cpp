#include "pid_controller.h"
#include "config.h"

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
void PIDController::setFeedForwardGains( float k_s )
{
  feed_forward_k_s_ = k_s;
}

void PIDController::reset()
{
  last_input_ = 0;
  integral_ = 0;
  last_error_ = 0;
  first_compute_ = true;
  feed_forward_term_ = 0.0f;
  feed_forward_active_ = false;
}

float PIDController::computeTorque( float goal, float current, float dt )
{
  if ( first_compute_ ) {
    last_input_ = current;
    dt = 0;
    first_compute_ = false;
  }

  const float error = goal - current;
  const float derivative = dt <= 0 ? 0 : ( error - last_error_ ) / dt;
  float output = 0.0f;

  // Feed-forward control logic
  bool is_stationary = std::abs( current ) < FEED_FORWARD_DEAD_ZONE;
  bool should_move = std::abs( goal ) > FEED_FORWARD_DEAD_ZONE && feed_forward_k_s_ > 0.0f;
  bool is_moving = !is_stationary && ( current * goal > 0 );

  bool enter_feed_forward = should_move && is_stationary && !feed_forward_active_;
  bool exit_feed_forward = feed_forward_active_ && ( !should_move || is_moving );

  if ( enter_feed_forward ) {
    feed_forward_term_ = last_output_;
    feed_forward_active_ = true;
  } else if ( exit_feed_forward ) {
    feed_forward_active_ = false;
    if ( ki_ != 0.0f ) {
      integral_ = ( last_output_ - ( kp_ * error + kd_ * derivative ) ) / ki_;
    }
  }

  const float max_output_change = max_output_change_ * dt;
  const float upper_limit = (max_output_ < last_output_ + max_output_change) ? max_output_ : (last_output_ + max_output_change);
  const float lower_limit = (min_output_ > last_output_ - max_output_change) ? min_output_ : (last_output_ - max_output_change);

  if ( feed_forward_active_ ) {
    // Feed-forward control
    feed_forward_term_ += std::copysign( feed_forward_k_s_, goal ) * dt;
    output = feed_forward_term_;
    debug_data_.raw_output = output;
  } else {
    // PID control
    float p_term = kp_ * error;
    float d_term = kd_ * derivative;
    
    // Conditional integration anti-windup
    float predicted_integral = integral_ + error * dt;
    float predicted_output = p_term + ki_ * predicted_integral + d_term;
    
    bool hitting_upper_limit = predicted_output > upper_limit && error > 0;
    bool hitting_lower_limit = predicted_output < lower_limit && error < 0;
    
    if ( !hitting_upper_limit && !hitting_lower_limit ) {
      integral_ = predicted_integral;
    }
    
    output = p_term + ki_ * integral_ + d_term;
    debug_data_.raw_output = output;
  }

  output = constrain( output, lower_limit, upper_limit );

  last_input_ = current;
  last_error_ = error;
  last_output_ = output;

  debug_data_.goal = goal;
  debug_data_.current = current;
  debug_data_.dt = dt;
  debug_data_.error = error;
  debug_data_.derivative = derivative;
  debug_data_.integral = integral_;
  debug_data_.output = output;

  return output;
}
