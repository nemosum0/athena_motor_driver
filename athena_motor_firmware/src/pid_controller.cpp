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

void PIDController::setFeedForwardGains( float k_v, float k_s )
{
  feed_forward_k_v_ = k_v;
  feed_forward_k_s_ = k_s;
}

void PIDController::reset()
{
  last_input_ = 0;
  integral_ = 0;
  last_error_ = 0;
  first_compute_ = true;
}

float PIDController::computeTorque( float goal, float current, float dt )
{
  if ( first_compute_ ) {
    last_input_ = current;
    dt = 0;
    first_compute_ = false;
  }

  const float error = goal - current;
  integral_ += error * dt;
  const float derivative = dt <= 0 ? 0 : ( error - last_error_ ) / dt;

  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  debug_data_.raw_output = output;

  // Feed-forward control: Add physics-based estimate to reduce PID workload and stick-slip effects
  // Formula: feed_forward = (target_velocity * k_v) + (sign(target_velocity) * k_s)
  // k_v: Velocity gain - proportional to target velocity
  // k_s: Static friction gain - constant "push" to overcome static friction (only when velocity != 0)
  if ( std::abs( goal ) > FEED_FORWARD_DEAD_ZONE ) {
    float feed_forward = goal * feed_forward_k_v_;
    feed_forward += std::copysign( feed_forward_k_s_, goal );
    output += feed_forward;
  }

  const float max_output_change = max_output_change_ * dt;
  output = constrain( output, last_output_ - max_output_change, last_output_ + max_output_change );
  output = constrain( output, min_output_, max_output_ );

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
