#include "ladrc_controller.h"
#include <algorithm>
#include <cmath>

LadrcController::LadrcController() : config_() { reset(); }

LadrcController::LadrcController( const Config &config ) : config_( config ) { reset(); }

void LadrcController::setConfig( const Config &config ) { config_ = config; }

void LadrcController::reset()
{
  x1_hat_ = 0.0;
  x2_hat_ = 0.0;
  x3_hat_ = 0.0;
  u_prev_ = 0.0;
  is_position_hold_ = false;
  p_hold_ = 0.0;
  zero_v_ticks_ = 0;
  breakaway_counter_ = 0;
  slip_holdoff_counter_ = 0;
  prev_torque_meas_ = 0.0;
  last_pos_meas_ = 0.0;
  last_torque_meas_ = 0.0;
  first_compute_ = true;
  last_output_ = 0.0;
  debug_data_ = LadrcDebugData();
}

void LadrcController::updateObserver( double pos_meas, double torque_meas, double dt )
{
  if ( first_compute_ ) {
    x1_hat_ = pos_meas;
    prev_torque_meas_ = torque_meas;
    first_compute_ = false;
    return;
  }

  // 1. Extended State Observer (ESO) Update - Forward Euler
  double beta1 = 3.0 * config_.omega_o;
  double beta2 = 3.0 * config_.omega_o * config_.omega_o;
  double beta3 = config_.omega_o * config_.omega_o * config_.omega_o;

  double e_obs = pos_meas - x1_hat_;
  x1_hat_ += dt * ( x2_hat_ + beta1 * e_obs );
  x2_hat_ += dt * ( x3_hat_ + config_.b0 * u_prev_ + beta2 * e_obs );
  x3_hat_ += dt * ( beta3 * e_obs );

  last_pos_meas_ = pos_meas;
  last_torque_meas_ = torque_meas;

  // Populate debug data with observer states
  debug_data_.x1_hat = x1_hat_;
  debug_data_.x2_hat = x2_hat_;
  debug_data_.x3_hat = x3_hat_;
  debug_data_.dt = dt;
}

float LadrcController::computeControlLaw( double v_ref, double dt )
{
  if ( first_compute_ ) {
    return 0.0f; // Observer must be initialized first
  }

  // 2. Mode select & Control Law
  double tau_raw = 0.0;
  if ( std::abs( v_ref ) <= config_.velocity_dead_zone ) {
    zero_v_ticks_++;
    if ( zero_v_ticks_ >= config_.position_hold_ticks && !is_position_hold_ ) {
      is_position_hold_ = true;
      p_hold_ = x1_hat_;
      breakaway_counter_ = 0;
    }
  } else {
    zero_v_ticks_ = 0;
    if ( is_position_hold_ ) {
      is_position_hold_ = false;
      breakaway_counter_ = config_.breakaway_ticks;
    }
  }

  if ( is_position_hold_ ) {
    double kp_vel = 2.0 * config_.omega_c;
    tau_raw = ( kp_vel * ( 0.0 - x2_hat_ ) + config_.kp_pos * ( p_hold_ - x1_hat_ ) - x3_hat_ ) /
              config_.b0;
  } else {
    double kp = config_.omega_c;
    tau_raw = ( kp * ( v_ref - x2_hat_ ) - x3_hat_ ) / config_.b0;
  }

  // 3. Feedforward Augmentation
  double tau_ff = 0.0;
  if ( std::abs( v_ref ) > config_.velocity_dead_zone ) {
    tau_ff += config_.f_c * ( v_ref > 0 ? 1.0 : -1.0 );
  }

  if ( breakaway_counter_ > 0 ) {
    tau_ff += 0.8 * config_.f_s * ( v_ref > 0 ? 1.0 : -1.0 );
    breakaway_counter_--;
  }

  double tau_aug = tau_raw + tau_ff;

  // 4. Slip Detection and Response
  double delta_tau = last_torque_meas_ - prev_torque_meas_;
  double vel_error = std::abs( x2_hat_ - v_ref );
  prev_torque_meas_ = last_torque_meas_;

  if ( delta_tau < -config_.slip_torque_threshold && vel_error > config_.slip_vel_threshold ) {
    slip_holdoff_counter_ = config_.slip_holdoff_ticks;
  }

  double tau_safe = tau_aug;
  if ( slip_holdoff_counter_ > 0 ) {
    tau_safe = tau_safe > 0 ? std::min( tau_safe, config_.f_c ) : std::max( tau_safe, -config_.f_c );
    slip_holdoff_counter_--;
  }

  // 5. Output Conditioning
  double max_delta_tau = config_.max_torque_change * dt;
  double output = tau_safe;

  // Apply rate limiter, optionally bypassing during breakaway
  if ( breakaway_counter_ == 0 ) {
    output = constrain( output, last_output_ - max_delta_tau, last_output_ + max_delta_tau );
  }

  // Dead-band compensation (if any) could go here.

  // Saturation
  output = constrain( output, -config_.max_torque, config_.max_torque );

  u_prev_ = output;
  last_output_ = output;

  // Populate debug data
  debug_data_.v_ref = v_ref;
  debug_data_.p_hold = p_hold_;
  debug_data_.is_position_hold = is_position_hold_;
  debug_data_.tau_raw = tau_raw;
  debug_data_.tau_ff = tau_ff;
  debug_data_.tau_aug = tau_aug;
  debug_data_.tau_safe = tau_safe;
  debug_data_.output = output;
  debug_data_.slip_holdoff_counter = slip_holdoff_counter_;

  return static_cast<float>( output );
}
