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
  torque_meas_prev_ = 0.0;
  last_pos_meas_ = 0.0;
  last_torque_meas_ = 0.0;
  observer_initialized_ = false;
  init_counter_ = 0;
  last_output_ = 0.0;
  debug_data_ = LadrcDebugData();
}

void LadrcController::updateObserver( double pos_meas, double torque_meas, double dt )
{
  if ( !observer_initialized_ ) {
    x1_hat_ = pos_meas;
    torque_meas_prev_ = torque_meas;
    observer_initialized_ = true;
    init_counter_ = 0;
    return;
  }

  // 1. Extended State Observer (ESO) Update - Tustin (Bilinear) discretization
  //    of 3rd-order ESO with bandwidth-parameterized gains (Gao, 2003)
  const double beta1 = 3.0 * config_.omega_o;
  const double beta2 = 3.0 * config_.omega_o * config_.omega_o;
  const double beta3 = config_.omega_o * config_.omega_o * config_.omega_o;

  // Warmup phase: Only track position, keep velocity/disturbance at zero
  if ( init_counter_ < OBSERVER_WARMUP_TICKS ) {
    x1_hat_ = pos_meas;
    x2_hat_ = 0.0;
    x3_hat_ = 0.0;
    init_counter_++;
  } else {
    const double T = dt / 2.0;
    const double T2 = T * T;
    const double T3 = T2 * T;

    const double K = T * beta1 + T2 * beta2 + T3 * beta3;

    const double y_k = last_pos_meas_;
    const double y_next = pos_meas;

    const double e_k = y_k - x1_hat_;

    const double x1_next = ( ( 1.0 - K ) * x1_hat_ + K * ( y_k + y_next ) + 2.0 * T * x2_hat_ +
                             2.0 * T2 * x3_hat_ + 2.0 * T2 * config_.b0 * u_prev_ ) /
                           ( 1.0 + K );

    const double e_next = y_next - x1_next;
    const double sum_e = e_k + e_next;

    const double x2_next = x2_hat_ + 2.0 * T * x3_hat_ + 2.0 * T * config_.b0 * u_prev_ +
                           ( T * beta2 + T2 * beta3 ) * sum_e;

    const double x3_next = x3_hat_ + T * beta3 * sum_e;

    x1_hat_ = x1_next;
    x2_hat_ = x2_next;
    x3_hat_ = x3_next;
  }

  last_pos_meas_ = pos_meas;
  last_torque_meas_ = torque_meas;

  // Populate debug data with observer states
  debug_data_.pos_meas = pos_meas;
  debug_data_.x1_hat = x1_hat_;
  debug_data_.x2_hat = x2_hat_;
  debug_data_.x3_hat = x3_hat_;
  debug_data_.dt = dt;
}

double LadrcController::computeControlLaw( double v_ref, double dt )
{
  if ( !observer_initialized_ || init_counter_ < OBSERVER_WARMUP_TICKS ) {
    return 0.0;
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
    const double kp_vel = config_.kd_pos * config_.omega_c;
    tau_raw = ( kp_vel * ( 0.0 - x2_hat_ ) + config_.kp_pos * ( p_hold_ - x1_hat_ ) - x3_hat_ ) /
              config_.b0;
  } else {
    tau_raw = ( config_.omega_c * ( v_ref - x2_hat_ ) - x3_hat_ ) / config_.b0;
  }

  // 3. Feedforward Augmentation
  double tau_ff = 0.0;
  const double direction = v_ref > 0 ? 1.0 : -1.0;
  if ( std::abs( v_ref ) > config_.velocity_dead_zone ) {
    tau_ff += config_.f_c * direction;
  }

  if ( breakaway_counter_ > 0 ) {
    tau_ff += config_.breakaway_fraction * config_.f_s * direction;
    breakaway_counter_--;
  }

  const double tau_aug = tau_raw + tau_ff;

  // 4. Slip Detection and Response
  const double delta_tau = last_torque_meas_ - torque_meas_prev_;
  const double vel_error = std::abs( x2_hat_ - v_ref );
  torque_meas_prev_ = last_torque_meas_;

  if ( delta_tau < -config_.slip_torque_threshold && vel_error > config_.slip_vel_threshold ) {
    slip_holdoff_counter_ = config_.slip_holdoff_ticks;
  }

  double tau_safe = tau_aug;
  if ( slip_holdoff_counter_ > 0 ) {
    tau_safe = tau_safe > 0 ? std::min( tau_safe, config_.f_c ) : std::max( tau_safe, -config_.f_c );
    slip_holdoff_counter_--;
  }

  // 5. Output Conditioning — saturation only; rate limiting is handled by MotorController
  const double output = constrain( tau_safe, -config_.max_torque, config_.max_torque );

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

  return output;
}
