#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include <elapsedMillis.h>

class LadrcController
{
public:
  static constexpr int OBSERVER_WARMUP_TICKS = 10;

  struct Config {
    double b0 = 1.0;
    double omega_c = 40.0;
    double omega_o = 150.0;
    double kp_pos = 1600.0;
    double kd_pos = 2.0;             // velocity damping gain multiplier for position hold
    double f_c = 0.5;                // Coulomb friction feedforward (Nm)
    double f_s = 1.0;                // static friction estimate (Nm)
    double breakaway_fraction = 0.8; // fraction of f_s applied during breakaway
    double max_torque = MOTOR_TORQUE_LIMIT;
    double slip_torque_threshold = 5.0; // Nm per tick
    double slip_vel_threshold = 1.0;    // rad/s
    int position_hold_ticks = LADRC_POSITION_HOLD_TICKS;
    int breakaway_ticks = LADRC_BREAKAWAY_TICKS;
    int slip_holdoff_ticks = LADRC_SLIP_HOLDOFF_TICKS;
    double velocity_dead_zone = VELOCITY_DEAD_ZONE;
  };

  LadrcController();
  LadrcController( const Config &config );

  void setConfig( const Config &config );

  const Config &getConfig() const { return config_; }

  void reset();

  /// Updates the internal ESO with new measurements. Must be called every tick even if in torque/brake mode.
  void updateObserver( double pos_meas, double torque_meas, double dt );

  /// Computes the required torque based on the target velocity and the internal ESO state.
  double computeControlLaw( double v_ref, double dt );

  /// Tell the observer what torque was actually applied (e.g. if saturated by driver, or when in raw torque mode)
  void setAppliedTorque( double torque )
  {
    u_prev_ = std::max( -config_.max_torque, std::min( torque, config_.max_torque ) );
  }

  const LadrcDebugData &debugData() const { return debug_data_; }

  double getX1Hat() const { return x1_hat_; }

  double getX2Hat() const { return x2_hat_; }

  double getX3Hat() const { return x3_hat_; }

  /// Shift all position-dependent state by offset. Used to re-center position
  /// near zero and prevent float precision loss during prolonged rotation.
  void recenterPosition( double offset )
  {
    x1_hat_ -= offset;
    last_pos_meas_ -= offset;
    p_hold_ -= offset;
  }

private:
  Config config_;

  // ESO states
  double x1_hat_ = 0.0;
  double x2_hat_ = 0.0;
  double x3_hat_ = 0.0;

  double u_prev_ = 0.0;

  // Control state
  bool is_position_hold_ = false;
  double p_hold_ = 0.0;
  int zero_v_ticks_ = 0;
  int breakaway_counter_ = 0;
  int slip_holdoff_counter_ = 0;
  double torque_meas_prev_ = 0.0; // torque measurement from two ticks ago (for derivative)
  double last_pos_meas_ = 0.0;
  double last_torque_meas_ = 0.0; // torque measurement from previous tick
  bool observer_initialized_ = false;
  int init_counter_ = 0;
  double last_output_ = 0.0;

  LadrcDebugData debug_data_;
};
