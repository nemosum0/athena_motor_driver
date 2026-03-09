#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include <elapsedMillis.h>

class LadrcController
{
public:
  struct Config {
    double b0 = 1.0;
    double omega_c = 40.0;
    double omega_o = 150.0;
    double kp_pos = 1600.0;
    double f_c = 0.5;
    double f_s = 1.0;
    double max_torque = MOTOR_TORQUE_LIMIT;
    double max_torque_change = MAX_TORQUE_CHANGE;
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
  void updateObserver( double pos_meas, double torque_meas );

  /// Computes the required torque based on the target velocity and the internal ESO state.
  float computeControlLaw( double v_ref );

  /// Tell the observer what torque was actually applied (e.g. if saturated by driver, or when in raw torque mode)
  void setAppliedTorque( double torque )
  {
    u_prev_ = std::max( -config_.max_torque, std::min( torque, config_.max_torque ) );
  }

  const LadrcDebugData &debugData() const { return debug_data_; }

  double getX1Hat() const { return x1_hat_; }

  double getX2Hat() const { return x2_hat_; }

  double getX3Hat() const { return x3_hat_; }

private:
  Config config_;
  elapsedMicros elapsed_;

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
  double prev_torque_meas_ = 0.0;
  double last_pos_meas_ = 0.0;
  double last_torque_meas_ = 0.0;
  bool first_compute_ = true;
  double last_output_ = 0.0;
  double last_dt_ = 0.0;

  LadrcDebugData debug_data_;
};
