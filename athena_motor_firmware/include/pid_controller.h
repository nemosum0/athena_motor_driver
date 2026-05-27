#ifndef ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H
#define ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H

#include "athena_motor_interface/athena_motor_interfaces.h"
#include <elapsedMillis.h>

class PIDController
{
public:
  /*
   * @param max_output Maximum torque output (Nm). This should be set to the maximum torque you want to allow the controller to command.
   * @param max_output_change Maximum change in output (Nm/s). This is used to limit acceleration and prevent
   *          sudden jumps in torque which can cause mechanical stress and instability.
   */
  PIDController( float kp, float ki, float kd, float min_output, float max_output,
                 float max_output_change );
  void setGains( float kp, float ki, float kd );
  void setOutputLimits( float min_output, float max_output );
  void setFeedForwardParams( float gain, float offset = 0.0f );
  void reset();

  //! Compute the torque required to reach the goal velocity
  float computeTorque( float goal, float current, float dt );

  const PIDDebugData &debugData() const { return debug_data_; }

private:
  PIDDebugData debug_data_;
  float kp_;
  float ki_;
  float kd_;
  float max_output_;
  float min_output_;
  float max_output_change_;
  float last_input_ = 0;
  float last_output_ = 0;
  float integral_ = 0;
  float filtered_derivative_ = 0;
  float last_error_ = 0;
  bool first_compute_;
  float feed_forward_gain_ = 0;
  float feed_forward_offset_ = 0;
  float derivative_filter_coeff_ = 0.8f;

  float feed_forward_term_ = 0;
  bool feed_forward_active_ = false;
};

#endif // ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H