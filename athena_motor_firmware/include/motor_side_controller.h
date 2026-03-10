#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include "math/mean_filter.h"
#include "pid_controller.h"
#include "position_measurement_filter.hpp"
#include "velocity_measurement_filter.hpp"
#include <elapsedMillis.h>

struct MotorCommStatus;

//! Manages the status, filtering, and PID control for one side (left or right) of the robot.
//! Each side has a front and rear motor whose measurements are fused for position and velocity.
class MotorSideController
{
public:
  MotorSideController();

  /// Update raw motor status from front comm result
  void updateFrontStatus( const MotorCommStatus &status, uint8_t expected_motor_id );

  /// Update raw motor status from rear comm result
  void updateRearStatus( const MotorCommStatus &status, uint8_t expected_motor_id );

  /// Feed current front/rear status to position and velocity filters
  void addMeasurements();

  /// Check if at least one motor (front or rear) is responding within timeout
  bool isWorking( int timeout_ms ) const;

  /// Reset position filter (called while position is not yet initialized)
  void resetPositionFilter();

  /// Capture current filtered position as the hold position
  void initializePosition();

  /// Compute torque output for the given target velocity using position/velocity PID
  float computeTorque( float target_velocity, float dt );

  /// Reset all PID controllers (e.g., on communication loss)
  void resetPIDControllers();

  // --- Gain setters ---
  void setPositionPIDGains( float kp, float ki, float kd );
  void setVelocityPIDGains( float kp, float ki, float kd );
  void setVelocityFeedForwardGains( float k_v, float k_s );
  void setPositionFeedForwardGains( float k_v, float k_s );

  // --- Accessors ---
  float filteredVelocity() const { return velocity_filter_.getFiltered(); }

  const MotorStatus &frontStatus() const { return front_status_; }

  const MotorStatus &rearStatus() const { return rear_status_; }

  unsigned long frontAgeMs() const { return front_age_; }

  unsigned long rearAgeMs() const { return rear_age_; }

  // --- Debug ---
  const PIDDebugData &velocityPIDDebugData() const { return velocity_pid_.debugData(); }

  const PIDDebugData &positionPIDDebugData() const { return position_pid_.debugData(); }

  float validFrontFreq( long age_ms ) const;
  float validRearFreq( long age_ms ) const;

private:
  void updateStatus( const MotorCommStatus &status, uint8_t expected_motor_id,
                     MotorStatus &out_status, MeanFilter<uint8_t, VALID_FILTER_SIZE> &valid_filter,
                     elapsedMillis &age );

  enum class ControlMode { POSITION, VELOCITY };

  MotorStatus front_status_;
  MotorStatus rear_status_;
  elapsedMillis front_age_;
  elapsedMillis rear_age_;
  MeanFilter<uint8_t, VALID_FILTER_SIZE> front_valid_;
  MeanFilter<uint8_t, VALID_FILTER_SIZE> rear_valid_;

  PositionMeasurementFilter position_filter_;
  VelocityMeasurementFilter velocity_filter_;
  PIDController velocity_pid_;
  PIDController position_pid_;

  ControlMode control_mode_ = ControlMode::POSITION;
  float hold_position_ = 0;
};
