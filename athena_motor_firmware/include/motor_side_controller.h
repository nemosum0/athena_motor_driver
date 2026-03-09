#pragma once

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include "ladrc_controller.h"
#include "math/mean_filter.h"
#include "position_measurement_filter.hpp"
#include <elapsedMillis.h>

struct MotorCommStatus;

//! Manages the status, filtering, and control for one side (left or right) of the robot.
//! Each side has a front and rear motor whose measurements are fused for position.
class MotorSideController
{
public:
  MotorSideController();

  /// Update raw motor status from front comm result
  void updateFrontStatus( const MotorCommStatus &status, uint8_t expected_motor_id );

  /// Update raw motor status from rear comm result
  void updateRearStatus( const MotorCommStatus &status, uint8_t expected_motor_id );

  /// Feed current front/rear status to position filters
  void addMeasurements();

  /// Check if at least one motor (front or rear) is responding within timeout
  bool isWorking( int timeout_ms ) const;

  /// Reset position filter (called while position is not yet initialized)
  void resetPositionFilter();

  /// Capture current filtered position as the hold position
  void initializePosition();

  /// Updates the ESO. Must be called every tick.
  void updateObserver();

  /// Compute torque output for the given target velocity
  float computeTorque( float target_velocity );

  /// Record actual torque applied to motors
  void setAppliedTorque( float torque );

  /// Reset controllers (e.g., on communication loss)
  void resetControllers();

  // --- Configuration setters ---
  void setLadrcConfig( const LadrcController::Config &config );

  LadrcController::Config getLadrcConfig() const { return ladrc_.getConfig(); }

  // --- Accessors ---
  float filteredVelocity() const { return ladrc_.getX2Hat(); }

  const MotorStatus &frontStatus() const { return front_status_; }

  const MotorStatus &rearStatus() const { return rear_status_; }

  unsigned long frontAgeMs() const { return front_age_; }

  unsigned long rearAgeMs() const { return rear_age_; }

  // --- Debug ---
  const LadrcDebugData &ladrcDebugData() const { return ladrc_.debugData(); }

  float validFrontFreq( long age_ms ) const;
  float validRearFreq( long age_ms ) const;

private:
  void updateStatus( const MotorCommStatus &status, uint8_t expected_motor_id,
                     MotorStatus &out_status, MeanFilter<uint8_t, VALID_FILTER_SIZE> &valid_filter,
                     elapsedMillis &age );

  MotorStatus front_status_;
  MotorStatus rear_status_;
  elapsedMillis front_age_;
  elapsedMillis rear_age_;
  MeanFilter<uint8_t, VALID_FILTER_SIZE> front_valid_;
  MeanFilter<uint8_t, VALID_FILTER_SIZE> rear_valid_;

  PositionMeasurementFilter position_filter_;
  LadrcController ladrc_;
};
