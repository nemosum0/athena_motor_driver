#include "config.h"
#include "pid_controller.h"
#include <gtest/gtest.h>

class PIDControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pid = new PIDController( kp, ki, kd, -100.0f, 100.0f, 1000.0f );
    pid->setStartupParams( startup_gain );
  }

  void TearDown() override { delete pid; }

  PIDController *pid;
  float kp = 1.0f;
  float ki = 0.5f;
  float kd = 0.1f;
  float startup_gain = 2.0f;
};

// Startup ramp activates when stationary and a valid goal is set
TEST_F( PIDControllerTest, StartupActivatesWhenStationaryAndCommanded )
{
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  // enter_startup happens. startup_term_ = last_output_ + offset(0) = 0.
  // startup_term_ += copysign(2.0, 1.0) * 0.01 = 0.02
  EXPECT_FLOAT_EQ( torque1, startup_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, 2 * startup_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, 3 * startup_gain * dt );
}

// Startup with a stiction-breaking offset applies the offset as an instantaneous step
// that bypasses the slew-rate limit, then ramps on top of it.
TEST_F( PIDControllerTest, StartupActivatesWithOffset )
{
  float startup_offset = 5.0f;
  pid->setStartupParams( startup_gain, startup_offset );
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, startup_offset + startup_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, startup_offset + 2 * startup_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, startup_offset + 3 * startup_gain * dt );
}

// Startup ramp does not activate if the goal is below the motion threshold
TEST_F( PIDControllerTest, StartupInactiveIfGoalBelowThreshold )
{
  float goal = MOTION_THRESHOLD * 0.5f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  float expected_p1 = kp * goal;
  float expected_i1 = ki * ( goal * dt );
  EXPECT_FLOAT_EQ( torque1, expected_p1 + expected_i1 );

  float torque2 = pid->computeTorque( goal, current, dt );
  float expected_i2 = ki * ( goal * dt * 2 );
  EXPECT_FLOAT_EQ( torque2, expected_p1 + expected_i2 );
}

// Startup ramp does not activate if the motor is already moving
TEST_F( PIDControllerTest, StartupInactiveIfAlreadyMoving )
{
  float goal = 2.0f;
  float current = 1.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  float error = goal - current;
  float expected_p1 = kp * error;
  float expected_i1 = ki * error * dt;
  EXPECT_FLOAT_EQ( torque1, expected_p1 + expected_i1 );

  float torque2 = pid->computeTorque( goal, current, dt );
  float expected_i2 = ki * ( error * dt * 2 );
  EXPECT_FLOAT_EQ( torque2, expected_p1 + expected_i2 );
}

// Startup ramp accumulates correctly in the negative direction
TEST_F( PIDControllerTest, StartupAccumulatesInNegativeDirection )
{
  float goal = -1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, -startup_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, -2 * startup_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, -3 * startup_gain * dt );
}

// Exits startup and transitions smoothly to standard PID when motion starts
TEST_F( PIDControllerTest, ExitsStartupAndTransitionsSeamlessly )
{
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float t1 = pid->computeTorque( goal, current, dt ); // 0.02
  float t2 = pid->computeTorque( goal, current, dt ); // 0.04
  float t3 = pid->computeTorque( goal, current, dt ); // 0.06

  // Now motor starts moving, exceeding the motion threshold
  current = 0.15f;
  float torque_pid = pid->computeTorque( goal, current, dt );

  // When exiting, the integral is set such that:
  // integral = (last_output_ - (kp * error + kd * filtered_derivative)) / ki
  // then it proceeds to update P, I, D.
  // With anti-windup, the exact output will be very close to `t3 + ki * error * dt`
  // because it pre-compensates for the new P and D, leaving only the new integration step.
  float expected_error = goal - current;
  float expected_output = t3 + ( ki * expected_error * dt );

  EXPECT_NEAR( torque_pid, expected_output, 0.0001f );
}

// Re-enters the startup ramp cleanly if the motor gets stuck again
TEST_F( PIDControllerTest, ReentersStartupWhenStuck )
{
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  pid->computeTorque( goal, current, dt ); // 0.02
  pid->computeTorque( goal, current, dt ); // 0.04

  // Motor starts moving
  current = 0.2f;
  float moving_torque = pid->computeTorque( goal, current, dt );

  // Motor gets stuck again
  current = 0.0f;
  float stuck_torque = pid->computeTorque( goal, current, dt );

  // When re-entering startup, it should resume exactly from last_output_
  // Expected: last_output_ (moving_torque) + startup_gain * dt * sign(goal)
  float expected_stuck_torque = moving_torque + ( startup_gain * dt * 1.0f );
  EXPECT_FLOAT_EQ( stuck_torque, expected_stuck_torque );
}

// The continuous startup ramp respects the slew-rate (max output change) limit
TEST_F( PIDControllerTest, StartupRampRespectsMaxOutputChange )
{
  // Reconfigure with a very strict max output change limit
  PIDController strict_pid( 1.0f, 0.5f, 0.1f, -100.0f, 100.0f, 0.5f ); // 0.5 Nm/s limit
  strict_pid.setStartupParams( 100.0f ); // Fast ramp: 100 Nm/s, no offset

  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  // Step 1: ramp wants 1.0 Nm, but is clamped by max_change of 0.5 * 0.01 = 0.005 Nm
  float torque1 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, 0.005f );

  // Step 2: ramps another 1.0 Nm internally, but output is clamped to last_output + max_change
  float torque2 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, 0.010f );

  float torque3 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, 0.015f );
}

// Feed-forward term adds kff * goal directly to the PID output
TEST_F( PIDControllerTest, FeedForwardTermIsAddedToOutput )
{
  float kff = 0.3f;
  pid->setGains( kp, ki, kd, kff );

  float goal = 2.0f;
  float current = 1.0f; // above MOTION_THRESHOLD, so startup ramp does not engage
  float dt = 0.01f;

  float torque = pid->computeTorque( goal, current, dt );
  float error = goal - current;
  float expected = kp * error + ki * error * dt + kff * goal;
  EXPECT_FLOAT_EQ( torque, expected );
}

// The stiction-breaking offset is applied as a step even under a strict slew-rate limit
TEST_F( PIDControllerTest, StartupOffsetBypassesMaxOutputChange )
{
  // Strict slew limit of 0.5 Nm/s would, without special handling, throttle the offset
  // down to a single 0.005 Nm increment. The offset must instead step through immediately.
  PIDController strict_pid( 1.0f, 0.5f, 0.1f, -100.0f, 100.0f, 0.5f );
  strict_pid.setStartupParams( 100.0f, 5.0f ); // gain 100 Nm/s, offset 5 Nm

  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  // The 5 Nm offset is applied instantaneously; the ramp adds gain*dt on top, but is
  // slew-limited to max_change (0.005) around the post-offset baseline.
  float torque1 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, 5.0f + 0.005f );
}
