#include "config.h"
#include "pid_controller.h"
#include <gtest/gtest.h>

class PIDControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pid = new PIDController( kp, ki, kd, -100.0f, 100.0f, 1000.0f );
    pid->setFeedForwardParams( feed_forward_gain );
  }

  void TearDown() override { delete pid; }

  PIDController *pid;
  float kp = 1.0f;
  float ki = 0.5f;
  float kd = 0.1f;
  float feed_forward_gain = 2.0f;
};

// Static feed-forward activates when stationary and a valid goal is set
TEST_F( PIDControllerTest, FeedForwardActivatesWhenStationaryAndCommanded )
{
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  // On first compute, dt logic acts differently if dt<=0, but it sets first_compute_=false
  // and correctly processes it.
  // enter_feed_forward happens. feed_forward_term = 0 + 0 = 0.
  // feed_forward_term += copysign(2.0, 1.0) * 0.01 = 0.02
  EXPECT_FLOAT_EQ( torque1, feed_forward_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, 2 * feed_forward_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, 3 * feed_forward_gain * dt );
}

// Static feed-forward with static offset activates correctly
TEST_F( PIDControllerTest, FeedForwardActivatesWithStaticOffset )
{
  float feed_forward_offset = 5.0f;
  pid->setFeedForwardParams( feed_forward_gain, feed_forward_offset );
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, feed_forward_offset + feed_forward_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, feed_forward_offset + 2 * feed_forward_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, feed_forward_offset + 3 * feed_forward_gain * dt );
}

// Static feed-forward does not activate if the goal is below the dead zone
TEST_F( PIDControllerTest, FeedForwardInactiveIfGoalBelowDeadzone )
{
  float goal = FEED_FORWARD_DEAD_ZONE * 0.5f;
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

// Static feed-forward does not activate if the motor is already moving
TEST_F( PIDControllerTest, FeedForwardInactiveIfAlreadyMoving )
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

// Static feed-forward accumulates correctly in the negative direction
TEST_F( PIDControllerTest, FeedForwardAccumulatesInNegativeDirection )
{
  float goal = -1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float torque1 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, -feed_forward_gain * dt );

  float torque2 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, -2 * feed_forward_gain * dt );

  float torque3 = pid->computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, -3 * feed_forward_gain * dt );
}

// Exits feed-forward and transitions smoothly to standard PID when motion starts
TEST_F( PIDControllerTest, ExitsFeedForwardAndTransitionsSeamlessly )
{
  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  float t1 = pid->computeTorque( goal, current, dt ); // 0.02
  float t2 = pid->computeTorque( goal, current, dt ); // 0.04
  float t3 = pid->computeTorque( goal, current, dt ); // 0.06

  // Now motor starts moving, exceeding dead zone
  current = 0.15f;
  float torque_pid = pid->computeTorque( goal, current, dt );

  // When exiting, the integral is set such that:
  // integral = (last_output_ - (kp * error + kd * filtered_derivative)) / ki
  // then it proceeds to update P, I, D.
  // With anti-windup, the exact output will be very close to the `t3 + ki * error * dt`
  // because it pre-compensates for the new P and D, leaving only the new integration step
  float expected_error = goal - current;
  float expected_output = t3 + ( ki * expected_error * dt );

  EXPECT_NEAR( torque_pid, expected_output, 0.0001f );
}

// Re-enters feed forward cleanly if the motor gets stuck again
TEST_F( PIDControllerTest, ReentersFeedForwardWhenStuck )
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

  // When re-entering feed-forward, it should resume exactly from last_output_
  // Expected: last_output_ (moving_torque) + k_s * dt * sign(goal)
  float expected_stuck_torque = moving_torque + ( feed_forward_gain * dt * 1.0f );
  EXPECT_FLOAT_EQ( stuck_torque, expected_stuck_torque );
}

// Ensure limits apply correctly during feed-forward
TEST_F( PIDControllerTest, FeedForwardRespectsMaxOutputChange )
{
  // Reconfigure with a very strict max output change limit
  PIDController strict_pid( 1.0f, 0.5f, 0.1f, -100.0f, 100.0f, 0.5f ); // 0.5 Nm/s limit
  strict_pid.setFeedForwardParams( 100.0f ); // Fast accumulation: 100 Nm/s

  float goal = 1.0f;
  float current = 0.0f;
  float dt = 0.01f;

  // Step 1: Accumulates 1.0Nm, but should be clamped by max_change of 0.5 * 0.01 = 0.005 Nm
  float torque1 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque1, 0.005f );

  // Step 2: Accumulates another 1.0Nm internally, but output clamped to last_output + max_change
  float torque2 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque2, 0.010f );

  float torque3 = strict_pid.computeTorque( goal, current, dt );
  EXPECT_FLOAT_EQ( torque3, 0.015f );
}
