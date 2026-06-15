#include "motor_controller.h"
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>

#include <deque>
#include <random>

uint32_t simulated_micros = 0;
uint32_t simulated_millis = 0;

DummySerial Serial;

HardwareSerialIMXRT dummy_serial;

std::shared_ptr<MotorComm> front_comm;
std::shared_ptr<MotorComm> rear_comm;

struct MotorState {
  float position = 0;
  float velocity = 0;
  float applied_torque = 0;
};

struct StatusDelayItem {
  uint32_t timestamp_ms;
  MotorState fl, fr, rl, rr;
  float robot_velocity;
};

struct PhysicsSim {
  MotorState fl, fr, rl, rr;
  float robot_velocity = 0;

  // Non-idealities
  std::deque<StatusDelayItem> history;
  uint32_t transport_delay_ms = 2;

  std::mt19937 gen{ 42 };
  std::normal_distribution<float> noise_dist{ 0.0f, 0.1f }; // Higher noise

  // Physical constants
  float robot_mass = 45.0f;    // kg
  float wheel_radius = 0.075f; // m
  float track_inertia = 0.1f;  // 5x lower than before
  float dt = 0.0005f;          // 0.5ms

  // Motor constants (Back-EMF simulation)
  float motor_max_omega = 50.0f; // rad/s (roughly 500 RPM)

  // Friction parameters
  float mu_static = 0.9f;
  float mu_dynamic = 0.7f;
  float mu_stair_slip = 0.1f;
  float mu_stair_hold = 0.8f;

  float rolling_resistance = 145.0f;
  float damping_coeff = 220.0f;

  bool stairs_mode = false;
  bool slipping_on_stairs = false;

  float max_oscillation = 0;

  void step()
  {
    // Torque is reduced by back-EMF as speed increases
    auto apply_back_emf = [&]( float torque, float omega ) {
      float speed_factor = 1.0f - std::abs( omega ) / motor_max_omega;
      if ( speed_factor < 0 )
        speed_factor = 0;
      // Back EMF only restricts torque in direction of motion
      if ( torque * omega > 0 )
        return torque * speed_factor;
      return torque;
    };

    float f_left = apply_back_emf( fl.applied_torque, fl.velocity ) +
                   apply_back_emf( rl.applied_torque, rl.velocity );
    float f_right = apply_back_emf( fr.applied_torque, fr.velocity ) +
                    apply_back_emf( rr.applied_torque, rr.velocity );

    float v_track_l = fl.velocity * wheel_radius;
    float v_track_r = -fr.velocity * wheel_radius;

    auto calc_traction = [&]( float v_track, float forward_torque ) {
      float N_side = ( robot_mass * 9.81f ) / 2.0f;
      float max_static_force = mu_static * N_side;
      float dynamic_force = mu_dynamic * N_side;

      if ( stairs_mode ) {
        if ( std::abs( v_track - robot_velocity ) > 0.3f )
          slipping_on_stairs = true;
        else if ( std::abs( v_track - robot_velocity ) < 0.05f )
          slipping_on_stairs = false;
        float mu = slipping_on_stairs ? mu_stair_slip : mu_stair_hold;
        max_static_force = dynamic_force = mu * N_side;
      }

      float slip_vel = v_track - robot_velocity;
      float traction_force = slip_vel * ( dynamic_force / 0.05f );
      if ( std::abs( traction_force ) > max_static_force )
        traction_force = std::copysign( dynamic_force, traction_force );
      return traction_force;
    };

    float F_l = calc_traction( v_track_l, f_left );
    float F_r = calc_traction( v_track_r, -f_right );

    float F_res =
        std::copysign( rolling_resistance, robot_velocity ) + robot_velocity * damping_coeff;
    if ( std::abs( robot_velocity ) < 0.01f && std::abs( F_l + F_r ) < rolling_resistance )
      F_res = F_l + F_r;

    float robot_accel = ( F_l + F_r - F_res ) / robot_mass;
    robot_velocity += robot_accel * dt;

    fl.velocity += ( f_left - F_l * wheel_radius ) / track_inertia * dt;
    fr.velocity += ( f_right + F_r * wheel_radius ) / track_inertia * dt;
    rl.velocity = fl.velocity;
    rr.velocity = fr.velocity;

    fl.position += fl.velocity * dt;
    fr.position += fr.velocity * dt;
    rl.position += rl.velocity * dt;
    rr.position += rr.velocity * dt;

    if ( simulated_micros % 1000 == 0 ) {
      history.push_back( { simulated_millis, fl, fr, rl, rr, robot_velocity } );
      // Keep history for jitter/delay simulation (2-4ms)
      uint32_t jittery_delay = transport_delay_ms + ( simulated_millis % 3 );
      while ( history.size() > jittery_delay + 1 ) history.pop_front();
    }
  }

  StatusDelayItem getDelayedState()
  {
    if ( history.empty() )
      return { simulated_millis, fl, fr, rl, rr, robot_velocity };
    auto state = history.front(); // Use oldest
    state.fl.velocity += noise_dist( gen );
    state.fr.velocity += noise_dist( gen );
    auto quantize = []( float p ) {
      return std::round( p * 16384.0f / ( 2 * M_PI ) ) * ( 2 * M_PI ) / 16384.0f;
    };
    state.fl.position = quantize( state.fl.position );
    state.fr.position = quantize( state.fr.position );
    return state;
  }
};

PhysicsSim sim;

MotorComm::MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type )
    : serial_( serial ), direction_pin_( direction_pin ), type_( type )
{
}

MotorCommStatus MotorComm::readStatus() { return MotorCommStatus{}; }

void MotorComm::writeData( const uint8_t *data, size_t size ) { }

static MotorCommCommand pending_command_front;
static MotorCommCommand pending_command_rear;

void MotorComm::sendCommand( const MotorCommCommand &command )
{
  auto &pending = ( this == front_comm.get() ) ? pending_command_front : pending_command_rear;
  pending = command;

  MotorState *m = nullptr;
  if ( this == front_comm.get() )
    m = ( command.motor_id == 0 ) ? &sim.fl : &sim.fr;
  else
    m = ( command.motor_id == 0 ) ? &sim.rl : &sim.rr;

  m->applied_torque = command.mode == MotorMode::FOC ? command.torque : -m->velocity * 2.0f;
}

MotorCommStatus MotorComm::receiveStatus()
{
  auto delayed = sim.getDelayedState();
  const auto &cmd = ( this == front_comm.get() ) ? pending_command_front : pending_command_rear;

  MotorState *dm = nullptr;
  if ( this == front_comm.get() )
    dm = ( cmd.motor_id == 0 ) ? &delayed.fl : &delayed.fr;
  else
    dm = ( cmd.motor_id == 0 ) ? &delayed.rl : &delayed.rr;

  MotorCommStatus status;
  status.valid = true;
  status.motor_id = cmd.motor_id;
  status.mode = cmd.mode;
  status.position = dm->position;
  status.velocity_high = dm->velocity;
  status.torque = dm->applied_torque;
  return status;
}

void MotorComm::sendReceive( const MotorCommCommand &left_command,
                             const MotorCommCommand &right_command, MotorCommStatus &left_status,
                             MotorCommStatus &right_status )
{
  sendCommand( left_command );
  left_status = receiveStatus();
  sendCommand( right_command );
  right_status = receiveStatus();
}

class MotorControllerTest : public ::testing::Test
{
protected:
  MotorController controller;
  std::ofstream log_file;

  void SetUp() override
  {
    front_comm = std::make_shared<MotorComm>( &dummy_serial, 1 );
    rear_comm = std::make_shared<MotorComm>( &dummy_serial, 0 );

    controller.init( front_comm, rear_comm );
    sim = PhysicsSim(); // Reset sim
    simulated_micros = 0;
    simulated_millis = 0;

    // Load params from params.yaml
    PIDGains vel_gains{ 2.0, 6.0, 0.3 };
    PIDGains pos_gains{ 2.0, 6.0, 0.3 };
    controller.setVelocityPIDGains( vel_gains, vel_gains );
    controller.setPositionPIDGains( pos_gains, pos_gains );
    controller.setVelocityStartupParams( 0.8f, 0.0f, 0.8f, 0.0f );

    std::string test_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    log_file.open( test_name + ".csv" );
    log_file << "time_ms,left_cmd_torque,right_cmd_torque,fl_vel,fr_vel,robot_vel\n";
  }

  void TearDown() override { log_file.close(); }

  void step( int ms = 1 )
  {
    bool debug = ::testing::Test::HasFailure();
    for ( int i = 0; i < ms; i++ ) {
      for ( int p = 0; p < 2; p++ ) {
        sim.step();
        simulated_micros += 500;
      }
      simulated_millis += 1;

      if ( simulated_millis % 2 == 0 ) {
        float v_before = sim.fl.velocity;
        controller.update();
        float v_after = sim.fl.velocity;
        // Track rapid oscillations (velocity changing direction or jumping)
        if ( simulated_millis > 500 ) { // Let it settle first
          sim.max_oscillation = std::max( sim.max_oscillation, std::abs( v_after - v_before ) );
        }
      }

      if ( debug && ( simulated_millis % 500 == 0 ) ) {
        std::cout << "t=" << simulated_millis << " fl_v=" << sim.fl.velocity
                  << " robot_v=" << sim.robot_velocity << " torque=" << sim.fl.applied_torque
                  << "\n";
      }

      log_file << simulated_millis << "," << sim.fl.applied_torque << "," << sim.fr.applied_torque
               << "," << sim.fl.velocity << "," << sim.fr.velocity << "," << sim.robot_velocity
               << "\n";
    }
  }
};

TEST_F( MotorControllerTest, NormalGroundMovement )
{
  // Send a velocity command
  MotorCommand cmd;
  cmd.mode = MotorCommand::MotorMode::VELOCITY;
  cmd.left = 5.0f; // rad/s
  cmd.right = 5.0f;
  controller.setCommand( cmd );

  // Simulate 4 seconds to allow settling with high dampening
  step( 4000 );

  // Check if velocities converge
  EXPECT_NEAR( sim.fl.velocity, 5.0f, 0.6f );
  EXPECT_NEAR( sim.fr.velocity, -5.0f, 0.6f );

  // Stop
  controller.stop();
  step( 2000 );

  EXPECT_NEAR( sim.fl.velocity, 0.0f, 0.5f );
  EXPECT_NEAR( sim.fr.velocity, 0.0f, 0.5f );
}

TEST_F( MotorControllerTest, StairClimbingSlip )
{
  // Start moving
  MotorCommand cmd;
  cmd.mode = MotorCommand::MotorMode::VELOCITY;
  cmd.left = 5.0f;
  cmd.right = 5.0f;
  controller.setCommand( cmd );

  step( 1000 );

  // Hit a stair (friction drops drastically)
  sim.stairs_mode = true;

  step( 500 );

  // Expected behaviour: motor may accelerate briefly due to slip,
  // but the controller should back off torque so it doesn't spin wildly out of control.
  // The peak torque shouldn't explode.
  EXPECT_LT( std::abs( sim.fl.applied_torque ), 30.0f );

  sim.stairs_mode = false;
  step( 2000 );

  // Should recover to normal speed
  EXPECT_NEAR( sim.fl.velocity, 5.0f, 0.6f );
}

TEST_F( MotorControllerTest, SlowStairClimbing )
{
  // Start moving slowly (0.15 m/s)
  MotorCommand cmd;
  cmd.mode = MotorCommand::MotorMode::VELOCITY;
  cmd.left = 2.0f;
  cmd.right = 2.0f;
  controller.setCommand( cmd );

  step( 1000 );

  // Hit a stair
  sim.stairs_mode = true;

  // Simulate some time
  step( 3000 );

  // Should NOT be slipping wildly (at 2 rad/s, track speed is 0.15 m/s, below slip threshold of 0.3)
  EXPECT_FALSE( sim.slipping_on_stairs );
  EXPECT_NEAR( sim.fl.velocity, 2.0f, 0.5f );
  EXPECT_GT( sim.robot_velocity, 0.1f ); // Making progress
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
