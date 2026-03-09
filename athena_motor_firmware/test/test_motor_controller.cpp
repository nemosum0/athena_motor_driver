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
  float robot_yaw_rate;
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
  float track_inertia = 0.1f;
  float track_separation = 0.55f; // m
  float robot_inertia_yaw = 5.0f; // kg*m^2
  float dt = 0.0005f;             // 0.5ms

  float robot_yaw_rate = 0; // rad/s
  float f_traction_l = 0;
  float f_traction_r = 0;

  // Motor constants (Back-EMF simulation)
  float motor_max_omega = 50.0f; // rad/s (roughly 500 RPM)

  // Friction parameters
  float mu_static = 0.9f;
  float mu_dynamic = 0.7f;
  float mu_stair_hold_static = 0.9f;
  float mu_stair_hold_dynamic = 0.8f;
  float mu_stair_slip_static = 0.3f;
  float mu_stair_slip_dynamic = 0.1f;

  float rolling_resistance = 145.0f;
  float damping_coeff = 220.0f;

  bool stairs_mode = false;
  bool slipping_on_stairs = false;

  float max_oscillation = 0;

  void step()
  {
    float v_track_l = fl.velocity * wheel_radius;
    float v_track_r = -fr.velocity * wheel_radius;

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

    // Ground velocities at track centers
    float v_ground_l = robot_velocity - robot_yaw_rate * ( track_separation / 2.0f );
    float v_ground_r = robot_velocity + robot_yaw_rate * ( track_separation / 2.0f );

    auto calc_traction = [&]( float v_track, float v_ground ) {
      float N_side = ( robot_mass * 9.81f ) / 2.0f;
      float mu_s = mu_static;
      float mu_d = mu_dynamic;

      if ( stairs_mode ) {
        if ( std::abs( v_track - v_ground ) > 0.3f )
          slipping_on_stairs = true;
        else if ( std::abs( v_track - v_ground ) < 0.05f )
          slipping_on_stairs = false;

        if ( slipping_on_stairs ) {
          mu_s = mu_stair_slip_static;
          mu_d = mu_stair_slip_dynamic;
        } else {
          mu_s = mu_stair_hold_static;
          mu_d = mu_stair_hold_dynamic;
        }
      }

      float max_static_force = mu_s * N_side;
      float dynamic_force = mu_d * N_side;

      float slip_vel = v_track - v_ground;
      float traction_force = slip_vel * ( dynamic_force / 0.02f ); // Stiffer contact
      if ( std::abs( traction_force ) > max_static_force )
        traction_force = std::copysign( dynamic_force, traction_force );
      return traction_force;
    };

    f_traction_l = calc_traction( v_track_l, v_ground_l );
    f_traction_r = calc_traction( v_track_r, v_ground_r );

    // Resistances
    float F_res =
        std::copysign( rolling_resistance, robot_velocity ) + robot_velocity * damping_coeff;
    if ( std::abs( robot_velocity ) < 0.01f &&
         std::abs( f_traction_l + f_traction_r ) < rolling_resistance )
      F_res = f_traction_l + f_traction_r;

    // Skid steering resistance (rotational friction is higher)
    float T_res = std::copysign( rolling_resistance * 2.0f, robot_yaw_rate ) +
                  robot_yaw_rate * damping_coeff * 2.0f;
    if ( std::abs( robot_yaw_rate ) < 0.01f &&
         std::abs( ( f_traction_r - f_traction_l ) * ( track_separation / 2.0f ) ) <
             rolling_resistance * 2.0f )
      T_res = ( f_traction_r - f_traction_l ) * ( track_separation / 2.0f );

    // Integrations
    float robot_accel = ( f_traction_l + f_traction_r - F_res ) / robot_mass;
    robot_velocity += robot_accel * dt;

    float yaw_accel = ( ( f_traction_r - f_traction_l ) * ( track_separation / 2.0f ) - T_res ) /
                      robot_inertia_yaw;
    robot_yaw_rate += yaw_accel * dt;

    fl.velocity += ( f_left - f_traction_l * wheel_radius ) / track_inertia * dt;
    fr.velocity += ( f_right + f_traction_r * wheel_radius ) / track_inertia * dt;
    rl.velocity = fl.velocity;
    rr.velocity = fr.velocity;

    fl.position += fl.velocity * dt;
    fr.position += fr.velocity * dt;
    rl.position += rl.velocity * dt;
    rr.position += rr.velocity * dt;

    if ( simulated_micros % 1000 == 0 ) {
      history.push_back( { simulated_millis, fl, fr, rl, rr, robot_velocity, robot_yaw_rate } );
      uint32_t jittery_delay = transport_delay_ms + ( simulated_millis % 3 );
      while ( history.size() > jittery_delay + 1 ) history.pop_front();
    }
  }

  StatusDelayItem getDelayedState()
  {
    if ( history.empty() )
      return { simulated_millis, fl, fr, rl, rr, robot_velocity, robot_yaw_rate };
    auto state = history.front(); // Use oldest
    state.fl.velocity += noise_dist( gen );
    state.fr.velocity += noise_dist( gen );
    state.rl.velocity += noise_dist( gen );
    state.rr.velocity += noise_dist( gen );

    // Position quantization
    auto quantize = []( float p ) {
      return std::round( p * 16384.0f / ( 2 * M_PI ) ) * ( 2 * M_PI ) / 16384.0f;
    };
    state.fl.position = quantize( state.fl.position );
    state.fr.position = quantize( state.fr.position );

    // Torque feedback simulation
    state.fl.applied_torque += noise_dist( gen ) * 0.1f;
    state.fr.applied_torque += noise_dist( gen ) * 0.1f;

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

void MotorComm::sendReceive( const MotorCommCommand &left_command,
                             const MotorCommCommand &right_command, MotorCommStatus &left_status,
                             MotorCommStatus &right_status )
{

  MotorState *ml = nullptr;
  MotorState *mr = nullptr;

  if ( this == front_comm.get() ) {
    ml = &sim.fl;
    mr = &sim.fr;
  } else {
    ml = &sim.rl;
    mr = &sim.rr;
  }

  // Commands apply immediately (with jitter/latency it would be better to delay this too,
  // but motor drivers usually have low TX latency).
  ml->applied_torque =
      left_command.mode == MotorMode::FOC ? left_command.torque : -ml->velocity * 2.0f;
  mr->applied_torque =
      right_command.mode == MotorMode::FOC ? right_command.torque : -mr->velocity * 2.0f;

  // Status is delayed
  auto delayed = sim.getDelayedState();
  MotorState *dml = ( this == front_comm.get() ) ? &delayed.fl : &delayed.rl;
  MotorState *dmr = ( this == front_comm.get() ) ? &delayed.fr : &delayed.rr;

  left_status.valid = true;
  left_status.motor_id = left_command.motor_id;
  left_status.mode = left_command.mode;
  left_status.position = dml->position;
  left_status.velocity_high = dml->velocity;
  left_status.torque = dml->applied_torque;

  right_status.valid = true;
  right_status.motor_id = right_command.motor_id;
  right_status.mode = right_command.mode;
  right_status.position = dmr->position;
  right_status.velocity_high = dmr->velocity;
  right_status.torque = dmr->applied_torque;
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
    LadrcGains gains;
    gains.omega_c = 40.0f;
    gains.omega_o = 120.0f; // Lowered from 150 for better stability at 500Hz
    gains.b0 = 10.0f;       // 1/I = 1/0.1 = 10.0
    gains.f_c = 0.5f;
    gains.f_s = 1.0f;
    controller.setLadrcGains( gains, gains );

    std::string test_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    log_file.open( test_name + ".csv" );
    log_file << "time_ms,left_cmd_torque,right_cmd_torque,fl_vel,fr_vel,robot_vel,yaw_rate,f_"
                "traction_l,f_traction_r\n";
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

      log_file << simulated_millis << "," << sim.fl.applied_torque << "," << -sim.fr.applied_torque
               << "," << sim.fl.velocity << "," << -sim.fr.velocity << "," << sim.robot_velocity
               << "," << sim.robot_yaw_rate << "," << sim.f_traction_l << "," << sim.f_traction_r
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
  // Start moving fast
  MotorCommand cmd;
  cmd.mode = MotorCommand::MotorMode::VELOCITY;
  cmd.left = 10.0f;
  cmd.right = 10.0f;
  controller.setCommand( cmd );

  step( 1000 );

  // Hit a stair (friction drops drastically)
  sim.stairs_mode = true;

  step( 1500 );

  // Expected behaviour: slip detection should trigger and clamp torque
  EXPECT_TRUE( sim.slipping_on_stairs );
  EXPECT_LT( std::abs( sim.fl.applied_torque ), 10.0f ); // Clamped near f_c

  sim.stairs_mode = false;
  step( 2000 );

  // Should recover to normal speed
  EXPECT_NEAR( sim.fl.velocity, 10.0f, 1.2f );
}

TEST_F( MotorControllerTest, SpinningInPlace )
{
  // Spin right (Left forward, Right backward)
  MotorCommand cmd;
  cmd.mode = MotorCommand::MotorMode::VELOCITY;
  cmd.left = 5.0f;
  cmd.right = -5.0f;
  controller.setCommand( cmd );

  step( 2000 );

  // Should have high yaw rate but low longitudinal velocity
  EXPECT_GT( std::abs( sim.robot_yaw_rate ), 1.0f );
  EXPECT_NEAR( sim.robot_velocity, 0.0f, 0.2f );
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
