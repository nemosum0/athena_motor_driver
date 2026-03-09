#include "motor_controller.h"
#include "config.h"
#include "motor_comm.h"
#include "throttle_printer.hpp"

#include <algorithm>

MotorController::MotorController() { }

MotorController::~MotorController() = default;

void MotorController::init( std::shared_ptr<MotorComm> front_comm,
                            std::shared_ptr<MotorComm> rear_comm )
{
  front_motor_comm_ = front_comm;
  rear_motor_comm_ = rear_comm;
}

void MotorController::setCommand( const MotorCommand &command ) { command_ = command; }

void MotorController::setLadrcGains( const LadrcGains &left_gains, const LadrcGains &right_gains )
{
  auto l_cfg = left_.getLadrcConfig();
  l_cfg.b0 = left_gains.b0;
  l_cfg.omega_c = left_gains.omega_c;
  l_cfg.omega_o = left_gains.omega_o;
  l_cfg.kp_pos = left_gains.kp_pos;
  l_cfg.f_c = left_gains.f_c;
  l_cfg.f_s = left_gains.f_s;
  l_cfg.slip_torque_threshold = left_gains.slip_torque_threshold;
  l_cfg.slip_vel_threshold = left_gains.slip_vel_threshold;
  left_.setLadrcConfig( l_cfg );

  auto r_cfg = right_.getLadrcConfig();
  r_cfg.b0 = right_gains.b0;
  r_cfg.omega_c = right_gains.omega_c;
  r_cfg.omega_o = right_gains.omega_o;
  r_cfg.kp_pos = right_gains.kp_pos;
  r_cfg.f_c = right_gains.f_c;
  r_cfg.f_s = right_gains.f_s;
  r_cfg.slip_torque_threshold = right_gains.slip_torque_threshold;
  r_cfg.slip_vel_threshold = right_gains.slip_vel_threshold;
  right_.setLadrcConfig( r_cfg );
}

void MotorController::stop()
{
  command_.mode = MotorCommand::MotorMode::BRAKE;
  command_.left = 0;
  command_.right = 0;
  target_velocity_.left = 0;
  target_velocity_.right = 0;
  left_.resetControllers();
  right_.resetControllers();
}

namespace
{
bool isAccelerating( float target_velocity, float current_velocity )
{
  return std::signbit( target_velocity ) == std::signbit( current_velocity ) &&
         std::abs( target_velocity ) > std::abs( current_velocity );
}

float limitVelocityChange( float target_velocity, float current_velocity, float max_velocity_change )
{
  if ( std::abs( target_velocity - current_velocity ) <= max_velocity_change ) {
    return target_velocity;
  }
  return current_velocity + std::copysign( max_velocity_change, target_velocity - current_velocity );
}

float limitTorqueChange( float target_torque, float current_torque, float max_torque_change )
{
  if ( std::abs( target_torque - current_torque ) <= max_torque_change ) {
    return target_torque;
  }
  return current_torque + std::copysign( max_torque_change, target_torque - current_torque );
}
} // namespace

MotorController::Torque MotorController::computeTorque()
{
  if ( !initialized_position_ ) {
    return { 0, 0 }; // Do not issue any torque commands until position is initialized
  }
  // Cap elapsed time to 30 ms to avoid large jumps after long delays
  long elapsed_micros = std::min<long>( time_since_last_command_, 30'000 );
  if ( command_.mode == MotorCommand::MotorMode::TORQUE ) {
    float max_torque_change = MAX_TORQUE_CHANGE * elapsed_micros / 1E6f;
    float torque_left = limitTorqueChange( command_.left, torque_.left, max_torque_change );
    float torque_right = limitTorqueChange( -command_.right, torque_.right, max_torque_change );
    torque_.left = torque_left;
    torque_.right = torque_right;
    return { torque_left, torque_right };
  } else if ( command_.mode == MotorCommand::MotorMode::BRAKE ) {
    return { 0, 0 };
  }
  target_velocity_.left = command_.left;
  target_velocity_.right = -command_.right;
  // Limit acceleration
  // Really simple ramp up and faster ramp down for breaking
  float acceleration = MAX_DECELERATION;
  if ( isAccelerating( target_velocity_.left, velocity_.left ) ||
       isAccelerating( target_velocity_.right, velocity_.right ) ) {
    acceleration = MAX_ACCELERATION;
  }

  const float max_velocity_change = acceleration * elapsed_micros / 1E6f;

  velocity_.left = limitVelocityChange( target_velocity_.left, velocity_.left, max_velocity_change );
  velocity_.right =
      limitVelocityChange( target_velocity_.right, velocity_.right, max_velocity_change );

  if ( disable_acceleration_limiting_ ) {
    // If acceleration limits are disabled, we just set the target velocity directly
    // This is useful for tuning the PID controller but should not be used in normal operation
    velocity_.left = target_velocity_.left;
    velocity_.right = target_velocity_.right;
  }

  float left_torque = left_.computeTorque( velocity_.left );
  float right_torque = right_.computeTorque( velocity_.right );

  if ( !std::isfinite( left_torque ) || !std::isfinite( right_torque ) ) {
    static ThrottlePrinter printer( 1000 );
    printer.print( "Invalid torque values detected. This is a bug!" );
    left_torque = 0;
    right_torque = 0;
  }

  // Detect rotation: wheels moving in opposite directions (or one moving, one still)
  const bool is_rotating = ( velocity_.left * velocity_.right < 0 ) ||
                           ( std::abs( velocity_.left ) > VELOCITY_DEAD_ZONE &&
                             std::abs( velocity_.right ) < VELOCITY_DEAD_ZONE ) ||
                           ( std::abs( velocity_.right ) > VELOCITY_DEAD_ZONE &&
                             std::abs( velocity_.left ) < VELOCITY_DEAD_ZONE );

  if ( is_rotating ) {
    if ( std::abs( velocity_.left ) > VELOCITY_DEAD_ZONE )
      left_torque += std::copysign( rotational_feed_forward_k_s_left_, velocity_.left );
    if ( std::abs( velocity_.right ) > VELOCITY_DEAD_ZONE )
      right_torque += std::copysign( rotational_feed_forward_k_s_right_, velocity_.right );
  }

  const float max_torque_change = MAX_TORQUE_CHANGE * elapsed_micros / 1E6f;
  left_torque = limitTorqueChange( left_torque, torque_.left, max_torque_change );
  right_torque = limitTorqueChange( right_torque, torque_.right, max_torque_change );
  torque_.left = left_torque;
  torque_.right = right_torque;

  return { left_torque, right_torque };
}

static void setCommandFromTorque( MotorCommCommand &command, float torque )
{
  if ( std::abs( torque ) > MotorController::MIN_TORQUE ) {
    command.mode = MotorMode::FOC;
    command.torque = constrain( torque, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT );
  } else {
    command.mode = MotorMode::BRAKE;
    command.torque = 0;
  }
}

void MotorController::computeMotorCommands( MotorCommCommand &left_command,
                                            MotorCommCommand &right_command )
{
  left_command.motor_id = 0;
  right_command.motor_id = 1;
  const bool left_working = left_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  const bool right_working = right_.isWorking( MOTOR_STATUS_TIMEOUT_MS );
  if ( !left_working || !right_working ) {
    // At least one motor on each side needs to be working, otherwise we stop
    velocity_.left = 0;
    velocity_.right = 0;
    left_command.mode = MotorMode::BRAKE;
    right_command.mode = MotorMode::BRAKE;
    left_command.torque = 0;
    right_command.torque = 0;
    // Reset all PID controllers so it will not try to jump back to a position when power is restored
    left_.resetControllers();
    right_.resetControllers();
    left_.setAppliedTorque( 0 );
    right_.setAppliedTorque( 0 );
    initialized_position_ = false;
    debug_data_.error = MotorDebugData::Error::NO_MOTOR_STATUS;
  } else {
    Torque torque = computeTorque();
    setCommandFromTorque( left_command, initialized_position_ ? torque.left : 0 );
    setCommandFromTorque( right_command, initialized_position_ ? torque.right : 0 );
    left_.setAppliedTorque( left_command.mode == MotorMode::FOC ? left_command.torque : 0 );
    right_.setAppliedTorque( right_command.mode == MotorMode::FOC ? right_command.torque : 0 );
  }
  time_since_last_command_ = 0;
}

void MotorController::sendReceiveBus( std::shared_ptr<MotorComm> &comm, int &reset_skip_count,
                                      const MotorCommCommand &left_command,
                                      const MotorCommCommand &right_command, bool bus_working,
                                      bool is_front )
{
  MotorCommStatus left_status;
  MotorCommStatus right_status;
  if ( bus_working || ++reset_skip_count > MAX_RESET_SKIP_COUNT ) {
    // When communication fails, skip commands for a few cycles so if motor comm is
    // misaligned it has time to recover
    reset_skip_count = 0;
    comm->sendReceive( left_command, right_command, left_status, right_status );
  } else {
    comm->resetComm();
  }

  if ( is_front ) {
    left_.updateFrontStatus( left_status, 0 );
    right_.updateFrontStatus( right_status, 1 );
  } else {
    left_.updateRearStatus( left_status, 0 );
    right_.updateRearStatus( right_status, 1 );
  }
}

void MotorController::tryInitializePosition()
{
  if ( initialized_position_ )
    return;

  left_.resetPositionFilter();
  right_.resetPositionFilter();
  // If at least one motor on each bus is valid, we can initialize the position
  const bool front_has_valid = left_.frontStatus().valid || right_.frontStatus().valid;
  const bool rear_has_valid = left_.rearStatus().valid || right_.rearStatus().valid;
  if ( front_has_valid && rear_has_valid ) {
    initialized_position_ = true;
    left_.initializePosition();
    right_.initializePosition();
  }
}

void MotorController::assembleMotorStatus( const MotorCommCommand &left_command,
                                           const MotorCommCommand &right_command )
{
  motor_status_.front_left = left_.frontStatus();
  motor_status_.front_right = right_.frontStatus();
  motor_status_.rear_left = left_.rearStatus();
  motor_status_.rear_right = right_.rearStatus();
  motor_status_.velocity_left = left_.filteredVelocity();
  motor_status_.velocity_right = right_.filteredVelocity();

  const float left_target_torque = left_command.mode == MotorMode::FOC ? left_command.torque : 0;
  motor_status_.front_left.target_torque = left_target_torque;
  motor_status_.rear_left.target_torque = left_target_torque;
  motor_status_.front_left.target_velocity = velocity_.left;
  motor_status_.rear_left.target_velocity = velocity_.left;

  const float right_target_torque = right_command.mode == MotorMode::FOC ? right_command.torque : 0;
  motor_status_.front_right.target_torque = right_target_torque;
  motor_status_.rear_right.target_torque = right_target_torque;
  motor_status_.front_right.target_velocity = velocity_.right;
  motor_status_.rear_right.target_velocity = velocity_.right;

  motor_status_.front_left.age_ms = left_.frontAgeMs();
  motor_status_.front_right.age_ms = right_.frontAgeMs();
  motor_status_.rear_left.age_ms = left_.rearAgeMs();
  motor_status_.rear_right.age_ms = right_.rearAgeMs();
}

void MotorController::collectDebugData()
{
  status_ages_.push( elapsedMillis() );
  long status_age_ms = std::max<long>( 1, status_ages_.front() ); // Avoid 0ms from first call.
  debug_data_.status.freq_front_left = left_.validFrontFreq( status_age_ms );
  debug_data_.status.freq_front_right = right_.validFrontFreq( status_age_ms );
  debug_data_.status.freq_rear_left = left_.validRearFreq( status_age_ms );
  debug_data_.status.freq_rear_right = right_.validRearFreq( status_age_ms );
  debug_data_.left_ladrc = left_.ladrcDebugData();
  debug_data_.right_ladrc = right_.ladrcDebugData();
}

const FullMotorStatus &MotorController::update()
{
  debug_data_.error = MotorDebugData::Error::NO_ERROR;

  const bool front_working =
      left_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.frontAgeMs() < MOTOR_STATUS_TIMEOUT_MS;
  const bool rear_working =
      left_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS || right_.rearAgeMs() < MOTOR_STATUS_TIMEOUT_MS;

  // 1. Actuate and Sense (Send previous block's command, get newest feedback)
  sendReceiveBus( front_motor_comm_, reset_skip_count_front_, left_command_, right_command_,
                  front_working, true );
  sendReceiveBus( rear_motor_comm_, reset_skip_count_rear_, left_command_, right_command_,
                  rear_working, false );

  // 2. Filter and Update state
  // Using newest measurements and the torque that just finished physical application
  tryInitializePosition();
  left_.addMeasurements();
  right_.addMeasurements();

  if ( initialized_position_ ) {
    left_.updateObserver();
    right_.updateObserver();
  }

  // 3. Compute control laws for the NEXT tick
  computeMotorCommands( left_command_, right_command_ );

  // 4. Telemetry
  assembleMotorStatus( left_command_, right_command_ );

  collectDebugData();

  return motor_status_;
}
