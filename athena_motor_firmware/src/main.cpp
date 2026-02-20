#include <Arduino.h>
#include <elapsedMillis.h>
#include <usb_serial.h>

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "config.h"
#include "crosstalk_teensy_usb_serial_wrapper.hpp"
#include "math/mean_filter.h"
#include "motor_comm.h"
#include "motor_controller.h"
#include "status_led.h"

namespace
{

struct AppState {
  StatusLED status_led{ LED_BUILTIN };
  crosstalk::CrossTalker<16384, 512> host_comm{
      std::make_unique<crosstalk::TeensyUSBSerialWrapper>( Serial ) };
  elapsedMillis time_since_last_command = 0;
  MotorController motor_controller;
  FullMotorStatus full_motor_status;
  MotorError::Error last_error = MotorError::Error::NO_ERROR;
  bool enable_debug = false;
  MeanFilter<uint32_t, LOOP_TIME_FILTER_SIZE> average_loop_time_filter;
} app;

IntervalTimer motor_timer;

} // anonymous namespace

void reboot() { SCB_AIRCR = 0x05FA0004; }

elapsedMicros loop_timer;

void motorControlLoop()
{
  loop_timer = 0;
  if ( app.time_since_last_command >= COMMAND_TIMEOUT_MS ) {
    app.motor_controller.stop();
    app.status_led.speed = StatusLED::SLOW;
  }
  app.full_motor_status = app.motor_controller.update();
  noInterrupts();
  app.average_loop_time_filter.addValue( loop_timer );
  interrupts();
}

void setup()
{
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, HIGH );
  delay( STARTUP_DELAY_MS );
  Serial.begin( BAUD_RATE );
  auto front_comm = std::make_shared<MotorComm>( &Serial1, 2 );
  auto rear_comm = std::make_shared<MotorComm>( &Serial2, 9 );
  app.motor_controller.init( front_comm, rear_comm );
  motor_timer.begin( motorControlLoop, MAIN_LOOP_PERIOD_US );
  digitalWrite( LED_BUILTIN, LOW );
}

void loop()
{
  app.host_comm.processSerialData();
  if ( app.host_comm.available() > 0 )
    app.host_comm.skip();
  while ( app.host_comm.hasObject() ) {
    switch ( app.host_comm.getObjectId() ) {
    case crosstalk::object_id<TeensyRebootCommand>(): {
      TeensyRebootCommand command;
      if ( app.host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
        break;
      }
      app.host_comm.sendObject( AckCommand{ CommandType::TEENSY_REBOOT } );
      delay( REBOOT_DELAY_MS );
      reboot();
      break;
    }
    case crosstalk::object_id<MotorCommand>(): {
      MotorCommand command;
      if ( app.host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
        break;
      }
      if ( command.mode == MotorCommand::MotorMode::VELOCITY &&
           ( !( std::abs( command.left ) <= MAX_PLAUSIBLE_VELOCITY_COMMAND ) ||
             !( std::abs( command.right ) <= MAX_PLAUSIBLE_VELOCITY_COMMAND ) ) ) {
        // If we receive a velocity command that is outside of the realm of plausibility, ignore it to prevent potential damage to the motors
        Serial.printf(
            "Received implausible velocity command: left=%.3f, right=%.3f. Setting to 0.\n",
            command.left, command.right );
        command.left = command.right = 0;
      } else if ( command.mode == MotorCommand::MotorMode::TORQUE &&
                  ( !( std::abs( command.left ) <= MAX_PLAUSIBLE_TORQUE_COMMAND ) ||
                    !( std::abs( command.right ) <= MAX_PLAUSIBLE_TORQUE_COMMAND ) ) ) {
        // If we receive a torque command that is outside of the realm of plausibility, ignore it to prevent potential damage to the motors
        Serial.printf(
            "Received implausible torque command: left=%.3f, right=%.3f. Setting to 0.\n",
            command.left, command.right );
        command.left = command.right = 0;
      }
      app.motor_controller.setCommand( command );
      app.time_since_last_command = 0;
      app.status_led.speed = StatusLED::FAST;
      app.host_comm.sendObject( AckCommand{ CommandType::MOTOR_COMMAND } );
      break;
    }
    case crosstalk::object_id<ChangePIDGainsCommand>(): {
      ChangePIDGainsCommand command;
      if ( app.host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
        break;
      }
      Serial.printf( "Received new velocity PID gains: left kP=%.3f, kI=%.3f, kD=%.3f; right "
                     "kP=%.3f, kI=%.3f, kD=%.3f\n",
                     command.left_velocity_pid_gains.k_p, command.left_velocity_pid_gains.k_i,
                     command.left_velocity_pid_gains.k_d, command.right_velocity_pid_gains.k_p,
                     command.right_velocity_pid_gains.k_i, command.right_velocity_pid_gains.k_d );
      Serial.printf( "Received new position PID gains: left kP=%.3f, kI=%.3f, kD=%.3f; right "
                     "kP=%.3f, kI=%.3f, kD=%.3f\n",
                     command.left_position_pid_gains.k_p, command.left_position_pid_gains.k_i,
                     command.left_position_pid_gains.k_d, command.right_position_pid_gains.k_p,
                     command.right_position_pid_gains.k_i, command.right_position_pid_gains.k_d );
      Serial.printf(
          "Received new velocity feed-forward gains: left kV=%.3f, kS=%.3f; right kV=%.3f, "
          "kS=%.3f\n",
          command.left_velocity_feed_forward_k_v, command.left_velocity_feed_forward_k_s,
          command.right_velocity_feed_forward_k_v, command.right_velocity_feed_forward_k_s );
      app.motor_controller.setVelocityPIDGains( command.left_velocity_pid_gains,
                                                command.right_velocity_pid_gains );
      app.motor_controller.setPositionPIDGains( command.left_position_pid_gains,
                                                command.right_position_pid_gains );
      app.motor_controller.setVelocityFeedForwardGains(
          command.left_velocity_feed_forward_k_v, command.left_velocity_feed_forward_k_s,
          command.right_velocity_feed_forward_k_v, command.right_velocity_feed_forward_k_s );
      app.motor_controller.setRotationalFeedForwardGains(
          command.left_velocity_feed_forward_k_s_rotational,
          command.right_velocity_feed_forward_k_s_rotational );

      app.motor_controller.setPositionFeedForwardGains( 0.0f, 0.0f, 0.0f, 0.0f );
      app.time_since_last_command = 0;
      app.status_led.speed = StatusLED::FAST;
      app.host_comm.sendObject( AckCommand{ CommandType::CHANGE_PID_GAINS } );
      break;
    }
    case crosstalk::object_id<UpdateSettings>(): {
      UpdateSettings settings;
      if ( app.host_comm.readObject( settings ) != crosstalk::ReadResult::Success ) {
        break;
      }
      app.enable_debug = settings.enable_debug;
      app.motor_controller.setDisableAccelerationLimiting( settings.disable_acceleration_limiting );
      app.host_comm.sendObject( AckCommand{ CommandType::UPDATE_SETTINGS } );
      break;
    }
    default:
      app.host_comm.skipObject();
      break;
    }
    if ( app.host_comm.available() > 0 )
      app.host_comm.skip();
  }

  app.host_comm.sendObject( app.full_motor_status );

  if ( const auto error = app.motor_controller.getError();
       error != MotorError::Error::NO_ERROR && error != app.last_error ) {
    app.host_comm.sendObject( MotorError{ error } );
    app.last_error = error;
  }
  if ( app.enable_debug ) {
    auto debug_data = app.motor_controller.debugData();
    noInterrupts();
    debug_data.average_loop_time_us = app.average_loop_time_filter.getMean();
    interrupts();
    app.host_comm.sendObject( debug_data );
  }
  app.status_led.update();

  delayMicroseconds( MAIN_LOOP_PERIOD_US );
}
