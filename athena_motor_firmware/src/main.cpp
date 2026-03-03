#include <Arduino.h>
#include <elapsedMillis.h>
#include <usb_serial.h>

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "crosstalk_teensy_usb_serial_wrapper.hpp"
#include "motor_comm.h"
#include "motor_controller.h"
#include "status_led.h"

constexpr int MAIN_LOOP_DELAY_IN_US = 2000;
constexpr int MAX_TIME_SINCE_LAST_COMMAND_IN_MILLISECONDS = 200;

crosstalk::CrossTalker<16384, 512>
    host_comm( std::make_unique<crosstalk::TeensyUSBSerialWrapper>( Serial ) );
elapsedMillis time_since_last_command = 0;
MotorController motor_controller;
IntervalTimer motor_timer;

FullMotorStatus motor_status;

void reboot() { SCB_AIRCR = 0x05FA0004; }

void motorControlLoop();

void setup()
{
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, HIGH );
  delay( 20 );
  Serial.begin( BAUD_RATE );
  auto front_comm = std::make_shared<MotorComm>( &Serial1, 2 );
  auto rear_comm = std::make_shared<MotorComm>( &Serial2, 9 );
  motor_controller.init( front_comm, rear_comm );
  // motor_timer.priority( 0 );
  // motor_timer.begin( motorControlLoop, 1000 );
  motor_timer.begin( motorControlLoop, MAIN_LOOP_DELAY_IN_US );

  digitalWrite( LED_BUILTIN, LOW );
}

FullMotorStatus full_motor_status;

void motorControlLoop()
{
  if ( time_since_last_command >= MAX_TIME_SINCE_LAST_COMMAND_IN_MILLISECONDS ) {
    motor_controller.stop();
    status_led.speed = StatusLED::SLOW;
  }
  full_motor_status = motor_controller.update();
}

MotorError::Error last_error;
elapsedMicros loop_timer;
bool enable_debug = false;
MeanFilter<uint32_t, 10> average_loop_time_filter;

void loop()
{
  loop_timer = 0;
  host_comm.processSerialData();
  if ( host_comm.available() > 0 )
    host_comm.skip();
  while ( host_comm.hasObject() ) {
    switch ( host_comm.getObjectId() ) {
    case crosstalk::object_id<TeensyRebootCommand>(): {
      TeensyRebootCommand command;
      if ( host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
        break;
      }
      host_comm.sendObject( AckCommand{ CommandType::TEENSY_REBOOT } );
      delay( 10 );
      reboot();
      break;
    }
    case crosstalk::object_id<MotorCommand>(): {
      MotorCommand command;
      if ( host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
        break;
      }
      motor_controller.setCommand( command );
      time_since_last_command = 0;
      status_led.speed = StatusLED::FAST;
      host_comm.sendObject( AckCommand{ CommandType::MOTOR_COMMAND } );
      break;
    }
    case crosstalk::object_id<ChangePIDGainsCommand>(): {
      ChangePIDGainsCommand command;
      if ( host_comm.readObject( command ) != crosstalk::ReadResult::Success ) {
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
      motor_controller.setVelocityPIDGains( command.left_velocity_pid_gains,
                                            command.right_velocity_pid_gains );
      motor_controller.setPositionPIDGains( command.left_position_pid_gains,
                                            command.right_position_pid_gains );
      motor_controller.setVelocityFeedForwardGains(
          command.left_velocity_feed_forward_k_v, command.left_velocity_feed_forward_k_s,
          command.right_velocity_feed_forward_k_v, command.right_velocity_feed_forward_k_s );

      motor_controller.setPositionFeedForwardGains( 0.0f, 0.0f, 0.0f, 0.0f );
      time_since_last_command = 0;
      status_led.speed = StatusLED::FAST;
      host_comm.sendObject( AckCommand{ CommandType::CHANGE_PID_GAINS } );
      break;
    }
    case crosstalk::object_id<UpdateSettings>(): {
      UpdateSettings settings;
      if ( host_comm.readObject( settings ) != crosstalk::ReadResult::Success ) {
        break;
      }
      enable_debug = settings.enable_debug;
      motor_controller.setDisableAccelerationLimiting( settings.disable_acceleration_limiting );
      host_comm.sendObject( AckCommand{ CommandType::UPDATE_SETTINGS } );
      break;
    }
    default:
      // Do nothing
      host_comm.skipObject();
      break;
    }
    if ( host_comm.available() > 0 )
      host_comm.skip();
  }

  host_comm.sendObject( full_motor_status );

  if ( const auto error = motor_controller.getError();
       error != MotorError::Error::NO_ERROR && error != last_error ) {
    host_comm.sendObject( MotorError{ error } );
    last_error = error;
  }
  if ( enable_debug ) {
    auto debug_data = motor_controller.debugData();
    debug_data.average_loop_time_us = average_loop_time_filter.getMean();
    host_comm.sendObject( debug_data );
  }
  status_led.update();

  average_loop_time_filter.addValue( loop_timer );
  int delay_time = MAIN_LOOP_DELAY_IN_US - loop_timer;
  if ( delay_time <= 0 )
    return;
  delayMicroseconds( delay_time );
}
