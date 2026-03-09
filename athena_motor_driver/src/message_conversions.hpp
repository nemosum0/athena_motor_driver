//
// Created by stefan on 16.07.25.
//

#ifndef ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP
#define ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP

#include <athena_motor_interface/athena_motor_interfaces.h>
#include <athena_motor_interface/msg/debug_data.hpp>
#include <athena_motor_interface/msg/ladrc_debug_data.hpp>

inline athena_motor_interface::msg::LadrcDebugData toMsg( const LadrcDebugData &ladrc_debug )
{
  athena_motor_interface::msg::LadrcDebugData msg;
  msg.v_ref = ladrc_debug.v_ref;
  msg.p_hold = ladrc_debug.p_hold;
  msg.is_position_hold = ladrc_debug.is_position_hold;
  msg.x1_hat = ladrc_debug.x1_hat;
  msg.x2_hat = ladrc_debug.x2_hat;
  msg.x3_hat = ladrc_debug.x3_hat;
  msg.tau_raw = ladrc_debug.tau_raw;
  msg.tau_ff = ladrc_debug.tau_ff;
  msg.tau_aug = ladrc_debug.tau_aug;
  msg.tau_safe = ladrc_debug.tau_safe;
  msg.output = ladrc_debug.output;
  msg.slip_holdoff_counter = ladrc_debug.slip_holdoff_counter;
  msg.dt = ladrc_debug.dt;
  return msg;
}

inline athena_motor_interface::msg::DebugData toMsg( const MotorDebugData &debug_data )
{
  athena_motor_interface::msg::DebugData msg;
  msg.left_ladrc = toMsg( debug_data.left_ladrc );
  msg.right_ladrc = toMsg( debug_data.right_ladrc );
  msg.status.freq_front_left = debug_data.status.freq_front_left;
  msg.status.freq_front_right = debug_data.status.freq_front_right;
  msg.status.freq_rear_left = debug_data.status.freq_rear_left;
  msg.status.freq_rear_right = debug_data.status.freq_rear_right;
  msg.average_loop_time_us = debug_data.average_loop_time_us;
  msg.error_code = static_cast<uint8_t>( debug_data.error );
  return msg;
}

#endif // ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP
