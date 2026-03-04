#pragma once

#include "motor_msg_A1B1.h"
#include <HardwareSerial.h>
#include <vector>

enum class MotorMode : uint8_t { INVALID, BRAKE, FOC, CALIBRATE };

struct MotorCommCommand {
  uint8_t motor_id = 0;
  MotorMode mode = MotorMode::BRAKE;

  float torque = 0;
  float velocity = 0;
  float position = 0;

  //! Joint stiffness coefficient
  //! When transmitting, this is multiplied by 2048 and transmitted as integer
  float k_p = 0;
  //! Joint velocity coefficient
  //! When transmitting, this is multiplied by 2048 and transmitted as integer
  float k_w = 0;
};

struct MotorCommStatus {
  bool valid = false;
  uint8_t motor_id = 0;
  MotorMode mode;
  int temperature;
  int error_code;

  float torque;
  float velocity_high;
  float velocity_low;
  float position;
  float acceleration;
};

// Only tested for A1
enum class MotorType { A1Motor, B1Motor, GO_M8010_6_Motor };

class MotorComm
{
public:
  MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type = MotorType::A1Motor );

  void sendReceive( const MotorCommCommand &command, MotorCommStatus &status );
  void sendReceive( const MotorCommCommand &left_command, const MotorCommCommand &right_command,
                    MotorCommStatus &left_status, MotorCommStatus &right_status );

  void resetComm()
  {
    serial_->clear();
    serial_->clearReadError();
    serial_->clearWriteError();
  }

private:
  MotorCommStatus readStatus();
  void writeData( const uint8_t *data, size_t size );

  HardwareSerialIMXRT *serial_;
  int direction_pin_;
  MotorType type_;

  uint8_t send_buffer_[4 * sizeof( MasterComdDataV3 )];
  uint8_t receive_buffer_[256];
  uint8_t status_buffer_[78];
  float torque_scale_ = 0.f;
  float velocity_scale_ = 0.f;
  float position_scale_ = 0.f;
  float kp_scale_ = 0.f;
  float kw_scale_ = 0.f;
};