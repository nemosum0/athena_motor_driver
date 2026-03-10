#include "motor_comm.h"
#include "crc32.h"

#include <Arduino.h>

// Protocol constants for Unitree motor communication
static constexpr uint8_t PACKET_START_BYTE_0 = 0xFE;
static constexpr uint8_t PACKET_START_BYTE_1 = 0xEE;
static constexpr float TORQUE_SCALE = 256.0f;
static constexpr float VELOCITY_SCALE = 128.0f;
static constexpr float ENCODER_TICKS_PER_RAD = 16384.0f / TWO_PI; // 14-bit encoder
static constexpr float KP_SCALE_DIVISOR = 26.7f;
static constexpr float KP_SCALE_MULTIPLIER = 2048.0f;
static constexpr float KP_SCALE = KP_SCALE_MULTIPLIER / KP_SCALE_DIVISOR;
static constexpr float KW_SCALE = 100.0f * 1024.0f;
static constexpr int INTER_MOTOR_DELAY_US = 50; // Collision avoidance between motor messages
static constexpr int HEADER_TIMEOUT_US = 200;
static constexpr int BODY_TIMEOUT_US = 400;
static constexpr int STATUS_BODY_SIZE = 77; // Bytes after header
static constexpr int CRC_DATA_WORDS = 74 / sizeof( uint32_t );
static constexpr int WRITE_POLL_DELAY_US = 5;

// Per-motor-type configuration: baud rate, gear ratio, and mode codes
// Baud rates compensate that cycles are calculated for 24MHz UART clock but we use 80MHz
struct MotorTypeConfig {
  int baud_rate;
  float gear_ratio;
  uint8_t mode_brake;
  uint8_t mode_foc;
  uint8_t mode_calibrate;
};

static constexpr MotorTypeConfig MOTOR_CONFIGS[] = {
    /* A1  */ { 4800000, 9.1f, 0, 10, 4 },
    /* B1  */ { 6000000, 8.66f, 0, 10, 4 },
    /* GO  */ { 4000000, 6.33f, 0, 1, 2 },
};

static_assert( sizeof( MOTOR_CONFIGS ) / sizeof( MOTOR_CONFIGS[0] ) == 3,
               "MOTOR_CONFIGS size must match MotorType enum count" );

static const MotorTypeConfig &getConfig( MotorType type )
{
  return MOTOR_CONFIGS[static_cast<int>( type )];
}

static uint8_t getMotorModeCode( MotorType type, MotorMode mode )
{
  const auto &cfg = getConfig( type );
  switch ( mode ) {
  case MotorMode::BRAKE:
    return cfg.mode_brake;
  case MotorMode::FOC:
    return cfg.mode_foc;
  case MotorMode::CALIBRATE:
    return cfg.mode_calibrate;
  case MotorMode::INVALID:
  default:
    return cfg.mode_brake;
  }
}

static MotorMode getMotorMode( MotorType type, uint8_t code )
{
  const auto &cfg = getConfig( type );
  if ( code == cfg.mode_brake )
    return MotorMode::BRAKE;
  if ( code == cfg.mode_foc )
    return MotorMode::FOC;
  if ( code == cfg.mode_calibrate )
    return MotorMode::CALIBRATE;
  return MotorMode::INVALID;
}

MotorComm::MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type )
    : serial_( serial ), direction_pin_( direction_pin ), type_( type )
{
  serial_->transmitterEnable( direction_pin );
  serial_->addMemoryForWrite( send_buffer_, sizeof( send_buffer_ ) );
  serial_->addMemoryForRead( receive_buffer_, sizeof( receive_buffer_ ) );
  serial_->begin( getConfig( type ).baud_rate, SERIAL_8N1 );

  const float gear_ratio = getConfig( type_ ).gear_ratio;
  torque_scale_ = TORQUE_SCALE / gear_ratio;
  velocity_scale_ = VELOCITY_SCALE * gear_ratio;
  position_scale_ = ENCODER_TICKS_PER_RAD * gear_ratio;
  kp_scale_ = KP_SCALE / ( gear_ratio * gear_ratio );
  kw_scale_ = KW_SCALE / ( gear_ratio * gear_ratio );
}

void MotorComm::sendCommand( const MotorCommCommand &command )
{
  MasterComdDataV3 data;
  memset( &data, 0, sizeof( data ) );
  data.head.start[0] = PACKET_START_BYTE_0;
  data.head.start[1] = PACKET_START_BYTE_1;
  data.head.motorID = command.motor_id;
  data.Mdata.mode = getMotorModeCode( type_, command.mode );
  data.Mdata.ModifyBit = 0xFF;
  data.Mdata.T = static_cast<int16_t>( command.torque * torque_scale_ );
  data.Mdata.W = static_cast<int16_t>( command.velocity * velocity_scale_ );
  data.Mdata.Pos = command.position * position_scale_;
  // Divide kp and kd by gear ratio squared to get the values at the rotor
  data.Mdata.K_P = static_cast<int16_t>( command.k_p * kp_scale_ );
  data.Mdata.K_W = static_cast<int16_t>( command.k_w * kw_scale_ );
  data.Mdata.Res[0].u32 = static_cast<uint32_t>( TORQUE_SCALE );
  data.CRCdata.u32 = crc32_core( (uint32_t *)&data, sizeof( data ) / sizeof( uint32_t ) - 1 );
  serial_->clear();
  writeData( (uint8_t *)&data, sizeof( MasterComdDataV3 ) );
}

MotorCommStatus MotorComm::receiveStatus() { return readStatus(); }

void MotorComm::sendReceive( const MotorCommCommand &command, MotorCommStatus &status )
{
  sendCommand( command );
  status = receiveStatus();
}

void MotorComm::sendReceive( const MotorCommCommand &left_command,
                             const MotorCommCommand &right_command, MotorCommStatus &left_status,
                             MotorCommStatus &right_status )
{
  sendCommand( left_command );
  left_status = receiveStatus();
  delayMicroseconds( INTER_MOTOR_DELAY_US );
  sendCommand( right_command );
  right_status = receiveStatus();
}

MotorCommStatus MotorComm::readStatus()
{
  MotorCommStatus result;
  result.valid = false;
  status_buffer_[0] = PACKET_START_BYTE_0;
  status_buffer_[1] = PACKET_START_BYTE_1;
  elapsedMicros time;
  while ( serial_->available() < 2 || serial_->read() != PACKET_START_BYTE_0 ||
          serial_->peek() != PACKET_START_BYTE_1 ) {
    if ( time > HEADER_TIMEOUT_US ) {
      return result;
    }
  }
  while ( serial_->available() < STATUS_BODY_SIZE ) {
    if ( time > BODY_TIMEOUT_US )
      return result;
  }
  serial_->readBytes( status_buffer_ + 1, STATUS_BODY_SIZE );
  serial_->clear();
  uint32_t crc = crc32_core( (uint32_t *)status_buffer_, CRC_DATA_WORDS );
  ServoComdDataV3 *status = (ServoComdDataV3 *)status_buffer_;
  if ( crc != status->CRCdata.u32 ) {
    return result;
  }

  result.valid = true;
  result.motor_id = status->head.motorID;
  result.mode = getMotorMode( type_, status->Mdata.mode );
  result.temperature = status->Mdata.Temp;
  result.error_code = status->Mdata.MError;
  result.torque = status->Mdata.T / TORQUE_SCALE * getConfig( type_ ).gear_ratio;
  result.velocity_high = status->Mdata.W / VELOCITY_SCALE / getConfig( type_ ).gear_ratio;
  result.velocity_low = status->Mdata.LW / getConfig( type_ ).gear_ratio;
  result.position = status->Mdata.Pos / ENCODER_TICKS_PER_RAD / getConfig( type_ ).gear_ratio;
  result.acceleration = status->Mdata.Acc;
  return result;
}

void MotorComm::writeData( const uint8_t *data, size_t size )
{
  serial_->clearWriteError();
  while ( serial_->availableForWrite() < static_cast<int>( size ) )
    delayMicroseconds( WRITE_POLL_DELAY_US );
  serial_->write( data, size );
  serial_->flush();
}
