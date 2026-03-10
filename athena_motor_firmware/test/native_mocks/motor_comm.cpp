#include "motor_comm.h"
#include <iostream>

// Provide empty implementations to avoid linker errors
MotorComm::MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type )
    : serial_( serial ), direction_pin_( direction_pin ), type_( type )
{
}

MotorCommStatus MotorComm::readStatus() { return MotorCommStatus{}; }

void MotorComm::writeData( const uint8_t *data, size_t size ) { }

void MotorComm::sendCommand( const MotorCommCommand &command ) { }

MotorCommStatus MotorComm::receiveStatus() { return MotorCommStatus{}; }
