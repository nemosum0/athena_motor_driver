#pragma once

#include <cstdint>

extern uint32_t simulated_micros;
extern uint32_t simulated_millis;

inline uint32_t micros() { return simulated_micros; }

inline uint32_t millis() { return simulated_millis; }

inline void delay( uint32_t ms )
{
  simulated_millis += ms;
  simulated_micros += ms * 1000;
}

template<typename T>
inline T constrain( T val, T min, T max )
{
  if ( val < min )
    return min;
  if ( val > max )
    return max;
  return val;
}

class elapsedMicros
{
private:
  uint32_t start_time;

public:
  elapsedMicros() : start_time( simulated_micros ) { }

  elapsedMicros( uint32_t val ) : start_time( simulated_micros - val ) { }

  elapsedMicros &operator=( uint32_t val )
  {
    start_time = simulated_micros - val;
    return *this;
  }

  operator uint32_t() const { return simulated_micros - start_time; }
};

class elapsedMillis
{
private:
  uint32_t start_time;

public:
  elapsedMillis() : start_time( simulated_millis ) { }

  elapsedMillis( uint32_t val ) : start_time( simulated_millis - val ) { }

  elapsedMillis &operator=( uint32_t val )
  {
    start_time = simulated_millis - val;
    return *this;
  }

  operator uint32_t() const { return simulated_millis - start_time; }
};

class DummySerial
{
public:
  void printf( const char *format, ... ) { }

  void print( const char *msg ) { }

  void println( const char *msg ) { }
};

extern DummySerial Serial;
