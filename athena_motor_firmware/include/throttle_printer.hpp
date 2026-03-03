#pragma once

#include <elapsedMillis.h>

class ThrottlePrinter
{
public:
  ThrottlePrinter( unsigned long ms ) : print_interval_ms_( ms ) { }

  void print( const char *message )
  {
    if ( elapsed_time_ >= print_interval_ms_ ) {
      Serial.println( message );
      elapsed_time_ = 0;
    }
  }

private:
  unsigned long print_interval_ms_;
  // Start with a long elapsed time so the first message is printed immediately
  elapsedMillis elapsed_time_ { 100000 };
};
