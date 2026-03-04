#pragma once

#include "config.h"
#include <Arduino.h>
#include <elapsedMillis.h>

struct StatusLED {
  enum Speed { SLOW = LED_SLOW_BLINK_MS, FAST = LED_FAST_BLINK_MS };

  StatusLED( int pin ) : led_pin_( pin ) { pinMode( pin, OUTPUT ); }

  void update()
  {
    if ( last_toggle >= speed ) {
      last_toggle = 0;
      last_led_state = !last_led_state;
      digitalWrite( led_pin_, last_led_state ? LOW : HIGH );
    }
  }

  int led_pin_;
  elapsedMillis last_toggle;
  bool last_led_state = false;
  Speed speed = SLOW;
};
