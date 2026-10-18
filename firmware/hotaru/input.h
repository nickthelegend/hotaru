#pragma once
/*
 * input.h — the one Cherry MX switch. Two legs, no polarity: one to the
 * GPIO, one to GND. INPUT_PULLUP, so the pin idles HIGH and a press is LOW.
 * 25 ms debounce covers MX bounce (typically ~5 ms).
 */
#include <Arduino.h>
#include "config.h"

class Key {
 public:
  void begin() { pinMode(PIN_MX_KEY, INPUT_PULLUP); level_ = raw(); stable_ = level_; }

  // true on the transition into "pressed"
  bool pressed() {
    bool r = raw();
    if (r != level_) { level_ = r; t_ = millis(); }
    if (millis() - t_ > 25 && stable_ != level_) {
      stable_ = level_;
      if (stable_) { held_since_ = millis(); return true; }
    }
    return false;
  }
  bool isDown() const { return stable_; }
  uint32_t heldMs() const { return stable_ ? millis() - held_since_ : 0; }

 private:
  bool raw() { return digitalRead(PIN_MX_KEY) == LOW; }   // active LOW
  bool level_ = false, stable_ = false;
  uint32_t t_ = 0, held_since_ = 0;
};
