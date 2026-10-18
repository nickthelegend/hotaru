#pragma once
/*
 * leds.h — WS2812 ring (16 x 5050) on one GPIO.
 *
 * LEVEL SHIFT WARNING. A WS2812 on 5 V wants DI above 0.7 x VDD = 3.5 V.
 * The ESP32 drives 3.3 V. It usually works and sometimes does not, and the
 * failure looks like random flicker or a wrong first pixel. Two honest fixes:
 *   a) drop the ring's supply to ~4.3 V with a 1N4148 in series on its 5V
 *      leg -- threshold falls to ~3.0 V and 3.3 V logic is comfortably clear.
 *   b) a 74AHCT125 buffer on DI.
 * (a) costs one diode and is what the wiring doc specifies.
 */
#include <Adafruit_NeoPixel.h>
#include "config.h"

class Ring {
 public:
  void begin() {
    px_.begin();
    px_.setBrightness(RING_MAX_BRIGHT);
    px_.clear();
    px_.show();
  }
  void solid(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < RING_LEDS; i++) px_.setPixelColor(i, r, g, b);
    px_.show();
  }
  void off() { px_.clear(); px_.show(); }

  // warm lamp white -- the default "on" look
  void lamp(uint8_t level = 255) {
    solid((level * 255) / 255, (level * 180) / 255, (level * 90) / 255);
  }

  // one dot chasing the ring; call repeatedly, non-blocking
  void spin(uint32_t period_ms = 60, uint8_t r=0, uint8_t g=140, uint8_t b=255) {
    if (millis() - t_ < period_ms) return;
    t_ = millis();
    px_.clear();
    px_.setPixelColor(head_, r, g, b);
    px_.setPixelColor((head_ + RING_LEDS - 1) % RING_LEDS, r/4, g/4, b/4);
    px_.show();
    head_ = (head_ + 1) % RING_LEDS;
  }

  // breathing glow, non-blocking
  void breathe(uint8_t r=255, uint8_t g=170, uint8_t b=70) {
    if (millis() - t_ < 16) return;
    t_ = millis();
    phase_ += 0.03f;
    float k = (sinf(phase_) + 1.0f) * 0.5f;
    solid(r * k, g * k, b * k);
  }

  // level meter driven by the mic, 0..1
  void meter(float level) {
    int n = (int)(constrain(level, 0.f, 1.f) * RING_LEDS + 0.5f);
    for (int i = 0; i < RING_LEDS; i++)
      px_.setPixelColor(i, i < n ? px_.Color(0, 255, 90) : 0);
    px_.show();
  }

 private:
  Adafruit_NeoPixel px_{RING_LEDS, PIN_RING_DI, NEO_GRB + NEO_KHZ800};
  uint32_t t_ = 0;
  uint16_t head_ = 0;
  float phase_ = 0;
};
