/* Simplest possible ring test. GPIO 15, 16 LEDs, full brightness.
 * Solid colours, 2 seconds each, forever:  WHITE -> RED -> GREEN -> BLUE
 * No animation, no timing subtleties, nothing to miss. */
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);

void solid(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < 16; i++) ring.setPixelColor(i, r, g, b);
  ring.show();
  delay(2000);
}

void setup() {
  ring.begin();
  ring.setBrightness(40);   // ~150 mA total -- safe off the 3V3 regulator
  ring.clear();
  ring.show();
}

void loop() {
  solid(255,255,255);
  solid(255,0,0);
  solid(0,255,0);
  solid(0,0,255);
}
