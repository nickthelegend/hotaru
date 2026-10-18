/* s3_ring.ino — full WS2812 ring exercise. GPIO 15, 16 x 5050.
 *
 * Runs five patterns on a loop. Between each, a short dark pause.
 *   1. PIXEL WALK   one white dot, 0 -> 15, slow. Every LED must light.
 *                   A dead pixel shows as a gap; a wrong-colour one shows here too.
 *   2. CHANNELS     whole ring red, then green, then blue, then white.
 *                   Proves all three dies in every LED.
 *   3. RAINBOW      hue sweep around the ring, rotating.
 *   4. BREATHE      warm lamp white, fading in and out.
 *   5. BRIGHTNESS   ramp 5 -> 120 -> 5, warm white. Watch for flicker at the
 *                   low end, which is where a marginal data line shows up.
 */
#include <Adafruit_NeoPixel.h>
#define PIN_RING 15
#define N 16
Adafruit_NeoPixel ring(N, PIN_RING, NEO_GRB + NEO_KHZ800);

void allOff() { ring.clear(); ring.show(); }
void fill(uint32_t c) { for (int i=0;i<N;i++) ring.setPixelColor(i,c); ring.show(); }
void gap() { allOff(); delay(500); }

void setup() {
  ring.begin();
  ring.setBrightness(70);
  allOff();
}

void loop() {
  // 1. every pixel, one at a time
  for (int i = 0; i < N; i++) {
    ring.clear();
    ring.setPixelColor(i, 255, 255, 255);
    ring.show();
    delay(120);
  }
  gap();

  // 2. each colour channel
  uint32_t cols[4] = { ring.Color(255,0,0), ring.Color(0,255,0),
                       ring.Color(0,0,255), ring.Color(255,255,255) };
  for (int k = 0; k < 4; k++) { fill(cols[k]); delay(700); }
  gap();

  // 3. rainbow, rotating
  for (int spin = 0; spin < 3 * N; spin++) {
    for (int i = 0; i < N; i++)
      ring.setPixelColor(i, ring.ColorHSV(((i + spin) * 65536L / N) & 0xFFFF, 255, 255));
    ring.show();
    delay(55);
  }
  gap();

  // 4. breathe, warm lamp white
  for (int k = 0; k < 2; k++) {
    for (int b = 0; b <= 100; b += 2) {
      fill(ring.Color(b*255/100, b*175/100, b*85/100)); delay(14);
    }
    for (int b = 100; b >= 0; b -= 2) {
      fill(ring.Color(b*255/100, b*175/100, b*85/100)); delay(14);
    }
  }
  gap();

  // 5. brightness ramp -- low end is where a marginal data line shows
  for (int b = 5; b <= 120; b += 3) { ring.setBrightness(b); fill(ring.Color(255,175,85)); delay(25); }
  for (int b = 120; b >= 5; b -= 3) { ring.setBrightness(b); fill(ring.Color(255,175,85)); delay(25); }
  ring.setBrightness(70);
  gap();
}
