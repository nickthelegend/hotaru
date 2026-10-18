/* Is the app running at all? Answers without needing serial:
 *  - your WS2812 ring on GPIO 15 cycles R -> G -> B, 1 s each
 *  - the devkit's onboard RGB (48, then 38) also cycles
 * If any of those light, the sketch runs and the problem is serial routing.
 */
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);
int step = 0;

void setup() {
  Serial.begin(115200);
  ring.begin();
  ring.setBrightness(60);
  ring.clear(); ring.show();
}

void loop() {
  uint32_t c = (step == 0) ? ring.Color(255,0,0)
             : (step == 1) ? ring.Color(0,255,0)
                           : ring.Color(0,0,255);
  for (int i = 0; i < 16; i++) ring.setPixelColor(i, c);
  ring.show();
  neopixelWrite(48, step==0?60:0, step==1?60:0, step==2?60:0);
  neopixelWrite(38, step==0?60:0, step==1?60:0, step==2?60:0);
  Serial.printf("step %d  t=%lu\n", step, millis());
  step = (step + 1) % 3;
  delay(1000);
}
