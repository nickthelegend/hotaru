/*
 * s3_eye.ino — light HOTARU's eye: the WS2812 ring, plus the S3's onboard RGB
 * as a "board is alive" tell. If the onboard LED lights and the ring does not,
 * the fault is the ring's power, ground or data wire -- not the board.
 *
 * FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=disabled,CDCOnBoot=cdc,USBMode=hwcdc
 *
 * Ring per docs/WIRING.md: DI on GPIO 15 through 330R, 5V on its own leg,
 * GND common. Starts at brightness 40 (~150 mA full white), safe even if the
 * ring is on 3V3. '+' never goes past RING_MAX_BRIGHT from config.h.
 *
 * Boot: one dot walks round all 16 pixels (a gap is a dead pixel), then the
 * ring holds warm lamp white.
 *
 * Serial @ 115200:
 *   w warm white   c cool white   r / g / b   o off
 *   + / -  brightness      p pixel walk      s rainbow spin
 *   i status               ? help
 */
#include <Adafruit_NeoPixel.h>
#include <esp_system.h>
#include "config.h"

Adafruit_NeoPixel ring(RING_LEDS, PIN_RING_DI, NEO_GRB + NEO_KHZ800);
int bright = 40;
uint32_t colour = 0;
const char *cname = "off";

// Onboard RGB: GPIO 48 on most S3 devkits, 38 on some. s3_onboard drives both
// for the same reason. Neither is used by anything else in config.h.
void onboard(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(48, r, g, b);
  neopixelWrite(38, r, g, b);
}

void show() {
  ring.setBrightness(bright);
  for (int i = 0; i < RING_LEDS; i++) ring.setPixelColor(i, colour);
  ring.show();
}

void set(uint32_t c, const char *nm) {
  colour = c;
  cname = nm;
  show();
  Serial.printf("ring: %s @ %d/255\n", cname, bright);
}

void pixelWalk() {
  Serial.println(F("pixel walk: one dot round all 16 -- a gap is a dead pixel"));
  ring.setBrightness(bright);
  for (int i = 0; i < RING_LEDS; i++) {
    ring.clear();
    ring.setPixelColor(i, 255, 255, 255);
    ring.show();
    delay(90);
  }
  show();
}

void spin() {
  Serial.println(F("rainbow, 3 turns"));
  ring.setBrightness(bright);
  for (int s = 0; s < 3 * RING_LEDS; s++) {
    for (int i = 0; i < RING_LEDS; i++)
      ring.setPixelColor(i, ring.ColorHSV(((i + s) * 65536L / RING_LEDS) & 0xFFFF, 255, 255));
    ring.show();
    delay(50);
  }
  show();
}

void reportReset() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print(F("reset reason: "));
  switch (r) {
    case ESP_RST_POWERON:  Serial.println(F("power-on (normal)")); break;
    case ESP_RST_SW:       Serial.println(F("software (normal)")); break;
    case ESP_RST_USB:      Serial.println(F("USB / flash (normal)")); break;
    case ESP_RST_BROWNOUT:
      Serial.println(F("*** BROWNOUT ***"));
      Serial.println(F("  The 3.3V rail collapsed. If the ring is on 3V3, send '-'"));
      Serial.println(F("  a few times, or move it to its own 5V leg."));
      break;
    case ESP_RST_PANIC:    Serial.println(F("panic/exception")); break;
    default:               Serial.printf("other (%d)\n", (int)r); break;
  }
}

void status() {
  Serial.printf("ring GPIO %d, %d LEDs: %s @ %d/255 (cap %d)\n",
                PIN_RING_DI, RING_LEDS, cname, bright, RING_MAX_BRIGHT);
}

void help() {
  Serial.println(F("\n--- HOTARU S3 eye ---"));
  Serial.println(F("  w warm  c cool  r g b  o off  +/- brightness"));
  Serial.println(F("  p pixel walk  s spin  i status"));
  Serial.println(F("  onboard LED green = this sketch is running\n"));
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nHOTARU S3 eye booting"));
  reportReset();
  onboard(0, 40, 0);
  ring.begin();
  ring.clear();
  ring.show();
  pixelWalk();
  set(ring.Color(255, 175, 85), "warm white");   // s3_ring's lamp white
  help();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  switch (cmd[0]) {
    case 'w': set(ring.Color(255, 175, 85), "warm white"); break;
    case 'c': set(ring.Color(255, 255, 255), "cool white"); break;
    case 'r': set(ring.Color(255, 0, 0), "red"); break;
    case 'g': set(ring.Color(0, 255, 0), "green"); break;
    case 'b': set(ring.Color(0, 0, 255), "blue"); break;
    case 'o': set(0, "off"); break;
    case '+': bright = (bright + 10 > RING_MAX_BRIGHT) ? RING_MAX_BRIGHT : bright + 10;
              set(colour, cname); break;
    case '-': bright = (bright - 10 < 5) ? 5 : bright - 10;
              set(colour, cname); break;
    case 'p': pixelWalk(); break;
    case 's': spin(); break;
    case 'i': status(); break;
    default:  help(); break;
  }
}
