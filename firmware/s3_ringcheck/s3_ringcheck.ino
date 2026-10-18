/*
 * s3_ringcheck.ino — is the ring fixed? Every pixel, every colour, no library.
 *
 * FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=disabled,CDCOnBoot=cdc,USBMode=hwcdc
 *
 * Raw RMT with the ESP32 core's own WS2812 timing -- the code that lit the
 * onboard LED -- so a fault here is wiring or power, never the firmware.
 *
 * Loops, with a short dark gap between steps:
 *   1. PIXEL WALK        one white dot, LED 1 -> 16. A gap is a dead pixel;
 *                        the dot stopping partway means data breaks at that LED.
 *   2. RED, GREEN, BLUE  whole ring, one colour at a time: every die in every LED.
 *   3. WHITE             pink or orange instead of white = the ring's supply is low.
 *   4. WARM WHITE        held 4 s -- the lamp look.
 *
 * Boot prints the reset reason (BROWNOUT = lighting the ring dragged the board's
 * supply down) and reads GPIO 15 with pull-up then pull-down:
 *   HIGH / LOW   high-impedance: normal for DI
 *   LOW  / LOW   held low: with the ring powered, the wire is on DO not DI,
 *                or shorted to GND
 *   HIGH / HIGH  shorted to a supply rail
 *
 * Serial @ 115200:  t line test   p pause / resume
 */
#include <esp_system.h>
#include "esp32-hal-rmt.h"
#include "config.h"

bool rmtReady = false;
bool running = true;
int cycle = 0;
rmt_data_t sym[RING_LEDS * 24];

bool rmtShow(const uint8_t *grb) {
  if (!rmtReady) {
    rmtReady = rmtInit(PIN_RING_DI, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
    if (!rmtReady) { Serial.println(F("rmtInit FAILED on the ring pin")); return false; }
  }
  int k = 0;
  for (int i = 0; i < RING_LEDS * 3; i++)
    for (int b = 7; b >= 0; b--) {
      bool one = grb[i] & (1 << b);
      sym[k].level0 = 1; sym[k].duration0 = one ? 8 : 4;   // 100 ns ticks
      sym[k].level1 = 0; sym[k].duration1 = one ? 4 : 8;
      k++;
    }
  bool ok = rmtWrite(PIN_RING_DI, sym, RING_LEDS * 24, RMT_WAIT_FOR_EVER);
  delay(2);   // idle low > 50 us: the pixels latch
  return ok;
}

bool paint(int n, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t px[RING_LEDS * 3] = {0};
  for (int i = 0; i < n; i++) { px[3*i] = g; px[3*i+1] = r; px[3*i+2] = b; }
  return rmtShow(px);
}

bool single(int idx, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t px[RING_LEDS * 3] = {0};
  px[3*idx] = g; px[3*idx+1] = r; px[3*idx+2] = b;
  return rmtShow(px);
}

void reportReset() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print(F("reset reason: "));
  switch (r) {
    case ESP_RST_POWERON:  Serial.println(F("power-on (normal)")); break;
    case ESP_RST_SW:       Serial.println(F("software (normal)")); break;
    case ESP_RST_USB:      Serial.println(F("USB / flash (normal)")); break;
    case ESP_RST_BROWNOUT: Serial.println(F("*** BROWNOUT *** -- the ring dragged the board's supply down")); break;
    case ESP_RST_PANIC:    Serial.println(F("panic/exception")); break;
    default:               Serial.printf("other (%d)\n", (int)r); break;
  }
}

void lineTest() {
  int pu = 0, pd = 0;
  for (int k = 0; k < 5; k++) {
    pinMode(PIN_RING_DI, INPUT_PULLUP);   delay(20); pu += digitalRead(PIN_RING_DI);
    pinMode(PIN_RING_DI, INPUT_PULLDOWN); delay(20); pd += digitalRead(PIN_RING_DI);
  }
  pinMode(PIN_RING_DI, OUTPUT);
  digitalWrite(PIN_RING_DI, LOW);
  rmtReady = false;   // pinMode took the pin back from RMT
  Serial.printf("data line GPIO %d: pull-up read HIGH %d/5, pull-down read HIGH %d/5\n",
                PIN_RING_DI, pu, pd);
  if (pu == 5 && pd == 0)      Serial.println(F("  -> high-impedance: normal for DI"));
  else if (pu == 0 && pd == 0) Serial.println(F("  -> held LOW: wire on DO not DI (ring powered), or shorted to GND"));
  else if (pu == 5 && pd == 5) Serial.println(F("  -> held HIGH: shorted to 3V3 or 5V"));
  else                         Serial.println(F("  -> unstable: marginal contact on the data wire"));
}

void wait(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available()) {
      String c = Serial.readStringUntil('\n');
      c.trim();
      if (c == "t") lineTest();
      else if (c == "p") {
        running = !running;
        if (!running) paint(0, 0, 0, 0);
        Serial.println(running ? F("resumed") : F("paused, ring off"));
      }
    }
    delay(10);
  }
}

void step(const char *label, int n, uint8_t r, uint8_t g, uint8_t b, uint32_t hold) {
  if (!running) { wait(200); return; }
  paint(0, 0, 0, 0);
  wait(400);
  bool ok = paint(n, r, g, b);
  Serial.printf("  %-11s (rmtWrite %s)\n", label, ok ? "ok" : "FAILED");
  wait(hold);
}

void pixelWalk() {
  if (!running) return;
  bool ok = true;
  for (int i = 0; i < RING_LEDS; i++) { ok = single(i, 60, 60, 60) && ok; wait(150); }
  Serial.printf("  %-11s (rmtWrite %s)\n", "pixel walk", ok ? "ok" : "FAILED");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nHOTARU ring check v2 -- raw RMT, no library"));
  reportReset();
  neopixelWrite(48, 0, 0, 0);   // onboard RGB off, so only the ring lights
  neopixelWrite(38, 0, 0, 0);
  lineTest();
}

void loop() {
  if (running) Serial.printf("cycle %d\n", ++cycle);
  pixelWalk();
  step("RED",        RING_LEDS, 40,  0,  0, 2000);
  step("GREEN",      RING_LEDS,  0, 40,  0, 2000);
  step("BLUE",       RING_LEDS,  0,  0, 40, 2000);
  step("WHITE",      RING_LEDS, 40, 40, 40, 2000);
  step("WARM WHITE", RING_LEDS, 40, 27, 13, 4000);
}
