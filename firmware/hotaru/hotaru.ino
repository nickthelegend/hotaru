/*
 * hotaru.ino — HOTARU hardware bring-up + motion firmware.
 *
 * Board : ESP32-S3 N16R8 (ROBODUINO, dual USB-C)
 * FQBN  : esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc
 *
 *   FQBN="esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc"
 *   arduino-cli compile --fqbn "$FQBN" firmware/hotaru
 *   arduino-cli upload -p <port> --fqbn "$FQBN" firmware/hotaru
 *   arduino-cli monitor -p <port> -c baudrate=115200
 *
 * The board has TWO USB-C ports. The UART bridge one enumerates as
 * /dev/cu.usbserial-* and always works. The native-USB one appears as
 * /dev/cu.usbmodem* and only carries Serial when CDCOnBoot=cdc is set --
 * which is why it is in the FQBN above.
 *
 * Every pin lives in config.h. Full wiring is in docs/WIRING.md.
 *
 * POWER, the short version: the servo bus gets its own 5 V leg with the
 * bulk cap AT THE BRANCH, and only the ground is shared with the ESP32.
 * Four MG996R stalling together is 10 A; nothing here survives that, which
 * is why every joint has software end stops in config.h.
 */
#include "config.h"
#include "motion.h"
#include "leds.h"
#include "audio.h"
#include "input.h"
#include <esp_system.h>

Motion arm;
Ring   ring;
Audio  audio;
Key    key;

enum Mode { M_IDLE, M_SPIN, M_BREATHE, M_METER, M_WAVE };
Mode mode = M_BREATHE;
uint32_t waveT = 0;
int waveStep = 0;

// ------------------------------------------------------------------ help --
void help() {
  Serial.println(F("\n================ HOTARU ================"));
  Serial.println(F(" JOINTS   p<deg> pan   s<deg> shoulder"));
  Serial.println(F("          e<deg> elbow h<deg> head"));
  Serial.println(F("          v<dps> speed  x detach  X attach"));
  Serial.println(F(" POSES    1 home   2 rest   3 alert   w wave"));
  Serial.println(F(" LEDS     L lamp   o off    r spin   b breathe  m meter"));
  Serial.println(F(" AUDIO    t tone   u chirp-up  d chirp-down  n mic level"));
  Serial.println(F(" INFO     ?  this   i  status"));
  Serial.println(F("========================================\n"));
}

void status() {
  Serial.println(F("--- status ---"));
  for (int i = 0; i < N_JOINTS; i++)
    Serial.printf("  %-9s GPIO%-3d %6.1f -> %6.1f deg  [%.0f..%.0f]  %.0f deg/s\n",
                  arm.j[i].name, arm.j[i].pin, arm.j[i].pos, arm.j[i].target,
                  arm.j[i].lo, arm.j[i].hi, arm.j[i].speed);
  Serial.printf("  ring      GPIO%-3d %d LEDs, brightness cap %d\n",
                PIN_RING_DI, RING_LEDS, RING_MAX_BRIGHT);
  Serial.printf("  mic       SCK%d WS%d SD%d   %s\n", PIN_MIC_SCK, PIN_MIC_WS,
                PIN_MIC_SD, audio.micOk() ? "OK" : "FAILED TO START");
  Serial.printf("  amp       BCLK%d LRC%d DIN%d %s\n", PIN_AMP_BCLK, PIN_AMP_LRC,
                PIN_AMP_DIN, audio.ampOk() ? "OK" : "FAILED TO START");
  Serial.printf("  key       GPIO%-3d %s\n", PIN_MX_KEY, key.isDown() ? "DOWN" : "up");
}

// a small idle animation that proves all four joints move together
void wave() {
  if (millis() - waveT < 1200) return;
  waveT = millis();
  switch (waveStep++ % 4) {
    case 0: arm.pose(70,  80, 100,  80); break;
    case 1: arm.pose(110, 95,  95, 110); break;
    case 2: arm.pose(90,  70, 115,  90); break;
    case 3: arm.home();                  break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nHOTARU booting"));
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_BROWNOUT) {
    Serial.println(F("*** LAST RESET WAS A BROWNOUT ***"));
    Serial.println(F("  the 5 V rail collapsed. Check: bulk cap at the servo"));
    Serial.println(F("  branch, charger not a laptop port, and that the servo"));
    Serial.println(F("  leg is separate from the ESP32's."));
  }

  ring.begin();
  ring.solid(20, 0, 0);          // red while we bring the rest up

  arm.begin();
  arm.home();
  Serial.println(F("servos attached, homed to 90"));

  key.begin();
  Serial.printf("mic  %s\n", audio.beginMic() ? "OK" : "FAILED");
  Serial.printf("amp  %s\n", audio.beginAmp() ? "OK" : "FAILED");

  ring.lamp();
  audio.chirpUp();
  help();
}

void loop() {
  arm.tick();                    // 50 Hz, non-blocking

  switch (mode) {
    case M_SPIN:    ring.spin();                  break;
    case M_BREATHE: ring.breathe();               break;
    case M_METER:   ring.meter(audio.micLevel() * 6.0f); break;
    case M_WAVE:    ring.breathe(); wave();       break;
    default: break;
  }

  if (key.pressed()) {
    Serial.println(F("[key] pressed"));
    audio.chirpUp();
    mode = (mode == M_WAVE) ? M_BREATHE : M_WAVE;
  }

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  char c = cmd[0];
  float val = cmd.substring(1).toFloat();

  switch (c) {
    case 'p': arm.moveTo(J_PAN, val);      Serial.printf("pan -> %.0f\n", val); break;
    case 's': arm.moveTo(J_SHOULDER, val); Serial.printf("shoulder -> %.0f\n", val); break;
    case 'e': arm.moveTo(J_ELBOW, val);    Serial.printf("elbow -> %.0f\n", val); break;
    case 'h': arm.moveTo(J_HEAD, val);     Serial.printf("head -> %.0f\n", val); break;
    case 'v': arm.allSpeed(val);           Serial.printf("speed -> %.0f deg/s\n", val); break;
    case 'x': arm.detachAll();             Serial.println(F("detached (limp)")); break;
    case 'X': arm.attachAll();             Serial.println(F("attached")); break;
    case '1': arm.home();  Serial.println(F("pose: home"));  break;
    case '2': arm.rest();  Serial.println(F("pose: rest"));  break;
    case '3': arm.alert(); Serial.println(F("pose: alert")); break;
    case 'w': mode = M_WAVE;    Serial.println(F("wave"));    break;
    case 'L': mode = M_IDLE; ring.lamp();    Serial.println(F("ring: lamp")); break;
    case 'o': mode = M_IDLE; ring.off();     Serial.println(F("ring: off"));  break;
    case 'r': mode = M_SPIN;    Serial.println(F("ring: spin"));    break;
    case 'b': mode = M_BREATHE; Serial.println(F("ring: breathe")); break;
    case 'm': mode = M_METER;   Serial.println(F("ring: mic meter -- talk to it")); break;
    case 't': audio.tone(880, 300); Serial.println(F("tone 880 Hz")); break;
    case 'u': audio.chirpUp();      break;
    case 'd': audio.chirpDown();    break;
    case 'n': {
      Serial.println(F("mic level for 3 s:"));
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) {
        Serial.printf("  %.4f\n", audio.micLevel());
        delay(150);
      }
      break;
    }
    case 'i': status(); break;
    default:  help();   break;
  }
}
