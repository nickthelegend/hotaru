/*
 * s3_check.ino — mic + amp + ring bring-up with NO serial required.
 *
 * This board flashes perfectly but produces no app serial on the port we
 * have, so every result here is reported through hardware you can see or
 * hear. Pins match firmware/hotaru/config.h (ESP32-S3).
 *
 * WHAT YOU WILL SEE AND HEAR, in order:
 *   1. WHITE spin, one lap ................ booting
 *   2. status flashes, 3x:
 *        GREEN  = mic OK  and amp OK
 *        RED    = mic FAILED to start
 *        BLUE   = amp FAILED to start
 *        YELLOW = both failed
 *   3. a rising two-tone chirp ............ THE AMP TEST. You should hear it.
 *   4. then forever: the ring is a LEVEL METER for the mic.
 *        silence  = 1-2 pixels lit
 *        talking  = it fills up green
 *        THE MIC TEST. Talk at it.
 *   Every ~6 s it re-chirps so you get another go at the amp.
 */
#include <Adafruit_NeoPixel.h>
#include <ESP_I2S.h>

#define PIN_RING   15
#define RING_N     16
#define PIN_MIC_SCK 10
#define PIN_MIC_WS  11
#define PIN_MIC_SD  12
#define PIN_AMP_BCLK 13
#define PIN_AMP_LRC  14
#define PIN_AMP_DIN  21
#define RATE 16000

Adafruit_NeoPixel ring(RING_N, PIN_RING, NEO_GRB + NEO_KHZ800);
I2SClass micI2S(I2S_NUM_0);
I2SClass ampI2S(I2S_NUM_1);
bool micOk = false, ampOk = false;
uint32_t lastChirp = 0;

void fill(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < RING_N; i++) ring.setPixelColor(i, r, g, b);
  ring.show();
}

void tone_(uint32_t hz, uint32_t ms, float vol = 0.30f) {
  if (!ampOk) return;
  int total = (int)((uint64_t)RATE * ms / 1000);
  int16_t blk[256];
  float ph = 0, inc = 2.0f * PI * hz / RATE;
  int done = 0;
  while (done < total) {
    int n = min(256, total - done);
    for (int i = 0; i < n; i++) { blk[i] = (int16_t)(sinf(ph) * 32767.0f * vol);
                                  ph += inc; if (ph > 2*PI) ph -= 2*PI; }
    ampI2S.write((const uint8_t *)blk, n * sizeof(int16_t));
    done += n;
  }
}

float micLevel() {
  if (!micOk) return 0;
  static int32_t buf[256];
  size_t n = micI2S.readBytes((char *)buf, sizeof(buf));
  size_t c = n / 4;
  if (!c) return 0;
  double acc = 0;
  for (size_t i = 0; i < c; i++) { float s = (float)(buf[i] >> 8) / 8388608.0f; acc += (double)s*s; }
  return sqrtf(acc / c);
}

void setup() {
  Serial.begin(115200);
  ring.begin(); ring.setBrightness(70); ring.clear(); ring.show();

  for (int i = 0; i < RING_N; i++) {          // 1. booting
    ring.clear(); ring.setPixelColor(i, 120,120,120); ring.show(); delay(45);
  }

  micI2S.setPins(PIN_MIC_SCK, PIN_MIC_WS, -1, PIN_MIC_SD);
  micOk = micI2S.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_32BIT,
                       I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT);
  ampI2S.setPins(PIN_AMP_BCLK, PIN_AMP_LRC, PIN_AMP_DIN, -1);
  ampOk = ampI2S.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_16BIT,
                       I2S_SLOT_MODE_MONO);

  uint8_t r = 0, g = 0, b = 0;                // 2. status
  if (micOk && ampOk)       g = 255;
  else if (!micOk && !ampOk){ r = 255; g = 160; }
  else if (!micOk)          r = 255;
  else                      b = 255;
  for (int k = 0; k < 3; k++) { fill(r,g,b); delay(400); fill(0,0,0); delay(250); }

  tone_(660, 160); tone_(990, 200);           // 3. amp
  lastChirp = millis();
}

void loop() {
  float lv = micLevel() * 8.0f;               // 4. mic meter
  int n = (int)(constrain(lv, 0.f, 1.f) * RING_N + 0.5f);
  if (micOk && n < 1) n = 1;
  for (int i = 0; i < RING_N; i++)
    ring.setPixelColor(i, i < n ? ring.Color(0,255,90) : 0);
  ring.show();

  if (millis() - lastChirp > 6000) { tone_(880, 140); lastChirp = millis(); }
  delay(30);
}
