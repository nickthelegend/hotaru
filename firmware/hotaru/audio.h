// audio.h -- the INMP441 microphone and the MAX98357A speaker, one I2S port
// each (the ESP32-S3 has two). Speech is recorded as 16 kHz 16-bit mono into
// PSRAM; speech out arrives from Deepgram as the same format and is streamed
// straight to the amplifier while it downloads.
#pragma once
#include <ESP_I2S.h>

I2SClass micI2S, spkI2S;
bool micOk = false, spkOk = false;

int16_t* recBuf = nullptr;
size_t recCap = 0, recLen = 0;          // samples
float volume = VOLUME_DEFAULT;
static float dcX = 0, dcY = 0;          // DC-blocking filter state

bool audioBegin() {
  recCap = (size_t)MIC_RATE * REC_MAX_S;
  recBuf = (int16_t*)ps_malloc(recCap * sizeof(int16_t));
  if (!recBuf) {                          // no PSRAM: 4 s in internal RAM
    recCap = (size_t)MIC_RATE * 4;
    recBuf = (int16_t*)malloc(recCap * sizeof(int16_t));
    Serial.println(F("no PSRAM (Tools > PSRAM: OPI PSRAM?): recordings limited to 4 s"));
  }
  if (!recBuf) recCap = 0;

  micI2S.setPins(PIN_MIC_SCK, PIN_MIC_WS, -1, PIN_MIC_SD);
  micOk = micI2S.begin(I2S_MODE_STD, MIC_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO,
                       I2S_STD_SLOT_LEFT);
  if (!micOk) Serial.println(F("*** microphone: I2S init failed ***"));

  spkI2S.setPins(PIN_AMP_BCLK, PIN_AMP_LRC, PIN_AMP_DIN);
  spkOk = spkI2S.begin(I2S_MODE_STD, SPK_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  if (!spkOk) Serial.println(F("*** speaker: I2S init failed ***"));
  return micOk && spkOk && recCap;
}

// ---------------------------------------------------------------- record ----
void recStart() {
  recLen = 0;
  dcX = dcY = 0;
  if (!micOk) return;
  int32_t junk[256];                      // drop what sat in the DMA buffers
  for (int i = 0; i < 4; i++) micI2S.readBytes((char*)junk, sizeof junk);
}

// Pull whatever the mic has (about 16 ms per call). False when full.
bool recPoll() {
  if (!micOk || !recCap) return false;
  int32_t raw[256];
  size_t n = micI2S.readBytes((char*)raw, sizeof raw) / sizeof(int32_t);
  for (size_t i = 0; i < n && recLen < recCap; i++) {
    float x = (float)(raw[i] >> MIC_SHIFT);
    dcY = x - dcX + 0.995f * dcY;         // remove the mic's DC offset
    dcX = x;
    recBuf[recLen++] = (int16_t)constrain(dcY, -32767.0f, 32767.0f);
  }
  return recLen < recCap;
}

// Lift a quiet recording to a healthy level (Deepgram copes either way,
// but it helps far from the lamp). Gain capped at 8x so hiss stays hiss.
void recNormalize() {
  int peak = 1;
  for (size_t i = 0; i < recLen; i++) peak = max(peak, abs((int)recBuf[i]));
  float g = min(8.0f, 24000.0f / peak);
  if (g <= 1.05f) return;
  for (size_t i = 0; i < recLen; i++) recBuf[i] = (int16_t)(recBuf[i] * g);
}

// ---------------------------------------------------------------- speaker ----
static void spkWriteMono(const int16_t* s, size_t n) {
  if (!spkOk) return;
  int16_t st[256];
  size_t i = 0;
  float env = 0;
  while (i < n) {
    size_t m = min((size_t)128, n - i);
    for (size_t k = 0; k < m; k++) {
      int v = (int)(s[i + k] * volume);
      st[2 * k] = st[2 * k + 1] = (int16_t)constrain(v, -32767, 32767);
      env = max(env, fabsf((float)s[i + k]) / 32768.0f);
    }
    spkI2S.write((uint8_t*)st, m * 4);
    i += m;
  }
  talkLevel = talkLevel * 0.6f + min(1.0f, env * 2.5f) * 0.4f;   // drives the LED pulse
}

void spkSilence(int ms) {
  int16_t z[256] = {0};
  int frames = SPK_RATE * ms / 1000;
  while (frames > 0) { int m = min(frames, 128); spkI2S.write((uint8_t*)z, m * 4); frames -= m; }
  talkLevel = 0;
}

// a short tone: listening started / stopped / something went wrong
void beep(float hz, int ms, float level = 0.25f) {
  int n = SPK_RATE * ms / 1000;
  int16_t buf[128];
  for (int i = 0; i < n; i += 128) {
    int m = min(128, n - i);
    for (int k = 0; k < m; k++) {
      float t = (float)(i + k) / SPK_RATE;
      float fade = min(1.0f, min((i + k) / 200.0f, (n - i - k) / 200.0f));
      buf[k] = (int16_t)(sinf(6.2831853f * hz * t) * 32767.0f * level * fade);
    }
    spkWriteMono(buf, m);
  }
  spkSilence(20);
}

// Deepgram's speech arrives as raw little-endian 16-bit samples in whatever
// chunk sizes the network delivers; this Stream turns them into I2S frames.
class SpeakerSink : public Stream {
 public:
  size_t total = 0;
  size_t write(const uint8_t* buf, size_t n) override {
    total += n;
    size_t i = 0;
    int16_t s[128];
    int m = 0;
    if (hasCarry && n) { s[m++] = (int16_t)(carry | (buf[0] << 8)); i = 1; hasCarry = false; }
    for (; i + 1 < n; i += 2) {
      s[m++] = (int16_t)(buf[i] | (buf[i + 1] << 8));
      if (m == 128) { spkWriteMono(s, m); m = 0; }
    }
    if (i < n) { carry = buf[i]; hasCarry = true; }
    if (m) spkWriteMono(s, m);
    return n;
  }
  size_t write(uint8_t c) override { return write(&c, 1); }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
 private:
  uint8_t carry = 0;
  bool hasCarry = false;
};
