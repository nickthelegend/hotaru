#pragma once
/*
 * audio.h — INMP441 in, MAX98357A out. Two SEPARATE I2S peripherals, so the
 * mic and the speaker never share a bus or fight over a clock.
 *
 * INMP441 is a 3.3 V part. Do not put 5 V on it.
 * MAX98357A runs from 5 V for full output into the 4R 2W driver.
 *
 * INMP441 L/R pin: tie to GND = LEFT channel (what this code reads).
 *                  Tie to VDD and it lands in the right slot and you get
 *                  silence, which is the usual "my mic is dead".
 */
#include <ESP_I2S.h>
#include "config.h"

class Audio {
 public:
  bool beginMic() {
    mic_.setPins(PIN_MIC_SCK, PIN_MIC_WS, -1 /*dout*/, PIN_MIC_SD);
    mic_ok_ = mic_.begin(I2S_MODE_STD, MIC_SAMPLE_RATE,
                         I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO,
                         I2S_STD_SLOT_LEFT);
    return mic_ok_;
  }
  bool beginAmp() {
    amp_.setPins(PIN_AMP_BCLK, PIN_AMP_LRC, PIN_AMP_DIN, -1 /*din*/);
    amp_ok_ = amp_.begin(I2S_MODE_STD, AMP_SAMPLE_RATE,
                         I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    return amp_ok_;
  }
  bool micOk() const { return mic_ok_; }
  bool ampOk() const { return amp_ok_; }

  // RMS of one block, normalised 0..1. The INMP441 is a 24-bit part packed
  // left-justified into 32-bit slots, so the sample is >> 8 before use.
  float micLevel() {
    if (!mic_ok_) return 0.0f;
    static int32_t buf[256];
    size_t n = mic_.readBytes((char *)buf, sizeof(buf));
    size_t cnt = n / sizeof(int32_t);
    if (!cnt) return 0.0f;
    double acc = 0;
    for (size_t i = 0; i < cnt; i++) {
      float s = (float)(buf[i] >> 8) / 8388608.0f;   // 2^23
      acc += (double)s * s;
    }
    return sqrtf(acc / cnt);
  }

  // Blocking square-ish tone. Fine for a bring-up beep; the real voice path
  // will stream instead of block.
  void tone(uint32_t hz, uint32_t ms, float vol = 0.25f) {
    if (!amp_ok_) return;
    const int rate = AMP_SAMPLE_RATE;
    const int total = (int)((uint64_t)rate * ms / 1000);
    int16_t blk[256];
    float phase = 0.0f, inc = 2.0f * PI * hz / rate;
    int done = 0;
    while (done < total) {
      int n = min((int)(sizeof(blk) / sizeof(blk[0])), total - done);
      for (int i = 0; i < n; i++) {
        blk[i] = (int16_t)(sinf(phase) * 32767.0f * vol);
        phase += inc;
        if (phase > 2 * PI) phase -= 2 * PI;
      }
      amp_.write((const uint8_t *)blk, n * sizeof(int16_t));
      done += n;
    }
  }
  void chirpUp()   { tone(660, 90); tone(990, 110); }
  void chirpDown() { tone(880, 90); tone(520, 120); }

 private:
  I2SClass mic_{I2S_NUM_0};
  I2SClass amp_{I2S_NUM_1};
  bool mic_ok_ = false, amp_ok_ = false;
};
