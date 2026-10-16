// leds.h -- the WS2812 ring in the shade: HOTARU's eye and its lamp.
// Driven straight through the ESP32-S3's RMT peripheral (core 3.x rmtWrite),
// so no LED library is needed. Rendered at 50 Hz from the motion task.
#pragma once

enum Mood : uint8_t {
  MOOD_CALM, MOOD_HAPPY, MOOD_EXCITED, MOOD_SAD, MOOD_SHY, MOOD_CURIOUS, MOOD_ANGRY,
  MOOD_SLEEPY, MOOD_LOVE, MOOD_SURPRISED, MOOD_SCARED, MOOD_PROUD, MOOD_THINKING,
  MOOD_PLAYFUL, MOOD_COUNT
};
static const char* MOOD_NAMES[MOOD_COUNT] = {
  "calm", "happy", "excited", "sad", "shy", "curious", "angry", "sleepy", "love",
  "surprised", "scared", "proud", "thinking", "playful"
};
struct RGB { uint8_t r, g, b; };
static const RGB MOOD_RGB[MOOD_COUNT] = {
  {255, 170,  90},   // calm: warm white
  {255, 190,  20},   // happy: firefly yellow
  {255, 110,   0},   // excited: orange
  { 30,  80, 255},   // sad: blue
  {255,  70, 140},   // shy: pink
  {  0, 210, 200},   // curious: cyan
  {255,  20,   0},   // angry: red
  {120,  40, 200},   // sleepy: violet
  {255,  30,  90},   // love: rose
  {255, 255, 255},   // surprised: white
  { 90,  30, 160},   // scared: dim purple
  {255, 200,  60},   // proud: gold
  {255, 140,  30},   // thinking: amber
  { 80, 255,  80},   // playful: green
};

Mood moodFromName(const String& n, Mood dflt) {
  for (int i = 0; i < MOOD_COUNT; i++) if (n.equalsIgnoreCase(MOOD_NAMES[i])) return (Mood)i;
  return dflt;
}

enum LedMode : uint8_t {
  LED_OFF, LED_MOOD, LED_BREATHE, LED_LISTEN, LED_THINK, LED_TALK, LED_RAINBOW, LED_LAMP,
  LED_ERROR, LED_BOOT,
  // GHOST modes, driven by the hub (ghost.h)
  LED_FOLLOW,   // brightness follows a live signal: someone's breathing, sensed over Wi-Fi
  LED_PATH,     // night path light: a low warm glow that fades in
  LED_AMBER,    // something is unusual: slow amber pulse
  LED_RED       // alert: fast red pulse
};

volatile LedMode ledMode = LED_BOOT;
volatile Mood ledMood = MOOD_CALM;
volatile uint8_t eyeBright = 110;       // eye modes, 0..255
volatile uint8_t lampBright = 230;      // reading-light mode
volatile bool lampOn = false;           // the lamp is a lamp: stays on until told off
volatile float talkLevel = 0.0f;        // 0..1, set by the speaker while it plays
volatile float followLevel = 0.0f;      // 0..1, set by the GHOST hub (LED_FOLLOW)
volatile uint8_t pathBright = 90;       // LED_PATH target brightness
volatile uint32_t pathFadeMs = 4000;    // LED_PATH fade-in time
volatile uint32_t pathSince = 0;        // millis() when LED_PATH started

static rmt_data_t ledBuf[LED_COUNT * 24];
static bool ledOk = false;

void ledsBegin() {
  // 10 MHz: one tick = 100 ns
  ledOk = rmtInit(PIN_LED, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  if (!ledOk) Serial.println(F("*** LED ring: RMT init failed ***"));
}

static void ledsShow(const RGB* px) {
  if (!ledOk) return;
  int k = 0;
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t grb[3] = {px[i].g, px[i].r, px[i].b};          // WS2812 wants G, R, B
    for (int c = 0; c < 3; c++)
      for (int b = 7; b >= 0; b--) {
        bool one = grb[c] & (1 << b);
        ledBuf[k].level0 = 1;
        ledBuf[k].duration0 = one ? 8 : 4;                   // 0.8 / 0.4 us high
        ledBuf[k].level1 = 0;
        ledBuf[k].duration1 = one ? 5 : 9;                   // 0.5 / 0.9 us low
        k++;
      }
  }
  rmtWrite(PIN_LED, ledBuf, LED_COUNT * 24, RMT_WAIT_FOR_EVER);
}

static RGB scale(RGB c, float f) {
  f = constrain(f, 0.0f, 1.0f);
  f = f * f;                                                  // eyes see brightness squared
  return {(uint8_t)(c.r * f), (uint8_t)(c.g * f), (uint8_t)(c.b * f)};
}

static RGB wheel(uint8_t h) {
  if (h < 85) return {(uint8_t)(255 - h * 3), (uint8_t)(h * 3), 0};
  if (h < 170) { h -= 85; return {0, (uint8_t)(255 - h * 3), (uint8_t)(h * 3)}; }
  h -= 170; return {(uint8_t)(h * 3), 0, (uint8_t)(255 - h * 3)};
}

void ledsRender(uint32_t ms) {
  RGB px[LED_COUNT];
  float eb = eyeBright / 255.0f;
  RGB mood = MOOD_RGB[ledMood];
  LedMode m = ledMode;
  if (m == LED_MOOD && lampOn) m = LED_LAMP;
  for (int i = 0; i < LED_COUNT; i++) {
    switch (m) {
      case LED_OFF: px[i] = {0, 0, 0}; break;
      case LED_MOOD:                                          // a slow gentle breath
        px[i] = scale(mood, eb * (0.85f + 0.15f * sinf(ms * 0.0021f))); break;
      case LED_BREATHE:                                       // deep slow breath (asleep)
        px[i] = scale(mood, eb * (0.12f + 0.35f * (0.5f + 0.5f * sinf(ms * 0.0012f)))); break;
      case LED_LISTEN:
      case LED_THINK: {                                       // a comet going round
        RGB c = (m == LED_LISTEN) ? RGB{40, 120, 255} : MOOD_RGB[MOOD_THINKING];
        float speed = (m == LED_LISTEN) ? 0.010f : 0.016f;
        float head = fmodf(ms * speed, (float)LED_COUNT);
        float d = fmodf(head - i + LED_COUNT, (float)LED_COUNT);
        px[i] = scale(c, eb * max(0.08f, 1.0f - d / 4.0f));
        break;
      }
      case LED_TALK:
        px[i] = scale(mood, eb * (0.35f + 0.65f * talkLevel)); break;
      case LED_RAINBOW:
        px[i] = scale(wheel((uint8_t)(i * 256 / LED_COUNT + ms / 8)), eb); break;
      case LED_LAMP:
        px[i] = scale({255, 196, 128}, lampBright / 255.0f); break;
      case LED_ERROR:
        px[i] = ((ms / 250) % 2) ? scale({255, 0, 0}, 0.6f) : RGB{0, 0, 0}; break;
      case LED_FOLLOW: {                                     // the ring breathes with them
        float v = followLevel;
        px[i] = scale({150, 200, 255}, max(0.06f, eb * (0.15f + 0.85f * v)));
        break;
      }
      case LED_PATH: {                                       // fade in, then hold
        float t = pathFadeMs ? min(1.0f, (ms - pathSince) / (float)pathFadeMs) : 1.0f;
        px[i] = scale({255, 150, 60}, (pathBright / 255.0f) * t);
        break;
      }
      case LED_AMBER:
        px[i] = scale({255, 130, 0}, 0.25f + 0.55f * (0.5f + 0.5f * sinf(ms * 0.0042f))); break;
      case LED_RED:
        px[i] = scale({255, 10, 0}, 0.2f + 0.8f * (0.5f + 0.5f * sinf(ms * 0.0126f))); break;
      case LED_BOOT: {
        int at = (ms / 70) % LED_COUNT;
        px[i] = (i == at) ? scale({255, 255, 255}, 0.7f) : RGB{0, 0, 0};
        break;
      }
    }
  }
  ledsShow(px);
}
