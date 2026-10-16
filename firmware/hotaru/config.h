// config.h -- HOTARU 3: pins, calibration defaults, voice and behaviour settings.
#pragma once

// ---------------------------------------------------------------- pins ----
// As on the "HOTARU 3 Wiring" page. I2C, button and servo channels match
// hotaru_motion.ino; mic, amp and LED ring are the suggested pins.
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_BTN       4      // Cherry MX to GND, internal pull-up
#define PIN_MIC_SCK   5      // INMP441 SCK
#define PIN_MIC_WS    6      // INMP441 WS
#define PIN_MIC_SD    7      // INMP441 SD (L/R tied to GND = left slot)
#define PIN_AMP_BCLK  15     // MAX98357A BCLK
#define PIN_AMP_LRC   16     // MAX98357A LRC
#define PIN_AMP_DIN   17     // MAX98357A DIN
#define PIN_LED       18     // WS2812 ring DIN
#define LED_COUNT     12

// ---------------------------------------------------------- calibration ----
// A servo angle 0..180 maps to SERVO_US_0..SERVO_US_180 microseconds.
#define SERVO_US_0    500
#define SERVO_US_180  2500

// Per joint, in SERVO degrees (the 0..180 sweep you calibrated with):
//   zero  servo angle with the joint at 0 (arm straight up, pan facing front,
//         head in line with the arm)
//   k     servo degrees per joint degree; the sign is the direction
//         (shoulder: 1.8 for the 30T pinion -> 54T gear)
//   smin, smax  the servo angles you measured as safe
// These are the first-boot defaults. Change them on the lamp with the serial
// "cal" commands; "cal save" keeps them in flash.
struct CalDefault { const char* name; uint8_t ch; float zero, k, smin, smax; };
static const CalDefault CAL_DEFAULT[4] = {
  //  name       ch  zero    k     smin  smax
  {"pan",      0,   90,  1.0f,    0, 180},
  {"shoulder", 1,    0,  1.8f,    0, 125},   // measured 2026-10-14: 0..125, 0 = upright
  {"elbow",    2,    0,  1.0f,    0, 120},   // measured 2026-10-14: 0..120, 0 = straight
  {"head",     3,   90,  1.0f,   15, 165},
};

// The CAD's joint limits (joint degrees). Never exceeded, whatever the
// calibration says: past these the arm hits its own parts.
static const float CAD_LO[4] = {-90, -30, -120, -75};
static const float CAD_HI[4] = { 90,  70,  120,  75};

// Speed caps at the joint (deg/s, deg/s^2). The shoulder carries the whole
// arm through a 1.8 : 1 gear, so it is the slow one. Same numbers as
// sw/h3_actions.py, which checked every gesture against them.
static const float VCAP[4] = {180,  60, 150,  300};
static const float ACAP[4] = {900, 240, 600, 2000};

// ---------------------------------------------------------------- voice ----
#define MIC_RATE       16000
#define MIC_SHIFT      14     // INMP441 32-bit slot >> this = 16-bit sample
#define REC_MAX_S      12     // longest press-to-talk recording
#define REC_MIN_MS     350    // shorter than this is a tap, not speech
#define SPK_RATE       16000
#define VOLUME_DEFAULT 0.6f

#define DG_STT_URL "https://api.deepgram.com/v1/listen?model=nova-3&smart_format=true" \
                   "&encoding=linear16&sample_rate=16000&channels=1&language=en&keyterm=Hotaru"
#define DG_TTS_VOICE "aura-2-thalia-en"
#define DG_TTS_URL "https://api.deepgram.com/v1/speak?model=" DG_TTS_VOICE \
                   "&encoding=linear16&sample_rate=16000&container=none"

#define OR_URL "https://openrouter.ai/api/v1/chat/completions"
// Free models, tried in this order (OpenRouter falls through to the next one
// when a model is down or rate-limited). All three answered in about a
// second when tested on 2026-10-14; free models come and go, so edit freely.
static const char* OR_MODELS[] = {
  "nvidia/nemotron-3.5-lightning:free",
  "nvidia/nemotron-3-super-120b-a12b:free",
  "google/gemma-4-26b-a4b-it:free",
};
#define HISTORY_TURNS  6      // exchanges the lamp remembers

// ------------------------------------------------------------ behaviour ----
#define TAP_MS          350   // a press shorter than this is a tap
#define IDLE_MIN_S      25    // random idle gesture every IDLE_MIN..IDLE_MAX s
#define IDLE_MAX_S      70
#define SLEEP_AFTER_S   600   // fold up and doze after 10 min alone
#define BOOT_GREETING   true  // say hello once the Wi-Fi is up
