/*
 * hotaru.ino -- HOTARU 3, the whole lamp: it listens, thinks, talks and moves.
 *
 * Board: ESP32-S3 dev board, esp32 core 3.x (tested against 3.3.x).
 *   Tools > PSRAM: "OPI PSRAM" (8 MB boards) -- recordings live there.
 *   Tools > Partition scheme: any with >= 1.5 MB app ("Huge APP" is fine).
 * No libraries to install: Wire, Preferences, WiFi, HTTPClient and I2S all
 * ship with the ESP32 core, and the LED ring is driven straight off RMT.
 *
 * Fill in secrets.h (Wi-Fi; the API keys are already in it).
 *
 * POWER: the servos need their own 5 V supply (3 A or more) on the PCA9685
 * V+ terminal. A laptop's USB port cannot start four MG996Rs: it cuts the
 * power and the board drops off USB. Hold the MX key while plugging in to
 * keep the servos off (testing voice and lights on USB only).
 *
 * TALK TO IT
 *   Hold the MX key, speak, let go. The ring spins blue while it listens,
 *   amber while it thinks, then it answers out loud and acts the answer out.
 *   A short tap plays a random gesture (or wakes it up when it's asleep).
 *   Left alone it fidgets now and then, and after 10 minutes it folds up and
 *   dozes until the key is pressed.
 *
 * HOW A TURN WORKS
 *   mic (I2S, 16 kHz) -> Deepgram nova-3 speech-to-text
 *   -> OpenRouter free model: {"say", "action", "mood", "light", "volume"}
 *   -> the action starts on the arm while Deepgram aura-2 speaks the words,
 *      streamed to the MAX98357A as they download.
 *   Every pose the arm takes passes the same pose guard and route planner as
 *   hotaru_motion.ino (holding torque, no part of the arm in the base), and
 *   all 60 actions were checked against them before they were written
 *   (sw/h3_actions.py).
 *
 * SERIAL @ 115200 (type ? for this list)
 *   <action>                 play an action: nod, shy, dance, ... (list)
 *   list                     every action
 *   ask <text>               talk to it in text (skips the microphone)
 *   say <text>               speak, nothing else
 *   rec                      record 4 s and print what Deepgram heard
 *   p <pan> <sh> <el> <hd>   move to a pose, joint degrees
 *   j <i> <deg>              one joint (0 pan, 1 shoulder, 2 elbow, 3 head)
 *   g <pan> <sh> <el> <hd>   ask the guard about a pose, don't move
 *   stop | off | on          stop / release every servo / engage again
 *   v <x>                    speed scale, 1 = normal
 *   vol <0..1>  bright <0..255>  lamp on|off  mood <name>
 *   cal                      show the calibration
 *   servo <i> <deg>          raw servo angle, 0..180 (releases that joint)
 *   cal <i> zero|min|max|k <value>   set one calibration number
 *   cal save | cal reset     keep it in flash / back to config.h
 *   wifi | forget            Wi-Fi status / clear the conversation
 */
#include "config.h"
#include "secrets.h"
#include "leds.h"
#include "motion.h"
#include "audio.h"
#include "net.h"
#include "brain.h"
#include "ghost.h"

enum State : uint8_t { S_IDLE, S_LISTEN, S_BUSY, S_SLEEP };
State st = S_IDLE;
bool online = false;
Mood mood = MOOD_CALM;
uint32_t lastTouch = 0, nextIdleAt = 0;

static void scheduleIdle() {
  nextIdleAt = millis() + 1000UL * (IDLE_MIN_S + esp_random() % (IDLE_MAX_S - IDLE_MIN_S + 1));
}
static void touch() { lastTouch = millis(); scheduleIdle(); }

static void setMood(Mood m) { mood = m; ledMood = m; }
static const char* stateName() {
  switch (st) { case S_LISTEN: return "listen"; case S_BUSY: return "busy"; case S_SLEEP: return "sleep"; default: return "idle"; }
}
static void eyes(LedMode m) { ledMode = m; }

static void waitMotion(uint32_t maxMs) {
  uint32_t t0 = millis();
  delay(60);                                    // let the task pick the command up
  while (!motionIdle && millis() - t0 < maxMs) delay(20);
}

// --------------------------------------------------------------- speech ----
static void speak(const String& text) {
  if (!text.length()) return;
  Serial.printf("HOTARU: %s\n", text.c_str());
  String err;
  talking = true;
  eyes(LED_TALK);
  bool ok = ttsSpeak(text, err);
  talking = false;
  eyes(LED_MOOD);
  if (!ok) { Serial.printf("  (speech failed: %s)\n", err.c_str()); beep(330, 160); }
}

static void apply(const Reply& r) {
  if (r.light == "on") lampOn = true;
  else if (r.light == "off") lampOn = false;
  else if (r.light == "brighter") { lampOn = true; lampBright = min(255, lampBright + 50); eyeBright = min(255, eyeBright + 40); }
  else if (r.light == "dimmer") { lampBright = max(40, lampBright - 50); eyeBright = max(25, eyeBright - 30); }
  if (r.volume == "up") volume = min(1.0f, volume + 0.15f);
  else if (r.volume == "down") volume = max(0.1f, volume - 0.15f);

  int a = findAction(r.action);
  if (a < 0 || !ACTIONS[a].pub) a = findAction("talk");
  setMood(moodFromName(r.mood, ACTIONS[a].mood));
  if (r.action == "celebrate" || r.action == "dance") eyes(LED_RAINBOW);
  playAction(a);
}

// One full turn: words in, words + motion out.
static void respond(const String& heard) {
  Serial.printf("you: %s\n", heard.c_str());
  st = S_BUSY;
  eyes(LED_THINK);
  playAction("hmm");
  Reply r;
  String err;
  if (!think(heard, r, err)) {
    Serial.printf("  (brain: %s)\n", err.c_str());
    setMood(MOOD_SAD);
    eyes(LED_MOOD);
    playAction("confused");
    speak("Sorry, my brain went fuzzy for a second. Try again?");
  } else {
    Serial.printf("  [%s] action=%s mood=%s light=%s\n", r.model.c_str(), r.action.c_str(),
                  r.mood.c_str(), r.light.c_str());
    apply(r);
    speak(r.say);
    if (r.action == "sleep") { waitMotion(6000); st = S_SLEEP; eyes(LED_BREATHE); touch(); return; }
  }
  waitMotion(8000);
  eyes(LED_MOOD);
  st = S_IDLE;
  touch();
}

// --------------------------------------------------------------- listen ----
static void listenAndRespond(uint32_t fixedMs = 0) {
  if (!online) { beep(330, 200); Serial.println(F("offline: fill in Wi-Fi in secrets.h")); return; }
  st = S_LISTEN;
  eyes(LED_LISTEN);
  playAction("listen");
  beep(880, 60, 0.15f);
  recStart();
  uint32_t t0 = millis();
  while (recPoll()) {
    if (fixedMs ? millis() - t0 >= fixedMs : digitalRead(PIN_BTN) == HIGH) break;
  }
  beep(660, 50, 0.12f);
  uint32_t ms = recLen * 1000UL / MIC_RATE;
  if (ms < REC_MIN_MS) { st = S_IDLE; eyes(LED_MOOD); touch(); return; }
  eyes(LED_THINK);
  recNormalize();
  String err;
  uint32_t t1 = millis();
  String heard = sttTranscribe(recBuf, recLen, err);
  Serial.printf("  (%.1f s of speech, heard in %lu ms)\n", ms / 1000.0f, millis() - t1);
  if (!heard.length()) {
    Serial.printf("  (heard nothing%s%s)\n", err.length() ? ": " : "", err.c_str());
    playAction("confused");
    speak(err.length() ? "I can't reach my ears right now." : "Hmm? I didn't catch that.");
    eyes(LED_MOOD);
    st = S_IDLE;
    touch();
    return;
  }
  respond(heard);
}

// --------------------------------------------------------------- button ----
static const char* TAP_ACTIONS[] = {"happy", "wave", "dance", "peek", "curious", "laugh",
                                    "nod", "shy", "bounce", "tada", "peekaboo", "dizzy"};

static void buttonPoll() {
  static bool down = false;
  static uint32_t t0 = 0;
  bool pressed = digitalRead(PIN_BTN) == LOW;
  if (pressed && !down) { down = true; t0 = millis(); }
  if (!down) return;
  if (pressed && millis() - t0 > TAP_MS) {      // a hold: talk
    down = false;
    touch();
    ghostEvent("key", "\"kind\":\"hold\"");
    if (ghostCheckin) {                          // GHOST asked "are you OK?": the answer goes to the hub
      st = S_LISTEN;
      ghostListen(0);
      st = S_IDLE;
      while (digitalRead(PIN_BTN) == LOW) delay(10);
      return;
    }
    if (st == S_SLEEP) { setMood(MOOD_HAPPY); playAction("wake_up"); waitMotion(5000); }
    listenAndRespond();
    while (digitalRead(PIN_BTN) == LOW) delay(10);
    return;
  }
  if (!pressed) {                               // a tap
    down = false;
    if (millis() - t0 < 30) return;             // bounce
    touch();
    if (ghostCheckin) {                          // a tap during a GHOST check-in: "I'm OK"
      ghostCheckin = false;
      ghostEvent("ok");
      if (st == S_SLEEP) st = S_IDLE;
      setMood(MOOD_HAPPY);
      eyes(LED_MOOD);
      playAction("nod");
      return;
    }
    ghostEvent("key", "\"kind\":\"tap\"");
    if (st == S_SLEEP) {
      st = S_IDLE; setMood(MOOD_HAPPY); eyes(LED_MOOD); playAction("wake_up");
      if (online) speak("Mm, I'm awake!");
      return;
    }
    const char* a = TAP_ACTIONS[esp_random() % (sizeof(TAP_ACTIONS) / sizeof(TAP_ACTIONS[0]))];
    setMood(ACTIONS[findAction(a)].mood);
    playAction(a);
  }
}

// ----------------------------------------------------------------- idle ----
static const char* IDLE_ACTIONS[] = {"idle", "idle", "scan", "look_left", "look_right",
                                     "look_up", "hmm", "stretch", "sigh", "curious", "stand"};

static void idlePoll() {
  if (st != S_IDLE || !motionIdle) return;
  if (ledMode >= LED_FOLLOW) return;          // GHOST is showing something: no fidgets, no dozing
  uint32_t now = millis();
  if (now - lastTouch > SLEEP_AFTER_S * 1000UL) {
    st = S_SLEEP;
    setMood(MOOD_SLEEPY);
    eyes(LED_BREATHE);
    lampOn = false;
    playAction("sleep");
    return;
  }
  if ((int32_t)(now - nextIdleAt) >= 0) {
    const char* a = IDLE_ACTIONS[esp_random() % (sizeof(IDLE_ACTIONS) / sizeof(IDLE_ACTIONS[0]))];
    playAction(a);
    scheduleIdle();
  }
}

// --------------------------------------------------------------- serial ----
static int jointIndex(const String& s) {
  for (int i = 0; i < 4; i++) if (s.equalsIgnoreCase(J[i].name)) return i;
  int i = s.toInt();
  return (s.length() && isdigit((uint8_t)s[0]) && i >= 0 && i < 4) ? i : -1;
}

static void help() {
  motionState();
  Serial.printf("Wi-Fi %s  volume %.2f  eye %d  lamp %s (%d)  mood %s\n",
                online ? "up" : "DOWN", volume, eyeBright, lampOn ? "on" : "off", lampBright,
                MOOD_NAMES[mood]);
  Serial.println(F("<action> | list | ask <text> | say <text> | rec | stop | off | on"));
  Serial.println(F("p <pan> <sh> <el> <hd> | j <i> <deg> | g <pose> | v <x> | vol <0..1> | bright <n>"));
  Serial.println(F("lamp on|off | mood <name> | cal | cal <i> zero|min|max|k <v> | cal save | cal reset"));
  Serial.println(F("servo <i> <deg> | wifi | forget"));
}

static void serialPoll() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  touch();
  if (st == S_SLEEP) { st = S_IDLE; eyes(LED_MOOD); }

  if (cmd.startsWith("ask ")) { if (online) respond(cmd.substring(4)); else Serial.println(F("offline")); }
  else if (cmd.startsWith("say ")) speak(cmd.substring(4));
  else if (cmd == "rec") {
    eyes(LED_LISTEN); recStart();
    uint32_t t0 = millis();
    while (recPoll() && millis() - t0 < 4000) {}
    eyes(LED_MOOD); recNormalize();
    String err, t = sttTranscribe(recBuf, recLen, err);
    Serial.printf("heard: \"%s\" %s\n", t.c_str(), err.c_str());
  }
  else if (cmd == "listen") listenAndRespond(4000);
  else if (cmd == "list") {
    for (int i = 0; i < N_ACTIONS; i++)
      Serial.printf("  %-14s %-10s %s%s\n", ACTIONS[i].name, MOOD_NAMES[ACTIONS[i].mood],
                    ACTIONS[i].desc, ACTIONS[i].pub ? "" : "  (internal)");
  }
  else if (cmd.startsWith("p ")) {
    float a[4];
    if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &a[0], &a[1], &a[2], &a[3]) == 4) movePose(a);
  }
  else if (cmd.startsWith("j ")) {
    int i; float d;
    if (sscanf(cmd.c_str() + 2, "%d %f", &i, &d) == 2 && i >= 0 && i < 4) {
      Cmd c{C_JOINT, (int16_t)i, {d, 0, 0, 0}, 0};
      motionSend(c);
    }
  }
  else if (cmd.startsWith("g ")) {
    float a[4];
    if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &a[0], &a[1], &a[2], &a[3]) == 4) {
      float cur[4], legs[3][4];
      for (int i = 0; i < 4; i++) cur[i] = J[i].pos;
      Serial.printf("guard %d  shoulder servo %.3f N.m (max %.3f)  clearance %.1f mm  route %d legs\n",
                    guard(a), fabsf(shoulderTorque(a[1], a[2], a[3])) / GEAR, TAU_MAX,
                    clearance(a[0], a[1], a[2], a[3]), plan(cur, a, legs));
      for (int i = 0; i < 4; i++)
        if (a[i] < J[i].lo || a[i] > J[i].hi)
          Serial.printf("  %s %.0f is outside the calibrated %.0f..%.0f (would be clamped)\n",
                        J[i].name, a[i], J[i].lo, J[i].hi);
    }
  }
  else if (cmd == "stop") { Cmd c{C_STOP, 0, {0, 0, 0, 0}, 0}; motionSend(c); }
  else if (cmd == "off")  { Cmd c{C_RELEASE, 0, {0, 0, 0, 0}, 0}; motionSend(c); }
  else if (cmd == "on")   { Cmd c{C_ENGAGE, 0, {0, 0, 0, 0}, 0}; motionSend(c); }
  else if (cmd.startsWith("v ")) speedScale = constrain(cmd.substring(2).toFloat(), 0.2f, 1.5f);
  else if (cmd.startsWith("vol ")) volume = constrain(cmd.substring(4).toFloat(), 0.0f, 1.0f);
  else if (cmd.startsWith("bright ")) eyeBright = constrain(cmd.substring(7).toInt(), 0, 255);
  else if (cmd == "lamp on") lampOn = true;
  else if (cmd == "lamp off") lampOn = false;
  else if (cmd.startsWith("mood ")) setMood(moodFromName(cmd.substring(5), mood));
  else if (cmd.startsWith("servo ")) {
    char name[16]; float deg;
    if (sscanf(cmd.c_str() + 6, "%15s %f", name, &deg) == 2) {
      int i = jointIndex(name);
      if (i >= 0) {
        Cmd c{C_SERVO, (int16_t)i, {constrain(deg, 0.0f, 180.0f), 0, 0, 0}, 0};
        motionSend(c);
        Serial.printf("%s servo -> %.0f deg (%.0f us), joint released; 'on' to re-engage\n",
                      J[i].name, deg, servoUs(deg));
      }
    }
  }
  else if (cmd == "cal") calPrint();
  else if (cmd == "cal save") Serial.println(calSave() ? F("calibration saved") : F("save failed"));
  else if (cmd == "cal reset") { calClear(); calPrint(); }
  else if (cmd.startsWith("cal ")) {
    char name[16], field[8]; float v;
    if (sscanf(cmd.c_str() + 4, "%15s %7s %f", name, field, &v) == 3) {
      int i = jointIndex(name);
      String f(field);
      if (i < 0) Serial.println(F("which joint? 0..3 or pan/shoulder/elbow/head"));
      else {
        if (f == "zero") J[i].zero = v;
        else if (f == "min") J[i].smin = constrain(v, 0.0f, 180.0f);
        else if (f == "max") J[i].smax = constrain(v, 0.0f, 180.0f);
        else if (f == "k") J[i].k = v;
        calDerive(i);
        calPrint();
        Serial.println(F("('cal save' to keep it after a restart)"));
      }
    }
  }
  else if (cmd == "wifi") {
    Serial.printf("Wi-Fi %s  %s  %d dBm\n", WiFi.status() == WL_CONNECTED ? "up" : "down",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
  else if (cmd == "forget") { histClear(); Serial.println(F("conversation cleared")); }
  else if (findAction(cmd) >= 0) {
    int a = findAction(cmd);
    setMood(ACTIONS[a].mood);
    playAction(a);
    Serial.printf("-> %s\n", ACTIONS[a].name);
  }
  else help();
}

// ---------------------------------------------------------------- main ----
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nHOTARU 3 -- waking up"));
  pinMode(PIN_BTN, INPUT_PULLUP);
  ledsBegin();
  // Hold the MX key while powering up to keep the servos OFF: for testing on a
  // laptop's USB, which cannot feed four MG996Rs (the port shuts down).
  bool servosOn = digitalRead(PIN_BTN) == HIGH;
  motionBegin(servosOn);            // engages the servos one at a time, at rest
  audioBegin();
  brainBegin();
  online = wifiBegin(15000);
  if (online) ghostBegin();
  setMood(online ? MOOD_HAPPY : MOOD_CALM);
  eyes(online ? LED_MOOD : LED_ERROR);
  if (servosOn) { playAction("wake_up"); waitMotion(6000); }
  if (!online) { delay(1500); eyes(LED_MOOD); }
  else if (BOOT_GREETING) speak("Hi! I'm Hotaru. Hold my button and talk to me.");
  touch();
  help();
}

void loop() {
  serialPoll();
  buttonPoll();
  idlePoll();
  ghostPoll();
  delay(5);
}
