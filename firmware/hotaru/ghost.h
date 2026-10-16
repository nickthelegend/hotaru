// ghost.h -- the GHOST link: HOTARU as the body of a camera-less home guardian.
//
// A GHOST hub (a laptop or server on the same network) senses the home through
// Wi-Fi and decides what the lamp should do. The lamp keeps a TCP connection
// to the hub and speaks newline-delimited JSON both ways. No libraries: it is
// WiFiClient plus the small JSON helpers in net.h.
//
// hub -> lamp, one object per line, "op" says what:
//   {"op":"action","name":"nod","mood":"happy"}        play one of the 60 actions
//   {"op":"look","pan":-40}                             turn the base, keep the rest
//   {"op":"pose","a":[pan,sh,el,hd]}                    move to a pose, joint degrees
//   {"op":"eye","mode":"follow|path|amber|red|mood|lamp|breathe|listen|think|off",
//    "mood":"calm","bright":110,"fade_ms":4000}         ring mode
//   {"op":"level","v":0.42}                             live 0..1 signal for "follow"
//   {"op":"checkin","on":true}                          a tap now means "I'm OK"
//   {"op":"say","text":"Are you okay?"}                 speak (Deepgram aura-2)
//   {"op":"listen","ms":5000}                           record, transcribe, send it back
//   {"op":"lamp","on":true,"bright":200}  {"op":"volume","v":0.8}
//   {"op":"stop"}  {"op":"ping"}
//
// lamp -> hub:
//   {"ev":"hello","id":"hotaru","fw":"hotaru3-ghost","ip":"...","rssi":-51,"csi":true}
//   {"ev":"key","kind":"tap|hold"}   {"ev":"ok"}  (tap during a check-in)
//   {"ev":"heard","text":"I'm fine","ms":3200}   {"ev":"said","text":"..."}
//   {"ev":"status","state":"idle","rssi":-52,"heap":123456,"csi_hz":98}
//   {"ev":"pong"}  {"ev":"error","msg":"..."}
//
// Wi-Fi sensing (GHOST_CSI): the lamp pings its router ~100 times a second and
// records the channel state information (CSI) of every reply -- 64 subcarrier
// amplitudes that change when someone moves or breathes between the router and
// the lamp. Batches go to the hub over UDP (port GHOST_HUB_PORT + 1):
//   "GC1" | u8 count | count x { u32 ms | i8 rssi | u8 n | n x u8 amplitude }
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef GHOST_HUB_HOST
#define GHOST_HUB_HOST ""            // set in secrets.h to turn the link on
#endif
#ifndef GHOST_HUB_PORT
#define GHOST_HUB_PORT 8770
#endif
#ifndef GHOST_CSI
#define GHOST_CSI 1                  // 0 to build without Wi-Fi sensing
#endif
#ifndef GHOST_CSI_HZ
#define GHOST_CSI_HZ 100
#endif

// defined in hotaru.ino
static void speak(const String& text);
static void setMood(Mood m);
static void eyes(LedMode m);
static const char* stateName();

static WiFiClient ghostHub;
static String ghostLine;
static uint32_t ghostRetryAt = 0, ghostStatusAt = 0, ghostLastRx = 0;
volatile bool ghostCheckin = false;       // hub is waiting for "are you OK?"
static bool ghostWasUp = false;

bool ghostEnabled() { return String(GHOST_HUB_HOST).length() > 0; }
bool ghostUp() { return ghostHub.connected(); }

void ghostSend(const String& json) {
  if (!ghostHub.connected()) return;
  ghostHub.print(json);
  ghostHub.print('\n');
}

static void ghostEvent(const char* ev, const String& extra = String()) {
  String s = String("{\"ev\":\"") + ev + '"';
  if (extra.length()) { s += ','; s += extra; }
  s += '}';
  ghostSend(s);
}

// --------------------------------------------------- a little more JSON ----
// Number at "key"; dflt if missing. Also reads true/false as 1/0.
static float jsonNum(const String& s, const char* key, float dflt) {
  String k = String('"') + key + '"';
  int i = s.indexOf(k);
  if (i < 0) return dflt;
  int j = s.indexOf(':', i + k.length());
  if (j < 0) return dflt;
  j++;
  while (j < (int)s.length() && isspace((uint8_t)s[j])) j++;
  if (s.startsWith("true", j)) return 1;
  if (s.startsWith("false", j)) return 0;
  char* end;
  float v = strtof(s.c_str() + j, &end);
  return end == s.c_str() + j ? dflt : v;
}

// "a":[n,n,n,n] -> out; false unless all four are there
static bool jsonArr4(const String& s, const char* key, float* out) {
  int i = s.indexOf(String('"') + key + '"');
  if (i < 0) return false;
  int j = s.indexOf('[', i);
  if (j < 0) return false;
  const char* p = s.c_str() + j + 1;
  for (int n = 0; n < 4; n++) {
    char* end;
    out[n] = strtof(p, &end);
    if (end == p) return false;
    p = end;
    while (*p == ' ' || *p == ',') p++;
  }
  return true;
}

// ------------------------------------------------------------- Wi-Fi CSI ----
#if GHOST_CSI
#include "esp_wifi.h"
#include "ping/ping_sock.h"

struct CsiRec { uint32_t ms; int8_t rssi; uint8_t n; uint8_t amp[64]; };
static QueueHandle_t csiQ = nullptr;
static uint8_t csiAp[6];
static WiFiUDP csiUdp;
static esp_ping_handle_t csiPing = nullptr;
static volatile uint32_t csiCount = 0;
static uint32_t csiRateAt = 0, csiRateN = 0, csiHz = 0;
static bool csiOn = false;

// Runs in the Wi-Fi task: keep it short, copy and queue.
static void csiCb(void*, wifi_csi_info_t* info) {
  if (!info || !info->buf || memcmp(info->mac, csiAp, 6) != 0) return;
  CsiRec r;
  r.ms = millis();
  r.rssi = info->rx_ctrl.rssi;
  int n = min(64, (int)(info->len / 2));
  r.n = n;
  const int8_t* b = info->buf;
  for (int k = 0; k < n; k++) {              // pairs of (imaginary, real)
    float im = b[2 * k], re = b[2 * k + 1];
    r.amp[k] = (uint8_t)min(255.0f, sqrtf(re * re + im * im));
  }
  csiCount++;
  xQueueSend(csiQ, &r, 0);                   // drop if the main loop is busy
}

static bool csiBegin() {
  if (!csiQ) csiQ = xQueueCreate(64, sizeof(CsiRec));
  memcpy(csiAp, WiFi.BSSID(), 6);
  wifi_csi_config_t c = {};
  c.lltf_en = true;
  c.htltf_en = false;
  c.stbc_htltf2_en = false;
  c.ltf_merge_en = true;
  c.channel_filter_en = false;
  c.manu_scale = false;
  c.shift = 0;
  if (esp_wifi_set_csi_config(&c) != ESP_OK || esp_wifi_set_csi_rx_cb(csiCb, nullptr) != ESP_OK ||
      esp_wifi_set_csi(true) != ESP_OK) {
    Serial.println(F("GHOST: CSI not available in this core build"));
    return false;
  }
  // Every ping reply comes from the router: GHOST_CSI_HZ channel snapshots a second.
  esp_ping_config_t pc = ESP_PING_DEFAULT_CONFIG();
  IPAddress gw = WiFi.gatewayIP();
  IP_ADDR4(&pc.target_addr, gw[0], gw[1], gw[2], gw[3]);
  pc.count = ESP_PING_COUNT_INFINITE;
  pc.interval_ms = 1000 / GHOST_CSI_HZ;
  pc.timeout_ms = 500;
  pc.data_size = 8;
  esp_ping_callbacks_t cb = {};
  if (esp_ping_new_session(&pc, &cb, &csiPing) != ESP_OK || esp_ping_start(csiPing) != ESP_OK) {
    Serial.println(F("GHOST: could not start the CSI ping"));
    return false;
  }
  csiUdp.begin(GHOST_HUB_PORT + 2);          // local port, only used to send
  Serial.printf("GHOST: Wi-Fi sensing on, %d Hz from router %02x:%02x:%02x:%02x:%02x:%02x\n",
                GHOST_CSI_HZ, csiAp[0], csiAp[1], csiAp[2], csiAp[3], csiAp[4], csiAp[5]);
  return true;
}

static void csiPump() {
  if (!csiOn) return;
  uint32_t now = millis();
  if (now - csiRateAt >= 2000) {
    uint32_t c = csiCount;
    csiHz = (c - csiRateN) * 1000 / max(1UL, (unsigned long)(now - csiRateAt));
    csiRateN = c;
    csiRateAt = now;
  }
  static uint8_t pkt[4 + 8 * (6 + 64)];
  CsiRec r;
  int count = 0, at = 4;
  while (count < 8 && xQueueReceive(csiQ, &r, 0) == pdTRUE) {
    memcpy(pkt + at, &r.ms, 4); at += 4;
    pkt[at++] = (uint8_t)r.rssi;
    pkt[at++] = r.n;
    memcpy(pkt + at, r.amp, r.n); at += r.n;
    count++;
  }
  if (!count || !ghostEnabled()) return;
  pkt[0] = 'G'; pkt[1] = 'C'; pkt[2] = '1'; pkt[3] = count;
  csiUdp.beginPacket(GHOST_HUB_HOST, GHOST_HUB_PORT + 1);
  csiUdp.write(pkt, at);
  csiUdp.endPacket();
}
#endif

// ----------------------------------------------------------- the commands ----
static LedMode ledFromName(const String& m, LedMode dflt) {
  if (m == "follow") return LED_FOLLOW;
  if (m == "path") return LED_PATH;
  if (m == "amber") return LED_AMBER;
  if (m == "red") return LED_RED;
  if (m == "mood") return LED_MOOD;
  if (m == "lamp") return LED_LAMP;
  if (m == "breathe") return LED_BREATHE;
  if (m == "listen") return LED_LISTEN;
  if (m == "think") return LED_THINK;
  if (m == "rainbow") return LED_RAINBOW;
  if (m == "off") return LED_OFF;
  return dflt;
}

// Record for ms (or while the key is held when ms == 0), transcribe, send.
void ghostListen(uint32_t ms) {
  LedMode was = ledMode;
  eyes(LED_LISTEN);
  beep(880, 60, 0.15f);
  recStart();
  uint32_t t0 = millis();
  while (recPoll()) {
    if (ms ? millis() - t0 >= ms : digitalRead(PIN_BTN) == HIGH) break;
  }
  beep(660, 50, 0.12f);
  eyes(LED_THINK);
  recNormalize();
  uint32_t got = recLen * 1000UL / MIC_RATE;
  String err, text;
  if (got >= REC_MIN_MS) text = sttTranscribe(recBuf, recLen, err);
  eyes(was);
  String extra = "\"text\":\"" + jsonEsc(text) + "\",\"ms\":" + String(got);
  if (err.length()) extra += ",\"error\":\"" + jsonEsc(err) + "\"";
  ghostEvent("heard", extra);
  Serial.printf("GHOST heard: \"%s\" %s\n", text.c_str(), err.c_str());
}

static void ghostHandle(const String& line) {
  String op = jsonStr(line, "op");
  if (op == "ping") { ghostEvent("pong"); return; }
  if (op == "action") {
    String name = jsonStr(line, "name");
    int a = findAction(name);
    if (a < 0) { ghostEvent("error", "\"msg\":\"no action " + jsonEsc(name) + "\""); return; }
    String m = jsonStr(line, "mood");
    setMood(m.length() ? moodFromName(m, ACTIONS[a].mood) : ACTIONS[a].mood);
    playAction(a);
  } else if (op == "look") {
    Cmd c{C_JOINT, 0, {jsonNum(line, "pan", 0), 0, 0, 0}, 0};
    motionSend(c);
  } else if (op == "pose") {
    float a[4];
    if (jsonArr4(line, "a", a)) movePose(a);
  } else if (op == "eye") {
    String m = jsonStr(line, "mode");
    LedMode lm = ledFromName(m, ledMode);
    String mood = jsonStr(line, "mood");
    if (mood.length()) setMood(moodFromName(mood, ledMood));
    float b = jsonNum(line, "bright", -1);
    if (lm == LED_PATH) {
      if (b >= 0) pathBright = constrain((int)b, 0, 255);
      pathFadeMs = (uint32_t)jsonNum(line, "fade_ms", 4000);
      pathSince = millis();
    } else if (b >= 0) eyeBright = constrain((int)b, 0, 255);
    eyes(lm);
  } else if (op == "level") {
    followLevel = constrain(jsonNum(line, "v", 0), 0.0f, 1.0f);
  } else if (op == "checkin") {
    ghostCheckin = jsonNum(line, "on", 1) != 0;
  } else if (op == "say") {
    String t = jsonStr(line, "text");
    LedMode was = ledMode;
    speak(t);
    eyes(was);                                 // keep an alert colour through the speech
    ghostEvent("said", "\"text\":\"" + jsonEsc(t) + "\"");
  } else if (op == "listen") {
    ghostListen((uint32_t)constrain(jsonNum(line, "ms", 5000), 500.0f, (float)(REC_MAX_S * 1000)));
  } else if (op == "lamp") {
    lampOn = jsonNum(line, "on", 1) != 0;
    float b = jsonNum(line, "bright", -1);
    if (b >= 0) lampBright = constrain((int)b, 0, 255);
  } else if (op == "volume") {
    volume = constrain(jsonNum(line, "v", volume), 0.0f, 1.0f);
  } else if (op == "stop") {
    Cmd c{C_STOP, 0, {0, 0, 0, 0}, 0};
    motionSend(c);
  } else {
    ghostEvent("error", "\"msg\":\"unknown op " + jsonEsc(op) + "\"");
  }
}

// -------------------------------------------------------- setup and loop ----
void ghostBegin() {
  if (!ghostEnabled()) {
    Serial.println(F("GHOST: no hub set (GHOST_HUB_HOST in secrets.h), link off"));
    return;
  }
  Serial.printf("GHOST: hub %s:%d\n", GHOST_HUB_HOST, GHOST_HUB_PORT);
#if GHOST_CSI
  if (WiFi.status() == WL_CONNECTED) csiOn = csiBegin();
#endif
}

static void ghostHello() {
  String extra = "\"id\":\"hotaru\",\"fw\":\"hotaru3-ghost\",\"ip\":\"" + WiFi.localIP().toString() +
                 "\",\"rssi\":" + String(WiFi.RSSI()) + ",\"actions\":" + String(N_ACTIONS) +
#if GHOST_CSI
                 ",\"csi\":" + String(csiOn ? "true" : "false");
#else
                 ",\"csi\":false";
#endif
  ghostEvent("hello", extra);
}

void ghostPoll() {
  if (!ghostEnabled() || WiFi.status() != WL_CONNECTED) return;
#if GHOST_CSI
  if (!csiOn && !csiQ) csiOn = csiBegin();     // Wi-Fi came up after boot
  csiPump();
#endif
  uint32_t now = millis();
  if (!ghostHub.connected()) {
    if (ghostWasUp) { ghostWasUp = false; ghostCheckin = false; Serial.println(F("GHOST: hub lost")); }
    if ((int32_t)(now - ghostRetryAt) < 0) return;
    ghostRetryAt = now + 5000;
    if (!ghostHub.connect(GHOST_HUB_HOST, GHOST_HUB_PORT, 400)) return;
    ghostHub.setNoDelay(true);
    ghostWasUp = true;
    ghostLine = "";
    ghostLastRx = now;
    Serial.println(F("GHOST: hub connected"));
    ghostHello();
  }
  // Read what's waiting, a bounded amount per loop so buttons stay responsive.
  int budget = 2048;
  while (ghostHub.available() && budget-- > 0) {
    char c = ghostHub.read();
    if (c == '\n') {
      ghostLine.trim();
      if (ghostLine.length()) { ghostLastRx = millis(); ghostHandle(ghostLine); }
      ghostLine = "";
    } else if (ghostLine.length() < 4096) {
      ghostLine += c;
    }
  }
  if ((int32_t)(now - ghostStatusAt) >= 0) {
    ghostStatusAt = now + 5000;
    String extra = "\"state\":\"" + String(stateName()) + "\",\"rssi\":" + String(WiFi.RSSI()) +
                   ",\"heap\":" + String(ESP.getFreeHeap()) + ",\"checkin\":" + (ghostCheckin ? "true" : "false");
#if GHOST_CSI
    extra += ",\"csi_hz\":" + String(csiHz);
#endif
    ghostEvent("status", extra);
  }
}
