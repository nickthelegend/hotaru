// net.h -- Wi-Fi, a little JSON, and the three cloud calls:
//   Deepgram speech-to-text  (nova-3)       recording -> words
//   OpenRouter chat          (free models)   words -> reply + action
//   Deepgram text-to-speech  (aura-2)       reply -> speaker, streamed
// HTTPS is encrypted but the server certificate is not checked
// (setInsecure): fine on your own Wi-Fi, not on an untrusted network.
#pragma once
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

bool wifiBegin(uint32_t timeoutMs) {
  if (String(WIFI_SSID) == "YOUR_WIFI_NAME") {
    Serial.println(F("*** Wi-Fi: put your network in secrets.h ***"));
    return false;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // sleeping radio = laggy replies
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi: %s  %s  %d dBm\n", WIFI_SSID, WiFi.localIP().toString().c_str(),
                  WiFi.RSSI());
    return true;
  }
  Serial.println(F("*** Wi-Fi: could not connect ***"));
  return false;
}

bool wifiUp() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.reconnect();
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) delay(200);
  return WiFi.status() == WL_CONNECTED;
}

// ----------------------------------------------------------------- JSON ----
String jsonEsc(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':  o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += c;
    }
  }
  return o;
}

static void utf8(String& o, uint32_t cp) {
  if (cp < 0x80) o += (char)cp;
  else if (cp < 0x800) { o += (char)(0xC0 | (cp >> 6)); o += (char)(0x80 | (cp & 0x3F)); }
  else if (cp < 0x10000) {
    o += (char)(0xE0 | (cp >> 12)); o += (char)(0x80 | ((cp >> 6) & 0x3F));
    o += (char)(0x80 | (cp & 0x3F));
  } else {
    o += (char)(0xF0 | (cp >> 18)); o += (char)(0x80 | ((cp >> 12) & 0x3F));
    o += (char)(0x80 | ((cp >> 6) & 0x3F)); o += (char)(0x80 | (cp & 0x3F));
  }
}

// The string value of the first "key" at or after `from`; "" if absent or
// not a string. Enough JSON for the three APIs and the model's reply.
String jsonStr(const String& s, const char* key, int from = 0, bool* found = nullptr) {
  String k = String('"') + key + '"';
  int n = s.length();
  if (found) *found = false;
  for (int i = s.indexOf(k, from); i >= 0; i = s.indexOf(k, i + 1)) {
    int j = i + k.length();
    while (j < n && isspace((uint8_t)s[j])) j++;
    if (j >= n || s[j] != ':') continue;
    j++;
    while (j < n && isspace((uint8_t)s[j])) j++;
    if (j >= n || s[j] != '"') return String();
    String o;
    for (j++; j < n && s[j] != '"'; j++) {
      char c = s[j];
      if (c != '\\' || j + 1 >= n) { o += c; continue; }
      char e = s[++j];
      if (e == 'n') o += '\n';
      else if (e == 't') o += '\t';
      else if (e == 'r') o += '\r';
      else if (e == 'b' || e == 'f') o += ' ';
      else if (e == 'u' && j + 4 < n) {
        uint32_t cp = strtoul(s.substring(j + 1, j + 5).c_str(), nullptr, 16);
        j += 4;
        if (cp >= 0xD800 && cp < 0xDC00 && j + 6 < n && s[j + 1] == '\\' && s[j + 2] == 'u') {
          uint32_t lo = strtoul(s.substring(j + 3, j + 7).c_str(), nullptr, 16);
          cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          j += 6;
        }
        utf8(o, cp);
      } else o += e;                    // \" \\ \/
    }
    if (found) *found = true;
    return o;
  }
  return String();
}

static void secure(WiFiClientSecure& c) { c.setInsecure(); }

// -------------------------------------------------------- speech -> text ----
String sttTranscribe(const int16_t* pcm, size_t samples, String& err) {
  err = "";
  if (!wifiUp()) { err = "no Wi-Fi"; return String(); }
  WiFiClientSecure c; secure(c);
  HTTPClient h;
  h.setConnectTimeout(8000);
  h.setTimeout(20000);
  if (!h.begin(c, DG_STT_URL)) { err = "Deepgram: bad URL"; return String(); }
  h.addHeader("Authorization", "Token " DEEPGRAM_KEY);
  h.addHeader("Content-Type", "audio/l16");     // raw 16 kHz mono; format is in the URL
  int code = h.POST((uint8_t*)pcm, samples * sizeof(int16_t));
  String body = h.getString();
  h.end();
  if (code != 200) { err = "Deepgram STT " + String(code) + ": " + body.substring(0, 160); return String(); }
  String t = jsonStr(body, "transcript");
  t.trim();
  return t;
}

// ------------------------------------------------------------ the brain ----
// messages: the JSON array of chat messages, already built. Returns the
// model's reply text (its "content").
String llmChat(const String& messages, String& err, String* modelUsed = nullptr) {
  err = "";
  if (!wifiUp()) { err = "no Wi-Fi"; return String(); }
  String body;
  body.reserve(messages.length() + 400);
  body = "{\"models\":[";
  for (size_t i = 0; i < sizeof(OR_MODELS) / sizeof(OR_MODELS[0]); i++) {
    if (i) body += ',';
    body += '"'; body += OR_MODELS[i]; body += '"';
  }
  body += "],\"max_tokens\":260,\"temperature\":0.8,\"reasoning\":{\"enabled\":false},"
          "\"messages\":";
  body += messages;
  body += '}';

  WiFiClientSecure c; secure(c);
  HTTPClient h;
  h.setConnectTimeout(8000);
  h.setTimeout(30000);
  if (!h.begin(c, OR_URL)) { err = "OpenRouter: bad URL"; return String(); }
  h.addHeader("Authorization", "Bearer " OPENROUTER_KEY);
  h.addHeader("Content-Type", "application/json");
  h.addHeader("HTTP-Referer", "https://github.com/nickthelegend/hotaru");
  h.addHeader("X-Title", "HOTARU 3");
  int code = h.POST(body);
  String resp = h.getString();
  h.end();
  if (code != 200) {
    String m = jsonStr(resp, "message");
    err = "OpenRouter " + String(code) + ": " + (m.length() ? m : resp.substring(0, 160));
    return String();
  }
  if (modelUsed) *modelUsed = jsonStr(resp, "model");
  int at = resp.indexOf("\"choices\"");
  bool found = false;
  String content = jsonStr(resp, "content", at < 0 ? 0 : at, &found);
  if (!found || !content.length()) err = "OpenRouter: empty reply";
  return content;
}

// -------------------------------------------------------- text -> speech ----
// Streams Deepgram's audio to the speaker as it arrives: the first words play
// about a second after the request goes out.
bool ttsSpeak(const String& text, String& err) {
  err = "";
  if (!text.length()) return true;
  if (!spkOk) { err = "no speaker"; return false; }
  if (!wifiUp()) { err = "no Wi-Fi"; return false; }
  WiFiClientSecure c; secure(c);
  HTTPClient h;
  h.setConnectTimeout(8000);
  h.setTimeout(20000);
  if (!h.begin(c, DG_TTS_URL)) { err = "Deepgram: bad URL"; return false; }
  h.addHeader("Authorization", "Token " DEEPGRAM_KEY);
  h.addHeader("Content-Type", "application/json");
  int code = h.POST("{\"text\":\"" + jsonEsc(text) + "\"}");
  if (code != 200) {
    err = "Deepgram TTS " + String(code) + ": " + h.getString().substring(0, 160);
    h.end();
    return false;
  }
  SpeakerSink sink;
  int n = h.writeToStream(&sink);
  h.end();
  spkSilence(120);
  if (n < 0) { err = "Deepgram TTS stream error " + String(n); return false; }
  return true;
}
