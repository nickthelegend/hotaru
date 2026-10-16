// brain.h -- who HOTARU is, what it remembers, and how its reply is read.
// The model answers with one JSON object: what to say, which action to act
// it out with, a mood (the ring's colour) and what to do with the lamp.
#pragma once

String sysPrompt;
String histUser[HISTORY_TURNS], histBot[HISTORY_TURNS];
int histN = 0;

void brainBegin() {
  sysPrompt =
    "You are HOTARU (Japanese for firefly), a small desk-lamp robot that lives on the "
    "user's desk, like Pixar's Luxo Jr. Your head is a lamp shade with a ring of LEDs; "
    "you move with four servos: a turntable base, a shoulder, an elbow and a tilting head. "
    "You can see nothing, but you hear through a microphone and talk through a small speaker. "
    "You are curious, playful, warm and a little shy, and you love light, desks, books and "
    "the person you live with. You speak in short, natural, spoken sentences: at most two "
    "sentences and about 30 words, because a speaker reads them aloud. No emojis, no "
    "markdown, no lists, no stage directions in the words.\n"
    "Reply with ONE JSON object and nothing else:\n"
    "{\"say\": \"what you say\", \"action\": \"an action name\", \"mood\": \"a mood\", "
    "\"light\": \"keep\", \"volume\": \"keep\"}\n"
    "action: pick the one that best acts out what you say, from this list:\n";
  for (int i = 0; i < N_ACTIONS; i++) {
    if (!ACTIONS[i].pub) continue;
    sysPrompt += "- ";
    sysPrompt += ACTIONS[i].name;
    sysPrompt += ": ";
    sysPrompt += ACTIONS[i].desc;
    sysPrompt += '\n';
  }
  sysPrompt += "mood: one of";
  for (int i = 0; i < MOOD_COUNT; i++) { sysPrompt += ' '; sysPrompt += MOOD_NAMES[i]; }
  sysPrompt +=
    ".\nlight: \"on\" to work as a reading lamp (asked for light, reading, writing, "
    "\"turn on\"), \"off\" to switch the lamp off, \"brighter\" or \"dimmer\", otherwise \"keep\".\n"
    "volume: \"up\", \"down\" or \"keep\".\n"
    "If asked to do a move, do it with the matching action. If the user says goodnight or "
    "asks you to sleep, use the sleep action.";
}

void histPush(const String& u, const String& b) {
  if (histN == HISTORY_TURNS) {
    for (int i = 1; i < HISTORY_TURNS; i++) { histUser[i - 1] = histUser[i]; histBot[i - 1] = histBot[i]; }
    histN--;
  }
  histUser[histN] = u;
  histBot[histN] = b;
  histN++;
}
void histClear() { histN = 0; }

struct Reply { String say, action, mood, light, volume, model; };

static String msg(const char* role, const String& content) {
  return String("{\"role\":\"") + role + "\",\"content\":\"" + jsonEsc(content) + "\"}";
}

bool think(const String& heard, Reply& r, String& err) {
  String m = "[";
  m += msg("system", sysPrompt);
  for (int i = 0; i < histN; i++) {
    m += ','; m += msg("user", histUser[i]);
    m += ','; m += msg("assistant", histBot[i]);
  }
  m += ','; m += msg("user", heard); m += ']';

  String content = llmChat(m, err, &r.model);
  if (!content.length()) return false;

  // models wrap JSON in code fences or chat before it: take the outermost {...}
  int a = content.indexOf('{'), b = content.lastIndexOf('}');
  String js = (a >= 0 && b > a) ? content.substring(a, b + 1) : String();
  bool hasSay = false;
  if (js.length()) r.say = jsonStr(js, "say", 0, &hasSay);
  if (!hasSay) {                       // plain text after all: say it as it is
    r.say = content;
    r.say.replace("\n", " ");
    r.say.trim();
  }
  r.action = jsonStr(js, "action");
  r.mood = jsonStr(js, "mood");
  r.light = jsonStr(js, "light");
  r.volume = jsonStr(js, "volume");
  r.action.trim(); r.mood.trim(); r.light.trim(); r.volume.trim();
  histPush(heard, js.length() ? js : String("{\"say\":\"") + jsonEsc(r.say) + "\"}");
  return true;
}
