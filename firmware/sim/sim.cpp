// sim.cpp -- run hotaru_motion.ino's real motion code off the board and
// watch every 20 ms servo frame against its own guard model.
//
//   node firmware/sim/sim.mjs        (builds with LLVM clang to wasm, runs)
//
// Pass: across every test, no executed frame comes closer than MARGIN to
// the tower, the base or the button pod. sw/h3_pan_sweep.py is what shows
// the guard model itself agrees with SolidWorks.
#include "Arduino.h"
uint32_t sim_us = 0;
SerialT Serial;
WireT Wire;
extern "C" void* memcpy(void* d, const void* s, size_t n) {
  char* o = (char*)d;
  const char* i = (const char*)s;
  while (n--) *o++ = *i++;
  return d;
}
#include "../hotaru_motion/hotaru_motion.ino"

// results go to sim.mjs, which prints them
extern "C" IMPORT("say") void say(int code, int i, float a, float b, float c, float d);

static float worst;
static long frames, bad;

static float clr(const float* p) { return clearance(p[0], p[1], p[2], p[3]); }

static void tick() {                      // one servo frame
  sim_us += 20000;
  loop();
  float p[4] = {J[0].pos, J[1].pos, J[2].pos, J[3].pos};
  float c = clr(p);
  worst = min(worst, c);
  frames++;
  if (c < MARGIN) bad++;
}
static bool runTo(int limit = 20000) {
  for (int k = 0; k < limit; k++) { tick(); if (idle()) return true; }
  return false;
}
static void place(const float* p) {
  for (int i = 0; i < 4; i++) { J[i].pos = J[i].target = goal[i] = p[i]; J[i].vel = 0; }
  legN = legAt = 0;
}
static float straightWorst(const float* a, const float* b) {
  float w = 1e9f, p[4];
  for (int k = 0; k <= 400; k++) {
    for (int i = 0; i < 4; i++) p[i] = a[i] + (b[i] - a[i]) * k / 400;
    w = min(w, clr(p));
  }
  return w;
}
static bool at(const float* p) {
  for (int i = 0; i < 4; i++) if (fabsf(J[i].pos - p[i]) > 0.01f) return false;
  return true;
}
static uint32_t rs = 7;
static float U(float lo, float hi) {      // xorshift32
  rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
  return lo + (hi - lo) * (rs >> 8) / 16777216.0f;
}

extern "C" __attribute__((export_name("run"))) int run() {
  setup();
  int fails = 0;

  // 1. pan an arm across the button pod
  struct { float a[4], b[4]; } routes[] = {
    {{-80, 0, -110, -45}, {80, 0, -110, -45}},
    {{-60, -10, -100, -60}, {60, -10, -100, -60}},
    {{-90, -30, -110, 0}, {90, -30, -110, 0}},
    {{-90, 15, 105, 70}, {90, 15, 105, 70}},
  };
  for (int r = 0; r < 4; r++) {
    place(routes[r].a);
    worst = 1e9f; bad = 0;
    setPose(routes[r].b);
    int legs = legN;
    bool ok = runTo() && at(routes[r].b) && !bad;
    fails += !ok;
    say(1, r, straightWorst(routes[r].a, routes[r].b), legs, worst, ok);
  }

  // 2. button / demo cycle, 3 times round
  place(POSES[1].a);
  worst = 1e9f; bad = 0;
  int missed = 0;
  for (int c = 0; c < 3 * N_POSES; c++) {
    int k = (c + 2) % N_POSES;
    setPose(POSES[k].a);
    if (!runTo() || !at(POSES[k].a)) missed++;
  }
  fails += (bad > 0) + (missed > 0);
  say(2, 3 * N_POSES, worst, !bad && !missed, missed, 0);

  // 3. a new command halfway through a pan
  {
    float a[4] = {-90, 15, 105, 70}, b[4] = {90, 15, 105, 70}, c[4] = {60, -10, -100, -60};
    place(a);
    worst = 1e9f; bad = 0;
    setPose(b);
    for (int k = 0; k < 60; k++) tick();
    float mid = J[0].pos, v = J[0].vel;
    setPose(c);
    bool ok = runTo() && at(c) && !bad;
    fails += !ok;
    say(3, 0, mid, v, worst, ok);
  }

  // 4. random targets, most of them interrupting a move under way
  {
    place(POSES[1].a);
    worst = 1e9f; bad = 0; frames = 0;
    int sent = 0, guardNo = 0, routeNo = 0, detour = 0;
    for (int n = 0; n < 5000; n++) {
      float t[4];
      for (int i = 0; i < 4; i++) t[i] = U(J[i].lo, J[i].hi);
      sent++;
      if (guard(t) == 2) { guardNo++; continue; }
      bool wasMoving = !settled();
      if (!setPose(t)) { routeNo++; continue; }
      if (legN - (int)wasMoving > 1) detour++;       // brake leg aside
      int run = (int)U(0, 150);                      // 0-3 s, then interrupt
      for (int k = 0; k < run && !idle(); k++) tick();
    }
    runTo();
    fails += bad > 0;
    say(4, sent, guardNo, routeNo, detour, 0);
    say(5, 0, frames, worst, bad, !bad);
  }
  return fails;
}
