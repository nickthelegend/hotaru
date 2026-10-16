// motion.h -- the motion core: PCA9685, calibrated joints, pose guard, route
// planner, trapezoid moves and the action player. Ported from
// firmware/hotaru_motion/hotaru_motion.ino (same guard, same planner; the
// Python twin is sw/h3_guard.py) with three changes:
//   * joints are calibrated in SERVO degrees (config.h, "cal" commands),
//     stored in flash, and the joint range follows from the calibration;
//   * every joint has its own speed cap, and a move can be given a duration;
//   * it runs in its own FreeRTOS task at 50 Hz, so the arm keeps moving
//     smoothly while the main loop waits on the network. Everything else
//     talks to it through a queue (motionSend).
#pragma once
#include <Wire.h>
#include <Preferences.h>

// SolidWorks mass properties of the three moving groups (sw/h3_guard.py --emit)
#define GUARD_M1 168.2f
#define GUARD_S1 61.82f
#define GUARD_M2 171.9f
#define GUARD_S2 75.56f
#define GUARD_M3 51.5f
#define GUARD_S3 40.24f

// ------------------------------------------------------------ PCA9685 ----
static const uint8_t PCA_ADDR = 0x40;
static const float PCA_OSC = 25000000.0f;   // calibrate if pulses read long
static const float SERVO_HZ = 50.0f;
bool pcaFound = false;

static void pcaWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
static uint8_t pcaRead(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom(PCA_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static void pcaBegin() {
  pcaWrite(0x00, 0x10);                                   // MODE1: sleep
  uint8_t pre = (uint8_t)lroundf(PCA_OSC / (4096.0f * SERVO_HZ)) - 1;   // 121
  pcaWrite(0xFE, pre);                                    // PRE_SCALE
  pcaWrite(0x00, 0x20);                                   // wake, auto-increment
  delay(1);
  pcaWrite(0x00, 0xA0);                                   // + restart
  pcaWrite(0x01, 0x04);                                   // MODE2: totem pole
}
// ON at `phase`, OFF `ticks` later: the phase staggers the four pulses
static void pcaPulse(uint8_t ch, uint16_t phase, uint16_t ticks) {
  uint16_t on = phase & 0x0FFF, off = (phase + ticks) & 0x0FFF;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  Wire.endTransmission();
}
static void pcaOff(uint8_t ch) {                          // full-OFF bit
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  Wire.endTransmission();
}
static uint16_t usToTicks(float us) {
  return (uint16_t)lroundf(us * 4096.0f * SERVO_HZ / 1e6f);
}
static float servoUs(float servoDeg) {
  servoDeg = constrain(servoDeg, 0.0f, 180.0f);
  return SERVO_US_0 + servoDeg * (SERVO_US_180 - SERVO_US_0) / 180.0f;
}

// -------------------------------------------------------------- joints ----
struct Joint {
  const char* name;
  uint8_t ch;
  float zero, k, smin, smax;   // calibration, servo degrees
  float lo, hi;                // joint range that follows from it (and the CAD)
  float pos, vel, target;      // motion state, joint degrees
  bool live;
};
Joint J[4];
float speedScale = 1.0f;       // "v" scales every cap

static float servoDeg(const Joint& j, float q) { return j.zero + j.k * q; }

void calDerive(int i) {
  Joint& j = J[i];
  if (fabsf(j.k) < 0.01f) j.k = (j.k < 0) ? -0.01f : 0.01f;
  float a = (j.smin - j.zero) / j.k, b = (j.smax - j.zero) / j.k;
  j.lo = max(min(a, b), CAD_LO[i]);
  j.hi = min(max(a, b), CAD_HI[i]);
  if (j.lo > j.hi) j.lo = j.hi = constrain(0.0f, CAD_LO[i], CAD_HI[i]);
}

void calDefaults() {
  for (int i = 0; i < 4; i++) {
    const CalDefault& c = CAL_DEFAULT[i];
    J[i].name = c.name; J[i].ch = c.ch;
    J[i].zero = c.zero; J[i].k = c.k; J[i].smin = c.smin; J[i].smax = c.smax;
    calDerive(i);
  }
}

void calLoad() {
  calDefaults();
  Preferences p;
  if (!p.begin("hotaru", true)) return;
  if (p.getUChar("calv", 0) == 1) {
    char key[8];
    for (int i = 0; i < 4; i++) {
      snprintf(key, sizeof key, "z%d", i); J[i].zero = p.getFloat(key, J[i].zero);
      snprintf(key, sizeof key, "k%d", i); J[i].k = p.getFloat(key, J[i].k);
      snprintf(key, sizeof key, "n%d", i); J[i].smin = p.getFloat(key, J[i].smin);
      snprintf(key, sizeof key, "x%d", i); J[i].smax = p.getFloat(key, J[i].smax);
      calDerive(i);
    }
  }
  p.end();
}

bool calSave() {
  Preferences p;
  if (!p.begin("hotaru", false)) return false;
  char key[8];
  for (int i = 0; i < 4; i++) {
    snprintf(key, sizeof key, "z%d", i); p.putFloat(key, J[i].zero);
    snprintf(key, sizeof key, "k%d", i); p.putFloat(key, J[i].k);
    snprintf(key, sizeof key, "n%d", i); p.putFloat(key, J[i].smin);
    snprintf(key, sizeof key, "x%d", i); p.putFloat(key, J[i].smax);
  }
  p.putUChar("calv", 1);
  p.end();
  return true;
}

void calClear() {
  Preferences p;
  if (p.begin("hotaru", false)) { p.remove("calv"); p.end(); }
  calDefaults();
}

static float jointUs(const Joint& j) {
  return servoUs(constrain(servoDeg(j, j.pos), j.smin, j.smax));
}

static void writeJoint(int i) {
  Joint& j = J[i];
  if (!j.live) return;
  pcaPulse(j.ch, 1024 * i, usToTicks(jointUs(j)));         // quarter-frame stagger
}

// ---------------------------------------------------------------- guard ----
// Numbers from SolidWorks (sw/h3_guard.py --emit). Lengths mm, masses g.
static const float L1 = 100.0f, L2H = 118.0f;
static const float Y_P = 102.5f, Z_P = 14.0f;
static const float G1_M = GUARD_M1, G1_S = GUARD_S1;
static const float G2_M = GUARD_M2, G2_S = GUARD_S2;
static const float G3_M = GUARD_M3, G3_S = GUARD_S3;
static const float GEAR = 1.8f;
static const float STALL = 9.4f * 0.0980665f;            // N.m @ 4.8 V
static const float TAU_MAX = STALL / 3.0f;
static const float TOWER_YMAX = 124.5f, TOWER_ZMIN = -60.0f, TOWER_ZMAX = 32.0f;
static const float BASE_YMAX = 59.0f, BASE_R = 82.0f;
static const float POD_YMAX = 80.0f, POD_R = 17.0f, POD_Z = -81.5f;
static const float SHADE_BACK = 30.0f, SHADE_LEN = 58.0f, SHADE_RB = 28.0f, SHADE_RM = 44.0f;
static const float L2_HALF = 19.0f, NOSE_R = 26.0f, MARGIN = 1.0f;

static inline float rad(float d) { return d * 0.01745329f; }

float shoulderTorque(float sh, float el, float hd) {      // N.m at the JOINT
  float a1 = rad(sh), a2 = rad(sh + el), a3 = rad(sh + el + hd);
  float ze = L1 * sinf(a1);
  float zh = ze + L2H * sinf(a2);
  float mz = G1_M * G1_S * sinf(a1) + G2_M * (ze + G2_S * sinf(a2)) + G3_M * (zh + G3_S * sinf(a3));
  return mz * 1e-6f * 9.81f;
}

static float sectionClear(float y, float z, float a, float hw, float cosPan, float worst) {
  float ny = -sinf(a), nz = cosf(a);
  for (int k = -1; k <= 1; k++) {
    float py = y + k * hw * ny, pz = z + k * hw * nz;
    float dT = max(max(py - TOWER_YMAX, TOWER_ZMIN - pz), pz - TOWER_ZMAX);
    float dB = max(py - BASE_YMAX, fabsf(pz) - BASE_R);
    worst = min(worst, min(dT, dB));
  }
  float dxz = sqrtf(max(z * z - 2 * z * POD_Z * cosPan + POD_Z * POD_Z, 0.0f));
  return min(worst, max(y - hw - POD_YMAX, dxz - hw - POD_R));
}

float clearance(float pan, float sh, float el, float hd) {
  float a1 = rad(sh), a2 = rad(sh + el), a3 = rad(sh + el + hd), cp = cosf(rad(pan));
  float ey = Y_P + L1 * cosf(a1), ez = Z_P + L1 * sinf(a1);
  float w = 1e9f;
  for (int k = 0; k <= 5; k++) {                         // link2 body
    float d = 36.0f + (90.0f - 36.0f) * k / 5.0f;
    w = sectionClear(ey + d * cosf(a2), ez + d * sinf(a2), a2, L2_HALF, cp, w);
  }
  float hy = ey + L2H * cosf(a2), hz = ez + L2H * sinf(a2);
  w = sectionClear(hy, hz, a2, NOSE_R, cp, w);
  w = sectionClear(hy, hz, a2 + 1.5708f, NOSE_R, cp, w);
  for (int k = 0; k <= 8; k++) {                         // shade
    float d = SHADE_BACK + SHADE_LEN * k / 8.0f;
    float r = SHADE_RB + (SHADE_RM - SHADE_RB) * k / 8.0f + 0.5f;
    w = sectionClear(hy + d * cosf(a3), hz + d * sinf(a3), a3, r, cp, w);
  }
  return w;
}

// 0 = ok, 1 = torque, 2 = collision
int guard(const float* a) {
  if (fabsf(shoulderTorque(a[1], a[2], a[3])) / GEAR > TAU_MAX) return 1;
  if (clearance(a[0], a[1], a[2], a[3]) < MARGIN) return 2;
  return 0;
}

// ----------------------------------------------------------- path guard ----
static const float PATH_STEP = 2.0f;

static bool pathClear(const float* a, const float* b) {
  float d = 0, p[4];
  for (int i = 0; i < 4; i++) d = max(d, fabsf(b[i] - a[i]));
  int n = max(1, (int)ceilf(d / PATH_STEP));
  float lowest = min(MARGIN, clearance(a[0], a[1], a[2], a[3]));
  for (int k = 1; k <= n; k++) {
    for (int i = 0; i < 4; i++) p[i] = a[i] + (b[i] - a[i]) * k / n;
    if (clearance(p[0], p[1], p[2], p[3]) < lowest) return false;
  }
  return true;
}

// Waypoints from cur to tgt, every leg path-clear; 0 = no clear route.
// Tried in order: straight; arm first then pan; pan first then arm; lift
// the arm straight up, pan, lower it.
int plan(const float* cur, const float* tgt, float out[3][4]) {
  float r[4][3][4];
  int n[4] = {1, 2, 2, 3};
  for (int i = 0; i < 4; i++) {
    r[0][0][i] = tgt[i];
    r[1][0][i] = i ? tgt[i] : cur[0];   r[1][1][i] = tgt[i];
    r[2][0][i] = i ? cur[i] : tgt[0];   r[2][1][i] = tgt[i];
    r[3][0][i] = i ? 0.0f : cur[0];     r[3][1][i] = i ? 0.0f : tgt[0];   r[3][2][i] = tgt[i];
  }
  for (int k = 0; k < 4; k++) {
    const float* at = cur;
    bool ok = true;
    for (int l = 0; l < n[k] && ok; l++) { ok = pathClear(at, r[k][l]); at = r[k][l]; }
    if (ok) { memcpy(out, r[k], sizeof(float) * 4 * n[k]); return n[k]; }
  }
  return 0;
}

// ----------------------------------------------------------- trapezoids ----
float vlim[4], alim[4];

static float moveTime(float d, float v, float a) {
  d = fabsf(d);
  return max(1.5f * d / v, sqrtf(4.5f * d / a));
}

// Every joint runs the same-shape trapezoid (ramps of T/3), stretched to the
// slowest joint's time -- or to `minT`, if the move was asked to take longer.
static float startLeg(const float* tgt, float minT) {
  float T = minT;
  for (int i = 0; i < 4; i++) {
    J[i].target = tgt[i];
    T = max(T, moveTime(J[i].target - J[i].pos, VCAP[i] * speedScale, ACAP[i] * speedScale));
  }
  for (int i = 0; i < 4; i++) {
    float d = fabsf(J[i].target - J[i].pos);
    if (T < 1e-3f || d < 1e-3f) { vlim[i] = VCAP[i]; alim[i] = ACAP[i]; continue; }
    float ta = T / 3.0f;
    vlim[i] = d / (T - ta);
    alim[i] = vlim[i] / ta;
  }
  return T;
}

float route[4][4], goal[4], legMinT = 0;
int legN = 0, legAt = 0;

static void clampPose(float* a) {
  for (int i = 0; i < 4; i++) a[i] = constrain(a[i], J[i].lo, J[i].hi);
}

// Plan and start a move. Returns false if the guard refuses it.
bool setPose(const float* req, float minT = 0.0f, bool quiet = false) {
  float tgt[4];
  memcpy(tgt, req, sizeof tgt);
  clampPose(tgt);
  int why = guard(tgt);
  if (why == 1) {                       // too much load: ease the shoulder upright
    float lim = max(0.0f, J[1].lo);
    while (why == 1 && fabsf(tgt[1] - lim) > 0.5f) {
      tgt[1] += (tgt[1] > lim ? -1.0f : 1.0f);
      why = guard(tgt);
    }
    if (!quiet) Serial.printf("guard: shoulder eased to %.0f deg (holding torque)\n", tgt[1]);
  }
  if (why) {
    if (!quiet) Serial.println(F("guard: REFUSED -- that pose puts the arm or shade into the base"));
    return false;
  }
  // plan from where the joints will actually stop (a running move brakes
  // along its own, already-checked line first)
  float from[4];
  bool moving = false;
  for (int i = 0; i < 4; i++) {
    from[i] = J[i].pos;
    if (J[i].vel == 0) continue;
    moving = true;
    float s = J[i].vel * fabsf(J[i].vel) / (2.0f * alim[i]);
    float left = J[i].target - J[i].pos;
    from[i] += (fabsf(s) < fabsf(left)) ? s : left;
  }
  float legs[3][4];
  int n = plan(from, tgt, legs);
  if (!n) {
    if (!quiet) Serial.println(F("guard: REFUSED -- no route there that keeps clear of the base"));
    return false;
  }
  legN = 0;
  if (moving) memcpy(route[legN++], from, sizeof(from));
  memcpy(route[legN], legs, sizeof(float) * 4 * n);
  legN += n;
  legAt = 0;
  legMinT = (n == 1) ? minT : 0.0f;      // a detour round the pod runs at full speed
  memcpy(goal, tgt, sizeof(goal));
  if (moving) for (int i = 0; i < 4; i++) J[i].target = from[i];
  else startLeg(route[0], legAt == legN - 1 ? legMinT : 0.0f);
  return true;
}

static void stepJoint(int i, float dt) {
  Joint& j = J[i];
  float d = j.target - j.pos;
  float dir = (d > 0) ? 1.0f : -1.0f;
  float a = alim[i], v = vlim[i];
  float stop = j.vel * j.vel / (2.0f * a);
  if (fabsf(d) < 0.05f && fabsf(j.vel) < a * dt) { j.pos = j.target; j.vel = 0; return; }
  if (fabsf(d) <= stop && j.vel * dir > 0) j.vel -= dir * a * dt;
  else j.vel += dir * a * dt;
  j.vel = constrain(j.vel, -v, v);
  float np = j.pos + j.vel * dt;
  if ((j.target - np) * dir < 0) { np = j.target; j.vel = 0; }
  j.pos = np;
}

bool settled() {
  for (int i = 0; i < 4; i++)
    if (J[i].pos != J[i].target || J[i].vel != 0) return false;
  return true;
}
bool routeDone() { return settled() && legAt + 1 >= legN; }

// ---------------------------------------------------------- engage/off ----
// Servos jump to the first pulse they see: engage one at a time, 250 ms
// apart, pan last (see hotaru_motion.ino).
void engageAll() {
  static const int ORDER[4] = {1, 2, 3, 0};
  for (int k = 0; k < 4; k++) {
    int i = ORDER[k];
    J[i].live = true; J[i].vel = 0;
    writeJoint(i);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
void releaseAll() {
  for (int i = 0; i < 4; i++) { J[i].live = false; pcaOff(J[i].ch); }
}

// ------------------------------------------------------------- actions ----
#include "actions.h"

int findAction(const String& n) {
  for (int i = 0; i < N_ACTIONS; i++) if (n.equalsIgnoreCase(ACTIONS[i].name)) return i;
  return -1;
}

static const float REST_POSE[4] = {0, 15, 105, 70};       // power-up / parked pose

// --------------------------------------------------------- the task ----
enum CmdType : uint8_t { C_POSE, C_ACTION, C_STOP, C_RELEASE, C_ENGAGE, C_SERVO, C_JOINT };
struct Cmd { CmdType t; int16_t idx; float a[4]; uint16_t ms; };

static QueueHandle_t motionQ;
volatile int curAction = -1;          // playing action, -1 = none
volatile bool motionIdle = true;      // no action and the route is finished
volatile bool talking = false;        // the main loop is speaking: fill gaps with "talk"
volatile uint32_t lastActionEnd = 0;

static const Action* act = nullptr;
static int actKey = 0;
static bool keyIssued = false;
static uint32_t keyDeadline = 0;

bool motionSend(const Cmd& c) { return motionQ && xQueueSend(motionQ, &c, 0) == pdTRUE; }
bool playAction(int idx) {
  if (idx < 0 || idx >= N_ACTIONS) return false;
  Cmd c{C_ACTION, (int16_t)idx, {0, 0, 0, 0}, 0};
  return motionSend(c);
}
bool playAction(const String& name) { return playAction(findAction(name)); }
bool movePose(const float* a, uint16_t ms = 0) {
  Cmd c{C_POSE, 0, {a[0], a[1], a[2], a[3]}, ms};
  return motionSend(c);
}

static void startAction(int idx) {
  act = &ACTIONS[idx];
  actKey = 0;
  keyIssued = false;
  curAction = idx;
}

static void playerTick(uint32_t now) {
  if (!act) {
    // a chosen action ended while still talking: keep the lamp lively
    if (talking && routeDone()) { int t = findAction("talk"); if (t >= 0) startAction(t); }
    return;
  }
  if (!keyIssued) {
    const Key& k = act->keys[actKey];
    float a[4] = {(float)k.a[0], (float)k.a[1], (float)k.a[2], (float)k.a[3]};
    setPose(a, k.ms / 1000.0f, true);      // a refused key is skipped
    keyDeadline = now + k.ms;
    keyIssued = true;
    return;
  }
  if (routeDone() && (int32_t)(now - keyDeadline) >= 0) {
    keyIssued = false;
    if (++actKey >= act->n) { act = nullptr; curAction = -1; lastActionEnd = now; }
  }
}

static void handle(const Cmd& c) {
  switch (c.t) {
    case C_ACTION: startAction(c.idx); break;
    case C_POSE:   act = nullptr; curAction = -1; setPose(c.a, c.ms / 1000.0f); break;
    case C_JOINT: {                                   // one joint, the rest stay
      act = nullptr; curAction = -1;
      float a[4]; memcpy(a, goal, sizeof a); a[c.idx] = c.a[0];
      setPose(a, 0.0f);
      break;
    }
    case C_STOP: {
      act = nullptr; curAction = -1;
      float a[4]; for (int i = 0; i < 4; i++) a[i] = J[i].pos;
      setPose(a, 0.0f, true);
      break;
    }
    case C_RELEASE: act = nullptr; curAction = -1; releaseAll(); break;
    case C_ENGAGE:
      for (int i = 0; i < 4; i++) { J[i].vel = 0; J[i].target = J[i].pos; }
      engageAll();
      break;
    case C_SERVO: {                                   // raw servo angle, calibration
      int i = c.idx;
      J[i].live = false;
      pcaPulse(J[i].ch, 1024 * i, usToTicks(servoUs(c.a[0])));
      break;
    }
  }
}

static void motionTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t last = micros();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));          // 50 Hz, one servo frame
    uint32_t now = micros();
    float dt = (now - last) / 1e6f;
    last = now;
    Cmd c;
    while (xQueueReceive(motionQ, &c, 0) == pdTRUE) handle(c);
    for (int i = 0; i < 4; i++) { stepJoint(i, dt); writeJoint(i); }
    if (settled() && legAt + 1 < legN) {
      ++legAt;
      startLeg(route[legAt], legAt == legN - 1 ? legMinT : 0.0f);
    }
    playerTick(millis());
    motionIdle = (act == nullptr) && routeDone();
    ledsRender(millis());
  }
}

void motionBegin(bool engage = true) {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  pcaFound = pcaRead(0x00) != 0xFF;
  if (!pcaFound) Serial.println(F("*** no PCA9685 at 0x40 -- check SDA/SCL/VCC ***"));
  pcaBegin();
  calLoad();
  releaseAll();
  for (int i = 0; i < 4; i++) {                 // assume parked at rest
    J[i].pos = J[i].target = goal[i] = constrain(REST_POSE[i], J[i].lo, J[i].hi);
    J[i].vel = 0; vlim[i] = VCAP[i]; alim[i] = ACAP[i];
  }
  if (engage) engageAll();
  else Serial.println(F("servos left OFF (MX key held at power-up) -- type 'on' to engage"));
  motionQ = xQueueCreate(16, sizeof(Cmd));
  xTaskCreatePinnedToCore(motionTask, "motion", 8192, nullptr, 3, nullptr, 1);
}

void motionState() {
  Serial.printf("speed x%.2f  action %s\n", speedScale,
                curAction >= 0 ? ACTIONS[curAction].name : "-");
  for (int i = 0; i < 4; i++) {
    const Joint& j = J[i];
    Serial.printf("  %-8s ch%d  %7.1f -> %7.1f deg   servo %6.1f (%4.0f us)  range %.0f..%.0f%s\n",
                  j.name, j.ch, j.pos, j.target, servoDeg(j, j.pos), jointUs(j), j.lo, j.hi,
                  j.live ? "" : "  off");
  }
}

void calPrint() {
  Serial.printf("servo angle 0..180 = %d..%d us\n", SERVO_US_0, SERVO_US_180);
  Serial.println(F("  joint     ch   zero      k   smin   smax   -> joint range"));
  for (int i = 0; i < 4; i++) {
    const Joint& j = J[i];
    Serial.printf("  %-8s  %d  %6.1f  %5.2f  %5.1f  %5.1f   -> %6.1f .. %6.1f deg\n",
                  j.name, j.ch, j.zero, j.k, j.smin, j.smax, j.lo, j.hi);
  }
}
