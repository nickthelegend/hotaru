/*
 * hotaru_motion.ino -- HOTARU 3 motion core: four MG996R on a PCA9685.
 *
 * Board: ESP32-S3 dev board, esp32 core 3.x. No libraries beyond Wire:
 * the PCA9685 is five registers, driven directly below.
 *
 * WIRING
 *   PCA9685  VCC  -- ESP32 3V3        (logic only)
 *            GND  -- ESP32 GND        (common ground, always)
 *            SDA  -- GPIO 8
 *            SCL  -- GPIO 9
 *            V+   -- 5 V servo rail, split from USB-C VBUS at the connector,
 *                    2200 uF across it AT the terminal block. Never from the
 *                    board's 3V3 regulator.
 *   channels 0 pan | 1 shoulder | 2 elbow | 3 head
 *   MX button  GPIO 4 -> GND  (internal pull-up): cycles the named poses
 *
 * WHY IT MOVES THE WAY IT DOES
 *   The old core stepped at a constant deg/s, so every move started and
 *   stopped at full speed -- the servo's worst case, current and torque
 *   both spike at the instant the target jumps. Every joint here follows a
 *   trapezoidal profile (accel-limited, speed-limited), and all four joints
 *   of a pose finish together. The PCA9685 pulses are also STAGGERED across
 *   the 20 ms frame so four servos never draw their pulse current at once.
 *
 *   The shoulder is geared 1.8 : 1 (30T pinion -> 54T gear on link1), and
 *   the pinion turns the opposite way to the arm. Everything above this
 *   file talks in JOINT degrees; jointToUs() is the one place that knows.
 *
 *   Every target pose passes a GUARD first (guard(), below): the shoulder
 *   servo may not be asked to hold more than a third of stall, and link2,
 *   the head and the shade may not enter the space the tower and base
 *   occupy. Its masses are SolidWorks' mass properties of the real parts;
 *   sw/h3_guard.py is the same code in Python, and it was run against
 *   SolidWorks' interference detection on random poses before it got here.
 *
 * SERIAL @ 115200
 *   p <pan> <sh> <el> <hd>   move to a pose, joint degrees
 *   j <i> <deg>              one joint
 *   rest | neutral | reach | look | lean | curious | shy | demo
 *   g <pan> <sh> <el> <hd>   ask the guard about a pose, don't move
 *   v <deg/s>   a <deg/s^2>  motion limits
 *   off | on                 release / re-engage all servos
 *   us <i> <us>              raw pulse on one channel (calibration)
 *   ?                        state
 */
#include <Wire.h>

// SolidWorks mass properties of the three moving groups (sw/h3_guard.py --emit)
#define GUARD_M1 168.2f
#define GUARD_S1 61.82f
#define GUARD_M2 171.9f
#define GUARD_S2 75.56f
#define GUARD_M3 51.5f
#define GUARD_S3 40.24f

// ------------------------------------------------------------ PCA9685 ----
static const uint8_t PCA_ADDR = 0x40;
static const int PIN_SDA = 8, PIN_SCL = 9, PIN_BTN = 4;
static const float PCA_OSC = 25000000.0f;   // calibrate if pulses read long
static const float SERVO_HZ = 50.0f;

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
// ON at `phase`, OFF `ticks` later: the phase is what staggers the pulses
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

// -------------------------------------------------------------- joints ----
struct Joint {
  const char* name;
  uint8_t ch;
  float lo, hi;         // joint limits, degrees (mechanical, from the CAD check)
  float ratio;          // servo degrees per joint degree (sign = direction)
  float us0;            // pulse at joint 0      <- calibrate per servo
  float usPerDeg;       // MG996R ~ 2000 us / 180 deg
  // motion state
  float pos = 0, vel = 0, target = 0;
  bool live = false;
};

Joint J[4] = {
  //  name       ch   lo     hi   ratio   us0    us/deg
  {"pan",      0, -90,   90,  1.0f,  1500, 11.11f},
  {"shoulder", 1, -30,   70, -1.8f,  1500, 11.11f},   // geared 1.8:1, pinion reversed
  {"elbow",    2, -120, 120,  1.0f,  1500, 11.11f},
  {"head",     3, -75,   75,  1.0f,  1500, 11.11f},
};

float VMAX = 90.0f;      // deg/s at the joint
float AMAX = 240.0f;     // deg/s^2 -- the knob that tames torque spikes
static const float US_MIN = 500, US_MAX = 2500;

static float jointToUs(const Joint& j, float deg) {
  float us = j.us0 + j.ratio * deg * j.usPerDeg;
  return constrain(us, US_MIN, US_MAX);
}

static void writeJoint(int i) {
  Joint& j = J[i];
  if (!j.live) return;
  // quarter-frame stagger: pulse i starts at i * 1024 ticks (5 ms apart)
  pcaPulse(j.ch, 1024 * i, usToTicks(jointToUs(j, j.pos)));
}

// ---------------------------------------------------------------- guard ----
// Numbers from SolidWorks (sw/h3_guard.py --emit). Lengths mm, masses g.
static const float L1 = 100.0f, L2H = 118.0f;            // shoulder->elbow, elbow->tilt axis
static const float Y_P = 102.5f, Z_P = 14.0f;            // shoulder pivot, arm plane
static const float G1_M = GUARD_M1, G1_S = GUARD_S1;     // link1 group: mass, CoM along link
static const float G2_M = GUARD_M2, G2_S = GUARD_S2;     // link2 group
static const float G3_M = GUARD_M3, G3_S = GUARD_S3;     // shade group
static const float GEAR = 1.8f;
static const float STALL = 9.4f * 0.0980665f;            // N.m @ 4.8 V
static const float TAU_MAX = STALL / 3.0f;
// keep-outs in the arm plane (y up, z along the arm's forward direction)
static const float TOWER_YMAX = 124.5f, TOWER_ZMIN = -60.0f, TOWER_ZMAX = 32.0f;
static const float BASE_YMAX = 59.0f, BASE_R = 82.0f;
static const float POD_YMAX = 80.0f, POD_R = 17.0f, POD_Z = -81.5f;   // keycap top
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

// smallest margin between a section of link2/head/shade and a keep-out
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

// Scale each joint's limits so all joints of a move arrive together: the
// joint with the longest trapezoid sets the time, the rest slow down.
float vlim[4], alim[4];

// Time for a move of d degrees using the SAME trapezoid setPose() then
// executes (ramps of T/3 each end): its cruise speed is 1.5 d / T and its
// acceleration 4.5 d / T^2, so the smallest T that respects both limits is
// the larger of the two below. (Timing with a textbook trapezoid and then
// executing this one overshot VMAX by 9% in simulation.)
static float moveTime(float d, float v, float a) {
  d = fabsf(d);
  return max(1.5f * d / v, sqrtf(4.5f * d / a));
}

bool setPose(const float* req) {
  float tgt[4];
  for (int i = 0; i < 4; i++) tgt[i] = constrain(req[i], J[i].lo, J[i].hi);
  int why = guard(tgt);
  if (why == 1) {
    // too much load: bring the shoulder back toward upright in 1 deg
    // steps until the servo is inside its holding budget
    while (why == 1 && fabsf(tgt[1]) > 0.5f) {
      tgt[1] -= (tgt[1] > 0 ? 1.0f : -1.0f);
      why = guard(tgt);
    }
    Serial.printf("guard: shoulder eased to %.0f deg (holding torque)\n", tgt[1]);
  }
  if (why == 2) {
    Serial.println(F("guard: REFUSED -- that pose puts the arm or shade into the base"));
    return false;
  }
  float T = 0.0f;
  for (int i = 0; i < 4; i++) {
    J[i].target = tgt[i];
    T = max(T, moveTime(J[i].target - J[i].pos, VMAX, AMAX));
  }
  for (int i = 0; i < 4; i++) {
    float d = fabsf(J[i].target - J[i].pos);
    if (T < 1e-3f || d < 1e-3f) { vlim[i] = VMAX; alim[i] = AMAX; continue; }
    // same-shape trapezoid, stretched to T: ramp time fixed at T/3
    float ta = T / 3.0f;
    vlim[i] = d / (T - ta);
    alim[i] = vlim[i] / ta;
  }
  return true;
}

// one control step: accelerate, cruise, or brake toward target
static void stepJoint(int i, float dt) {
  Joint& j = J[i];
  float d = j.target - j.pos;
  float dir = (d > 0) ? 1.0f : -1.0f;
  float a = alim[i], v = vlim[i];
  float stop = j.vel * j.vel / (2.0f * a);             // braking distance
  if (fabsf(d) < 0.05f && fabsf(j.vel) < a * dt) { j.pos = j.target; j.vel = 0; return; }
  if (fabsf(d) <= stop && j.vel * dir > 0) j.vel -= dir * a * dt;   // brake
  else j.vel += dir * a * dt;                                       // go
  j.vel = constrain(j.vel, -v, v);
  float np = j.pos + j.vel * dt;
  if ((j.target - np) * dir < 0) { np = j.target; j.vel = 0; }       // no overshoot
  j.pos = np;
}

// ---------------------------------------------------------------- poses ----
struct Pose { const char* name; float a[4]; };
// every pose here was checked in SolidWorks for zero interference
const Pose POSES[] = {
  {"neutral", {0, 0, 0, 0}},
  {"rest",    {0, 15, 105, 70}},
  {"reach",   {0, 60, 40, 30}},
  {"look",    {0, 10, 30, -70}},
  {"lean",    {0, -30, 60, -30}},
  {"curious", {35, 45, 75, -20}},
  {"shy",     {-40, -20, 110, 60}},
};
const int N_POSES = sizeof(POSES) / sizeof(POSES[0]);
int poseIdx = 1;

bool goPose(const String& n) {
  for (int k = 0; k < N_POSES; k++)
    if (n == POSES[k].name) { setPose(POSES[k].a); poseIdx = k; return true; }
  return false;
}

bool settled() {
  for (int i = 0; i < 4; i++)
    if (J[i].pos != J[i].target || J[i].vel != 0) return false;
  return true;
}

// ------------------------------------------------------------ engaging ----
// Servos jump to the first pulse they see. Engage one at a time, 250 ms
// apart, at the REST pose the lamp is parked in -- four MG996Rs lurching at
// once is the brownout the old bench logs recorded.
void engageAll() {
  for (int i = 0; i < 4; i++) {
    J[i].live = true; J[i].vel = 0;
    writeJoint(i);
    delay(250);
  }
}
void releaseAll() {
  for (int i = 0; i < 4; i++) { J[i].live = false; pcaOff(J[i].ch); }
}

void state() {
  Serial.printf("vmax %.0f deg/s  amax %.0f deg/s^2\n", VMAX, AMAX);
  for (int i = 0; i < 4; i++)
    Serial.printf("  %-8s ch%d  %7.1f deg -> %7.1f  (%4.0f us)%s\n", J[i].name, J[i].ch,
                  J[i].pos, J[i].target, jointToUs(J[i], J[i].pos), J[i].live ? "" : "  off");
}

// ---------------------------------------------------------------- main ----
uint32_t lastUs = 0;
bool demo = false;
uint32_t demoAt = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BTN, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  if (pcaRead(0x00) == 0xFF) Serial.println(F("*** no PCA9685 at 0x40 -- check SDA/SCL/VCC ***"));
  pcaBegin();
  releaseAll();
  for (int i = 0; i < 4; i++) {                 // assume parked at rest
    J[i].pos = J[i].target = POSES[1].a[i];
    vlim[i] = VMAX; alim[i] = AMAX;
  }
  engageAll();
  Serial.println(F("\nHOTARU 3 motion core -- PCA9685, geared shoulder. ? for help"));
  lastUs = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastUs >= 20000) {                  // 50 Hz, one servo frame
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;
    for (int i = 0; i < 4; i++) { stepJoint(i, dt); writeJoint(i); }
    if (demo && settled() && millis() > demoAt) {
      poseIdx = (poseIdx + 1) % N_POSES;
      setPose(POSES[poseIdx].a);
      demoAt = millis() + 1500;
    }
  }

  static uint32_t btnAt = 0;                    // MX key: next pose
  if (digitalRead(PIN_BTN) == LOW && millis() - btnAt > 400) {
    btnAt = millis();
    poseIdx = (poseIdx + 1) % N_POSES;
    setPose(POSES[poseIdx].a);
    Serial.printf("button -> %s\n", POSES[poseIdx].name);
  }

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  demo = false;
  if (cmd.startsWith("p ")) {
    float a[4];
    if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &a[0], &a[1], &a[2], &a[3]) == 4) setPose(a);
  } else if (cmd.startsWith("j ")) {
    int i; float d;
    if (sscanf(cmd.c_str() + 2, "%d %f", &i, &d) == 2 && i >= 0 && i < 4) {
      float a[4]; for (int k = 0; k < 4; k++) a[k] = J[k].target;
      a[i] = d; setPose(a);
    }
  } else if (cmd.startsWith("v ")) VMAX = constrain(cmd.substring(2).toFloat(), 5, 300);
  else if (cmd.startsWith("a ")) AMAX = constrain(cmd.substring(2).toFloat(), 20, 2000);
  else if (cmd.startsWith("us ")) {
    int i; float us;
    if (sscanf(cmd.c_str() + 3, "%d %f", &i, &us) == 2 && i >= 0 && i < 4) {
      J[i].live = false;                        // calibration owns the channel
      pcaPulse(J[i].ch, 1024 * i, usToTicks(constrain(us, US_MIN, US_MAX)));
      Serial.printf("ch%d raw %.0f us (joint released from motion)\n", J[i].ch, us);
    }
  }
  else if (cmd.startsWith("g ")) {
    float a[4];
    if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &a[0], &a[1], &a[2], &a[3]) == 4)
      Serial.printf("guard %d  shoulder servo %.3f N.m (max %.3f)  clearance %.1f mm\n",
                    guard(a), fabsf(shoulderTorque(a[1], a[2], a[3])) / GEAR, TAU_MAX,
                    clearance(a[0], a[1], a[2], a[3]));
  }
  else if (cmd == "off") releaseAll();
  else if (cmd == "on")  engageAll();
  else if (cmd == "demo") { demo = true; demoAt = 0; }
  else if (goPose(cmd)) Serial.printf("-> %s\n", cmd.c_str());
  else {
    state();
    Serial.println(F("p <pan> <sh> <el> <hd> | j <i> <deg> | v <deg/s> | a <deg/s2>"));
    Serial.println(F("rest neutral reach look lean curious shy | demo | g <pose> | off on | us <i> <us>"));
  }
}
