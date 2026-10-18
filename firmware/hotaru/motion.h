#pragma once
/*
 * motion.h — four-joint non-blocking motion core.
 *
 * WHY NOT step-and-delay: measured on this board, a 140 deg move took 5.57 s
 * whether the per-step delay was 2 ms or 27 ms. servo.write() blocks on the
 * 50 Hz PWM frame, so a "one degree per delay()" loop is pinned near 25
 * steps/s and its speed knob does nothing. It also blocks, which is fatal
 * once servos, I2S audio and a LED ring share one loop.
 *
 * Instead each joint holds a float position advanced by (speed * dt) once per
 * 20 ms frame -- exactly the rate the servo consumes. Real deg/s, smooth,
 * and loop() never stalls.
 */
#include <ESP32Servo.h>
#include "config.h"

enum JointId { J_PAN = 0, J_SHOULDER, J_ELBOW, J_HEAD, N_JOINTS };

struct Joint {
  Servo   servo;
  uint8_t pin;
  const char *name;
  float   lo, hi;          // live limits, calibrated deg (cut to what the servo reaches)
  float   cfgLo, cfgHi;    // the limits as written in config.h
  float   pos, target, speed;
  float   trim;       // servo deg added at the write: makes 90 the true neutral
  bool    attached;
};

class Motion {
 public:
  Joint j[N_JOINTS];

  void begin() {
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    setup(J_PAN,      PIN_SERVO_PAN,      "pan",      PAN_MIN,      PAN_MAX,      PAN_TRIM);
    setup(J_SHOULDER, PIN_SERVO_SHOULDER, "shoulder", SHOULDER_MIN, SHOULDER_MAX, SHOULDER_TRIM);
    setup(J_ELBOW,    PIN_SERVO_ELBOW,    "elbow",    ELBOW_MIN,    ELBOW_MAX,    ELBOW_TRIM);
    setup(J_HEAD,     PIN_SERVO_HEAD,     "head",     HEAD_MIN,     HEAD_MAX,     HEAD_TRIM);
    last_ = micros();
  }

  // call every loop(); returns true while anything is still moving
  bool tick() {
    uint32_t now = micros();
    if (now - last_ < FRAME_US) return moving_;
    float dt = (now - last_) / 1e6f;
    last_ = now;
    moving_ = false;
    for (int i = 0; i < N_JOINTS; i++) {
      Joint &q = j[i];
      if (!q.attached) continue;
      float d = q.target - q.pos, step = q.speed * dt;
      if (fabsf(d) <= step) q.pos = q.target;
      else                  q.pos += (d > 0 ? step : -step);
      q.servo.write((int)lroundf(rawOf(q)));
      if (q.pos != q.target) moving_ = true;
    }
    return moving_;
  }

  // Clamped to the joint's software end stops -- a stalled MG996R pulls
  // 2.5 A and browns out the lamp, so nothing gets to command past them.
  void moveTo(int i, float deg) {
    if (i < 0 || i >= N_JOINTS) return;
    j[i].target = constrain(deg, j[i].lo, j[i].hi);
  }
  void setSpeed(int i, float dps) { j[i].speed = constrain(dps, 5.f, 500.f); }
  void allSpeed(float dps) { for (int i=0;i<N_JOINTS;i++) setSpeed(i, dps); }
  bool busy() const { return moving_; }

  // The angle actually sent to the servo: calibrated position plus trim.
  static float rawOf(const Joint &q) { return constrain(q.pos + q.trim, 0.0f, 180.0f); }

  // Calibration: make the joint's CURRENT physical spot read as 90 from now
  // on. Nothing moves -- pos and trim change together, so the angle written
  // (pos + trim) is the same before and after.
  void setHomeHere(int i) {
    if (i < 0 || i >= N_JOINTS) return;
    j[i].trim = j[i].pos + j[i].trim - 90.0f;
    j[i].pos = j[i].target = 90.0f;
    applyLimits(i);
  }

  // config.h's limits, cut to what the servo can physically reach at this trim
  // (raw 0..180). A big trim pushes one end of the range past the servo's
  // travel; commanding into that zone would park it against its end stop.
  void applyLimits(int i) {
    Joint &q = j[i];
    float rawLo = -q.trim, rawHi = 180.0f - q.trim;
    q.lo = q.cfgLo > rawLo ? q.cfgLo : rawLo;
    q.hi = q.cfgHi < rawHi ? q.cfgHi : rawHi;
  }

  void pose(float pan, float sh, float el, float hd) {
    moveTo(J_PAN, pan); moveTo(J_SHOULDER, sh);
    moveTo(J_ELBOW, el); moveTo(J_HEAD, hd);
  }
  void home()  { pose(90, 90, 90, 90); }
  void rest()  { pose(90, 140, 40, 60); }   // folded down, low load
  void alert() { pose(90, 70, 110, 100); }  // upright, looking forward

  void detachAll() {
    for (int i=0;i<N_JOINTS;i++) { j[i].servo.detach(); j[i].attached=false; }
  }
  void attachAll() { for (int i=0;i<N_JOINTS;i++) attach(i); }

 private:
  uint32_t last_ = 0;
  bool moving_ = false;

  void setup(int i, uint8_t pin, const char *nm, float lo, float hi, float trim) {
    j[i].pin = pin; j[i].name = nm; j[i].cfgLo = lo; j[i].cfgHi = hi;
    j[i].trim = trim;
    applyLimits(i);
    j[i].pos = j[i].target = 90.0f;
    j[i].speed = DEFAULT_SPEED;
    j[i].attached = false;
    attach(i);
  }
 public:
  // Public so a bench harness can bring joints up ONE at a time -- with no
  // bulk cap on the rail, never starting two servos together is the
  // mitigation that replaces it.
  void attach(int i) {
    j[i].servo.setPeriodHertz(SERVO_HZ);
    j[i].servo.attach(j[i].pin, SERVO_US_MIN, SERVO_US_MAX);
    j[i].servo.write((int)lroundf(rawOf(j[i])));
    j[i].attached = true;
  }
};
