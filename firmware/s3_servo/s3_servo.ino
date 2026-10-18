/*
 * s3_servo.ino — HOTARU four-joint bench test, ESP32-S3.
 *
 * REPLACES servo_test/ and servo_move/, which are v1 sketches for the classic
 * WROOM-32 on /dev/cu.usbserial-0001. Their pins (13/14/26/27) are wrong here
 * and two are dangerous: on the S3, 26..32 are the in-package SPI flash.
 * Pins come from config.h now -- pan 4, shoulder 5, elbow 6, head 7.
 *
 * FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=disabled,CDCOnBoot=cdc,USBMode=hwcdc
 *
 * POWER, given there is no 2200uF cap on the rail yet:
 *   - servos run off the charger, NOT the board. The 5V pin is dead anyway.
 *   - the ONLY wire between charger and board is ground. Never feed the
 *     charger's + into 5V/VIN.
 *   - so a servo dip cannot brown out the S3: separate rails, common ground.
 *
 * SAFE BY DEFAULT: boots with every servo DETACHED. Nothing moves until you
 * ask. Attach one joint at a time and confirm each before adding the next --
 * that also keeps peak current to one servo's worth, which is the mitigation
 * that stands in for the missing cap.
 *
 * Serial @ 115200:
 *   ?         help                 i        status
 *   j0..j3    select joint         a / d    attach / detach the active joint
 *   A / D     attach / detach ALL
 *   0..180    move active joint (clamped to its end stops)
 *   n         nudge +10 deg        b        nudge -10 deg
 *   c         centre active (90)   C        centre all
 *   s         gentle sweep, active joint only, 70<->110
 *   w         staggered walk: each attached joint moves alone, in turn
 *   v<n>      speed deg/s (default 60 here, not config's 120)
 *   x         STOP: freeze every joint where it stands
 *
 * CALIBRATION (trims live in config.h):
 *   + / -     jog the active joint 1 deg
 *   h         this spot is 90 from now on (nothing moves)
 *   k         print the trims, ready to paste into config.h
 */
#include <esp_system.h>
#include "config.h"
#include "motion.h"

Motion M;
int active = J_PAN;

// Deliberately half of config.h's DEFAULT_SPEED. Current scales with
// acceleration, so 60 deg/s roughly halves the peak draw of 120. Raise it
// once a bulk cap is actually fitted.
static const float BENCH_SPEED = 60.0f;

static const char *jname(int i) { return M.j[i].name; }

void reportReset() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print(F("reset reason: "));
  switch (r) {
    case ESP_RST_POWERON: Serial.println(F("power-on (normal)")); break;
    case ESP_RST_SW:      Serial.println(F("software (normal)")); break;
    case ESP_RST_BROWNOUT:
      Serial.println(F("*** BROWNOUT ***"));
      Serial.println(F("  The board's own 3.3V rail collapsed. With split"));
      Serial.println(F("  rails this should be impossible -- so check that"));
      Serial.println(F("  the charger's + has NOT been wired to 5V/VIN, and"));
      Serial.println(F("  that the USB cable is a data cable in a real port."));
      break;
    case ESP_RST_PANIC:   Serial.println(F("panic/exception")); break;
    case ESP_RST_WDT:
    case ESP_RST_TASK_WDT:Serial.println(F("watchdog")); break;
    default:              Serial.printf("other (%d)\n", (int)r); break;
  }
}

void status() {
  Serial.println(F("\n  joint      pin  attached   pos   target   range    trim    raw"));
  for (int i = 0; i < N_JOINTS; i++) {
    Joint &q = M.j[i];
    Serial.printf("  %-9s %3d  %-8s  %5.1f  %5.1f    %.0f..%.0f  %+5.1f  %5.1f%s\n",
                  q.name, q.pin, q.attached ? "yes" : "no",
                  q.pos, q.target, q.lo, q.hi, q.trim, Motion::rawOf(q),
                  i == active ? "   <- active" : "");
  }
  Serial.printf("  speed %.0f deg/s\n\n", M.j[active].speed);
}

void help() {
  Serial.println(F("\n--- HOTARU S3 servo bench ---"));
  Serial.println(F("  j0 pan   j1 shoulder   j2 elbow   j3 head"));
  Serial.println(F("  a/d attach/detach active   A/D all   x STOP"));
  Serial.println(F("  0..180 move   n/b +-10   c centre   C centre all"));
  Serial.println(F("  s sweep active   w staggered walk   v<n> speed   i status"));
  Serial.println(F("  CALIBRATE: jN, then +/- 1 deg until neutral, h = home here, k = trims"));
  Serial.println(F("  servos on the charger, grounds tied, + NEVER to 5V\n"));
}

// Block until every attached joint reaches its target, ticking the core
// throughout. NOT "while (M.tick())": tick() returns early with a stale
// moving_ flag inside the 20 ms frame window, so that loop exits instantly.
// The timeout means a jammed joint cannot hang the bench.
void settle(uint32_t timeout_ms = 8000) {
  uint32_t t0 = millis();
  for (;;) {
    M.tick();
    bool moving = false;
    for (int i = 0; i < N_JOINTS; i++)
      if (M.j[i].attached && fabsf(M.j[i].pos - M.j[i].target) > 0.5f) moving = true;
    if (!moving) break;
    if (millis() - t0 > timeout_ms) { Serial.println(F("  (settle timeout)")); break; }
    delay(2);
  }
  delay(120);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nHOTARU S3 servo bench booting"));
  reportReset();

  M.begin();          // attaches all four
  M.detachAll();      // ...then immediately goes limp: nothing moves on boot
  M.allSpeed(BENCH_SPEED);

  Serial.println(F("all joints DETACHED -- nothing will move until you attach"));
  Serial.println(F("suggested: j0 then a, confirm pan alone, then add joints"));
  help();
  status();
}

void loop() {
  M.tick();

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  char c = cmd[0];

  if (isDigit(c)) {
    float target = cmd.toFloat();
    M.moveTo(active, target);
    Serial.printf("%s -> %.1f (clamped %.1f)\n",
                  jname(active), target, M.j[active].target);

  } else if (c == 'j' && cmd.length() > 1) {
    int i = cmd.substring(1).toInt();
    if (i >= 0 && i < N_JOINTS) { active = i; Serial.printf("active: %s\n", jname(active)); }
    else Serial.println(F("joint must be 0..3"));

  } else if (c == 'v') {
    float s = cmd.substring(1).toFloat();
    if (s > 0) { M.allSpeed(s); Serial.printf("speed %.0f deg/s\n", M.j[active].speed); }

  } else if (c == 'a') {
    M.j[active].target = M.j[active].pos;   // no lurch on attach
    M.attach(active);                       // THIS joint only
    Serial.printf("%s attached at %.1f\n", jname(active), M.j[active].pos);

  } else if (c == 'd') {
    M.j[active].servo.detach(); M.j[active].attached = false;
    Serial.printf("%s detached (limp)\n", jname(active));

  } else if (c == 'A') {
    for (int i = 0; i < N_JOINTS; i++) M.j[i].target = M.j[i].pos;
    M.attachAll(); Serial.println(F("ALL attached"));

  } else if (c == 'D') {
    M.detachAll(); Serial.println(F("ALL detached (limp)"));

  } else if (c == 'x') {
    for (int i = 0; i < N_JOINTS; i++) M.j[i].target = M.j[i].pos;
    Serial.println(F("STOP -- every joint frozen where it stands"));

  } else if (c == 'n') {
    M.moveTo(active, M.j[active].target + 10);
    Serial.printf("%s -> %.1f\n", jname(active), M.j[active].target);

  } else if (c == 'b') {
    M.moveTo(active, M.j[active].target - 10);
    Serial.printf("%s -> %.1f\n", jname(active), M.j[active].target);

  } else if (c == 'c') {
    M.moveTo(active, 90); Serial.printf("%s -> 90\n", jname(active));

  } else if (c == 'C') {
    M.home(); Serial.println(F("all -> 90"));

  } else if (c == 's') {
    // Deliberately narrow: +-20 deg about centre, nowhere near the end stops.
    Serial.printf("sweeping %s 70<->110, send anything to stop\n", jname(active));
    while (!Serial.available()) {
      M.moveTo(active, 110); settle();
      if (Serial.available()) break;
      M.moveTo(active, 70);  settle();
    }
    while (Serial.available()) Serial.read();
    M.moveTo(active, 90);
    Serial.println(F("stopped, returning to 90"));

  } else if (c == 'w') {
    // The no-capacitor mitigation: never start two servos together.
    Serial.println(F("staggered walk -- one joint at a time"));
    for (int i = 0; i < N_JOINTS; i++) {
      if (!M.j[i].attached) { Serial.printf("  %-9s skipped (detached)\n", jname(i)); continue; }
      Serial.printf("  %-9s 90 -> 110 -> 70 -> 90\n", jname(i));
      int prev = active; active = i;
      M.moveTo(i, 110); settle();
      M.moveTo(i,  70); settle();
      M.moveTo(i,  90); settle();
      active = prev;
    }
    Serial.println(F("walk done"));

  } else if (c == '+' || c == '-') {
    M.moveTo(active, M.j[active].target + (c == '+' ? 1.0f : -1.0f));
    Serial.printf("%s -> %.1f (raw %.1f)\n", jname(active), M.j[active].target,
                  constrain(M.j[active].target + M.j[active].trim, 0.0f, 180.0f));

  } else if (c == 'h') {
    if (!M.j[active].attached) {
      Serial.println(F("attach it first (a) -- a limp joint has no position to keep"));
    } else {
      settle();
      M.setHomeHere(active);
      Serial.printf("%s: this spot is 90 now (trim %+.1f)\n", jname(active), M.j[active].trim);
      if (fabsf(M.j[active].trim) > 7.2f)
        Serial.println(F("  over half a spline tooth: re-seat the horn one tooth (14.4 deg) instead"));
    }

  } else if (c == 'k') {
    const char *macro[N_JOINTS] = {"PAN_TRIM", "SHOULDER_TRIM", "ELBOW_TRIM", "HEAD_TRIM"};
    Serial.println(F("\n// paste into firmware/hotaru/config.h, calibration block:"));
    for (int i = 0; i < N_JOINTS; i++) {
      Joint &q = M.j[i];
      Serial.printf("#define %-14s %5.1ff\n", macro[i], q.trim);
      if (q.lo > q.cfgLo || q.hi < q.cfgHi)
        Serial.printf("//   %s range cut to %.0f..%.0f (config %.0f..%.0f): the servo can't reach the rest\n",
                      q.name, q.lo, q.hi, q.cfgLo, q.cfgHi);
    }
    Serial.println();

  } else if (c == 'i') {
    status();

  } else {
    help();
  }
}
