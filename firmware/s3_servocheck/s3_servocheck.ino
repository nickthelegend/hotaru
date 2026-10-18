/*
 * s3_servocheck.ino — why don't the servos move? Checks every signal pin from
 * the board's side: firmware vs wiring vs power.
 *
 * FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=disabled,CDCOnBoot=cdc,USBMode=hwcdc
 *
 * For each joint (pan 4, shoulder 5, elbow 6, head 7):
 *
 * 1. PULSES AT THE PIN. Attaches with ESP32Servo exactly as the bench does,
 *    commands 90 deg, then switches the pad's input buffer on underneath the
 *    PWM and samples what the pin is really doing for 400 ms. A good pin shows
 *    ~20 pulses of ~1500 us. None means the firmware never drove it.
 *
 * 2. WHAT THE WIRE IS CONNECTED TO. Detaches, reads the pin with the internal
 *    pull-up, then pull-down:
 *      HIGH / LOW   high-impedance: a powered servo's input -- but also a
 *                   disconnected wire, or a servo whose ground isn't shared
 *      LOW  / LOW   held low: a servo with no power on its red wire clamps the
 *                   signal (or a short to GND, or the servo's own pull-down)
 *      HIGH / HIGH  shorted to a supply rail
 *
 * Servos end DETACHED and limp.   Serial @ 115200:  t  run again
 */
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "config.h"

struct Joint { const char *name; int pin; };
const Joint J[4] = {
  {"pan",      PIN_SERVO_PAN},
  {"shoulder", PIN_SERVO_SHOULDER},
  {"elbow",    PIN_SERVO_ELBOW},
  {"head",     PIN_SERVO_HEAD},
};
Servo sv[4];
#define FIRST_JOINT 1   // 1 = leave the base (pan) untouched; 0 = check all four

struct Pwm { int ch; unsigned long pulses; unsigned long avgUs; };

Pwm measure(int i) {
  const int pin = J[i].pin;
  Pwm r = {0, 0, 0};
  sv[i].setPeriodHertz(SERVO_HZ);
  r.ch = sv[i].attach(pin, SERVO_US_MIN, SERVO_US_MAX);
  sv[i].write(90);
  delay(80);
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[pin]);   // read the pad; PWM routing untouched
  unsigned long t0 = micros(), riseAt = 0, sum = 0, n = 0;
  int prev = gpio_get_level((gpio_num_t)pin);
  while (micros() - t0 < 400000UL) {
    int v = gpio_get_level((gpio_num_t)pin);
    if (v != prev) {
      unsigned long now = micros();
      if (v) { r.pulses++; riseAt = now; }
      else if (riseAt) { sum += now - riseAt; n++; }
      prev = v;
    }
  }
  PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[pin]);
  r.avgUs = n ? sum / n : 0;
  return r;
}

void lineTest(int i, int &pu, int &pd) {
  const int pin = J[i].pin;
  sv[i].detach();
  pu = pd = 0;
  for (int k = 0; k < 5; k++) {
    pinMode(pin, INPUT_PULLUP);   delay(20); pu += digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN); delay(20); pd += digitalRead(pin);
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);   // no pulses: servo stays limp and quiet
}

void runChecks() {
  Serial.println(F("\njoint     pin  pulses/400ms  avg high   | pull-up  pull-down  line"));
  int good = 0;
  for (int i = FIRST_JOINT; i < 4; i++) {
    Pwm p = measure(i);
    int pu, pd;
    lineTest(i, pu, pd);
    const char *lv = (pu == 5 && pd == 0) ? "high-impedance"
                   : (pu == 0 && pd == 0) ? "held LOW"
                   : (pu == 5 && pd == 5) ? "held HIGH" : "unstable";
    bool ok = p.pulses >= 15 && p.avgUs > 1300 && p.avgUs < 1700;
    if (ok) good++;
    Serial.printf("%-9s %3d  %6lu      %6lu us  |  %d/5      %d/5      %-15s %s\n",
                  J[i].name, J[i].pin, p.pulses, p.avgUs, pu, pd, lv,
                  ok ? "PWM ok" : "NO VALID PWM");
  }
  if (good == 4 - FIRST_JOINT) Serial.println(F("\nVERDICT: every checked pin outputs correct servo pulses -- firmware is fine"));
  else if (good == 0) Serial.println(F("\nVERDICT: no pin outputs valid pulses -- firmware/library, not wiring"));
  else                Serial.println(F("\nVERDICT: some pins pulse, some don't -- see the table"));
  Serial.println(F("servos now DETACHED (limp). 't' runs again."));
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nHOTARU servo check -- pulses at each pin + what each wire is on"));
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  runChecks();
}

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n');
    c.trim();
    if (c == "t") runChecks();
  }
  delay(10);
}
