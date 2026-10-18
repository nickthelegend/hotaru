#pragma once
/*
 * config.h — HOTARU pin map and hardware constants. SINGLE SOURCE OF TRUTH.
 * Change a pin here and nowhere else.
 *
 * BOARD: ESP32-S3 N16R8 (ROBODUINO, dual USB-C). 16 MB flash, 8 MB OCTAL PSRAM.
 * FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc
 *
 * The S3 map shares almost nothing with the classic ESP32's. Rules that
 * shaped it:
 *   GPIO 22,23,24,25  DO NOT EXIST on the S3. The S3 is 0..21 and 26..48.
 *   GPIO 26..32       SPI flash, in-package. Never.
 *   GPIO 33..37       OCTAL PSRAM on an -R8 part. Never on this board.
 *                     (Free on quad-PSRAM parts like N8R2 — not this one.)
 *   GPIO 19,20        native USB D-/D+. Leave them to the USB-C port.
 *   GPIO 43,44        UART0 = the serial port you flash and debug over.
 *   GPIO 0,3,45,46    strapping. 45 sets VDD_SPI; 46 is boot mode.
 *   GPIO 48           the devkit's onboard RGB LED on most S3 boards.
 *   GPIO 39..42       JTAG. Usable, but you lose hardware debug.
 *
 * That leaves 1,2,4..18,21,38,47 genuinely free. Everything below is in it.
 *
 * Unlike the classic ESP32 there are no input-only pins — every GPIO here
 * can drive as well as read.
 */

// ---------------------------------------------------------------- servos --
// Physical order up the lamp. PAN is the one at the bottom, in the tub,
// spline pointing up; HEAD is the one at the top in the cone's block.
#define PIN_SERVO_PAN        4   // tub, turntable      z  2.4 .. 45.3
#define PIN_SERVO_SHOULDER   5   // tower on the disc   z 64.8 ..105.5
#define PIN_SERVO_ELBOW      6   // link1's far end     z164.8 ..205.5
#define PIN_SERVO_HEAD       7   // head block, cone    z254.8 ..295.5

// ------------------------------------------------------------- WS2812 ---
#define PIN_RING_DI         15   // -> ring DI (its DO stays unconnected)
#define RING_LEDS           16
// 16 x 5050 at full white is ~0.96 A. This cap keeps the ring under ~0.35 A
// so it cannot fight the servos for the 5 V rail. Raise it only if the ring
// has its own feed.
#define RING_MAX_BRIGHT     80   // 0..255

// ------------------------------------------------------------ MX switch --
// 8 and 9 are left free on purpose -- they are the S3's default I2C pair,
// so an IMU or ToF sensor drops straight on later.
#define PIN_MX_KEY          16   // other leg to GND; INPUT_PULLUP, active LOW

// ------------------------------------- I2S mic: INMP441 (RX, I2S port 0) --
#define PIN_MIC_SCK         10
#define PIN_MIC_WS          11
#define PIN_MIC_SD          12   // data in from the mic
#define MIC_SAMPLE_RATE     16000

// -------------------------------- I2S amp: MAX98357A (TX, I2S port 1) --
#define PIN_AMP_BCLK        13
#define PIN_AMP_LRC         14
#define PIN_AMP_DIN         21
#define AMP_SAMPLE_RATE     16000

// ----------------------------------------------------------- servo timing --
#define SERVO_US_MIN        500   // MG996R full travel. The library default
#define SERVO_US_MAX        2500  // of 544..2400 loses a few degrees each end.
#define SERVO_HZ            50
#define FRAME_US            (1000000UL / SERVO_HZ)

// ------------------------------------------------------------ joint limits --
// Software end stops. A stalled MG996R pulls 2.5 A and will brown out the
// whole lamp, so every commanded angle is clamped BEFORE it reaches a servo.
// Widen these only after checking the joint physically clears at the new value.
#define PAN_MIN        10.0f
#define PAN_MAX       170.0f
#define SHOULDER_MIN   25.0f
#define SHOULDER_MAX  155.0f
#define ELBOW_MIN      20.0f
#define ELBOW_MAX     160.0f
#define HEAD_MIN       30.0f
#define HEAD_MAX      150.0f

// ------------------------------------------------------------ calibration --
// Servo degrees added to every write so that 90 is each joint's true neutral
// pose -- the horn goes onto the spline at whatever tooth it goes on. Find
// them with the servo bench (s3_servo): select a joint, jog it with + / -
// until it sits at neutral, press h, then k prints these lines to paste.
// More than ~7 deg means the horn is a whole tooth out (MG996R spline: 25T,
// 14.4 deg per tooth) -- re-seat it rather than trimming.
// The limits above are in calibrated degrees: re-check them after trimming.
#define PAN_TRIM        -19.0f
#define SHOULDER_TRIM   -6.0f
#define ELBOW_TRIM      -62.0f
#define HEAD_TRIM       -26.0f

#define DEFAULT_SPEED 120.0f   // deg/s
