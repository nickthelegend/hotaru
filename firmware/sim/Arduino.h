// Just enough Arduino for hotaru_motion.ino to compile and run on a PC,
// freestanding: no C library, built to WebAssembly and run by sim.mjs.
// The clock only moves when the harness moves it (sim_us).
#pragma once
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#define IMPORT(n) __attribute__((import_module("env"), import_name(n)))
extern "C" {
IMPORT("sinf") float sinf(float);
IMPORT("cosf") float cosf(float);
void* memcpy(void* d, const void* s, size_t n);
}
inline float sqrtf(float x) { return __builtin_sqrtf(x); }
inline float fabsf(float x) { return __builtin_fabsf(x); }
inline float ceilf(float x) { return __builtin_ceilf(x); }
inline long lroundf(float x) { return (long)(x < 0 ? x - 0.5f : x + 0.5f); }

template <class T> T min(T a, T b) { return b < a ? b : a; }
template <class T> T max(T a, T b) { return a < b ? b : a; }
inline float min(float a, double b) { return min(a, (float)b); }
inline float max(float a, double b) { return max(a, (float)b); }
template <class T, class L, class H> T constrain(T v, L lo, H hi) {
  return v < lo ? (T)lo : (v > hi ? (T)hi : v);
}
#define F(s) (s)
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1

extern uint32_t sim_us;
inline uint32_t micros() { return sim_us; }
inline uint32_t millis() { return sim_us / 1000; }
inline void delay(uint32_t ms) { sim_us += ms * 1000; }
inline void pinMode(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline int sscanf(const char*, const char*, ...) { return 0; }

// Serial never has input here, so the command parser is compiled but never
// entered; String only has to type-check.
struct String {
  char s[64] = {0};
  bool startsWith(const char*) const { return false; }
  String substring(size_t) const { return *this; }
  float toFloat() const { return 0; }
  const char* c_str() const { return s; }
  void trim() {}
  size_t length() const { return 0; }
  bool operator==(const char*) const { return false; }
  bool operator==(const String&) const { return false; }
};
struct SerialT {
  void begin(long) {}
  int available() { return 0; }
  String readStringUntil(char) { return String(); }
  void println(const char* = "") {}
  void printf(const char*, ...) {}
};
extern SerialT Serial;
struct WireT {
  void begin(int, int, long) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission() { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 1; }
  int available() { return 1; }
  int read() { return 0; }
};
extern WireT Wire;
