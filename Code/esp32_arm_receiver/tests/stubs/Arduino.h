#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <deque>
#include <string>

#define CONFIG_IDF_TARGET_ESP32 1
#define OUTPUT 1
#define LOW 0
#define SERIAL_8N1 0
static uint32_t fakeMillis = 0;
static uint32_t fakeDuty[40]{};
static bool fakeAttached[40]{};
inline uint32_t millis() { return fakeMillis; }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}
inline void yield() {}
inline bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t bits) {
  if (freq != 50 || bits != 16) return false;
  fakeAttached[pin] = true;
  return true;
}
inline bool ledcDetach(uint8_t pin) { fakeAttached[pin] = false; return true; }
inline bool ledcWrite(uint8_t pin, uint32_t duty) {
  if (!fakeAttached[pin]) return false;
  fakeDuty[pin] = duty;
  return true;
}

class FakeSerial0 {
 public:
  std::deque<char> rx;
  std::string tx;
  size_t rxSize = 0;
  uint32_t baud = 0;
  void setRxBufferSize(size_t size) { rxSize = size; }
  void begin(uint32_t speed, int, int rxPin, int txPin) {
    baud = rxPin == 3 && txPin == 1 ? speed : 0;
  }
  void println(const char* text) { tx += std::string(text)+"\n"; }
  void printf(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    tx += buffer;
  }
  int available() { return static_cast<int>(rx.size()); }
  int read() { char byte = rx.front(); rx.pop_front(); return byte; }
  void inject(const std::string& text) { for (char byte : text) rx.push_back(byte); }
};
static FakeSerial0 Serial0;
