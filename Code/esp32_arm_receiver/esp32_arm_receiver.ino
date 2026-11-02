/* Classic ESP32/WROOM, Arduino-ESP32 3.x. No external servo library.
 * UART0: RX GPIO3 / TX GPIO1, 115200 8N1. PC uses the board's USB-UART bridge.
 * Input: J1,J2,J3\n, integers already converted to servo coordinates by the app.
 * J1 direction is corrected in app/MATLAB mapping (J1=90-theta1).
 * Do not invert J1 again here: UART values are the final servo commands.
 * PWM: OT5320M, GPIO25/26/27, 50 Hz, 500..2500 us.
 */
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "servo_protocol.h"
#include "servo_motion.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "This pin assignment targets the classic ESP32/WROOM, not C3/S3/etc."
#endif
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Install esp32 by Espressif Systems version 3.x in Boards Manager."
#endif

constexpr uint8_t SERVO_PINS[3] = {25, 26, 27};
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t PWM_HZ = 50;
constexpr uint8_t PWM_BITS = 16;
constexpr uint32_t PWM_PERIOD_US = 1000000UL/PWM_HZ;
constexpr uint32_t LINK_TIMEOUT_MS = 1000;
constexpr uint32_t HOME_SETTLE_MS = 2000;
// Longest-travel joint sets the shared duration. Others slow down to finish
// with it. Increase this to slow all moves: 20 ms/degree = max 50 deg/s.
// Intermediate delay between the original direct target (0) and prior 40 ms/deg.
constexpr uint32_t SERVO_STEP_INTERVAL_MS = 20;
static_assert(SERVO_STEP_INTERVAL_MS > 0, "Servo speed interval must be positive.");

arm::FrameParser parser;
arm::Command lastCommand = arm::HOME;
arm::ServoMotion motion;
bool pwmReady = false;
bool haveCommand = false;
bool linkTimedOut = false;
bool pwmFault = false;
uint32_t lastValidMs = 0;
uint32_t lastErrorMs = 0;
uint32_t homeStartedMs = 0;
bool bootHomeActive = false;

bool attachPwm() {
  for (uint8_t i = 0; i < 3; ++i) {
    if (!ledcAttach(SERVO_PINS[i], PWM_HZ, PWM_BITS)) {
      for (uint8_t j = 0; j < i; ++j) ledcDetach(SERVO_PINS[j]);
      for (uint8_t pin : SERVO_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
      return false;
    }
  }
  return true;
}

bool applyPulses(const uint16_t* pulseWidthsUs) {
  if (pwmFault) return false;
  if (!pwmReady) {
    pwmReady = attachPwm();
    if (!pwmReady) { pwmFault = true; return false; }
  }
  for (uint8_t i = 0; i < 3; ++i) {
    const uint32_t duty = (static_cast<uint64_t>(pulseWidthsUs[i]) *
                           (1UL << PWM_BITS) + PWM_PERIOD_US/2)/PWM_PERIOD_US;
    if (!ledcWrite(SERVO_PINS[i], duty)) { pwmFault = true; return false; }
  }
  return true;
}

bool applyCommand(const arm::Command& command) {
  uint16_t pulseWidthsUs[3];
  for (uint8_t i = 0; i < 3; ++i) pulseWidthsUs[i] = arm::pulseUs(command.joint[i]);
  return applyPulses(pulseWidthsUs);
}

void reportDone() {
  Serial0.printf("DONE,%u,%u,%u\n", static_cast<unsigned>(motion.target.joint[0]),
                 static_cast<unsigned>(motion.target.joint[1]), static_cast<unsigned>(motion.target.joint[2]));
}

void setup() {
  for (uint8_t pin : SERVO_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial0.setRxBufferSize(512);  // Must precede begin().
  Serial0.begin(UART_BAUD, SERIAL_8N1, 3, 1);
  // Requested startup/reset behavior: command home before accepting live moves.
  if (applyCommand(arm::HOME)) {
    // Initial shaft positions are unknown at reset; only subsequent moves can
    // be slewed from a known commanded PWM value.
    motion.reset(arm::HOME, millis());
    haveCommand = true;
    homeStartedMs = lastValidMs = millis();
    bootHomeActive = true;
    Serial0.printf("HOLD,STARTUP_HOME,%u,%u,%u\n", static_cast<unsigned>(arm::HOME.joint[0]),
                   static_cast<unsigned>(arm::HOME.joint[1]), static_cast<unsigned>(arm::HOME.joint[2]));
  } else {
    Serial0.println("ERR,PWM_FAILURE_RESTART_REQUIRED");
  }
}

void loop() {
  const uint32_t loopNow = millis();
  if (bootHomeActive && static_cast<uint32_t>(loopNow-homeStartedMs) >= HOME_SETTLE_MS) {
    bootHomeActive = false;
    lastValidMs = loopNow;
    Serial0.println("READY,ARM_UART0,115200");
    reportDone();
  }
  parser.expire(loopNow);
  // Bounded work per loop; no readString/parseInt/delay-based motion or sweep.
  for (uint16_t budget = 0; budget < 256 && Serial0.available(); ++budget) {
    arm::Command candidate{};
    const uint32_t now = millis();
    const auto result = parser.feed(static_cast<char>(Serial0.read()), now, candidate);
    if (result == arm::Result::Valid) {
      if (bootHomeActive) continue;  // Drain input; never replay moves queued during home.
      if (pwmFault) {
        if (static_cast<uint32_t>(now-lastErrorMs) >= 250) {
          Serial0.println("ERR,PWM_FAILURE_RESTART_REQUIRED");
          lastErrorMs = now;
        }
        continue;
      }
      motion.retarget(candidate, now, SERVO_STEP_INTERVAL_MS);
      lastCommand = candidate;
      lastValidMs = now;
      haveCommand = true;
      linkTimedOut = false;
      // Target accepted. DONE is emitted once the PWM setpoint reaches it.
      Serial0.printf("OK,%u,%u,%u\n", static_cast<unsigned>(lastCommand.joint[0]),
                     static_cast<unsigned>(lastCommand.joint[1]), static_cast<unsigned>(lastCommand.joint[2]));
      if (motion.settled()) reportDone();
    } else if (result == arm::Result::Invalid && static_cast<uint32_t>(now-lastErrorMs) >= 250) {
      Serial0.println("ERR,INVALID_FRAME_OR_JOINT_LIMIT");
      lastErrorMs = now;
    }
  }
  if (haveCommand && !bootHomeActive && !linkTimedOut && static_cast<uint32_t>(millis()-lastValidMs) > LINK_TIMEOUT_MS) {
    linkTimedOut = true;
    motion.hold(millis());
    Serial0.println("HOLD,LINK_TIMEOUT");
    // LEDC keeps the last pulse widths active. Do not detach or move home.
  }
  if (!bootHomeActive && !linkTimedOut && !pwmFault && motion.step(millis())) {
    if (!applyPulses(motion.currentPulseUs)) {
      Serial0.println("ERR,PWM_FAILURE_RESTART_REQUIRED");
    } else if (motion.settled()) {
      reportDone(); // Commanded PWM arrival only, not encoder feedback.
    }
  }
  yield();
}
