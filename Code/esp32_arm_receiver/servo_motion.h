#pragma once
#include "servo_protocol.h"

namespace arm {
inline bool sameCommand(const Command& a, const Command& b) {
  for (uint8_t i = 0; i < 3; ++i) if (a.joint[i] != b.joint[i]) return false;
  return true;
}

// One shared trajectory for all three PWM channels. Integer microseconds allow
// small coordinated steps while UART targets remain whole-degree integers.
class ServoMotion {
 public:
  Command target = HOME;
  uint16_t currentPulseUs[3]{};
  uint32_t lastStepMs = 0;
  static constexpr uint32_t UPDATE_MS = 20; // Same update period as 50 Hz PWM.

  ServoMotion() { reset(HOME, 0); }

  void reset(const Command& command, uint32_t now) {
    target = command;
    for (uint8_t i = 0; i < 3; ++i) {
      currentPulseUs[i] = startPulseUs[i] = endPulseUs[i] = pulseUs(command.joint[i]);
    }
    elapsedMs = durationMs = 0;
    active = held = false;
    lastStepMs = now;
  }
  bool settled() const { return !active; }
  void retarget(const Command& command, uint32_t now, uint32_t msPerDegree) {
    // A heartbeat must not restart the trajectory or change its finish time.
    if (!held && sameCommand(command, target)) return;
    target = command;
    uint16_t longest = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      startPulseUs[i] = currentPulseUs[i];
      endPulseUs[i] = pulseUs(command.joint[i]);
      const uint16_t travel = startPulseUs[i] < endPulseUs[i] ?
          endPulseUs[i]-startPulseUs[i] : startPulseUs[i]-endPulseUs[i];
      if (travel > longest) longest = travel;
    }
    // 2000 us spans 180 degrees. Round duration UP to preserve the speed cap.
    durationMs = static_cast<uint32_t>((static_cast<uint64_t>(longest)*180U*msPerDegree+1999U)/2000U);
    elapsedMs = 0;
    lastStepMs = now;
    held = false;
    active = longest != 0;
  }
  void hold(uint32_t now) {
    // Retain the exact intermediate PWM widths, including sub-degree values.
    active = false;
    held = true;
    lastStepMs = now;
  }
  bool step(uint32_t now) {
    if (!active || static_cast<uint32_t>(now-lastStepMs) < UPDATE_MS) return false;
    // Do not replay missed frames through a large jump after an MCU stall.
    lastStepMs = now;
    elapsedMs = durationMs-elapsedMs <= UPDATE_MS ? durationMs : elapsedMs+UPDATE_MS;
    for (uint8_t i = 0; i < 3; ++i) {
      const int32_t travel = static_cast<int32_t>(endPulseUs[i])-startPulseUs[i];
      // Signed integer division truncates toward the start, so every moving
      // channel first reaches its final PWM value on the shared last frame.
      currentPulseUs[i] = static_cast<uint16_t>(startPulseUs[i]+static_cast<int64_t>(travel)*elapsedMs/durationMs);
    }
    if (elapsedMs == durationMs) active = false;
    return true;
  }

 private:
  uint16_t startPulseUs[3]{};
  uint16_t endPulseUs[3]{};
  uint32_t elapsedMs = 0;
  uint32_t durationMs = 0;
  bool active = false;
  bool held = false;
};
} // namespace arm
