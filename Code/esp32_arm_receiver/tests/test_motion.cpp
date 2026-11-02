#include "../servo_motion.h"
#include <assert.h>
#include <cstdlib>
#include <iostream>

bool atTarget(const arm::ServoMotion& motion) {
  for (uint8_t i = 0; i < 3; ++i)
    if (motion.currentPulseUs[i] != arm::pulseUs(motion.target.joint[i])) return false;
  return true;
}

int main() {
  arm::ServoMotion motion;
  motion.reset(arm::Command{{90,45,180}}, 0);
  motion.retarget(arm::Command{{180,0,0}}, 0, 40);
  for (uint32_t time = 0; time <= 7200; ++time) {
    if (time % 50 == 0) motion.retarget(arm::Command{{180,0,0}}, time, 40);
    motion.step(time);
    if (time == 20) { // All three channels start in the same PWM update.
      assert(motion.currentPulseUs[0] > 1500);
      assert(motion.currentPulseUs[1] < 1000);
      assert(motion.currentPulseUs[2] < 2500);
    }
    if (time == 3600) { // Unequal 90/45/180-degree moves are all halfway.
      assert(motion.currentPulseUs[0] == 2000);
      assert(motion.currentPulseUs[1] == 750);
      assert(motion.currentPulseUs[2] == 1500);
    }
    for (uint8_t i = 0; i < 3; ++i) {
      if (time < 7200) assert(motion.currentPulseUs[i] != arm::pulseUs(motion.target.joint[i]));
      else assert(motion.currentPulseUs[i] == arm::pulseUs(motion.target.joint[i]));
    }
  }
  assert(motion.settled());
  motion.retarget(arm::Command{{0,180,180}}, 7200, 40);
  motion.step(10000); // A delayed MCU must advance only one PWM update.
  assert(motion.currentPulseUs[0] == 2495 && motion.currentPulseUs[2] == 505);
  motion.hold(10001);
  assert(!motion.step(20000));
  motion.retarget(arm::Command{{0,180,180}}, 20000, 40); // Resume SAME target after link hold.
  assert(!motion.settled() && motion.currentPulseUs[0] == 2495);
  motion.step(20020);
  assert(motion.currentPulseUs[0] < 2495);
  const uint16_t intermediate = motion.currentPulseUs[0];
  motion.retarget(arm::Command{{90,90,90}}, 20021, 40); // Mid-move retarget starts here.
  assert(motion.currentPulseUs[0] == intermediate);
  assert(!motion.step(20040));
  assert(motion.step(20041));
  motion.reset(arm::Command{{90,45,180}}, UINT32_MAX-10);
  motion.retarget(arm::Command{{91,45,180}}, UINT32_MAX-10, 40);
  assert(!motion.step(8));
  assert(motion.step(9) && !motion.settled());
  assert(motion.step(29) && motion.settled() && atTarget(motion));

  // New 50 deg/s setting halves the same coordinated move to 3.6 seconds.
  motion.reset(arm::Command{{90,45,180}}, 0);
  motion.retarget(arm::Command{{180,0,0}}, 0, 20);
  for (uint32_t time = 20; time <= 3600; time += 20) {
    motion.retarget(arm::Command{{180,0,0}}, time, 20);
    motion.step(time);
    if (time == 1800) {
      assert(motion.currentPulseUs[0] == 2000);
      assert(motion.currentPulseUs[1] == 750);
      assert(motion.currentPulseUs[2] == 1500);
    }
    assert(motion.settled() == (time == 3600));
  }
  assert(atTarget(motion));

  // Independent invariants across varied travel lengths and directions:
  // bounded PWM changes, monotonic travel, and no moving axis finishes early.
  std::srand(45);
  for (int trial = 0; trial < 400; ++trial) {
    const uint32_t msPerDegree = trial < 200 ? 40 : 20;
    arm::Command start{}, goal{};
    for (uint8_t i = 0; i < 3; ++i) {
      start.joint[i] = std::rand()%181;
      goal.joint[i] = std::rand()%181;
    }
    motion.reset(start, 0);
    motion.retarget(goal, 0, msPerDegree);
    for (uint32_t time = 20; time <= 7240 && !motion.settled(); time += 20) {
      uint16_t before[3];
      for (uint8_t i = 0; i < 3; ++i) before[i] = motion.currentPulseUs[i];
      motion.retarget(goal, time, msPerDegree);
      motion.step(time);
      for (uint8_t i = 0; i < 3; ++i) {
        const int32_t after = motion.currentPulseUs[i];
        const int32_t end = arm::pulseUs(goal.joint[i]);
        assert(std::abs(after-before[i]) <= (msPerDegree == 40 ? 6 : 12)); // Integer-us quantization.
        assert(after >= 500 && after <= 2500);
        if (start.joint[i] < goal.joint[i]) assert(after >= before[i] && after <= end);
        else assert(after <= before[i] && after >= end);
        if (start.joint[i] != goal.joint[i]) assert((after == end) == motion.settled());
      }
    }
    assert(motion.settled() && atTarget(motion));
  }
  std::cout << "PASS: shared finish, integer PWM, 25/50 deg/s caps, 400 synchronized moves, heartbeats, retarget, hold/resume, no catch-up and rollover\n";
}
