#include <assert.h>
#include <cmath>
#include <iostream>
#include "../esp32_arm_receiver.ino"

bool pulseNear(uint8_t pin, uint16_t microseconds) {
  return std::abs(static_cast<double>(fakeDuty[pin])*20000/65536-microseconds) < .2;
}

void moveTo(const std::string& frame, const arm::Command& expected) {
  const uint32_t began = fakeMillis;
  const uint16_t start[3] = {motion.currentPulseUs[0],motion.currentPulseUs[1],motion.currentPulseUs[2]};
  for (int count = 0; count < 400 && !motion.settled(); ++count) {
    fakeMillis += arm::ServoMotion::UPDATE_MS;
    Serial0.inject(frame); // Heartbeats must not stall or restart the ramp.
    loop();
    for (uint8_t i = 0; i < 3; ++i) {
      assert(pulseNear(SERVO_PINS[i],motion.currentPulseUs[i]));
      const uint16_t end = arm::pulseUs(expected.joint[i]);
      if (start[i] != end) assert((motion.currentPulseUs[i] == end) == motion.settled());
    }
  }
  assert(arm::sameCommand(motion.target, expected));
  for (uint8_t i = 0; i < 3; ++i) assert(motion.currentPulseUs[i] == arm::pulseUs(expected.joint[i]));
  assert(motion.settled() && !linkTimedOut);
  assert(fakeMillis >= began);
}

int main() {
  setup();
  assert(Serial0.baud == 115200 && Serial0.rxSize == 512);
  assert(pulseNear(25,1500) && pulseNear(26,1000) && pulseNear(27,2500));
  assert(Serial0.tx.find("HOLD,STARTUP_HOME,90,45,180") != std::string::npos);
  assert(bootHomeActive && haveCommand && !pwmFault);
  fakeMillis = 100;
  Serial0.inject("120,60,90\n");
  loop();
  assert(lastCommand.joint[0] == 90 && lastCommand.joint[1] == 45 && lastCommand.joint[2] == 180);
  assert(Serial0.tx.find("OK,") == std::string::npos); // Input during startup is discarded.
  fakeMillis = 2000;
  loop();
  assert(!bootHomeActive && Serial0.rx.empty());
  assert(Serial0.tx.find("READY,ARM_UART0,115200") != std::string::npos);
  fakeMillis = 2001;
  Serial0.inject("90,45,60\n");
  loop();
  assert(lastCommand.joint[2] == 60 && pulseNear(27,2500)); // No instant jump.
  assert(Serial0.tx.find("OK,90,45,60\n") != std::string::npos);
  fakeMillis = 2020;
  loop();
  assert(motion.currentPulseUs[2] == 2500); // First PWM update interval not elapsed.
  fakeMillis = 2021;
  loop();
  assert(SERVO_STEP_INTERVAL_MS == 20);
  assert(motion.currentPulseUs[2] == 2489 && !motion.settled());
  moveTo("90,45,60\n", arm::Command{{90,45,60}});
  assert(pulseNear(27,1167));
  assert(Serial0.tx.find("DONE,90,45,60\n") != std::string::npos);
  const uint32_t before[3] = {fakeDuty[25],fakeDuty[26],fakeDuty[27]};
  fakeMillis += 100;
  Serial0.inject("10,181,90\n90.0,45,60\n");
  loop();
  assert(fakeDuty[25] == before[0] && fakeDuty[26] == before[1] && fakeDuty[27] == before[2]);
  assert(lastCommand.joint[0] == 90 && lastCommand.joint[2] == 60);
  fakeMillis += 1001;
  loop();
  assert(linkTimedOut && fakeAttached[25] && fakeAttached[26] && fakeAttached[27]);
  assert(fakeDuty[25] == before[0] && fakeDuty[26] == before[1] && fakeDuty[27] == before[2]);
  assert(Serial0.tx.find("HOLD,LINK_TIMEOUT") != std::string::npos);
  Serial0.inject("120,90,0\n");
  loop();
  assert(!linkTimedOut && lastCommand.joint[0] == 120 && lastCommand.joint[2] == 0);
  fakeMillis += arm::ServoMotion::UPDATE_MS;
  loop();
  assert(motion.currentPulseUs[0] > 1500 && motion.currentPulseUs[1] > 1000 && motion.currentPulseUs[2] < 1167);
  assert(Serial0.tx.find("DONE,120,90,0\n") == std::string::npos);
  fakeMillis += LINK_TIMEOUT_MS+1;
  loop();
  const uint16_t frozen[3] = {motion.currentPulseUs[0],motion.currentPulseUs[1],motion.currentPulseUs[2]};
  fakeMillis += 500;
  loop();
  for (uint8_t i = 0; i < 3; ++i) assert(motion.currentPulseUs[i] == frozen[i]); // Freeze mid-move.
  Serial0.inject("0,0,0\n");
  loop();
  moveTo("0,0,0\n", arm::Command{{0,0,0}});
  assert(pulseNear(25,500) && pulseNear(26,500) && pulseNear(27,500));
  Serial0.inject("90,45,180\n"); // Unequal joint travel must finish on one common frame.
  loop();
  moveTo("90,45,180\n", arm::Command{{90,45,180}});
  assert(Serial0.tx.find("DONE,90,45,180\n") != std::string::npos);
  Serial0.inject("180,180,180\n");
  loop();
  moveTo("180,180,180\n", arm::Command{{180,180,180}});
  assert(pulseNear(25,2500) && pulseNear(26,2500) && pulseNear(27,2500));
  std::cout << "PASS: startup home/dwell, coordinated PWM, target ACK/DONE, heartbeat continuity, invalid-frame hold, mid-move timeout freeze, endpoints\n";
}
