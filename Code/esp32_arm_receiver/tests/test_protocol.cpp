#include "../servo_protocol.h"
#include <assert.h>
#include <iostream>
#include <string>

arm::Result feed(arm::FrameParser& parser, const std::string& text, arm::Command& result, uint32_t now=0) {
  arm::Result last = arm::Result::None;
  for (char byte : text) last = parser.feed(byte, now, result);
  return last;
}

int main() {
  assert(arm::HOME.joint[0] == 90 && arm::HOME.joint[1] == 45 && arm::HOME.joint[2] == 180);
  assert(arm::pulseUs(arm::HOME.joint[0]) == 1500);
  assert(arm::pulseUs(arm::HOME.joint[1]) == 1000);
  assert(arm::pulseUs(arm::HOME.joint[2]) == 2500);
  arm::FrameParser parser;
  arm::Command command{{90, 90, 90}};
  assert(feed(parser, "90,45,60\n", command) == arm::Result::Valid);
  assert(command.joint[0] == 90 && command.joint[1] == 45 && command.joint[2] == 60);
  for (auto valid : {"0,0,0\n", "180,180,180\n", "90,14,60\n", "90,166,60\n", " 90 , 45 , 60 \r\n"})
    assert(feed(parser, valid, command) == arm::Result::Valid);
  command = {{90,45,60}};
  for (auto invalid : {"", "90,-1,60", "90,181,60", "181,45,60", "-1,45,60", "+1,45,60",
                       "90.0,45,60", "90,45", "90,45,60,1", "nan,45,60", "9e1,45,60",
                       "90,,60", ",45,60", "90,45,60junk", "999999999,45,60", "9\r0,45,60"}) {
    assert(feed(parser, std::string(invalid)+"\n", command) == arm::Result::Invalid);
    assert(command.joint[0] == 90 && command.joint[1] == 45 && command.joint[2] == 60);
  }
  assert(feed(parser, std::string(200,'9')+",45,60\n", command) == arm::Result::Invalid);
  assert(feed(parser, "120,45,60\n", command) == arm::Result::Valid);
  assert(feed(parser, std::string("90,45,")+char(0)+"60\n", command) == arm::Result::Invalid);
  assert(command.joint[0] == 120);
  assert(feed(parser, "90,45", command, 100) == arm::Result::None);
  parser.expire(351);
  assert(feed(parser, ",60\n", command, 352) == arm::Result::Invalid);
  assert(command.joint[0] == 120);
  assert(feed(parser, "90,45,60\n", command, 353) == arm::Result::Valid);
  assert(feed(parser, "120,", command, UINT32_MAX-10) == arm::Result::None);
  assert(feed(parser, "45,60\n", command, 10) == arm::Result::Valid); // millis rollover
  for (int i = 0; i < 10000; ++i)
    assert(feed(parser, std::to_string(i%181)+",45,60\n", command) == arm::Result::Valid);
  assert(arm::pulseUs(0) == 500 && arm::pulseUs(90) == 1500 && arm::pulseUs(180) == 2500);
  assert(arm::pulseUs(15) == 667 && arm::pulseUs(165) == 2333);
  std::cout << "PASS: parser, atomic rejection, limits, framing, timeout, rollover, 10000 live frames, PWM mapping\n";
}
