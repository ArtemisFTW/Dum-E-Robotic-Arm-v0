#pragma once
#include <stddef.h>
#include <stdint.h>

namespace arm {
constexpr uint16_t MIN_ANGLE[3] = {0, 0, 0};
constexpr uint16_t MAX_ANGLE[3] = {180, 180, 180};
constexpr uint32_t PARTIAL_FRAME_TIMEOUT_MS = 250;

struct Command { uint16_t joint[3]; };
constexpr Command HOME{{90, 45, 180}};

// OT5320M: 0..180 degrees -> 500..2500 microseconds, integer arithmetic.
inline uint16_t pulseUs(uint16_t angle) {
  return static_cast<uint16_t>(500U + (static_cast<uint32_t>(angle)*2000U + 90U)/180U);
}

inline void skipSpaces(const char* text, size_t size, size_t& index) {
  while (index < size && (text[index] == ' ' || text[index] == '\t')) ++index;
}

inline bool parse(const char* text, size_t size, Command& output) {
  Command candidate{};
  size_t index = 0;
  for (size_t joint = 0; joint < 3; ++joint) {
    skipSpaces(text, size, index);
    const size_t start = index;
    uint16_t value = 0;
    while (index < size && text[index] >= '0' && text[index] <= '9') {
      if (index-start >= 3) return false;
      value = static_cast<uint16_t>(value*10U + (text[index++]-'0'));
    }
    if (index == start || value < MIN_ANGLE[joint] || value > MAX_ANGLE[joint]) return false;
    candidate.joint[joint] = value;
    skipSpaces(text, size, index);
    if (joint != 2) {
      if (index >= size || text[index++] != ',') return false;
    }
  }
  if (index < size && text[index] == '\r') ++index;
  if (index != size) return false;
  output = candidate;  // Publish all three values only after the entire frame passes.
  return true;
}

enum class Result { None, Valid, Invalid };

class FrameParser {
 public:
  Result feed(char byte, uint32_t now, Command& output) {
    expire(now);
    lastByte_ = now;
    if (byte == '\n') {
      const bool valid = !discard_ && parse(buffer_, used_, output);
      used_ = 0;
      discard_ = false;
      return valid ? Result::Valid : Result::Invalid;
    }
    if (discard_) return Result::None;
    if (used_ == sizeof(buffer_) ||
        (byte != '\r' && byte != '\t' && (byte < ' ' || byte > '~'))) {
      discard_ = true;
      used_ = 0;
      return Result::None;
    }
    buffer_[used_++] = byte;
    return Result::None;
  }

  void expire(uint32_t now) {
    if (used_ && static_cast<uint32_t>(now-lastByte_) > PARTIAL_FRAME_TIMEOUT_MS) {
      used_ = 0;
      discard_ = true;  // Discard the rest of a stale frame through its newline.
    }
  }

 private:
  char buffer_[48]{};
  size_t used_ = 0;
  bool discard_ = false;
  uint32_t lastByte_ = 0;
};
}  // namespace arm
