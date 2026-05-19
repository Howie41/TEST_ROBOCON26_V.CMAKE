/** 
 * @file Hwt101.cpp
 * @author YE
 * @brief HWT101 IMU传感器驱动源文件
 * @version 0.1
 * @date 2026-05-12
 *
 */
#include "Hwt101.hpp"

namespace {

static float decodeAngleDeg(uint8_t lo, uint8_t hi) {
  const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) |
                                           static_cast<uint16_t>(lo));
  return static_cast<float>(raw) / 32768.0f * 180.0f;
}

} // namespace

void Hwt101Parser::reset() { index_ = 0; }

bool Hwt101Parser::processByte(uint8_t byte) {
  if (index_ == 0U) {
    if (byte != 0x55U) {
      return false;
    }
    frame_[index_++] = byte;
    return false;
  }

  if (index_ == 1U) {
    if (byte == 0x55U) {
      frame_[0] = byte;
      return false;
    }
    frame_[index_++] = byte;
    return false;
  }

  frame_[index_++] = byte;
  if (index_ < kFrameSize) {
    return false;
  }

  uint8_t sum = 0U;
  for (uint8_t i = 0; i < kFrameSize - 1U; ++i) {
    sum = static_cast<uint8_t>(sum + frame_[i]);
  }

  const bool valid = (sum == frame_[kFrameSize - 1U]);
  if (valid && frame_[1] == 0x53U) {
    parseAngleFrame();
  }

  reset();
  return valid;
}

void Hwt101Parser::parseAngleFrame() {
  //roll_deg_ = decodeAngleDeg(frame_[2], frame_[3]);
 // pitch_deg_ = decodeAngleDeg(frame_[4], frame_[5]);
  yaw_deg_ = decodeAngleDeg(frame_[6], frame_[7]);
  frame_count_++;
}
