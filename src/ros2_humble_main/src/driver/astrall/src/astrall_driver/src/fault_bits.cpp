// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/fault_bits.hpp"

#include <array>
#include <iomanip>
#include <sstream>

#include "astrall_sdk/interface.h"

namespace astrall_driver {
namespace {

struct BitLabel {
  uint32_t bit_;
  const char* label_;
};

constexpr std::array<BitLabel, 10> kErrors = { {
  { ASTRALL_ERR_MOTOR_VERSION,
    "ASTRALL_ERR_MOTOR_VERSION: 电机驱动版本错误" },
  { ASTRALL_ERR_MOTOR_COMM_LOSE,
    "ASTRALL_ERR_MOTOR_COMM_LOSE: 电机通讯丢失" },
  { ASTRALL_ERR_MOTOR_STATUS,
    "ASTRALL_ERR_MOTOR_STATUS: 电机异常" },
  { ASTRALL_ERR_MOTOR_OVER_TEMP,
    "ASTRALL_ERR_MOTOR_OVER_TEMP: 电机过温" },
  { ASTRALL_ERR_STO_TRIGGERED,
    "ASTRALL_ERR_STO_TRIGGERED: STO 触发" },
  { ASTRALL_ERR_IMU_ERROR,
    "ASTRALL_ERR_IMU_ERROR: IMU 错误" },
  { ASTRALL_ERR_LOW_BATTERY_LEVEL,
    "ASTRALL_ERR_LOW_BATTERY_LEVEL: 电量过低" },
  { ASTRALL_ERR_OVER_LOAD,
    "ASTRALL_ERR_OVER_LOAD: 电机过载" },
  { ASTRALL_ERR_MBUS_DRIVER_FAIL,
    "ASTRALL_ERR_MBUS_DRIVER_FAIL: 驱动版本错误" },
  { ASTRALL_ERR_LOSE_CAMERA,
    "ASTRALL_ERR_LOSE_CAMERA: 未发现相机设备" }
} };

constexpr std::array<BitLabel, 8> kWarnings = { {
  { ASTRALL_WARN_MOTOR_OVER_TEMP,
    "ASTRALL_WARN_MOTOR_OVER_TEMP: 电机即将过温" },
  { ASTRALL_WARN_JOYSTICK_LOCK,
    "ASTRALL_WARN_JOYSTICK_LOCK: 遥控器已锁定" },
  { ASTRALL_INFO_JOYSTICK_INTERVENTION,
    "ASTRALL_INFO_JOYSTICK_INTERVENTION: 遥控器已获取控制权" },
  { ASTRALL_WARN_MOTOR_COMM_UNSTABLE,
    "ASTRALL_WARN_MOTOR_COMM_UNSTABLE: 电机通讯不稳定" },
  { ASTRALL_WARN_LOW_BATTERY_LEVEL,
    "ASTRALL_WARN_LOW_BATTERY_LEVEL: 电量不足" },
  { ASTRALL_WARN_JOYSTICK_LOSE,
    "ASTRALL_WARN_JOYSTICK_LOSE: 遥控器离线" },
  { ASTRALL_WARN_MOTOR_OVER_LOAD,
    "ASTRALL_WARN_MOTOR_OVER_LOAD: 电机即将过载" },
  { ASTRALL_WARNING_CONNECT_SDK_LOSE,
    "ASTRALL_WARNING_CONNECT_SDK_LOSE: 已连接 SDK 离线" }
} };

template <size_t N>
std::vector<std::string> Decode(uint32_t bits,
    const std::array<BitLabel, N>& labels,
    const char* empty, const char* unknown_name) {
  if (bits == 0) {
    return { empty };
  }
  uint32_t known = 0;
  std::vector<std::string> result;
  for (const auto& label : labels) {
    known |= label.bit_;
    if (bits & label.bit_) {
      result.emplace_back(label.label_);
    }
  }
  const auto unknown = bits & ~known;
  if (unknown != 0) {
    constexpr int kHexWidth = 8;
    std::ostringstream text;
    text << unknown_name << " (0x" << std::uppercase << std::hex
         << std::setw(kHexWidth) << std::setfill('0') << unknown << ')';
    result.push_back(text.str());
  }
  return result;
}

}  // namespace

std::vector<std::string> DecodeErrorBits(uint32_t bits) {
  return Decode(bits, kErrors,
             "ASTRALL_ERR_NONE: 无错误", "UNKNOWN_ERROR_BITS");
}

std::vector<std::string> DecodeWarnBits(uint32_t bits) {
  return Decode(bits, kWarnings,
             "ASTRALL_WARN_NONE: 无警告", "UNKNOWN_WARN_BITS");
}

}  // namespace astrall_driver
