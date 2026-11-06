// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__TELEMETRY_DATA_HPP_
#define ASTRALL_DRIVER__TELEMETRY_DATA_HPP_

#include <chrono>
#include <cstdint>
#include <variant>

#include "astrall_sdk/interface.h"

namespace astrall_driver {

/** @brief S3 原始值类型；模式枚举与 SDK 结果码不同，不推测原始时钟单位。 */
using TelemetryData = std::variant<AstrallImuData, AstrallSportData,
        AstrallJoystickData, AstrallSystemStatus, AstrallPowerStatus,
        AstrallSportStatus, AstrallDeviceInfo>;

/** @brief 接收 ROS 时间 ns 用于消息头，稳态接收时间只用于超时丢弃。 */
struct TelemetrySample {
  TelemetryData data_;
  int64_t received_ros_ns_ = 0;
  uint64_t session_generation_ = 0;
  std::chrono::steady_clock::time_point received_steady_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__TELEMETRY_DATA_HPP_
