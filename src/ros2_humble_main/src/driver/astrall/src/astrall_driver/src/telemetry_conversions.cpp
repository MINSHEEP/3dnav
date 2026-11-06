// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/telemetry_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "astrall_driver/fault_bits.hpp"

namespace astrall_driver {
namespace {

constexpr int64_t kNanosecondsPerSecond = 1000000000;
constexpr double kMinimumQuaternionNorm = 1e-6;
constexpr float kPercentScale = 100.0F;
constexpr int64_t kMaxStamp =
    int64_t{ std::numeric_limits<int32_t>::max() } *kNanosecondsPerSecond +
kNanosecondsPerSecond - 1;

template <typename Message>
std::optional<Message> WithHeader(int64_t stamp, const std::string& frame) {
  if (stamp < 0 || stamp > kMaxStamp || frame.empty()) {
    return std::nullopt;
  }
  Message message;
  message.header.stamp.sec =
      static_cast<int32_t>(stamp / kNanosecondsPerSecond);
  message.header.stamp.nanosec =
      static_cast<uint32_t>(stamp % kNanosecondsPerSecond);
  message.header.frame_id = frame;
  return message;
}

template <size_t N>
bool Finite(const float (& values)[N]) {
  return std::all_of(std::begin(values), std::end(values),
             [](float value) {
        return std::isfinite(value);
      });
}

}  // namespace

std::optional<sensor_msgs::msg::Imu> ConvertImu(
  const AstrallImuData& data, int64_t stamp_ns, const std::string& frame) {
  if (!Finite(data.accelerometer) || !Finite(data.gyroscope) ||
      !Finite(data.quaternion)) {
    return std::nullopt;
  }
  double squared_norm = 0;
  for (auto component : data.quaternion) {
    squared_norm += static_cast<double>(component) * component;
  }
  const double norm = std::sqrt(squared_norm);
  if (norm < kMinimumQuaternionNorm) {
    return std::nullopt;
  }
  auto message = WithHeader<sensor_msgs::msg::Imu>(stamp_ns, frame);
  if (!message) {
    return std::nullopt;
  }
  message->orientation.w = data.quaternion[0] / norm;
  message->orientation.x = data.quaternion[1] / norm;
  message->orientation.y = data.quaternion[2] / norm;
  message->orientation.z = data.quaternion[3] / norm;
  message->linear_acceleration.x = data.accelerometer[0];
  message->linear_acceleration.y = data.accelerometer[1];
  message->linear_acceleration.z = data.accelerometer[2];
  message->angular_velocity.x = data.gyroscope[0];
  message->angular_velocity.y = data.gyroscope[1];
  message->angular_velocity.z = data.gyroscope[2];
  return message;
}

std::optional<astrall_interfaces::msg::WheelSpeeds> ConvertWheelSpeeds(
  const AstrallSportData& data, int64_t stamp_ns, const std::string& frame) {
  if (!Finite(data.wheelSpeed)) {
    return std::nullopt;
  }
  auto message = WithHeader<astrall_interfaces::msg::WheelSpeeds>(stamp_ns,
          frame);
  if (message) {
    std::copy(std::begin(data.wheelSpeed), std::end(data.wheelSpeed),
        message->wheel_speed.begin());
    message->sdk_timestamp = data.timestamp;
  }
  return message;
}

std::optional<astrall_interfaces::msg::JoystickState> ConvertJoystick(
  const AstrallJoystickData& data, int64_t stamp_ns, const std::string& frame) {
  auto message = WithHeader<astrall_interfaces::msg::JoystickState>(stamp_ns,
          frame);
  if (!message) {
    return std::nullopt;
  }
  message->sdk_timestamp = data.timestamp;
  std::copy(std::begin(data.rocker),
      std::end(data.rocker),
      message->rocker.begin());
  message->l1 = data.l1;
  message->l2 = data.l2;
  message->l3 = data.l3;
  message->r1 = data.r1;
  message->r2 = data.r2;
  message->r3 = data.r3;
  message->sl = data.sl;
  message->sr = data.sr;
  message->h = data.h;
  message->switch_position = data.sw;
  return message;
}

std::optional<astrall_interfaces::msg::SystemStatus> ConvertSystemStatus(
  const AstrallSystemStatus& data, int64_t stamp_ns, const std::string& frame) {
  auto message = WithHeader<astrall_interfaces::msg::SystemStatus>(stamp_ns,
          frame);
  if (message) {
    message->system_status = data.sysStatus;
    message->error_code = data.errorCode;
    message->warn_code = data.warnCode;
    message->error_messages = DecodeErrorBits(data.errorCode);
    message->warn_messages = DecodeWarnBits(data.warnCode);
  }
  return message;
}

std::optional<astrall_interfaces::msg::PowerStatus> ConvertPowerStatus(
  const AstrallPowerStatus& data, int64_t stamp_ns, const std::string& frame) {
  if (!std::isfinite(data.soc) || !std::isfinite(data.temp) ||
      !std::isfinite(data.voltage) || data.soc < 0 ||
      data.soc > kPercentScale ||
      data.voltage < 0) {
    return std::nullopt;
  }
  auto message = WithHeader<astrall_interfaces::msg::PowerStatus>(stamp_ns,
          frame);
  if (message) {
    message->percentage = data.soc / kPercentScale;
    message->temperature = data.temp;
    message->voltage = data.voltage;
    message->cycle_count = data.cycleCount;
    message->charged = data.charged;
  }
  return message;
}

std::optional<astrall_interfaces::msg::SportStatus> ConvertSportStatus(
  AstrallSportStatus data, int64_t stamp_ns, const std::string& frame) {
  auto message = WithHeader<astrall_interfaces::msg::SportStatus>(stamp_ns,
          frame);
  if (message) {
    message->status = data;
  }
  return message;
}

std::optional<astrall_interfaces::msg::DeviceInfo> ConvertDeviceInfo(
  const AstrallDeviceInfo& data, int64_t stamp_ns, const std::string& frame) {
  auto message =
      WithHeader<astrall_interfaces::msg::DeviceInfo>(stamp_ns, frame);
  if (message) {
    message->version = data.version;
    message->serial_number = data.serialNumber;
    message->model = data.model;
  }
  return message;
}

}  // namespace astrall_driver
