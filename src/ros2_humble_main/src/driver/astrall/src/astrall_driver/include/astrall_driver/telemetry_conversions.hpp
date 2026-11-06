// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__TELEMETRY_CONVERSIONS_HPP_
#define ASTRALL_DRIVER__TELEMETRY_CONVERSIONS_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "astrall_interfaces/msg/device_info.hpp"
#include "astrall_interfaces/msg/joystick_state.hpp"
#include "astrall_interfaces/msg/power_status.hpp"
#include "astrall_interfaces/msg/sport_status.hpp"
#include "astrall_interfaces/msg/system_status.hpp"
#include "astrall_interfaces/msg/wheel_speeds.hpp"
#include "astrall_sdk/interface.h"
#include "sensor_msgs/msg/imu.hpp"

namespace astrall_driver {

/**
 * @brief 手册 5.10；接收 ROS 时间 ns，四元数 wxyz→xyzw 并归一化。
 * 仅使用 SI 加速度/角速度与四元数；拒绝非有限/零范数，不使用冲突 Euler。
 * SDK 没有协方差，ROS 协方差保持全零表示未知，不能伪称无姿态估计。
 */
std::optional<sensor_msgs::msg::Imu> ConvertImu(
  const AstrallImuData& data, int64_t stamp_ns, const std::string& frame);
/** @brief 四轮顺序与数值不变，单位未确认；保留原始时间戳。 */
std::optional<astrall_interfaces::msg::WheelSpeeds> ConvertWheelSpeeds(
  const AstrallSportData& data, int64_t stamp_ns, const std::string& frame);
/** @brief 六路摇杆、按钮及开关原值；不推测摇杆归一化范围。 */
std::optional<astrall_interfaces::msg::JoystickState> ConvertJoystick(
  const AstrallJoystickData& data, int64_t stamp_ns, const std::string& frame);
/** @brief 手册 5.7；原始位域与已知/未知位文本同时保留。 */
std::optional<astrall_interfaces::msg::SystemStatus> ConvertSystemStatus(
  const AstrallSystemStatus& data, int64_t stamp_ns, const std::string& frame);
/** @brief 手册 5.8；soc 百分比转 [0,1]，越界/非有限拒绝，其他量保留。 */
std::optional<astrall_interfaces::msg::PowerStatus> ConvertPowerStatus(
  const AstrallPowerStatus& data, int64_t stamp_ns, const std::string& frame);
/** @brief 手册 5.9；模式是状态值，不是 SDK 调用返回码，保留未知值。 */
std::optional<astrall_interfaces::msg::SportStatus> ConvertSportStatus(
  AstrallSportStatus data, int64_t stamp_ns, const std::string& frame);
/** @brief 手册 5.6；按值复制版本、序列号与型号，不缓存失败查询。 */
std::optional<astrall_interfaces::msg::DeviceInfo> ConvertDeviceInfo(
  const AstrallDeviceInfo& data, int64_t stamp_ns, const std::string& frame);

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__TELEMETRY_CONVERSIONS_HPP_
