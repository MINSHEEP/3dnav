// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__TELEMETRY_PUBLISHER_HPP_
#define ASTRALL_DRIVER__TELEMETRY_PUBLISHER_HPP_

#include <cstdint>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/telemetry_conversions.hpp"
#include "astrall_driver/telemetry_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace astrall_driver {

/**
 * @brief S3 ROS 转换/发布，不访问 SDK；调用方须检查样本的当前会话有效性。
 * 由节点的互斥 callback group 串行调用；无 transient 历史重发。
 */
class TelemetryPublisher {
 public:
  /** @brief 创建可配置话题；ROS 拒绝非法名称时抛异常，此时尚未连接 SDK。 */
  TelemetryPublisher(rclcpp_lifecycle::LifecycleNode& node,
      const DriverParameters& parameters);
  /** @brief 转换有效样本后发布；无效四元数/数值/时间返回 false 并计数。 */
  bool Publish(const TelemetrySample& sample);
  /** @brief 只计转换失败，不将丢弃样本冒充为默认零值。 */
  uint64_t InvalidSamples() const;

 private:
  bool PublishData(const AstrallImuData& data, int64_t stamp);
  bool PublishData(const AstrallSportData& data, int64_t stamp);
  bool PublishData(const AstrallJoystickData& data, int64_t stamp);
  bool PublishData(const AstrallSystemStatus& data, int64_t stamp);
  bool PublishData(const AstrallPowerStatus& data, int64_t stamp);
  bool PublishData(AstrallSportStatus data, int64_t stamp);
  bool PublishData(const AstrallDeviceInfo& data, int64_t stamp);

  const DriverParameters parameters_;
  uint64_t invalid_samples_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<astrall_interfaces::msg::WheelSpeeds>::SharedPtr wheels_;
  rclcpp::Publisher<astrall_interfaces::msg::JoystickState>::SharedPtr
      joystick_;
  rclcpp::Publisher<astrall_interfaces::msg::SystemStatus>::SharedPtr system_;
  rclcpp::Publisher<astrall_interfaces::msg::PowerStatus>::SharedPtr power_;
  rclcpp::Publisher<astrall_interfaces::msg::SportStatus>::SharedPtr sport_;
  rclcpp::Publisher<astrall_interfaces::msg::DeviceInfo>::SharedPtr device_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__TELEMETRY_PUBLISHER_HPP_
