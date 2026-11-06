// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/telemetry_publisher.hpp"

#include "rclcpp/create_publisher.hpp"

namespace astrall_driver {

TelemetryPublisher::TelemetryPublisher(rclcpp_lifecycle::LifecycleNode& node,
    const DriverParameters& parameters)
  : parameters_(parameters) {
  const auto sensor_qos = rclcpp::SensorDataQoS();
  constexpr size_t kStatusDepth = 1;
  const auto status_qos = rclcpp::QoS(kStatusDepth).reliable();
  imu_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(node,
          parameters.imu_topic_, sensor_qos);
  wheels_ = rclcpp::create_publisher<astrall_interfaces::msg::WheelSpeeds>(node,
          parameters.wheel_speed_topic_, sensor_qos);
  joystick_ = rclcpp::create_publisher<astrall_interfaces::msg::JoystickState>(
    node,
    parameters.joystick_topic_,
    sensor_qos);
  system_ = rclcpp::create_publisher<astrall_interfaces::msg::SystemStatus>(
    node,
    parameters.system_status_topic_,
    status_qos);
  power_ = rclcpp::create_publisher<astrall_interfaces::msg::PowerStatus>(node,
          parameters.power_status_topic_, status_qos);
  sport_ = rclcpp::create_publisher<astrall_interfaces::msg::SportStatus>(node,
          parameters.sport_status_topic_, status_qos);
  device_ = rclcpp::create_publisher<astrall_interfaces::msg::DeviceInfo>(node,
          parameters.device_info_topic_, status_qos);
}

bool TelemetryPublisher::Publish(const TelemetrySample& sample) {
  const bool valid = std::visit([&](const auto& data) {
      return PublishData(data, sample.received_ros_ns_);
    }, sample.data_);
  if (!valid) {
    ++invalid_samples_;
  }
  return valid;
}

uint64_t TelemetryPublisher::InvalidSamples() const {
  return invalid_samples_;
}

bool TelemetryPublisher::PublishData(const AstrallImuData& data,
    int64_t stamp) {
  auto message = ConvertImu(data, stamp, parameters_.imu_frame_id_);
  if (message) {
    imu_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(const AstrallSportData& data,
    int64_t stamp) {
  auto message = ConvertWheelSpeeds(data, stamp, parameters_.base_frame_id_);
  if (message) {
    wheels_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(const AstrallJoystickData& data,
    int64_t stamp) {
  auto message = ConvertJoystick(data, stamp, parameters_.base_frame_id_);
  if (message) {
    joystick_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(const AstrallSystemStatus& data,
    int64_t stamp) {
  auto message = ConvertSystemStatus(data, stamp, parameters_.base_frame_id_);
  if (message) {
    system_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(const AstrallPowerStatus& data,
    int64_t stamp) {
  auto message = ConvertPowerStatus(data, stamp, parameters_.base_frame_id_);
  if (message) {
    power_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(AstrallSportStatus data, int64_t stamp) {
  auto message = ConvertSportStatus(data, stamp, parameters_.base_frame_id_);
  if (message) {
    sport_->publish(*message);
  }
  return message.has_value();
}

bool TelemetryPublisher::PublishData(const AstrallDeviceInfo& data,
    int64_t stamp) {
  auto message = ConvertDeviceInfo(data, stamp, parameters_.base_frame_id_);
  if (message) {
    device_->publish(*message);
  }
  return message.has_value();
}

}  // namespace astrall_driver
