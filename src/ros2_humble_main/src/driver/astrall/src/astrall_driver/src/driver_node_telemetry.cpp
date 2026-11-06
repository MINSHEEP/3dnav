// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_node.hpp"

#include "astrall_driver/telemetry_publisher.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace astrall_driver {

void DriverNode::DeclareTelemetryParameters() {
  const DriverParameters defaults;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description =
      "S3 telemetry startup configuration; see package README";
  declare_parameter("telemetry_publish_period_ms",
      defaults.telemetry_publish_period_ms_,
      descriptor);
  declare_parameter("telemetry_query_period_ms",
      defaults.telemetry_query_period_ms_,
      descriptor);
  declare_parameter("telemetry_max_age_ms",
      defaults.telemetry_max_age_ms_,
      descriptor);
  declare_parameter("imu_frequency_hz", defaults.imu_frequency_hz_, descriptor);
  declare_parameter("wheel_speed_frequency_hz",
      defaults.wheel_speed_frequency_hz_,
      descriptor);
  declare_parameter("joystick_frequency_hz",
      defaults.joystick_frequency_hz_,
      descriptor);
  declare_parameter("enable_rgb", defaults.enable_rgb_, descriptor);
  declare_parameter("enable_lidar", defaults.enable_lidar_, descriptor);
  declare_parameter("imu_topic", defaults.imu_topic_, descriptor);
  declare_parameter("wheel_speed_topic", defaults.wheel_speed_topic_,
      descriptor);
  declare_parameter("joystick_topic", defaults.joystick_topic_, descriptor);
  declare_parameter("system_status_topic",
      defaults.system_status_topic_,
      descriptor);
  declare_parameter("power_status_topic",
      defaults.power_status_topic_,
      descriptor);
  declare_parameter("sport_status_topic",
      defaults.sport_status_topic_,
      descriptor);
  declare_parameter("device_info_topic", defaults.device_info_topic_,
      descriptor);
  declare_parameter("imu_frame_id", defaults.imu_frame_id_, descriptor);
  declare_parameter("base_frame_id", defaults.base_frame_id_, descriptor);
}

void DriverNode::ReadTelemetryParameters(DriverParameters* parameters) const {
  parameters->telemetry_publish_period_ms_ = get_parameter(
    "telemetry_publish_period_ms").as_int();
  parameters->telemetry_query_period_ms_ = get_parameter(
    "telemetry_query_period_ms").as_int();
  parameters->telemetry_max_age_ms_ =
      get_parameter("telemetry_max_age_ms").as_int();
  parameters->imu_frequency_hz_ = get_parameter("imu_frequency_hz").as_int();
  parameters->wheel_speed_frequency_hz_ = get_parameter(
    "wheel_speed_frequency_hz").as_int();
  parameters->joystick_frequency_hz_ =
      get_parameter("joystick_frequency_hz").as_int();
  parameters->enable_rgb_ = get_parameter("enable_rgb").as_bool();
  parameters->enable_lidar_ = get_parameter("enable_lidar").as_bool();
  parameters->imu_topic_ = get_parameter("imu_topic").as_string();
  parameters->wheel_speed_topic_ =
      get_parameter("wheel_speed_topic").as_string();
  parameters->joystick_topic_ = get_parameter("joystick_topic").as_string();
  parameters->system_status_topic_ =
      get_parameter("system_status_topic").as_string();
  parameters->power_status_topic_ =
      get_parameter("power_status_topic").as_string();
  parameters->sport_status_topic_ =
      get_parameter("sport_status_topic").as_string();
  parameters->device_info_topic_ =
      get_parameter("device_info_topic").as_string();
  parameters->imu_frame_id_ = get_parameter("imu_frame_id").as_string();
  parameters->base_frame_id_ = get_parameter("base_frame_id").as_string();
}

void DriverNode::PublishTelemetry() {
  if (!telemetry_publisher_ ||
      !get_node_base_interface()->get_context()->is_valid()) {
    return;
  }
  for (const auto& sample : core_->TakeTelemetry()) {
    if (core_->TelemetryCurrent(sample.session_generation_)) {
      telemetry_publisher_->Publish(sample);
    }
  }
  if (lidar_publisher_) {
    lidar_publisher_->Publish();
  }
}

}  // namespace astrall_driver
