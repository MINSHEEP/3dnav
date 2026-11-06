// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_node.hpp"

#include <algorithm>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace astrall_driver {

void DriverNode::DeclareLidarParameters() {
  const DriverParameters defaults;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "S5 lidar layout/resources; see README and Q04/Q30";
  declare_parameter("lidar_bind_address",
      defaults.lidar_bind_address_,
      descriptor);
  declare_parameter("lidar_wire_format", defaults.lidar_wire_format_,
      descriptor);
  declare_parameter("point_cloud_topic", defaults.point_cloud_topic_,
      descriptor);
  declare_parameter("lidar_odometry_topic",
      defaults.lidar_odometry_topic_,
      descriptor);
  declare_parameter("odom_frame_id", defaults.odom_frame_id_, descriptor);
  declare_parameter("lidar_frame_id", defaults.lidar_frame_id_, descriptor);
  declare_parameter("lidar_pose_reference",
      defaults.lidar_pose_reference_,
      descriptor);
  declare_parameter("lidar_timestamp_mode",
      defaults.lidar_timestamp_mode_,
      descriptor);
  declare_parameter("point_cloud_port", defaults.point_cloud_port_, descriptor);
  declare_parameter("lidar_odometry_port",
      defaults.lidar_odometry_port_,
      descriptor);
  declare_parameter("lidar_max_points", defaults.lidar_max_points_, descriptor);
  declare_parameter("lidar_frame_timeout_ms",
      defaults.lidar_frame_timeout_ms_,
      descriptor);
  declare_parameter("lidar_max_age_ms", defaults.lidar_max_age_ms_, descriptor);
  declare_parameter("lidar_source_time_tolerance_ms",
      defaults.lidar_source_time_tolerance_ms_,
      descriptor);
  declare_parameter("lidar_quaternion_scale",
      defaults.lidar_quaternion_scale_,
      descriptor);
  declare_parameter("lidar_max_position_m",
      defaults.lidar_max_position_m_,
      descriptor);
}

void DriverNode::ReadLidarParameters(DriverParameters* parameters) const {
  parameters->lidar_bind_address_ =
      get_parameter("lidar_bind_address").as_string();
  parameters->lidar_wire_format_ =
      get_parameter("lidar_wire_format").as_string();
  parameters->point_cloud_topic_ =
      get_parameter("point_cloud_topic").as_string();
  parameters->lidar_odometry_topic_ =
      get_parameter("lidar_odometry_topic").as_string();
  parameters->odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  parameters->lidar_frame_id_ = get_parameter("lidar_frame_id").as_string();
  parameters->lidar_pose_reference_ =
      get_parameter("lidar_pose_reference").as_string();
  parameters->lidar_timestamp_mode_ =
      get_parameter("lidar_timestamp_mode").as_string();
  parameters->point_cloud_port_ = get_parameter("point_cloud_port").as_int();
  parameters->lidar_odometry_port_ =
      get_parameter("lidar_odometry_port").as_int();
  parameters->lidar_max_points_ = get_parameter("lidar_max_points").as_int();
  parameters->lidar_frame_timeout_ms_ =
      get_parameter("lidar_frame_timeout_ms").as_int();
  parameters->lidar_max_age_ms_ = get_parameter("lidar_max_age_ms").as_int();
  parameters->lidar_source_time_tolerance_ms_ = get_parameter(
    "lidar_source_time_tolerance_ms").as_int();
  parameters->lidar_quaternion_scale_ =
      get_parameter("lidar_quaternion_scale").as_double();
  parameters->lidar_max_position_m_ =
      get_parameter("lidar_max_position_m").as_double();
}

void DriverNode::CreateLidarPublisher() {
  lidar_publisher_.reset();
  if (parameters_.enable_lidar_) {
    lidar_publisher_ = std::make_unique<LidarPublisher>(*this, parameters_,
            [this](uint64_t generation) {
        return core_->TelemetryEpoch(generation);
      });
  }
}

bool DriverNode::StartLidar() {
  if (!lidar_publisher_) {
    return true;
  }
  if (lidar_publisher_->Start(core_->Snapshot().session_generation_)) {
    return true;
  }
  const auto state = lidar_publisher_->Snapshot();
  RCLCPP_ERROR(get_logger(), "LIDAR_START %s; cloud=%s; odometry=%s",
      state.last_error_.c_str(), state.cloud_receiver_.last_error_.c_str(),
      state.odometry_receiver_.last_error_.c_str());
  return false;
}

void DriverNode::StopLidar() {
  if (lidar_publisher_) {
    lidar_publisher_->Stop();
  }
}

LidarSnapshot DriverNode::LidarState() const {
  return lidar_publisher_ ? lidar_publisher_->Snapshot() : LidarSnapshot{};
}

void DriverNode::AddLidarDiagnostics(
  diagnostic_msgs::msg::DiagnosticStatus* status) const {
  const auto state = LidarState();
  if (!state.enabled_) {
    return;
  }
  const auto add = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = "lidar_" + key;
        item.value = value;
        status->values.push_back(std::move(item));
      };
  add("active", std::to_string(state.active_));
  add("epoch", std::to_string(state.epoch_));
  add("point_cloud_port", std::to_string(state.cloud_receiver_.port_));
  add("odometry_port", std::to_string(state.odometry_receiver_.port_));
  add("invalid_packets", std::to_string(state.invalid_packets_));
  add("dropped_frames", std::to_string(state.dropped_frames_));
  add("overwritten", std::to_string(state.overwritten_));
  add("stale_samples", std::to_string(state.stale_samples_));
  add("published_clouds", std::to_string(state.published_clouds_));
  add("published_odometry", std::to_string(state.published_odometry_));
  add("last_cloud_source", std::to_string(state.last_cloud_source_));
  add("last_odometry_source", std::to_string(state.last_odometry_source_));
  add("last_error", state.last_error_);
  add("cloud_receiver_error", state.cloud_receiver_.last_error_);
  add("odometry_receiver_error", state.odometry_receiver_.last_error_);
  add("truncated", std::to_string(state.cloud_receiver_.truncated_ +
      state.odometry_receiver_.truncated_));
  const bool failed = state.cloud_receiver_.errors_ != 0 ||
      state.odometry_receiver_.errors_ != 0 ||
      (state.active_ && (!state.cloud_receiver_.running_ ||
      !state.odometry_receiver_.running_));
  if (failed) {
    status->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status->message += "; LIDAR UDP receive/start failure";
  } else if (state.invalid_packets_ || state.dropped_frames_) {
    status->level = std::max<uint8_t>(status->level,
            diagnostic_msgs::msg::DiagnosticStatus::WARN);
  }
}

}  // namespace astrall_driver
