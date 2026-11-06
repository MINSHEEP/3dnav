// Copyright 2026 Hypertron ROS 2 contributors

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <cmath>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/lidar_packet_parser.hpp"

namespace astrall_driver {
namespace {

bool Name(const std::string& value, bool frame) {
  return !value.empty() && value.back() != '/' &&
         (!frame || value.front() != '/') &&
         std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isalnum(character) || character == '_' || character == '/';
      });
}

}  // namespace

std::string DriverParameters::ValidateLidar() const {
  in_addr address{};
  if (inet_pton(AF_INET, lidar_bind_address_.c_str(), &address) != 1 ||
      point_cloud_port_ < 0 || point_cloud_port_ > 65535 ||
      lidar_odometry_port_ < 0 || lidar_odometry_port_ > 65535 ||
      (point_cloud_port_ != 0 && point_cloud_port_ == lidar_odometry_port_)) {
    return "lidar requires IPv4 bind address and distinct ports 1..65535; "
           "0 explicitly selects an offline ephemeral port";
  }
  if (lidar_wire_format_ != "native_le" && lidar_wire_format_ != "packed_le") {
    return "lidar_wire_format must explicitly be native_le or packed_le";
  }
  if (lidar_max_points_ < 1 || lidar_max_points_ > kMaxLidarFramePoints ||
      lidar_frame_timeout_ms_ < 10 || lidar_frame_timeout_ms_ > 5000 ||
      lidar_max_age_ms_ < telemetry_publish_period_ms_ ||
      lidar_max_age_ms_ > 5000) {
    return "lidar max_points 1..1000000, frame timeout 10..5000ms, "
           "max age at least telemetry publish period and at most 5000ms";
  }
  if (!Name(point_cloud_topic_, false) || !Name(lidar_odometry_topic_, false) ||
      point_cloud_topic_ == lidar_odometry_topic_ ||
      !Name(odom_frame_id_, true) || !Name(lidar_frame_id_, true) ||
      odom_frame_id_ == base_frame_id_ || odom_frame_id_ == lidar_frame_id_ ||
      base_frame_id_ == lidar_frame_id_ ||
      (lidar_pose_reference_ != "base" && lidar_pose_reference_ != "lidar")) {
    return "lidar topics/frames must be plain names; frames distinct; "
           "lidar_pose_reference must be base or lidar (no invented extrinsic)";
  }
  if (!std::isfinite(lidar_quaternion_scale_) ||
      lidar_quaternion_scale_ < 1e-18 || lidar_quaternion_scale_ > 1e18 ||
      !std::isfinite(lidar_max_position_m_) || lidar_max_position_m_ <= 0 ||
      lidar_max_position_m_ > 1e9 ||
      (lidar_timestamp_mode_ != "receive" &&
      lidar_timestamp_mode_ != "source_unix_ns") ||
      lidar_source_time_tolerance_ms_ < 1 ||
      lidar_source_time_tolerance_ms_ > 60000) {
    return "lidar scale 1e-18..1e18, position limit (0,1e9]m, timestamp "
           "receive/source_unix_ns, source tolerance 1..60000ms";
  }
  return {};
}

}  // namespace astrall_driver
