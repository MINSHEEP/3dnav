// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/lidar_conversions.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace astrall_driver {
namespace {

constexpr int64_t kNsPerSecond = 1000000000;
constexpr int64_t kMaxRosNs =
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) *
    kNsPerSecond + kNsPerSecond - 1;
constexpr uint32_t kPointStride = 16;
constexpr double kPointScale = 0.0001;
constexpr double kUnknownVariance = 1e6;
constexpr uint32_t kMaxColorChannel = 255;

std::optional<builtin_interfaces::msg::Time> Stamp(uint64_t source,
    int64_t received, const DriverParameters& parameters) {
  if (received < 0 || received > kMaxRosNs) {
    return std::nullopt;
  }
  int64_t selected = received;
  if (parameters.lidar_timestamp_mode_ == "source_unix_ns") {
    if (source > static_cast<uint64_t>(kMaxRosNs) ||
        parameters.lidar_source_time_tolerance_ms_ < 1 ||
        parameters.lidar_source_time_tolerance_ms_ > 60000) {
      return std::nullopt;
    }
    selected = static_cast<int64_t>(source);
    if (std::abs(selected - received) >
        parameters.lidar_source_time_tolerance_ms_ * 1000000) {
      return std::nullopt;
    }
  } else if (parameters.lidar_timestamp_mode_ != "receive") {
    return std::nullopt;
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(selected / kNsPerSecond);
  result.nanosec = static_cast<uint32_t>(selected % kNsPerSecond);
  return result;
}

void Put(std::vector<uint8_t>* bytes, size_t offset, uint32_t bits) {
  for (size_t i = 0; i < 4; ++i) {
    (*bytes)[offset + i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}

void Coordinate(std::vector<uint8_t>* bytes, size_t offset, int32_t raw) {
  const float meters = static_cast<float>(raw * kPointScale);
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(meters), "PointCloud2 float32 required");
  std::memcpy(&bits, &meters, sizeof(bits));
  Put(bytes, offset, bits);
}

}  // namespace

std::optional<sensor_msgs::msg::PointCloud2> ConvertLidarPoints(
  const LidarPointFrame& frame, int64_t received_ros_ns,
  const DriverParameters& parameters) {
  auto stamp = Stamp(frame.timestamp_ns_, received_ros_ns, parameters);
  if (!stamp || frame.points_.empty() || parameters.lidar_max_points_ < 1 ||
      frame.points_.size() > kMaxLidarFramePoints ||
      frame.points_.size() >
      static_cast<size_t>(parameters.lidar_max_points_)) {
    return std::nullopt;
  }
  sensor_msgs::msg::PointCloud2 result;
  result.header.stamp = *stamp;
  result.header.frame_id = parameters.odom_frame_id_;
  result.height = 1;
  result.width = static_cast<uint32_t>(frame.points_.size());
  result.point_step = kPointStride;
  result.row_step = result.width * kPointStride;
  result.is_bigendian = false;
  result.is_dense = true;
  const std::array<std::string, 4> names{ "x", "y", "z", "rgba" };
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<uint32_t>(i * 4);
    field.datatype = i == 3 ? sensor_msgs::msg::PointField::UINT32 :
        sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    result.fields.push_back(std::move(field));
  }
  result.data.resize(result.row_step);
  for (size_t i = 0; i < frame.points_.size(); ++i) {
    const auto& point = frame.points_[i];
    for (auto channel : point.rgba_) {
      if (channel > kMaxColorChannel) {
        return std::nullopt;
      }
    }
    const auto offset = i * kPointStride;
    Coordinate(&result.data, offset, point.x_);
    Coordinate(&result.data, offset + 4, point.y_);
    Coordinate(&result.data, offset + 8, point.z_);
    Put(&result.data, offset + 12, point.rgba_[3] << 24 |
        point.rgba_[0] << 16 | point.rgba_[1] << 8 | point.rgba_[2]);
  }
  return result;
}

std::optional<LidarPoseMessages> ConvertLidarOdometry(
  const LidarOdometryPacket& packet, int64_t received_ros_ns,
  const DriverParameters& parameters) {
  auto stamp = Stamp(packet.timestamp_ns_, received_ros_ns, parameters);
  if (!std::isfinite(parameters.lidar_quaternion_scale_) ||
      parameters.lidar_quaternion_scale_ <= 0 ||
      (parameters.lidar_pose_reference_ != "base" &&
      parameters.lidar_pose_reference_ != "lidar")) {
    return std::nullopt;
  }
  std::array<double, 4> quaternion;
  for (size_t i = 0; i < quaternion.size(); ++i) {
    quaternion[i] = packet.quaternion_[i] * parameters.lidar_quaternion_scale_;
    if (!std::isfinite(quaternion[i])) {
      return std::nullopt;
    }
  }
  const auto norm = std::hypot(std::hypot(quaternion[0], quaternion[1]),
          std::hypot(quaternion[2], quaternion[3]));
  if (!stamp || !std::isfinite(norm) || norm == 0 ||
      !std::isfinite(parameters.lidar_max_position_m_) ||
      parameters.lidar_max_position_m_ <= 0) {
    return std::nullopt;
  }
  for (auto position : packet.position_) {
    if (std::abs(static_cast<double>(position)) >
        parameters.lidar_max_position_m_) {
      return std::nullopt;
    }
  }
  LidarPoseMessages result;
  auto& odom = result.odometry_;
  odom.header.stamp = *stamp;
  odom.header.frame_id = parameters.odom_frame_id_;
  odom.child_frame_id = parameters.lidar_pose_reference_ == "lidar" ?
      parameters.lidar_frame_id_ : parameters.base_frame_id_;
  odom.pose.pose.position.x = static_cast<double>(packet.position_[0]);
  odom.pose.pose.position.y = static_cast<double>(packet.position_[1]);
  odom.pose.pose.position.z = static_cast<double>(packet.position_[2]);
  odom.pose.pose.orientation.x = quaternion[0] / norm;
  odom.pose.pose.orientation.y = quaternion[1] / norm;
  odom.pose.pose.orientation.z = quaternion[2] / norm;
  odom.pose.pose.orientation.w = quaternion[3] / norm;
  for (size_t i = 0; i < 6; ++i) {
    odom.pose.covariance[i * 7] = kUnknownVariance;
    odom.twist.covariance[i * 7] = kUnknownVariance;
  }
  result.transform_.header = odom.header;
  result.transform_.child_frame_id = odom.child_frame_id;
  result.transform_.transform.translation.x = odom.pose.pose.position.x;
  result.transform_.transform.translation.y = odom.pose.pose.position.y;
  result.transform_.transform.translation.z = odom.pose.pose.position.z;
  result.transform_.transform.rotation = odom.pose.pose.orientation;
  return result;
}

}  // namespace astrall_driver
