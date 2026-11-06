// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__LIDAR_CONVERSIONS_HPP_
#define ASTRALL_DRIVER__LIDAR_CONVERSIONS_HPP_

#include <optional>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/point_cloud_assembler.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace astrall_driver {

struct LidarPoseMessages {
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::TransformStamped transform_;
};

/**
 * @brief 完整帧0.1mm→m，xyz/rgba小端16B；点已在odom，不施加外参。
 * 默认以完整帧接收ROS ns为stamp；源Unix ns仅显式启用且在容差/ROS范围内。
 */
std::optional<sensor_msgs::msg::PointCloud2> ConvertLidarPoints(
  const LidarPointFrame& frame, int64_t received_ros_ns,
  const DriverParameters& parameters);

/**
 * @brief 米制整数位姿，四元数缩放/有限检查/归一化，Odometry和TF共用stamp。
 * 默认假设参考点为base_link，可显式选择lidar；不生成未知lidar→base外参。
 * 无速度观测；twist零和协方差对角1e6均为未知占位，禁止冒充实测，见Q30。
 */
std::optional<LidarPoseMessages> ConvertLidarOdometry(
  const LidarOdometryPacket& packet, int64_t received_ros_ns,
  const DriverParameters& parameters);

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__LIDAR_CONVERSIONS_HPP_
