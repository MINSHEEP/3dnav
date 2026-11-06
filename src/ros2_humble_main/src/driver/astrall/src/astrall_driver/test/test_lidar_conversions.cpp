// Copyright 2026 Hypertron ROS 2 contributors

#include <cmath>
#include <cstring>
#include <limits>

#include "astrall_driver/lidar_conversions.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

uint32_t Bits(const std::vector<uint8_t>& data, size_t offset) {
  uint32_t bits = 0;
  for (size_t i = 0; i < 4; ++i) {
    bits |= static_cast<uint32_t>(data.at(offset + i)) << (8 * i);
  }
  return bits;
}

float Coordinate(const std::vector<uint8_t>& data, size_t offset) {
  const auto bits = Bits(data, offset);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

LidarPointFrame Frame() {
  return { 123, { { 10000, -20000, 5000, { 10, 20, 30, 255 } } } };
}

LidarOdometryPacket Pose() {
  return { 123, { 1, -2, 3 }, { 1, 2, 3, 4 } };
}

TEST(LidarConversions, CompleteCloudUnitsFieldsStrideRgbaAndReceiveStamp) {
  auto cloud = ConvertLidarPoints(Frame(), 2015000000, {});
  ASSERT_TRUE(cloud);
  EXPECT_EQ(cloud->header.frame_id, "odom");
  EXPECT_EQ(cloud->header.stamp.sec, 2);
  EXPECT_EQ(cloud->header.stamp.nanosec, 15000000U);
  EXPECT_EQ(cloud->height, 1U);
  EXPECT_EQ(cloud->width, 1U);
  EXPECT_EQ(cloud->point_step, 16U);
  EXPECT_EQ(cloud->row_step, 16U);
  EXPECT_EQ(cloud->data.size(), 16U);
  EXPECT_FALSE(cloud->is_bigendian);
  ASSERT_EQ(cloud->fields.size(), 4U);
  EXPECT_EQ(cloud->fields[0].name, "x");
  EXPECT_EQ(cloud->fields[1].offset, 4U);
  EXPECT_EQ(cloud->fields[2].offset, 8U);
  EXPECT_EQ(cloud->fields[3].name, "rgba");
  EXPECT_EQ(cloud->fields[3].offset, 12U);
  EXPECT_EQ(cloud->fields[3].datatype,
      sensor_msgs::msg::PointField::UINT32);
  EXPECT_FLOAT_EQ(Coordinate(cloud->data, 0), 1);
  EXPECT_FLOAT_EQ(Coordinate(cloud->data, 4), -2);
  EXPECT_FLOAT_EQ(Coordinate(cloud->data, 8), 0.5);
  EXPECT_EQ(Bits(cloud->data, 12), 0xff0a141eU);
}

TEST(LidarConversions,
    NormalizedMeterPoseAndTfShareHeaderAndUnknownCovariance) {
  DriverParameters parameters;
  parameters.lidar_quaternion_scale_ = 0.001;
  auto messages = ConvertLidarOdometry(Pose(), 4000000002, parameters);
  ASSERT_TRUE(messages);
  const auto& odom = messages->odometry_;
  const auto& transform = messages->transform_;
  EXPECT_EQ(odom.header, transform.header);
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_EQ(transform.child_frame_id, odom.child_frame_id);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, -2);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 3);
  EXPECT_EQ(transform.transform.rotation, odom.pose.pose.orientation);
  EXPECT_NEAR(odom.pose.pose.orientation.w, 4 / std::sqrt(30), 1e-12);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(odom.pose.covariance[0], 1e6);
  EXPECT_DOUBLE_EQ(odom.twist.covariance[35], 1e6);
  parameters.lidar_pose_reference_ = "lidar";
  parameters.lidar_frame_id_ = "raw_lidar";
  messages = ConvertLidarOdometry(Pose(), 0, parameters);
  ASSERT_TRUE(messages);
  EXPECT_EQ(messages->transform_.child_frame_id, "raw_lidar");
  EXPECT_EQ(messages->transform_.header.frame_id, "odom");
}

TEST(LidarConversions, ZeroNonfiniteQuaternionAndImplausiblePoseAreRejected) {
  auto pose = Pose();
  pose.quaternion_ = {};
  EXPECT_FALSE(ConvertLidarOdometry(pose, 0, {}));
  DriverParameters parameters;
  for (auto scale : { 0.0, -1.0, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN() }) {
    parameters.lidar_quaternion_scale_ = scale;
    EXPECT_FALSE(ConvertLidarOdometry(Pose(), 0, parameters));
  }
  pose = Pose();
  pose.position_[0] = std::numeric_limits<int64_t>::min();
  EXPECT_FALSE(ConvertLidarOdometry(pose, 0, {}));
  auto frame = Frame();
  frame.points_[0].rgba_[0] = 256;
  EXPECT_FALSE(ConvertLidarPoints(frame, 0, {}));
  frame.points_.clear();
  EXPECT_FALSE(ConvertLidarPoints(frame, 0, {}));
}

TEST(LidarConversions, SourceTimeRequiresExplicitUnixDomainToleranceAndRange) {
  DriverParameters parameters;
  auto frame = Frame();
  frame.timestamp_ns_ = 5000000000;
  auto cloud = ConvertLidarPoints(frame, 4000000001, parameters);
  ASSERT_TRUE(cloud);
  EXPECT_EQ(cloud->header.stamp.sec, 4);
  parameters.lidar_timestamp_mode_ = "source_unix_ns";
  cloud = ConvertLidarPoints(frame, 4000000001, parameters);
  ASSERT_TRUE(cloud);
  EXPECT_EQ(cloud->header.stamp.sec, 5);
  EXPECT_FALSE(ConvertLidarPoints(frame, 3000000000, parameters));
  frame.timestamp_ns_ = std::numeric_limits<uint64_t>::max();
  EXPECT_FALSE(ConvertLidarPoints(frame, 0, parameters));
  EXPECT_FALSE(ConvertLidarPoints(Frame(), -1, {}));
  EXPECT_FALSE(ConvertLidarOdometry(Pose(),
      std::numeric_limits<int64_t>::max(), {}));
  parameters.lidar_source_time_tolerance_ms_ =
      std::numeric_limits<int64_t>::max();
  EXPECT_FALSE(ConvertLidarOdometry(Pose(), 0, parameters));
}

TEST(LidarConversions, ParametersBoundWireResourcesFramesAndClockAssumptions) {
  DriverParameters parameters;
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.point_cloud_port_ = parameters.lidar_odometry_port_;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters.point_cloud_port_ = parameters.lidar_odometry_port_ = 0;
  parameters.lidar_wire_format_ = "packed_le";
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.lidar_wire_format_ = "guess";
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.lidar_max_points_ = kMaxLidarFramePoints + 1;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.odom_frame_id_ = parameters.base_frame_id_;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.lidar_quaternion_scale_ = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(parameters.Validate().empty());
}

}  // namespace
}  // namespace astrall_driver
