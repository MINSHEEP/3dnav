// Copyright 2026 Hypertron ROS 2 contributors

#include <chrono>
#include <cstddef>
#include <limits>
#include <string>

#include "astrall_driver/lidar_packet_parser.hpp"
#include "astrall_driver/point_cloud_assembler.hpp"
#include "astrall_sdk/interface.h"
#include "gtest/gtest.h"
#include "lidar_capture_payloads.hpp"
#include "lidar_test_packets.hpp"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;
constexpr auto kNative = LidarWireFormat::kNativeLittleEndian;
constexpr auto kPacked = LidarWireFormat::kPackedLittleEndian;

LidarPointPacket Fragment(uint64_t stamp, uint32_t total, uint32_t index,
    size_t count) {
  return { stamp, total, index, std::vector<LidarPoint>(count) };
}

TEST(LidarPackets, CapturedPackedPayloadsDecodeWithoutNativePadding) {
  LidarPacketParser parser(kPacked);
  LidarPacketParser native(kNative);
  std::string error;
  const auto& first = test::kCapturedPointsFirst;
  const auto& second = test::kCapturedPointsSecond;
  auto one = parser.ParsePoints(first.data(), first.size(), &error);
  auto two = parser.ParsePoints(second.data(), second.size(), &error);
  ASSERT_TRUE(one) << error;
  ASSERT_TRUE(two) << error;
  EXPECT_EQ(one->timestamp_ns_, 2267017221919U);
  EXPECT_EQ(two->timestamp_ns_, one->timestamp_ns_);
  EXPECT_EQ(one->total_, 30396U);
  EXPECT_EQ(two->total_, one->total_);
  EXPECT_EQ(one->index_, 15600U);
  EXPECT_EQ(two->index_, one->index_ + one->points_.size());
  ASSERT_EQ(one->points_.size(), 50U);
  EXPECT_EQ(one->points_[0].x_, 24501);
  EXPECT_EQ(one->points_[0].y_, 48043);
  EXPECT_EQ(one->points_[0].z_, 3902);
  EXPECT_FALSE(native.ParsePoints(first.data(), first.size(), &error));
  EXPECT_NE(error.find("1422 bytes received, 1432 expected"),
      std::string::npos);
  PointCloudAssembler assembler;
  EXPECT_FALSE(assembler.Push(*one, PointCloudAssembler::Time{}));
  EXPECT_FALSE(assembler.Push(*two, PointCloudAssembler::Time{}));
  const auto& bytes = test::kCapturedOdometry;
  auto odom = parser.ParseOdometry(bytes.data(), bytes.size(), &error);
  ASSERT_TRUE(odom) << error;
  EXPECT_EQ(odom->timestamp_ns_, 2279883648753U);
  EXPECT_EQ(odom->position_, (std::array<int64_t, 3>{ 33847, 116806, 10217 }));
  EXPECT_EQ(odom->quaternion_,
      (std::array<int64_t, 4>{ -2272, -521, 130606, 991431 }));
  EXPECT_FALSE(native.ParseOdometry(bytes.data(), bytes.size(), &error));
}

TEST(LidarPackets, NativeOffsetsMatchLocalSdkWithoutCopyingWireStruct) {
  EXPECT_EQ(sizeof(AstrallPosCloudData), 1432U);
  EXPECT_EQ(offsetof(AstrallPosCloudData, timestampNs), 8U);
  EXPECT_EQ(offsetof(AstrallPosCloudData, pos), 28U);
  EXPECT_EQ(offsetof(AstrallPosCloudData, tail), 1428U);
  EXPECT_EQ(sizeof(AstrallOdometryData), 80U);
  EXPECT_EQ(offsetof(AstrallOdometryData, tail), 72U);
}

TEST(LidarPackets, ExplicitLayoutsDecodeIndependentBytesAndSignedExtremes) {
  for (bool packed : { false, true }) {
    LidarPacketParser parser(packed ? kPacked : kNative);
    auto bytes = test::PointBytes(123, 3, 1, 2, packed);
    std::string error;
    auto cloud = parser.ParsePoints(bytes.data(), bytes.size(), &error);
    ASSERT_TRUE(cloud) << error;
    EXPECT_EQ(cloud->timestamp_ns_, 123U);
    EXPECT_EQ(cloud->total_, 3U);
    EXPECT_EQ(cloud->index_, 1U);
    ASSERT_EQ(cloud->points_.size(), 2U);
    EXPECT_EQ(cloud->points_[1].x_, 10002);
    EXPECT_EQ(cloud->points_[1].y_, -20000);
    EXPECT_EQ(cloud->points_[1].rgba_[3], 255U);
    bytes = test::OdometryBytes(456, packed);
    test::Put(&bytes, packed ? 10 : 16, uint64_t{ 1 } << 63, 8);
    auto odom = parser.ParseOdometry(bytes.data(), bytes.size(), &error);
    ASSERT_TRUE(odom) << error;
    EXPECT_EQ(odom->position_[0], std::numeric_limits<int64_t>::min());
    EXPECT_EQ(odom->position_[1], -2);
    EXPECT_EQ(odom->quaternion_[3], 2);
  }
}

TEST(LidarPackets, BothLayoutsRejectEveryTruncationExtraBytesAndBadMagic) {
  for (bool packed : { false, true }) {
    LidarPacketParser parser(packed ? kPacked : kNative);
    auto bytes = test::PointBytes(1, 1, 0, 1, packed);
    std::string error;
    for (size_t length = 0; length < bytes.size(); ++length) {
      EXPECT_FALSE(parser.ParsePoints(bytes.data(), length, &error));
    }
    EXPECT_FALSE(parser.ParsePoints(nullptr, bytes.size(), &error));
    bytes.push_back(0);
    EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
    for (size_t offset : { size_t{ 0 }, packed ? size_t{ 1420 } :
                           size_t{ 1428 } }) {
      bytes = test::PointBytes(1, 1, 0, 1, packed);
      bytes[offset] ^= 1;
      EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
    }
    auto odom = test::OdometryBytes(1, packed);
    for (size_t length = 0; length < odom.size(); ++length) {
      EXPECT_FALSE(parser.ParseOdometry(odom.data(), length, &error));
    }
    odom[packed ? 66 : 72] ^= 1;
    EXPECT_FALSE(parser.ParseOdometry(odom.data(), odom.size(), &error));
    bytes = test::PointBytes(1, 1, 0, 1, !packed);
    EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  }
}

TEST(LidarPackets, CountsOverflowLimitsAndColorCannotAllocateOrOverread) {
  LidarPacketParser parser(kNative, 100);
  std::string error;
  for (auto count : { 0, 51, 65535 }) {
    auto bytes = test::PointBytes(1, 100, 0, count);
    EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  }
  for (uint32_t total : { 0U, 101U, 0xffffffffU }) {
    auto bytes = test::PointBytes(1, total, 0, 1);
    EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  }
  for (uint32_t index : { 100U, 0xffffffffU }) {
    auto bytes = test::PointBytes(1, 100, index, 1);
    EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  }
  auto bytes = test::PointBytes(1, 100, 99, 2);
  EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  bytes = test::PointBytes(1, 1, 0, 1);
  test::Put(&bytes, 40, 256, 4);
  EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
  bytes = test::PointBytes(0, 1, 0, 1);
  EXPECT_FALSE(parser.ParsePoints(bytes.data(), bytes.size(), &error));
}

TEST(LidarAssembler, CompleteInOrderOnlyAndOldFramesNeverReplay) {
  PointCloudAssembler assembler(100, 100ms);
  const auto time = PointCloudAssembler::Time{};
  EXPECT_FALSE(assembler.Push(Fragment(10, 3, 0, 2), time));
  auto result = assembler.Push(Fragment(10, 3, 2, 1), time + 1ms);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->points_.size(), 3U);
  EXPECT_FALSE(assembler.Push(Fragment(10, 3, 0, 3), time + 2ms));
  EXPECT_FALSE(assembler.Push(Fragment(9, 3, 0, 3), time + 3ms));
  EXPECT_TRUE(assembler.Push(Fragment(11, 1, 0, 1), time + 4ms));
}

TEST(LidarAssembler, GapDuplicateConflictAndMissingStartDiscardWholeFrame) {
  for (auto second : { Fragment(1, 3, 2, 1), Fragment(1, 3, 0, 1),
                       Fragment(1, 4, 1, 1) }) {
    PointCloudAssembler assembler;
    const auto time = PointCloudAssembler::Time{};
    EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 1), time));
    EXPECT_FALSE(assembler.Push(second, time + 1ms));
    EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 3), time + 2ms));
    EXPECT_TRUE(assembler.Push(Fragment(2, 1, 0, 1), time + 3ms));
    EXPECT_GE(assembler.DroppedFrames(), 1U);
  }
  PointCloudAssembler assembler;
  const auto time = PointCloudAssembler::Time{};
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 1, 2), time));
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 3), time + 1ms));
  EXPECT_TRUE(assembler.Push(Fragment(2, 1, 0, 1), time + 2ms));
}

TEST(LidarAssembler, InclusiveTimeoutCannotBeExtendedOrRestartedAfterStop) {
  PointCloudAssembler assembler(100, 100ms);
  const auto time = PointCloudAssembler::Time{};
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 1), time));
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 1, 1), time + 99ms));
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 2, 1), time + 100ms));
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 3), time + 101ms));
  EXPECT_TRUE(assembler.Push(Fragment(2, 1, 0, 1), time + 102ms));
  EXPECT_FALSE(assembler.Push(Fragment(3, 3, 0, 1), time + 103ms));
  assembler.Discard();
  EXPECT_FALSE(assembler.Push(Fragment(3, 3, 0, 3), time + 104ms));
  EXPECT_TRUE(assembler.Push(Fragment(4, 1, 0, 1), time + 105ms));
}

TEST(LidarAssembler, DefensiveBoundsAndNewFrameDiscardIncompleteOldFrame) {
  PointCloudAssembler assembler(2);
  const auto time = PointCloudAssembler::Time{};
  EXPECT_FALSE(assembler.Push(Fragment(1, 3, 0, 1), time));
  EXPECT_FALSE(assembler.Push(Fragment(2, 0xffffffffU, 0xffffffffU, 1), time));
  EXPECT_FALSE(assembler.Push(Fragment(3, 2, 0, 1), time));
  EXPECT_TRUE(assembler.Push(Fragment(4, 1, 0, 1), time));
  EXPECT_FALSE(assembler.Push(Fragment(3, 2, 1, 1), time));
}

}  // namespace
}  // namespace astrall_driver
