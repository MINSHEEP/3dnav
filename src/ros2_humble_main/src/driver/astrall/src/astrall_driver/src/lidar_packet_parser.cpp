// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/lidar_packet_parser.hpp"

#include <limits>
#include <stdexcept>

namespace astrall_driver {
namespace {

constexpr uint16_t kHead = 0xAA55;
constexpr uint16_t kTail = 0xFF00;
constexpr size_t kPointBytes = 28;
constexpr uint32_t kMaxColor = 255;

uint64_t Read(const uint8_t* data, size_t offset, size_t bytes) {
  uint64_t value = 0;
  for (size_t i = 0; i < bytes; ++i) {
    value |= static_cast<uint64_t>(data[offset + i]) << (i * 8);
  }
  return value;
}

int64_t Signed(uint64_t value, size_t bytes) {
  const uint64_t maximum = bytes == 8 ?
      std::numeric_limits<uint64_t>::max() :
      std::numeric_limits<uint32_t>::max();
  return value <= maximum / 2 ? static_cast<int64_t>(value) :
         -1 - static_cast<int64_t>(maximum - value);
}

bool Envelope(const uint8_t* data, size_t size, size_t expected,
    size_t tail, std::string* error) {
  if (!data || size != expected || tail > size || size - tail < 2) {
    *error = "lidar packet length does not match configured fixed layout: " +
        std::to_string(size) + " bytes received, " +
        std::to_string(expected) + " expected";
    return false;
  }
  if (Read(data, 0, 2) != kHead || Read(data, tail, 2) != kTail) {
    *error = "lidar packet requires 0xAA55 head and 0xFF00 tail";
    return false;
  }
  return true;
}

}  // namespace

LidarPacketParser::LidarPacketParser(LidarWireFormat format,
    uint32_t max_points) : format_(format), max_points_(max_points) {
  if (max_points == 0 || max_points > kMaxLidarFramePoints ||
      (format != LidarWireFormat::kNativeLittleEndian &&
      format != LidarWireFormat::kPackedLittleEndian)) {
    throw std::invalid_argument("lidar max_points must be 1..1000000");
  }
}

size_t LidarPacketParser::PointPacketSize() const {
  return format_ == LidarWireFormat::kNativeLittleEndian ? 1432 : 1422;
}

size_t LidarPacketParser::OdometryPacketSize() const {
  return format_ == LidarWireFormat::kNativeLittleEndian ? 80 : 68;
}

std::optional<LidarPointPacket> LidarPacketParser::ParsePoints(
  const uint8_t* data, size_t size, std::string* error) const {
  std::string ignored;
  if (!error) {
    error = &ignored;
  }
  error->clear();
  const size_t stamp =
      format_ == LidarWireFormat::kNativeLittleEndian ? 8 : 2;
  const size_t points =
      format_ == LidarWireFormat::kNativeLittleEndian ? 28 : 20;
  if (!Envelope(data, size, PointPacketSize(), points +
      kLidarPointSlots * kPointBytes, error)) {
    return std::nullopt;
  }
  LidarPointPacket result;
  result.timestamp_ns_ = Read(data, stamp, 8);
  result.total_ = static_cast<uint32_t>(Read(data, stamp + 8, 4));
  result.index_ = static_cast<uint32_t>(Read(data, stamp + 12, 4));
  const auto count = Read(data, stamp + 16, 2);
  if (result.timestamp_ns_ == 0 || result.total_ == 0 ||
      result.total_ > max_points_ || count == 0 || count > kLidarPointSlots ||
      result.index_ >= result.total_ || count > result.total_ - result.index_) {
    *error = "lidar timestamp/total/index/posNum outside configured bounds";
    return std::nullopt;
  }
  for (size_t i = 0; i < count; ++i) {
    const size_t offset = points + i * kPointBytes;
    LidarPoint point;
    point.x_ = static_cast<int32_t>(Signed(Read(data, offset, 4), 4));
    point.y_ = static_cast<int32_t>(Signed(Read(data, offset + 4, 4), 4));
    point.z_ = static_cast<int32_t>(Signed(Read(data, offset + 8, 4), 4));
    for (size_t channel = 0; channel < point.rgba_.size(); ++channel) {
      point.rgba_[channel] = static_cast<uint32_t>(
        Read(data, offset + 12 + channel * 4, 4));
      if (point.rgba_[channel] > kMaxColor) {
        *error = "lidar RGBA uint32 channels must contain 0..255";
        return std::nullopt;
      }
    }
    result.points_.push_back(point);
  }
  return result;
}

std::optional<LidarOdometryPacket> LidarPacketParser::ParseOdometry(
  const uint8_t* data, size_t size, std::string* error) const {
  std::string ignored;
  if (!error) {
    error = &ignored;
  }
  error->clear();
  const size_t stamp =
      format_ == LidarWireFormat::kNativeLittleEndian ? 8 : 2;
  if (!Envelope(data, size, OdometryPacketSize(), stamp + 64, error)) {
    return std::nullopt;
  }
  LidarOdometryPacket result;
  result.timestamp_ns_ = Read(data, stamp, 8);
  if (result.timestamp_ns_ == 0) {
    *error = "lidar timestamp zero cannot identify an odometry sample";
    return std::nullopt;
  }
  for (size_t i = 0; i < result.position_.size(); ++i) {
    result.position_[i] = Signed(Read(data, stamp + 8 + i * 8, 8), 8);
  }
  for (size_t i = 0; i < result.quaternion_.size(); ++i) {
    result.quaternion_[i] = Signed(Read(data, stamp + 32 + i * 8, 8), 8);
  }
  return result;
}

}  // namespace astrall_driver
