// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/point_cloud_assembler.hpp"

#include <stdexcept>
#include <utility>

namespace astrall_driver {

PointCloudAssembler::PointCloudAssembler(uint32_t max_points,
    std::chrono::milliseconds timeout)
  : max_points_(max_points), timeout_(timeout) {
  if (max_points == 0 || max_points > kMaxLidarFramePoints ||
      timeout.count() <= 0) {
    throw std::invalid_argument("invalid lidar frame limit/timeout");
  }
}

void PointCloudAssembler::Discard() {
  if (active_) {
    ++dropped_;
  }
  active_ = false;
  points_.clear();
}

void PointCloudAssembler::Expire(Time now) {
  if (active_ && now - started_ >= timeout_) {
    Discard();
  }
}

uint64_t PointCloudAssembler::DroppedFrames() const {
  return dropped_;
}

std::optional<LidarPointFrame> PointCloudAssembler::Push(
  LidarPointPacket packet, Time now) {
  Expire(now);
  if (packet.timestamp_ns_ == 0 || packet.timestamp_ns_ < newest_) {
    return std::nullopt;
  }
  if (packet.timestamp_ns_ > newest_) {
    Discard();
    newest_ = packet.timestamp_ns_;
    total_ = packet.total_;
    if (packet.index_ != 0) {
      ++dropped_;
      return std::nullopt;
    }
    active_ = true;
    started_ = now;
  }
  if (!active_) {
    return std::nullopt;
  }
  if (total_ == 0 || total_ > max_points_ || packet.total_ != total_ ||
      packet.index_ != points_.size() || packet.points_.empty() ||
      packet.points_.size() > kLidarPointSlots || packet.index_ >= total_ ||
      packet.points_.size() > total_ - packet.index_) {
    Discard();
    return std::nullopt;
  }
  points_.insert(points_.end(), packet.points_.begin(), packet.points_.end());
  if (points_.size() != total_) {
    return std::nullopt;
  }
  active_ = false;
  auto complete = LidarPointFrame{ newest_, std::move(points_) };
  points_.clear();
  return complete;
}

}  // namespace astrall_driver
