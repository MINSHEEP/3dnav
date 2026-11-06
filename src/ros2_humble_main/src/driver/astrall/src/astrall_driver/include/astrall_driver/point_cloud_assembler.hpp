// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__POINT_CLOUD_ASSEMBLER_HPP_
#define ASTRALL_DRIVER__POINT_CLOUD_ASSEMBLER_HPP_

#include <chrono>
#include <optional>
#include <vector>

#include "astrall_driver/lidar_packet_parser.hpp"

namespace astrall_driver {

struct LidarPointFrame {
  uint64_t timestamp_ns_ = 0;
  std::vector<LidarPoint> points_;
};

/**
 * @brief 单帧有界缓存，外部串行化；严格顺序，重复/缺口/冲突使该时间戳整帧作废。
 * timestamp仅作为严格递增帧ID；已结束/超时的ID不重启，见Q04/Q28。
 */
class PointCloudAssembler {
 public:
  using Time = std::chrono::steady_clock::time_point;
  /** @brief 从首包计时，期限不因后续碎片刷新；最大点数1..1000000。 */
  explicit PointCloudAssembler(uint32_t max_points = 100000,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
  /** @brief 只返回完整帧；即使调用者绕过parser也检查全部范围。 */
  std::optional<LidarPointFrame> Push(LidarPointPacket packet, Time now);
  /** @brief 空闲时也可推进超时，迟到包不能重新建立同一帧。 */
  void Expire(Time now);
  /** @brief 清部分帧但保留高水位；停用/断线后旧ID不能重新发出。 */
  void Discard();
  /** @brief 被丢弃的部分帧/非法起始帧累计数，不是丢包精确计数。 */
  uint64_t DroppedFrames() const;

 private:
  const uint32_t max_points_;
  const std::chrono::milliseconds timeout_;
  uint64_t newest_ = 0;
  uint32_t total_ = 0;
  bool active_ = false;
  Time started_{};
  std::vector<LidarPoint> points_;
  uint64_t dropped_ = 0;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__POINT_CLOUD_ASSEMBLER_HPP_
