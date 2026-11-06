// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__LIDAR_PACKET_PARSER_HPP_
#define ASTRALL_DRIVER__LIDAR_PACKET_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astrall_driver {

inline constexpr uint32_t kMaxLidarFramePoints = 1000000;
inline constexpr size_t kLidarPointSlots = 50;

enum class LidarWireFormat { kNativeLittleEndian, kPackedLittleEndian };

struct LidarPoint {
  int32_t x_ = 0;
  int32_t y_ = 0;
  int32_t z_ = 0;
  std::array<uint32_t, 4> rgba_{};
};

struct LidarPointPacket {
  uint64_t timestamp_ns_ = 0;
  uint32_t total_ = 0;
  uint32_t index_ = 0;
  std::vector<LidarPoint> points_;
};

struct LidarOdometryPacket {
  uint64_t timestamp_ns_ = 0;
  std::array<int64_t, 3> position_{};
  std::array<int64_t, 4> quaternion_{};
};

/**
 * @brief 手册5.10固定50点槽的显式线格式，逐字节小端读，不解引用未对齐结构。
 * native_le为本地x86_64布局，packed_le须显式选；不据长度猜测，见Q04。
 * 点坐标保留0.1mm，odom位置保留米，四元数比例由ROS转换参数指定。
 */
class LidarPacketParser {
 public:
  /** @brief max_points 限制外部 total；构造/之后只读，Parse可并发调用。 */
  explicit LidarPacketParser(LidarWireFormat format,
      uint32_t max_points = 100000);
  /** @brief 错误返回nullopt并给出原因；nullptr/非精确长度均拒绝。 */
  std::optional<LidarPointPacket> ParsePoints(const uint8_t* data, size_t size,
      std::string* error) const;
  /** @brief 仅解析整数位姿，不猜测时间域或四元数比例。 */
  std::optional<LidarOdometryPacket> ParseOdometry(const uint8_t* data,
      size_t size, std::string* error) const;
  /** @brief 每种布局都有固定包长，包括未使用点槽和尾部对齐。 */
  size_t PointPacketSize() const;
  /** @brief 里程计包的固定长度。 */
  size_t OdometryPacketSize() const;

 private:
  const LidarWireFormat format_;
  const uint32_t max_points_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__LIDAR_PACKET_PARSER_HPP_
