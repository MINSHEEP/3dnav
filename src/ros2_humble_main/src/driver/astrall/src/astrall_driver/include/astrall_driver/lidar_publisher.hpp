// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__LIDAR_PUBLISHER_HPP_
#define ASTRALL_DRIVER__LIDAR_PUBLISHER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "astrall_driver/lidar_conversions.hpp"
#include "astrall_driver/udp_receiver.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace astrall_driver {

struct LidarSnapshot {
  bool enabled_ = false;
  bool active_ = false;
  uint64_t generation_ = 0;
  uint64_t epoch_ = 0;
  uint64_t invalid_packets_ = 0;
  uint64_t dropped_frames_ = 0;
  uint64_t overwritten_ = 0;
  uint64_t stale_samples_ = 0;
  uint64_t published_clouds_ = 0;
  uint64_t published_odometry_ = 0;
  uint64_t last_cloud_source_ = 0;
  uint64_t last_odometry_source_ = 0;
  std::string last_error_;
  UdpReceiverSnapshot cloud_receiver_;
  UdpReceiverSnapshot odometry_receiver_;
};

/**
 * @brief S5双UDP接收/单份最新样本/ROS发布，复用S3会话门，不申请控制权。
 * Start/Stop/Publish由ROS同一互斥组串行调用，Snapshot可并发；UDP不调用SDK。
 * Gate须快速返回有效会话的断线epoch；nullopt关闭接收/交付，不依赖运动持权。
 */
class LidarPublisher {
 public:
  using Gate = std::function<std::optional<uint64_t>(uint64_t)>;
  LidarPublisher(rclcpp_lifecycle::LifecycleNode& node,
      const DriverParameters& parameters, Gate gate);
  ~LidarPublisher();
  /** @brief 激活后绑定两端口，失败清理已启动线程并保留诊断。 */
  bool Start(uint64_t generation);
  /** @brief 清队列/部分帧，保留源高水位；join接收线程后关闭FD，可重复。 */
  void Stop();
  /** @brief 仅交付当前代次且未过稳态TTL的最新完整帧/pose；转换不持核心锁。 */
  void Publish();
  /** @brief 错包/过期/覆盖/发布及真实端口统计。 */
  LidarSnapshot Snapshot() const;

 private:
  template <typename Data>
  struct Sample {
    Data data_;
    int64_t ros_ns_ = 0;
    std::chrono::steady_clock::time_point received_;
  };
  void ClearLocked();
  void SyncEpochLocked(uint64_t epoch);
  bool CurrentLocked(uint64_t generation) const;
  void ReceivePoints(const uint8_t* data, size_t size, uint64_t generation);
  void ReceiveOdometry(const uint8_t* data, size_t size, uint64_t generation);
  bool CheckReceive(uint64_t generation, std::optional<uint64_t> before,
      std::optional<uint64_t> after);

  const DriverParameters parameters_;
  const Gate gate_;
  const rclcpp::Clock::SharedPtr clock_;
  const LidarPacketParser parser_;
  mutable std::mutex mutex_;
  PointCloudAssembler assembler_;
  LidarSnapshot snapshot_;
  std::optional<Sample<LidarPointFrame>> cloud_;
  std::optional<Sample<LidarOdometryPacket>> odometry_;
  UdpReceiver cloud_receiver_;
  UdpReceiver odometry_receiver_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_publisher_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__LIDAR_PUBLISHER_HPP_
