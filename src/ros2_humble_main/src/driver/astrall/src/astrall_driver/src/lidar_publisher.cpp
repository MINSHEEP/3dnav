// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/lidar_publisher.hpp"

#include <stdexcept>
#include <utility>

#include "rclcpp/create_publisher.hpp"

namespace astrall_driver {

LidarPublisher::LidarPublisher(rclcpp_lifecycle::LifecycleNode& node,
    const DriverParameters& parameters, Gate gate)
  : parameters_(parameters), gate_(std::move(gate)), clock_(node.get_clock()),
  parser_(parameters.lidar_wire_format_ == "packed_le" ?
      LidarWireFormat::kPackedLittleEndian :
      LidarWireFormat::kNativeLittleEndian, parameters.lidar_max_points_),
  assembler_(parameters.lidar_max_points_,
      std::chrono::milliseconds(parameters.lidar_frame_timeout_ms_)) {
  const auto error = parameters.ValidateLidar();
  if (!error.empty() || !gate_) {
    throw std::invalid_argument(error.empty() ? "lidar gate required" : error);
  }
  snapshot_.enabled_ = true;
  cloud_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
    node, parameters.point_cloud_topic_, rclcpp::SensorDataQoS().keep_last(1));
  odometry_publisher_ = rclcpp::create_publisher<nav_msgs::msg::Odometry>(
    node, parameters.lidar_odometry_topic_, rclcpp::QoS(1).reliable());
  transform_publisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

LidarPublisher::~LidarPublisher() {
  Stop();
}

void LidarPublisher::ClearLocked() {
  assembler_.Discard();
  cloud_.reset();
  odometry_.reset();
}

void LidarPublisher::SyncEpochLocked(uint64_t epoch) {
  if (snapshot_.epoch_ != epoch) {
    ClearLocked();
    snapshot_.epoch_ = epoch;
  }
}

bool LidarPublisher::CurrentLocked(uint64_t generation) const {
  return snapshot_.active_ && snapshot_.generation_ == generation;
}

bool LidarPublisher::Start(uint64_t generation) {
  Stop();
  const auto epoch = gate_(generation);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!epoch) {
      snapshot_.last_error_ = "lidar session gate is closed on activation";
      return false;
    }
    snapshot_.generation_ = generation;
    snapshot_.epoch_ = *epoch;
    snapshot_.active_ = true;
    snapshot_.last_error_.clear();
  }
  const auto points = [this, generation](const uint8_t* data, size_t size) {
        ReceivePoints(data, size, generation);
      };
  const auto odometry = [this, generation](const uint8_t* data, size_t size) {
        ReceiveOdometry(data, size, generation);
      };
  if (!cloud_receiver_.Start(parameters_.lidar_bind_address_,
      static_cast<uint16_t>(parameters_.point_cloud_port_),
      parser_.PointPacketSize(), points) ||
      !odometry_receiver_.Start(parameters_.lidar_bind_address_,
      static_cast<uint16_t>(parameters_.lidar_odometry_port_),
      parser_.OdometryPacketSize(), odometry)) {
    Stop();
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.last_error_ = "lidar UDP bind/start failed; inspect receivers";
    return false;
  }
  return true;
}

void LidarPublisher::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.active_ = false;
    ClearLocked();
  }
  cloud_receiver_.Stop();
  odometry_receiver_.Stop();
}

LidarSnapshot LidarPublisher::Snapshot() const {
  LidarSnapshot result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = snapshot_;
    result.dropped_frames_ = assembler_.DroppedFrames();
  }
  result.cloud_receiver_ = cloud_receiver_.Snapshot();
  result.odometry_receiver_ = odometry_receiver_.Snapshot();
  return result;
}

bool LidarPublisher::CheckReceive(uint64_t generation,
    std::optional<uint64_t> before, std::optional<uint64_t> after) {
  if (!CurrentLocked(generation)) {
    return false;
  }
  if (after && *after < snapshot_.epoch_) {
    ++snapshot_.stale_samples_;
    return false;
  }
  if (!before || !after || before != after) {
    ClearLocked();
    if (after) {
      snapshot_.epoch_ = *after;
    }
    ++snapshot_.stale_samples_;
    return false;
  }
  SyncEpochLocked(*after);
  return true;
}

void LidarPublisher::ReceivePoints(const uint8_t* data, size_t size,
    uint64_t generation) {
  const auto before = gate_(generation);
  std::string error;
  auto packet = before ? parser_.ParsePoints(data, size, &error) : std::nullopt;
  const auto stamp = before ? clock_->now().nanoseconds() : 0;
  const auto after = gate_(generation);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!CheckReceive(generation, before, after)) {
    return;
  }
  if (!packet) {
    ++snapshot_.invalid_packets_;
    snapshot_.last_error_ = error;
    assembler_.Discard();
    return;
  }
  snapshot_.last_cloud_source_ = packet->timestamp_ns_;
  const auto received = std::chrono::steady_clock::now();
  auto frame = assembler_.Push(std::move(*packet), received);
  if (frame) {
    snapshot_.overwritten_ += cloud_.has_value();
    cloud_ = Sample<LidarPointFrame>{ std::move(*frame), stamp, received };
  }
}

void LidarPublisher::ReceiveOdometry(const uint8_t* data, size_t size,
    uint64_t generation) {
  const auto before = gate_(generation);
  std::string error;
  auto packet = before ? parser_.ParseOdometry(data, size, &error) :
      std::nullopt;
  const auto stamp = before ? clock_->now().nanoseconds() : 0;
  const auto after = gate_(generation);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!CheckReceive(generation, before, after)) {
    return;
  }
  if (!packet || packet->timestamp_ns_ <= snapshot_.last_odometry_source_) {
    ++snapshot_.invalid_packets_;
    snapshot_.last_error_ = packet ? "old/duplicate odometry source ID" : error;
    return;
  }
  snapshot_.last_odometry_source_ = packet->timestamp_ns_;
  snapshot_.overwritten_ += odometry_.has_value();
  odometry_ = Sample<LidarOdometryPacket>{ *packet, stamp,
                                           std::chrono::steady_clock::now() };
}

void LidarPublisher::Publish() {
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = snapshot_.generation_;
  }
  const auto epoch = gate_(generation);
  std::optional<Sample<LidarPointFrame>> cloud;
  std::optional<Sample<LidarOdometryPacket>> odometry;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!CurrentLocked(generation) || !epoch) {
      ClearLocked();
      return;
    }
    if (*epoch < snapshot_.epoch_) {
      return;
    }
    SyncEpochLocked(*epoch);
    const auto now = std::chrono::steady_clock::now();
    assembler_.Expire(now);
    const auto age = std::chrono::milliseconds(parameters_.lidar_max_age_ms_);
    if (cloud_ && now - cloud_->received_ > age) {
      cloud_.reset();
      ++snapshot_.stale_samples_;
    }
    if (odometry_ && now - odometry_->received_ > age) {
      odometry_.reset();
      ++snapshot_.stale_samples_;
    }
    cloud.swap(cloud_);
    odometry.swap(odometry_);
  }
  if (cloud) {
    auto message =
        ConvertLidarPoints(cloud->data_, cloud->ros_ns_, parameters_);
    if (message && gate_(generation) == epoch) {
      cloud_publisher_->publish(*message);
      std::lock_guard<std::mutex> lock(mutex_);
      ++snapshot_.published_clouds_;
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      ++snapshot_.invalid_packets_;
      snapshot_.last_error_ = "cloud conversion/time or session gate rejected";
    }
  }
  if (odometry) {
    auto messages = ConvertLidarOdometry(odometry->data_, odometry->ros_ns_,
            parameters_);
    if (messages && gate_(generation) == epoch) {
      odometry_publisher_->publish(messages->odometry_);
      transform_publisher_->sendTransform(messages->transform_);
      std::lock_guard<std::mutex> lock(mutex_);
      ++snapshot_.published_odometry_;
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      ++snapshot_.invalid_packets_;
      snapshot_.last_error_ =
          "odometry quaternion/position/time or gate rejected";
    }
  }
}

}  // namespace astrall_driver
