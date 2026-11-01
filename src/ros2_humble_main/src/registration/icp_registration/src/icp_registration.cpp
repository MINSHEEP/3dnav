#include "icp_registration/icp_registration.hpp"
#include "icp_registration/search_seeds.hpp"
#include "icp_registration/candidate_quality.hpp"
#include "icp_registration/refinement_budget.hpp"
#include "icp_registration/tracking_validation.hpp"
#include "icp_registration/target_sampling.hpp"
#include "icp_registration/pose_output.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lzp_interface/action/relocalize.hpp>
#include <lzp_interface/msg/current_map.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace
{

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using Relocalize = lzp_interface::action::Relocalize;
using GoalHandleRelocalize = rclcpp_action::ServerGoalHandle<Relocalize>;
using SteadyClock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

enum ResultCode : uint16_t
{
  CODE_OK = 0,
  CODE_CANCELLED = 1,
  CODE_INVALID_GOAL = 10,
  CODE_MAP_NAME_INVALID = 11,
  CODE_METADATA_MISSING = 12,
  CODE_METADATA_INVALID = 13,
  CODE_PCD_MISSING = 14,
  CODE_PCD_CHECKSUM_MISMATCH = 15,
  CODE_PCD_LOAD_FAILED = 16,
  CODE_CLOUD_TIMEOUT = 20,
  CODE_CLOUD_INVALID = 21,
  CODE_TF_UNAVAILABLE = 22,
  CODE_NDT_FAILED = 30,
  CODE_GICP_FAILED = 31,
  CODE_VALIDATION_FAILED = 32,
  CODE_TIMEOUT = 33,
  CODE_INTERNAL_ERROR = 34
};

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::string normalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

bool isFiniteMatrix(const Eigen::Matrix4f & transform)
{
  return transform.array().isFinite().all();
}

bool poseToMatrix(
  const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d & transform,
  std::string & error)
{
  const auto & p = pose.position;
  const auto & o = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(o.x) || !std::isfinite(o.y) ||
    !std::isfinite(o.z) || !std::isfinite(o.w))
  {
    error = "粗初始位姿包含 NaN 或无穷值";
    return false;
  }

  Eigen::Quaterniond quaternion(o.w, o.x, o.y, o.z);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1.0e-6) {
    error = "粗初始位姿的四元数无效";
    return false;
  }
  quaternion.normalize();

  transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  transform(0, 3) = p.x;
  transform(1, 3) = p.y;
  transform(2, 3) = p.z;
  return true;
}

Eigen::Matrix4d transformToMatrix(
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  const auto & t = transform_msg.transform.translation;
  const auto & r = transform_msg.transform.rotation;
  Eigen::Quaterniond quaternion(r.w, r.x, r.y, r.z);
  quaternion.normalize();

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  transform(0, 3) = t.x;
  transform(1, 3) = t.y;
  transform(2, 3) = t.z;
  return transform;
}

double matrixYaw(const Eigen::Matrix4d & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}

std::pair<double, double> matrixRollPitch(const Eigen::Matrix4d & transform)
{
  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  return {roll, pitch};
}

Eigen::Matrix4d planarPoseMatrix(double x, double y, double z, double yaw)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  return transform;
}

PointCloudPtr voxelFilter(const PointCloudPtr & input, double leaf_size)
{
  PointCloudPtr output(new PointCloud);
  if (!input || input->empty()) {
    return output;
  }
  if (!(leaf_size > 0.0)) {
    *output = *input;
    return output;
  }

  pcl::VoxelGrid<PointT> filter;
  const float leaf = static_cast<float>(leaf_size);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}

PointCloudPtr limitCloud(const PointCloudPtr & input, std::size_t maximum_points)
{
  if (!input || input->size() <= maximum_points || maximum_points == 0U) {
    return input;
  }

  PointCloudPtr output(new PointCloud);
  output->reserve(maximum_points);
  const double stride =
    static_cast<double>(input->size()) / static_cast<double>(maximum_points);
  for (std::size_t i = 0; i < maximum_points; ++i) {
    const auto index = std::min(
      static_cast<std::size_t>(std::floor(static_cast<double>(i) * stride)),
      input->size() - 1U);
    output->push_back((*input)[index]);
  }
  output->width = static_cast<uint32_t>(output->size());
  output->height = 1U;
  output->is_dense = true;
  return output;
}

// Small dependency-free SHA-256 implementation.  It avoids adding OpenSSL to
// the 4 GB controller solely to validate an optional localization-PCD digest.
class Sha256
{
public:
  Sha256()
  : data_length_(0), bit_length_(0),
    state_{
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}
  {
  }

  void update(const uint8_t * data, std::size_t length)
  {
    for (std::size_t i = 0; i < length; ++i) {
      data_[data_length_++] = data[i];
      if (data_length_ == 64U) {
        transform();
        bit_length_ += 512U;
        data_length_ = 0U;
      }
    }
  }

  std::array<uint8_t, 32> finalize()
  {
    std::size_t i = data_length_;
    data_[i++] = 0x80U;
    if (i > 56U) {
      while (i < 64U) {
        data_[i++] = 0U;
      }
      transform();
      i = 0U;
    }
    while (i < 56U) {
      data_[i++] = 0U;
    }

    bit_length_ += static_cast<uint64_t>(data_length_) * 8U;
    for (int shift = 56; shift >= 0; shift -= 8) {
      data_[i++] = static_cast<uint8_t>((bit_length_ >> shift) & 0xffU);
    }
    transform();

    std::array<uint8_t, 32> digest{};
    for (std::size_t word = 0; word < state_.size(); ++word) {
      digest[word * 4U] = static_cast<uint8_t>((state_[word] >> 24U) & 0xffU);
      digest[word * 4U + 1U] = static_cast<uint8_t>((state_[word] >> 16U) & 0xffU);
      digest[word * 4U + 2U] = static_cast<uint8_t>((state_[word] >> 8U) & 0xffU);
      digest[word * 4U + 3U] = static_cast<uint8_t>(state_[word] & 0xffU);
    }
    return digest;
  }

private:
  static uint32_t rotateRight(uint32_t value, uint32_t amount)
  {
    return (value >> amount) | (value << (32U - amount));
  }

  void transform()
  {
    static constexpr std::array<uint32_t, 64> constants{
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
      0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
      0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
      0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
      0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
      0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
      0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
      0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
      0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
      0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
      0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
      0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

    std::array<uint32_t, 64> words{};
    for (std::size_t i = 0; i < 16U; ++i) {
      const std::size_t offset = i * 4U;
      words[i] =
        (static_cast<uint32_t>(data_[offset]) << 24U) |
        (static_cast<uint32_t>(data_[offset + 1U]) << 16U) |
        (static_cast<uint32_t>(data_[offset + 2U]) << 8U) |
        static_cast<uint32_t>(data_[offset + 3U]);
    }
    for (std::size_t i = 16U; i < words.size(); ++i) {
      const uint32_t s0 =
        rotateRight(words[i - 15U], 7U) ^ rotateRight(words[i - 15U], 18U) ^
        (words[i - 15U] >> 3U);
      const uint32_t s1 =
        rotateRight(words[i - 2U], 17U) ^ rotateRight(words[i - 2U], 19U) ^
        (words[i - 2U] >> 10U);
      words[i] = words[i - 16U] + s0 + words[i - 7U] + s1;
    }

    uint32_t a = state_[0];
    uint32_t b = state_[1];
    uint32_t c = state_[2];
    uint32_t d = state_[3];
    uint32_t e = state_[4];
    uint32_t f = state_[5];
    uint32_t g = state_[6];
    uint32_t h = state_[7];

    for (std::size_t i = 0; i < words.size(); ++i) {
      const uint32_t sum1 =
        rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const uint32_t choose = (e & f) ^ ((~e) & g);
      const uint32_t temporary1 = h + sum1 + choose + constants[i] + words[i];
      const uint32_t sum0 =
        rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temporary2 = sum0 + majority;

      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<uint8_t, 64> data_{};
  std::size_t data_length_;
  uint64_t bit_length_;
  std::array<uint32_t, 8> state_;
};

bool sha256File(
  const std::filesystem::path & path, std::string & digest, std::string & error)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "无法读取 PCD 以校验 SHA-256: " + path.string();
    return false;
  }

  Sha256 sha;
  std::array<char, 64 * 1024> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0) {
      sha.update(
        reinterpret_cast<const uint8_t *>(buffer.data()),
        static_cast<std::size_t>(count));
    }
  }
  if (!stream.eof()) {
    error = "读取 PCD 进行 SHA-256 校验时发生错误: " + path.string();
    return false;
  }

  const auto bytes = sha.finalize();
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  digest = output.str();
  return true;
}

std::string normalizeDigest(std::string value)
{
  constexpr const char * prefix = "sha256:";
  if (value.rfind(prefix, 0) == 0U) {
    value.erase(0, std::char_traits<char>::length(prefix));
  }
  value.erase(
    std::remove_if(value.begin(), value.end(), [](unsigned char character) {
      return std::isspace(character) != 0;
    }),
    value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

struct RegistrationMetrics
{
  double rmse{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  double planar_eigen_ratio{0.0};
  std::size_t matched_points{0U};
};

RegistrationMetrics computeMetrics(
  const PointCloudPtr & source, const PointCloudPtr & target,
  const Eigen::Matrix4f & source_to_target, double maximum_distance)
{
  RegistrationMetrics metrics;
  if (!source || source->empty() || !target || target->empty()) {
    return metrics;
  }

  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  const float maximum_squared =
    static_cast<float>(maximum_distance * maximum_distance);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  double squared_error_sum = 0.0;
  Eigen::Vector2d target_sum = Eigen::Vector2d::Zero();
  Eigen::Matrix2d target_second_moment = Eigen::Matrix2d::Zero();

  for (const auto & point : *source) {
    Eigen::Vector4f input(point.x, point.y, point.z, 1.0F);
    const Eigen::Vector4f transformed = source_to_target * input;
    PointT query;
    query.x = transformed.x();
    query.y = transformed.y();
    query.z = transformed.z();

    if (tree.nearestKSearch(query, 1, indices, squared_distances) != 1 ||
      squared_distances[0] > maximum_squared)
    {
      continue;
    }

    squared_error_sum += squared_distances[0];
    const auto & matched = (*target)[static_cast<std::size_t>(indices[0])];
    const Eigen::Vector2d xy(matched.x, matched.y);
    target_sum += xy;
    target_second_moment += xy * xy.transpose();
    ++metrics.matched_points;
  }

  metrics.overlap =
    static_cast<double>(metrics.matched_points) /
    static_cast<double>(source->size());
  if (metrics.matched_points == 0U) {
    return metrics;
  }
  metrics.rmse = std::sqrt(
    squared_error_sum / static_cast<double>(metrics.matched_points));

  if (metrics.matched_points >= 3U) {
    const double count = static_cast<double>(metrics.matched_points);
    const Eigen::Vector2d mean = target_sum / count;
    const Eigen::Matrix2d covariance =
      target_second_moment / count - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() == Eigen::Success) {
      const auto eigenvalues = solver.eigenvalues();
      const double largest = std::max(eigenvalues(1), 0.0);
      const double smallest = std::max(eigenvalues(0), 0.0);
      if (largest > 1.0e-9) {
        metrics.planar_eigen_ratio = smallest / largest;
      }
    }
  }
  return metrics;
}

}  // namespace

namespace icp
{

class IcpNode::Impl
{
public:
  explicit Impl(IcpNode & node)
  : node_(node)
  {
    declareParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, &node_, false);
    // Registration runs in its own worker while the ROS executor continues to
    // service /tf on another thread.  Tell tf2 about that arrangement so
    // timestamped lookups do not emit the "no dedicated thread" error.
    tf_buffer_->setUsingDedicatedThread(true);

    auto pointcloud_qos = rclcpp::SensorDataQoS();
    pointcloud_qos.keep_last(3);
    pointcloud_subscription_ =
      node_.create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, pointcloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        pointcloudCallback(std::move(message));
      });

    action_server_ = rclcpp_action::create_server<Relocalize>(
      &node_, "/relocalize",
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Relocalize::Goal> goal)
      {
        return handleGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleRelocalize> goal_handle) {
        return handleCancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleRelocalize> goal_handle) {
        handleAccepted(goal_handle);
      });

    auto global_map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    global_map_qos.reliable().transient_local();
    global_map_publisher_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      global_map_topic_, global_map_qos);
    global_map_status_publisher_ =
      node_.create_publisher<std_msgs::msg::String>(
      global_map_topic_ + "/status", global_map_qos);
    auto current_map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    current_map_qos.reliable().transient_local();
    current_map_subscription_ =
      node_.create_subscription<lzp_interface::msg::CurrentMap>(
      "/current_map_info", current_map_qos,
      [this](lzp_interface::msg::CurrentMap::ConstSharedPtr message) {
        const std::string next_map_name =
          message->success ? message->map_name : "";
        const std::string next_load_id =
          message->success ? message->load_id : "";
        const std::string announcement = message->message;
        const bool confirmed_reload =
          announcement.rfind("Successfully loaded and confirmed map:", 0) == 0 ||
          announcement.rfind("Successfully loaded map:", 0) == 0;
        bool identity_changed = false;
        {
          std::lock_guard<std::mutex> lock(active_map_mutex_);
          identity_changed =
            active_map_name_ != next_map_name ||
            active_map_load_id_ != next_load_id;
          if (identity_changed || confirmed_reload) {
            ++active_map_generation_;
          }
          active_map_name_ = next_map_name;
          active_map_load_id_ = next_load_id;
        }
        std::lock_guard<std::mutex> lock(published_map_mutex_);
        if (identity_changed || confirmed_reload) {
          published_map_name_.clear();
        }
      });
    global_map_timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(global_map_publish_period_sec_)),
      [this]() {publishActiveGlobalMap();});

    RCLCPP_INFO(
      node_.get_logger(),
      "Relocalization action ready on /relocalize. It is on-demand only and "
      "publishes neither TF nor /initialpose.");
    RCLCPP_INFO(node_.get_logger(),
      "REGISTRATION_CONTRACT model=%s robust_only=true legacy_projection_unchanged=true",
      icp::kRigidTransformModel);
    RCLCPP_INFO(node_.get_logger(),
      "REGISTRATION_TARGET_POLICY model=%s robust_only=true max_refinements=3",
      icp::kTargetSamplingModel);
    RCLCPP_INFO(
      node_.get_logger(),
      "PCD directory: %s; metadata schema: %s v%d; internal registration "
      "worker limit: 1 (PCL NDT/GICP are used single-threaded).",
      pcd_directory_.c_str(), metadata_schema_.c_str(), metadata_version_);
    RCLCPP_INFO(
      node_.get_logger(),
      "Saved global map publication ready on %s: period %.1f s, max %zu "
      "points, published only while subscribers exist.",
      global_map_topic_.c_str(), global_map_publish_period_sec_,
      global_map_maximum_points_);
  }

  ~Impl()
  {
    shutting_down_.store(true);
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      collecting_clouds_ = false;
    }
    cloud_condition_.notify_all();
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  struct LoadedMap
  {
    std::string map_name;
    PointCloudPtr cloud;
    double voxel_size{0.0};
    std::filesystem::file_time_type pcd_write_time{};
    std::filesystem::file_time_type metadata_write_time{};
    uintmax_t pcd_file_size{0U};
  };

  struct FrameData
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    PointCloudPtr cloud;
    Eigen::Matrix4d odom_from_base{Eigen::Matrix4d::Identity()};
  };

  struct ExecutionContext
  {
    std::shared_ptr<GoalHandleRelocalize> goal_handle;
    std::shared_ptr<Relocalize::Result> result;
    SteadyClock::time_point deadline;
    double search_xy{0.0};
    double search_yaw{0.0};
    double timeout_sec{10.0};
    std::string last_candidate_error{};
  };

  void publishActiveGlobalMap()
  {
    if (!global_map_publisher_ ||
      global_map_publisher_->get_subscription_count() == 0U)
    {
      return;
    }

    std::string map_name;
    std::string map_load_id;
    uint64_t map_generation = 0;
    {
      std::lock_guard<std::mutex> lock(active_map_mutex_);
      map_name = active_map_name_;
      map_load_id = active_map_load_id_;
      map_generation = active_map_generation_;
    }
    if (map_name.empty()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(published_map_mutex_);
      if (published_map_name_ == map_name) {
        return;
      }
    }

    LoadedMap loaded_map;
    uint16_t error_code = CODE_INTERNAL_ERROR;
    std::string error;
    if (!loadMap(map_name, loaded_map, error_code, error)) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 10000,
        "Global map preview skipped: %s", error.c_str());
      return;
    }
    const auto preview = limitCloud(
      loaded_map.cloud, global_map_maximum_points_);
    if (!preview || preview->empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*preview, message);
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = map_frame_;
    // PointCloud2 has only a frame_id, not a map identity. Publish a second
    // latched message keyed by the exact cloud timestamp so the browser can
    // reject a stale cloud after a map switch or same-name reload. Keep the
    // nanoseconds as a JSON string: JavaScript Number loses 64-bit precision.
    std_msgs::msg::String status;
    std::ostringstream status_json;
    status_json << "{\"map_name\":\"" << map_name
                << "\",\"load_id\":\"" << map_load_id
                << "\",\"stamp_ns\":\""
                << stampNanoseconds(message.header.stamp) << "\"}";
    status.data = status_json.str();
    {
      // A load can complete while this cloud is being prepared. Publish the
      // cloud and its identity only for the same active-map generation.
      std::lock_guard<std::mutex> active_lock(active_map_mutex_);
      if (active_map_name_ != map_name ||
        active_map_load_id_ != map_load_id ||
        active_map_generation_ != map_generation)
      {
        return;
      }
      global_map_publisher_->publish(message);
      global_map_status_publisher_->publish(status);
      std::lock_guard<std::mutex> published_lock(published_map_mutex_);
      published_map_name_ = map_name;
    }
  }

  void declareParameters()
  {
    pointcloud_topic_ =
      node_.declare_parameter<std::string>("pointcloud_topic", "/cloud_registered");
    pcd_directory_ = node_.declare_parameter<std::string>(
      "pcd_directory",
      "/home/a/3dnav/src/ros2_humble_main/src/navigation/"
      "robot_navigation2/pcdmaps");
    metadata_schema_ = node_.declare_parameter<std::string>(
      "metadata_schema", "ros2_mid360_localization_map");
    metadata_version_ = static_cast<int>(
      node_.declare_parameter<int64_t>("metadata_version", 1));
    global_map_topic_ = node_.declare_parameter<std::string>(
      "global_map_topic", "/global_localization_map");
    global_map_publish_period_sec_ = node_.declare_parameter<double>(
      "global_map_publish_period_sec", 3.0);
    global_map_maximum_points_ = static_cast<std::size_t>(std::max<int64_t>(
        1000, node_.declare_parameter<int64_t>(
          "global_map_maximum_points", 120000)));
    map_frame_ = normalizeFrame(node_.declare_parameter<std::string>("map_frame", "map"));
    odom_frame_ = normalizeFrame(node_.declare_parameter<std::string>("odom_frame", "odom"));
    base_frame_ = normalizeFrame(
      node_.declare_parameter<std::string>("base_frame", "base_footprint"));

    default_search_xy_ =
      node_.declare_parameter<double>("default_search_xy", 3.0);
    default_search_yaw_ = degreesToRadians(
      node_.declare_parameter<double>("default_search_yaw_deg", 90.0));
    hard_max_search_xy_ =
      node_.declare_parameter<double>("hard_max_search_xy", 3.0);
    hard_max_search_yaw_ = degreesToRadians(
      node_.declare_parameter<double>("hard_max_search_yaw_deg", 90.0));
    default_timeout_sec_ =
      node_.declare_parameter<double>("default_timeout_sec", 30.0);
    maximum_timeout_sec_ =
      node_.declare_parameter<double>("maximum_timeout_sec", 30.0);

    accumulation_frames_ = static_cast<int>(std::max<int64_t>(
        3, node_.declare_parameter<int64_t>("accumulation_frames", 5)));
    minimum_consistent_frames_ = static_cast<int>(std::max<int64_t>(
        2, node_.declare_parameter<int64_t>("minimum_consistent_frames", 5)));
    cloud_wait_timeout_sec_ =
      node_.declare_parameter<double>("cloud_wait_timeout_sec", 3.0);
    tf_lookup_retry_sec_ =
      node_.declare_parameter<double>("tf_lookup_retry_sec", 0.25);
    source_min_range_ =
      node_.declare_parameter<double>("source_min_range", 0.55);
    source_max_range_ =
      node_.declare_parameter<double>("source_max_range", 12.0);
    source_min_z_ =
      node_.declare_parameter<double>("source_min_z", -0.15);
    source_max_z_ =
      node_.declare_parameter<double>("source_max_z", 2.5);
    source_self_radius_ =
      node_.declare_parameter<double>("source_self_radius", 0.55);
    source_self_min_z_ =
      node_.declare_parameter<double>("source_self_min_z", -0.5);
    source_self_max_z_ =
      node_.declare_parameter<double>("source_self_max_z", 0.7);
    rough_leaf_size_ =
      node_.declare_parameter<double>("rough_leaf_size", 0.45);
    fine_leaf_size_ =
      node_.declare_parameter<double>("fine_leaf_size", 0.15);
    minimum_points_per_frame_ =
      static_cast<std::size_t>(std::max<int64_t>(
        50, node_.declare_parameter<int64_t>("minimum_points_per_frame", 300)));
    minimum_source_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        100, node_.declare_parameter<int64_t>("minimum_source_points", 900)));
    minimum_rough_source_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        100, node_.declare_parameter<int64_t>(
          "minimum_rough_source_points", 300)));
    minimum_target_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        200, node_.declare_parameter<int64_t>("minimum_target_points", 1500)));
    maximum_raw_points_per_frame_ =
      static_cast<std::size_t>(std::max<int64_t>(
        1000, node_.declare_parameter<int64_t>(
          "maximum_raw_points_per_frame", 120000)));
    maximum_rough_source_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        500, node_.declare_parameter<int64_t>(
          "maximum_rough_source_points", 6000)));
    maximum_fine_source_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        1000, node_.declare_parameter<int64_t>(
          "maximum_fine_source_points", 12000)));
    maximum_rough_target_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        1000, node_.declare_parameter<int64_t>(
          "maximum_rough_target_points", 30000)));
    maximum_fine_target_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        2000, node_.declare_parameter<int64_t>(
          "maximum_fine_target_points", 60000)));
    maximum_map_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        10000, node_.declare_parameter<int64_t>("maximum_map_points", 1000000)));
    maximum_pcd_bytes_ =
      static_cast<uintmax_t>(std::max<int64_t>(
        1, node_.declare_parameter<int64_t>("maximum_pcd_mebibytes", 64))) *
      1024U * 1024U;

    roi_radius_ = node_.declare_parameter<double>("roi_radius", 12.0);
    roi_min_z_ = node_.declare_parameter<double>("roi_min_z", -1.5);
    roi_max_z_ = node_.declare_parameter<double>("roi_max_z", 3.0);
    ndt_resolution_ =
      node_.declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ =
      node_.declare_parameter<double>("ndt_step_size", 0.2);
    ndt_transformation_epsilon_ =
      node_.declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    ndt_maximum_iterations_ = static_cast<int>(std::max<int64_t>(
        1, node_.declare_parameter<int64_t>("ndt_maximum_iterations", 12)));
    ndt_yaw_hypotheses_ = static_cast<int>(std::clamp<int64_t>(
        node_.declare_parameter<int64_t>("ndt_yaw_hypotheses", 5), 1, 5));
    ndt_maximum_fitness_ =
      node_.declare_parameter<double>("ndt_maximum_fitness", 2.0);

    gicp_maximum_iterations_ = static_cast<int>(std::max<int64_t>(
        1, node_.declare_parameter<int64_t>("gicp_maximum_iterations", 15)));
    gicp_maximum_optimizer_iterations_ =
      static_cast<int>(std::max<int64_t>(
        1, node_.declare_parameter<int64_t>(
          "gicp_maximum_optimizer_iterations", 8)));
    initial_budget_.ndt = static_cast<int>(std::clamp<int64_t>(
        node_.declare_parameter<int64_t>("initial_ndt_maximum_iterations", 20), 1, 40));
    initial_budget_.gicp = static_cast<int>(std::clamp<int64_t>(
        node_.declare_parameter<int64_t>("initial_gicp_maximum_iterations", 40), 1, 80));
    initial_budget_.optimizer = static_cast<int>(std::clamp<int64_t>(
        node_.declare_parameter<int64_t>("initial_gicp_maximum_optimizer_iterations", 12), 1, 30));
    gicp_maximum_correspondence_distance_ =
      node_.declare_parameter<double>(
      "gicp_maximum_correspondence_distance", 0.6);
    gicp_transformation_epsilon_ =
      node_.declare_parameter<double>("gicp_transformation_epsilon", 0.001);
    gicp_fitness_epsilon_ =
      node_.declare_parameter<double>("gicp_fitness_epsilon", 0.0001);
    gicp_correspondence_randomness_ =
      static_cast<int>(std::max<int64_t>(
        5, node_.declare_parameter<int64_t>(
          "gicp_correspondence_randomness", 20)));

    metric_maximum_distance_ =
      node_.declare_parameter<double>("metric_maximum_distance", 0.4);
    maximum_rmse_ =
      node_.declare_parameter<double>("maximum_rmse", 0.20);
    minimum_overlap_ =
      node_.declare_parameter<double>("minimum_overlap", 0.55);
    minimum_planar_eigen_ratio_ =
      node_.declare_parameter<double>("minimum_planar_eigen_ratio", 0.01);
    minimum_matched_points_ =
      static_cast<std::size_t>(std::max<int64_t>(
        100, node_.declare_parameter<int64_t>("minimum_matched_points", 500)));
    maximum_vertical_correction_ =
      node_.declare_parameter<double>("maximum_vertical_correction", 0.35);
    tracking_maximum_rmse_ =
      node_.declare_parameter<double>("tracking_maximum_rmse", 0.22);
    tracking_maximum_vertical_correction_ =
      node_.declare_parameter<double>("tracking_maximum_vertical_correction", 0.45);
    maximum_tilt_ = degreesToRadians(
      node_.declare_parameter<double>("maximum_tilt_deg", 8.0));
    refined_yaw_stddev_ = degreesToRadians(
      node_.declare_parameter<double>("refined_yaw_stddev_deg", 5.0));
    minimum_refined_xy_stddev_ =
      node_.declare_parameter<double>("minimum_refined_xy_stddev", 0.05);

    validateParameters();
  }

  void validateParameters()
  {
    const bool invalid =
      pcd_directory_.empty() || metadata_schema_.empty() ||
      global_map_topic_.empty() ||
      !std::isfinite(global_map_publish_period_sec_) ||
      global_map_publish_period_sec_ < 0.5 ||
      metadata_version_ <= 0 ||
      map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      hard_max_search_xy_ <= 0.0 || hard_max_search_yaw_ <= 0.0 ||
      default_search_xy_ <= 0.0 ||
      default_search_xy_ > hard_max_search_xy_ ||
      default_search_yaw_ <= 0.0 ||
      default_search_yaw_ > hard_max_search_yaw_ ||
      maximum_timeout_sec_ < 0.1 || default_timeout_sec_ <= 0.0 ||
      default_timeout_sec_ > maximum_timeout_sec_ ||
      cloud_wait_timeout_sec_ <= 0.0 || tf_lookup_retry_sec_ < 0.0 ||
      minimum_consistent_frames_ > accumulation_frames_ ||
      source_min_range_ < 0.0 ||
      source_max_range_ <= source_min_range_ ||
      source_max_z_ <= source_min_z_ ||
      rough_leaf_size_ <= 0.0 || fine_leaf_size_ <= 0.0 ||
      fine_leaf_size_ > rough_leaf_size_ ||
      minimum_rough_source_points_ > minimum_source_points_ ||
      minimum_rough_source_points_ > maximum_rough_source_points_ ||
      roi_radius_ <= 0.0 || roi_max_z_ <= roi_min_z_ ||
      ndt_resolution_ <= 0.0 || ndt_step_size_ <= 0.0 ||
      gicp_maximum_correspondence_distance_ <= 0.0 ||
      metric_maximum_distance_ <= 0.0 || maximum_rmse_ <= 0.0 ||
      minimum_overlap_ <= 0.0 || minimum_overlap_ > 1.0 ||
      minimum_planar_eigen_ratio_ < 0.0 ||
      minimum_planar_eigen_ratio_ > 1.0 ||
      maximum_vertical_correction_ < 0.0 || maximum_tilt_ <= 0.0 ||
      !std::isfinite(tracking_maximum_rmse_) || tracking_maximum_rmse_ <= 0.0 ||
      tracking_maximum_rmse_ > 0.25 ||
      !std::isfinite(tracking_maximum_vertical_correction_) ||
      tracking_maximum_vertical_correction_ <= 0.0 ||
      tracking_maximum_vertical_correction_ > 0.60;
    if (invalid) {
      const std::string message =
        "icp_registration 参数无效；请检查搜索/超时、点云范围、"
        "体素、ROI、NDT/GICP 和质量阈值";
      RCLCPP_FATAL(node_.get_logger(), "%s", message.c_str());
      throw std::invalid_argument(message);
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Relocalize::Goal>)
  {
    bool expected = false;
    if (!active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "Rejected /relocalize goal because another relocalization is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleRelocalize>)
  {
    cloud_condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleRelocalize> goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    try {
      worker_thread_ = std::thread([this, goal_handle]() {
          runGoal(goal_handle);
        });
    } catch (const std::exception & exception) {
      active_.store(false);
      auto result = std::make_shared<Relocalize::Result>();
      result->success = false;
      result->code = CODE_INTERNAL_ERROR;
      result->message =
        std::string("无法启动三维重定位工作线程: ") + exception.what();
      RCLCPP_ERROR(node_.get_logger(), "%s", result->message.c_str());
      if (goal_handle->is_canceling()) {
        result->code = CODE_CANCELLED;
        goal_handle->canceled(result);
      } else if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }
  }

  void runGoal(const std::shared_ptr<GoalHandleRelocalize> goal_handle)
  {
    try {
      executeGoal(goal_handle);
    } catch (const std::exception & exception) {
      auto result = std::make_shared<Relocalize::Result>();
      result->success = false;
      result->code = CODE_INTERNAL_ERROR;
      result->message =
        std::string("三维重定位发生未处理异常: ") + exception.what();
      RCLCPP_ERROR(node_.get_logger(), "%s", result->message.c_str());
      if (!goal_handle->is_active()) {
        // A terminal transition can itself throw during shutdown. Do not
        // attempt a second terminal transition on the same goal.
      } else if (goal_handle->is_canceling()) {
        result->code = CODE_CANCELLED;
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (...) {
      auto result = std::make_shared<Relocalize::Result>();
      result->success = false;
      result->code = CODE_INTERNAL_ERROR;
      result->message = "三维重定位发生未知异常";
      RCLCPP_ERROR(node_.get_logger(), "%s", result->message.c_str());
      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      collecting_clouds_ = false;
      collected_clouds_.clear();
    }
    active_.store(false);
  }

  void pointcloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!collecting_clouds_) {
      return;
    }
    if (normalizeFrame(message->header.frame_id) != odom_frame_) {
      rejected_cloud_frame_ = message->header.frame_id;
      cloud_condition_.notify_all();
      return;
    }

    const int64_t stamp = stampNanoseconds(message->header.stamp);
    const auto duplicate = std::any_of(
      collected_clouds_.begin(), collected_clouds_.end(),
      [stamp](const auto & existing) {
        return stampNanoseconds(existing->header.stamp) == stamp;
      });
    if (duplicate) {
      return;
    }

    if (collected_clouds_.size() < static_cast<std::size_t>(accumulation_frames_)) {
      collected_clouds_.push_back(std::move(message));
      cloud_condition_.notify_all();
    }
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandleRelocalize> & goal_handle,
    const std::string & stage, float progress, const std::string & message)
  {
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<Relocalize::Feedback>();
    feedback->stage = stage;
    feedback->progress = std::clamp(progress, 0.0F, 1.0F);
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
      node_.get_logger(), "[%s %.0f%%] %s", stage.c_str(),
      static_cast<double>(feedback->progress) * 100.0, message.c_str());
  }

  bool stopRequested(ExecutionContext & context)
  {
    if (!context.goal_handle->is_active() && !context.goal_handle->is_canceling()) {
      return true;
    }
    if (shutting_down_.load() || context.goal_handle->is_canceling()) {
      context.result->success = false;
      context.result->code = CODE_CANCELLED;
      context.result->message = "三维重定位已取消";
      RCLCPP_WARN(node_.get_logger(), "%s", context.result->message.c_str());
      context.goal_handle->canceled(context.result);
      return true;
    }
    if (SteadyClock::now() >= context.deadline) {
      context.result->success = false;
      context.result->code = CODE_TIMEOUT;
      std::ostringstream message;
      message << "三维重定位超过 " << context.timeout_sec
              << " 秒截止时间";
      context.result->message = message.str();
      RCLCPP_ERROR(node_.get_logger(), "%s", context.result->message.c_str());
      context.goal_handle->abort(context.result);
      return true;
    }
    return false;
  }

  void abortGoal(
    ExecutionContext & context, uint16_t code, const std::string & message)
  {
    context.result->success = false;
    context.result->code = code;
    context.result->message = message;
    RCLCPP_ERROR(node_.get_logger(), "%s", message.c_str());
    context.goal_handle->abort(context.result);
  }

  bool validateGoal(
    const Relocalize::Goal & goal, ExecutionContext & context,
    Eigen::Matrix4d & rough_map_from_base)
  {
    if (normalizeFrame(goal.rough_pose.header.frame_id) != map_frame_) {
      abortGoal(
        context, CODE_INVALID_GOAL,
        "粗初始位姿必须使用 '" + map_frame_ + "' 坐标系，实际为 '" +
        goal.rough_pose.header.frame_id + "'");
      return false;
    }

    static const std::regex safe_map_name("^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$");
    if (!std::regex_match(goal.map_name, safe_map_name))
    {
      abortGoal(
        context, CODE_MAP_NAME_INVALID,
        "地图名称无效；仅允许 1–64 个字母、数字、下划线和连字符，且必须以字母或数字开头");
      return false;
    }

    std::string pose_error;
    if (!poseToMatrix(goal.rough_pose.pose.pose, rough_map_from_base, pose_error)) {
      abortGoal(context, CODE_INVALID_GOAL, pose_error);
      return false;
    }

    if (!std::isfinite(goal.search_xy) || !std::isfinite(goal.search_yaw) ||
      !std::isfinite(goal.timeout_sec))
    {
      abortGoal(context, CODE_INVALID_GOAL, "搜索范围或超时时间不是有限数值");
      return false;
    }

    context.search_xy =
      goal.search_xy > 0.0F ? static_cast<double>(goal.search_xy) :
      default_search_xy_;
    context.search_yaw =
      goal.search_yaw > 0.0F ? static_cast<double>(goal.search_yaw) :
      default_search_yaw_;
    if (context.search_xy <= 0.0 || context.search_xy > hard_max_search_xy_ + 1.0e-6 ||
      context.search_yaw <= 0.0 ||
      context.search_yaw > hard_max_search_yaw_ + 1.0e-6)
    {
      std::ostringstream message;
      message << "请求的精修范围超过安全上限（平移 "
              << hard_max_search_xy_ << " m，旋转 "
              << hard_max_search_yaw_ * 180.0 / kPi << "°）";
      abortGoal(context, CODE_INVALID_GOAL, message.str());
      return false;
    }
    if (goal.robust_validation) {
      std::lock_guard<std::mutex> lock(active_map_mutex_);
      if (goal.map_load_id.empty() || goal.map_name != active_map_name_ ||
        goal.map_load_id != active_map_load_id_)
      {
        abortGoal(context, CODE_INVALID_GOAL, "三维定位地图加载身份不一致");
        return false;
      }
    }
    return true;
  }

  bool loadMap(
    const std::string & map_name, LoadedMap & loaded_map,
    uint16_t & error_code, std::string & error)
  {
    std::lock_guard<std::mutex> cache_lock(map_cache_mutex_);
    const std::filesystem::path directory(pcd_directory_);
    const auto metadata_path =
      directory / (map_name + ".localization.yaml");
    const auto pcd_path = directory / (map_name + ".pcd");

    if (!std::filesystem::exists(metadata_path)) {
      error_code = CODE_METADATA_MISSING;
      error =
        "地图 '" + map_name +
        "' 缺少新版三维定位元数据 " + metadata_path.string() +
        "；旧地图必须重新建图并保存";
      return false;
    }
    if (!std::filesystem::exists(pcd_path)) {
      error_code = CODE_PCD_MISSING;
      error =
        "地图 '" + map_name + "' 缺少同名定位 PCD: " + pcd_path.string();
      return false;
    }

    std::filesystem::file_time_type pcd_write_time;
    std::filesystem::file_time_type metadata_write_time;
    uintmax_t pcd_file_size = 0U;
    try {
      pcd_write_time = std::filesystem::last_write_time(pcd_path);
      metadata_write_time = std::filesystem::last_write_time(metadata_path);
      pcd_file_size = std::filesystem::file_size(pcd_path);
    } catch (const std::filesystem::filesystem_error & exception) {
      error_code = CODE_METADATA_INVALID;
      error = std::string("无法检查定位地图文件: ") + exception.what();
      return false;
    }
    if (pcd_file_size > maximum_pcd_bytes_) {
      error_code = CODE_PCD_LOAD_FAILED;
      std::ostringstream message;
      message << "定位 PCD 大小为 "
              << static_cast<double>(pcd_file_size) / (1024.0 * 1024.0)
              << " MiB，超过低内存上限 "
              << static_cast<double>(maximum_pcd_bytes_) /
                 (1024.0 * 1024.0)
              << " MiB；请重新降采样保存";
      error = message.str();
      return false;
    }

    if (map_cache_.cloud && map_cache_.map_name == map_name &&
      map_cache_.pcd_write_time == pcd_write_time &&
      map_cache_.metadata_write_time == metadata_write_time &&
      map_cache_.pcd_file_size == pcd_file_size)
    {
      loaded_map = map_cache_;
      RCLCPP_INFO(
        node_.get_logger(), "Using cached localization PCD '%s' (%zu points).",
        map_name.c_str(), loaded_map.cloud->size());
      return true;
    }

    YAML::Node metadata;
    try {
      metadata = YAML::LoadFile(metadata_path.string());
    } catch (const YAML::Exception & exception) {
      error_code = CODE_METADATA_INVALID;
      error =
        std::string("无法解析三维定位元数据: ") + exception.what();
      return false;
    }

    const std::array<const char *, 9> required_fields{
      "schema", "version", "map_name", "pcd_file", "frame_id",
      "point_count", "pcd_size_bytes", "pcd_sha256", "voxel_size"};
    for (const auto * field : required_fields) {
      if (!metadata[field] || !metadata[field].IsScalar()) {
        error_code = CODE_METADATA_INVALID;
        error =
          "三维定位元数据缺少标量字段 '" + std::string(field) + "'";
        return false;
      }
    }

    uint64_t expected_point_count = 0U;
    double voxel_size = 0.0;
    try {
      if (metadata["schema"].as<std::string>() != metadata_schema_) {
        throw std::runtime_error(
                "schema 必须为 '" + metadata_schema_ + "'");
      }
      if (metadata["version"].as<int>() != metadata_version_) {
        throw std::runtime_error(
                "version 必须为 " + std::to_string(metadata_version_));
      }
      if (metadata["map_name"].as<std::string>() != map_name) {
        throw std::runtime_error("map_name 与请求地图不一致");
      }
      if (metadata["pcd_file"].as<std::string>() != map_name + ".pcd") {
        throw std::runtime_error("pcd_file 必须为同名 PCD 文件");
      }
      if (normalizeFrame(metadata["frame_id"].as<std::string>()) != map_frame_) {
        throw std::runtime_error("frame_id 必须为 'map'");
      }
      expected_point_count = metadata["point_count"].as<uint64_t>();
      voxel_size = metadata["voxel_size"].as<double>();
      if (expected_point_count == 0U || !std::isfinite(voxel_size) ||
        voxel_size <= 0.0 || voxel_size > 1.0)
      {
        throw std::runtime_error("point_count 或 voxel_size 超出有效范围");
      }
      if (metadata["pcd_size_bytes"].as<uint64_t>() != pcd_file_size)
      {
        throw std::runtime_error("pcd_size_bytes 与 PCD 实际大小不一致");
      }
    } catch (const std::exception & exception) {
      error_code = CODE_METADATA_INVALID;
      error =
        std::string("三维定位元数据校验失败: ") + exception.what();
      return false;
    }
    if (expected_point_count > maximum_map_points_) {
      error_code = CODE_PCD_LOAD_FAILED;
      std::ostringstream message;
      message << "定位 PCD 元数据声明 " << expected_point_count
              << " 个点，超过低内存上限 " << maximum_map_points_
              << "；请用新版流程提高体素尺寸后重新保存";
      error = message.str();
      return false;
    }

    std::string expected_digest;
    try {
      expected_digest =
        normalizeDigest(metadata["pcd_sha256"].as<std::string>());
    } catch (const YAML::Exception & exception) {
      error_code = CODE_METADATA_INVALID;
      error =
        std::string("PCD SHA-256 字段无效: ") + exception.what();
      return false;
    }
    if (!std::regex_match(expected_digest, std::regex("^[0-9a-f]{64}$"))) {
      error_code = CODE_METADATA_INVALID;
      error = "PCD SHA-256 必须是 64 位十六进制字符串";
      return false;
    }
    std::string actual_digest;
    if (!sha256File(pcd_path, actual_digest, error)) {
      error_code = CODE_PCD_CHECKSUM_MISMATCH;
      return false;
    }
    if (actual_digest != expected_digest) {
      error_code = CODE_PCD_CHECKSUM_MISMATCH;
      error =
        "地图 '" + map_name +
        "' 的定位 PCD SHA-256 与元数据不一致，禁止使用";
      return false;
    }

    PointCloudPtr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(pcd_path.string(), *cloud) < 0) {
      error_code = CODE_PCD_LOAD_FAILED;
      error = "无法加载定位 PCD: " + pcd_path.string();
      return false;
    }
    if (cloud->size() != expected_point_count) {
      error_code = CODE_METADATA_INVALID;
      std::ostringstream message;
      message << "定位 PCD 点数与元数据不一致（文件 " << cloud->size()
              << "，元数据 " << expected_point_count << "）";
      error = message.str();
      return false;
    }
    if (cloud->size() > maximum_map_points_) {
      error_code = CODE_PCD_LOAD_FAILED;
      std::ostringstream message;
      message << "定位 PCD 有 " << cloud->size()
              << " 个点，超过低内存上限 " << maximum_map_points_
              << "；请用新版流程重新降采样保存";
      error = message.str();
      return false;
    }
    const auto invalid_point = std::find_if(
      cloud->begin(), cloud->end(), [](const PointT & point) {
        return !pcl::isFinite(point);
      });
    if (invalid_point != cloud->end()) {
      error_code = CODE_PCD_LOAD_FAILED;
      error = "定位 PCD 含有 NaN/无穷点，禁止用于精修";
      return false;
    }

    cloud->is_dense = true;
    map_cache_.map_name = map_name;
    map_cache_.cloud = cloud;
    map_cache_.voxel_size = voxel_size;
    map_cache_.pcd_write_time = pcd_write_time;
    map_cache_.metadata_write_time = metadata_write_time;
    map_cache_.pcd_file_size = pcd_file_size;
    loaded_map = map_cache_;

    RCLCPP_INFO(
      node_.get_logger(),
      "Loaded validated localization map '%s': %zu points, voxel %.3f m, %.1f MiB.",
      map_name.c_str(), cloud->size(), voxel_size,
      static_cast<double>(pcd_file_size) / (1024.0 * 1024.0));
    return true;
  }

  bool collectClouds(
    ExecutionContext & context,
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & messages)
  {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      collected_clouds_.clear();
      rejected_cloud_frame_.clear();
      collecting_clouds_ = true;
    }

    const auto collection_deadline = std::min(
      context.deadline,
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(cloud_wait_timeout_sec_)));
    auto next_feedback = SteadyClock::now();

    std::unique_lock<std::mutex> lock(cloud_mutex_);
    while (collected_clouds_.size() <
      static_cast<std::size_t>(accumulation_frames_))
    {
      if (shutting_down_.load() || context.goal_handle->is_canceling()) {
        collecting_clouds_ = false;
        lock.unlock();
        (void)stopRequested(context);
        return false;
      }
      if (!rejected_cloud_frame_.empty()) {
        const std::string frame = rejected_cloud_frame_;
        collecting_clouds_ = false;
        lock.unlock();
        abortGoal(
          context, CODE_CLOUD_INVALID,
          "点云 '" + pointcloud_topic_ + "' 必须位于 '" + odom_frame_ +
          "' 坐标系，实际为 '" + frame + "'");
        return false;
      }
      if (SteadyClock::now() >= collection_deadline) {
        break;
      }

      if (SteadyClock::now() >= next_feedback) {
        const auto count = collected_clouds_.size();
        lock.unlock();
        const float fraction = static_cast<float>(count) /
          static_cast<float>(accumulation_frames_);
        publishFeedback(
          context.goal_handle, "collecting_cloud",
          0.18F + 0.17F * fraction,
          "正在采集不同时间戳点云 " + std::to_string(count) + "/" +
          std::to_string(accumulation_frames_));
        lock.lock();
        next_feedback = SteadyClock::now() + std::chrono::milliseconds(250);
      }
      cloud_condition_.wait_for(lock, std::chrono::milliseconds(50));
    }

    collecting_clouds_ = false;
    messages.assign(collected_clouds_.begin(), collected_clouds_.end());
    collected_clouds_.clear();
    lock.unlock();

    if (messages.size() < static_cast<std::size_t>(accumulation_frames_)) {
      if (stopRequested(context)) {
        return false;
      }
      abortGoal(
        context, CODE_CLOUD_TIMEOUT,
        "在限定时间内未收到 " + std::to_string(accumulation_frames_) +
        " 帧不同时间戳的 " + pointcloud_topic_ + " 点云");
      return false;
    }

    publishFeedback(
      context.goal_handle, "collecting_cloud", 0.35F,
      "已采集不同时间戳点云 " + std::to_string(messages.size()) + "/" +
      std::to_string(accumulation_frames_));
    std::sort(
      messages.begin(), messages.end(), [](const auto & left, const auto & right) {
        return stampNanoseconds(left->header.stamp) <
               stampNanoseconds(right->header.stamp);
      });
    return true;
  }

  PointCloudPtr filterSourceFrame(
    const PointCloudPtr & raw_cloud, const Eigen::Matrix4d & odom_from_base)
  {
    PointCloudPtr filtered(new PointCloud);
    if (!raw_cloud) {
      return filtered;
    }

    const Eigen::Matrix4d base_from_odom = odom_from_base.inverse();
    const std::size_t stride = std::max<std::size_t>(
      1U, (raw_cloud->size() + maximum_raw_points_per_frame_ - 1U) /
      maximum_raw_points_per_frame_);
    filtered->reserve(std::min(raw_cloud->size(), maximum_raw_points_per_frame_));

    for (std::size_t index = 0; index < raw_cloud->size(); index += stride) {
      const auto & point = (*raw_cloud)[index];
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector4d odom_point(point.x, point.y, point.z, 1.0);
      const Eigen::Vector4d base_point = base_from_odom * odom_point;
      const double range =
        std::hypot(base_point.x(), base_point.y());
      if (range < source_min_range_ || range > source_max_range_ ||
        base_point.z() < source_min_z_ || base_point.z() > source_max_z_)
      {
        continue;
      }
      if (range < source_self_radius_ &&
        base_point.z() >= source_self_min_z_ &&
        base_point.z() <= source_self_max_z_)
      {
        continue;
      }
      filtered->push_back(point);
    }
    filtered->width = static_cast<uint32_t>(filtered->size());
    filtered->height = 1U;
    filtered->is_dense = true;
    return voxelFilter(filtered, fine_leaf_size_);
  }

  bool prepareFrames(
    ExecutionContext & context,
    const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & messages,
    std::vector<FrameData> & frames, PointCloudPtr & aggregate)
  {
    aggregate.reset(new PointCloud);
    for (const auto & message : messages) {
      if (stopRequested(context)) {
        return false;
      }

      geometry_msgs::msg::TransformStamped odom_from_base_msg;
      bool transform_found = false;
      std::string last_transform_error;
      const auto transform_deadline = std::min(
        context.deadline,
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(tf_lookup_retry_sec_)));
      do {
        try {
          odom_from_base_msg = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, rclcpp::Time(message->header.stamp));
          transform_found = true;
          break;
        } catch (const tf2::TransformException & exception) {
          last_transform_error = exception.what();
        }
        if (stopRequested(context)) {
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } while (SteadyClock::now() < transform_deadline);

      if (!transform_found) {
        abortGoal(
          context, CODE_TF_UNAVAILABLE,
          "无法按点云时间戳查询 " + odom_frame_ + "→" + base_frame_ +
          " TF: " + last_transform_error);
        return false;
      }

      PointCloudPtr raw_cloud(new PointCloud);
      try {
        pcl::fromROSMsg(*message, *raw_cloud);
      } catch (const std::exception & exception) {
        abortGoal(
          context, CODE_CLOUD_INVALID,
          std::string("PointCloud2 转换失败: ") + exception.what());
        return false;
      }

      FrameData frame;
      frame.message = message;
      frame.odom_from_base = transformToMatrix(odom_from_base_msg);
      frame.cloud = filterSourceFrame(raw_cloud, frame.odom_from_base);
      RCLCPP_INFO(
        node_.get_logger(),
        "Prepared source cloud stamp %ld: raw=%zu filtered_fine=%zu.",
        static_cast<long>(stampNanoseconds(message->header.stamp)),
        raw_cloud->size(), frame.cloud->size());
      if (frame.cloud->size() < minimum_points_per_frame_) {
        RCLCPP_WARN(
          node_.get_logger(),
          "Discarding cloud stamp %ld: only %zu usable points (need %zu).",
          static_cast<long>(stampNanoseconds(message->header.stamp)),
          frame.cloud->size(), minimum_points_per_frame_);
        continue;
      }
      *aggregate += *frame.cloud;
      frames.push_back(std::move(frame));
    }

    if (frames.size() < static_cast<std::size_t>(accumulation_frames_)) {
      abortGoal(
        context, CODE_CLOUD_INVALID,
        "点云经过自体、地面和距离过滤后，有效的独立帧不足 " +
        std::to_string(accumulation_frames_) + " 帧");
      return false;
    }
    aggregate = voxelFilter(aggregate, fine_leaf_size_);
    aggregate = limitCloud(aggregate, maximum_fine_source_points_);
    RCLCPP_INFO(
      node_.get_logger(),
      "Prepared %zu/%d source frames: fine aggregate=%zu points "
      "(minimum=%zu).",
      frames.size(), accumulation_frames_, aggregate->size(),
      minimum_source_points_);
    if (aggregate->size() < minimum_source_points_) {
      abortGoal(
        context, CODE_CLOUD_INVALID,
        "聚合点云只有 " + std::to_string(aggregate->size()) +
        " 个有效点，少于最低要求 " + std::to_string(minimum_source_points_));
      return false;
    }
    return true;
  }

  bool prepareMapRoi(
    ExecutionContext & context, const LoadedMap & loaded_map,
    const Eigen::Matrix4d & rough_map_from_base,
    PointCloudPtr & rough_target, PointCloudPtr & fine_target)
  {
    PointCloudPtr roi(new PointCloud);
    const double center_x = rough_map_from_base(0, 3);
    const double center_y = rough_map_from_base(1, 3);
    const double radius = roi_radius_ + context.search_xy;
    const double radius_squared = radius * radius;
    roi->reserve(std::min(loaded_map.cloud->size(), maximum_fine_target_points_ * 2U));

    for (const auto & point : *loaded_map.cloud) {
      const double dx = static_cast<double>(point.x) - center_x;
      const double dy = static_cast<double>(point.y) - center_y;
      if (dx * dx + dy * dy > radius_squared ||
        point.z < roi_min_z_ || point.z > roi_max_z_)
      {
        continue;
      }
      roi->push_back(point);
    }
    roi->width = static_cast<uint32_t>(roi->size());
    roi->height = 1U;
    roi->is_dense = true;

    if (roi->size() < minimum_target_points_) {
      abortGoal(
        context, CODE_VALIDATION_FAILED,
        "粗初始点周围的定位 PCD 只有 " + std::to_string(roi->size()) +
        " 个点（最低要求 " + std::to_string(minimum_target_points_) +
        "，半径 " + std::to_string(radius) +
        " m）；地图覆盖不足或初始点不在有效三维地图内");
      return false;
    }

    const auto sampling = prepareTargetSampling(
      context.goal_handle->get_goal()->robust_validation, roi->size(),
      loaded_map.voxel_size, fine_leaf_size_, rough_leaf_size_,
      minimum_target_points_, maximum_fine_target_points_,
      maximum_rough_target_points_,
      [&](bool rough, double leaf, std::size_t maximum) {
        auto sampled = limitCloud(voxelFilter(roi, leaf), maximum);
        if (rough) {rough_target = sampled;} else {fine_target = sampled;}
        return sampled->size();
      });
    RCLCPP_INFO(
      node_.get_logger(),
      "REGISTRATION_TARGET model=%s center=[%.3f,%.3f] radius=%.3f "
      "z=[%.3f,%.3f] raw=%zu fine=%zu rough=%zu initial_rough=%zu "
      "fine_leaf=%.4f rough_leaf=%.4f minimum=%zu refinements=%zu ready=%s",
      kTargetSamplingModel, center_x, center_y, radius, roi_min_z_, roi_max_z_,
      sampling.raw_count, sampling.fine_count, sampling.rough_count,
      sampling.initial_rough_count, sampling.fine_leaf, sampling.rough_leaf,
      minimum_target_points_, sampling.refinements, sampling.ready ? "true" : "false");
    if (!sampling.ready) {
      abortGoal(
        context, CODE_VALIDATION_FAILED,
        "定位 PCD 局部点数不足：原始 " + std::to_string(sampling.raw_count) +
        "，细配准 " + std::to_string(sampling.fine_count) +
        "，粗配准 " + std::to_string(sampling.rough_count) +
        "（初次 " + std::to_string(sampling.initial_rough_count) +
        "，最低要求 " + std::to_string(minimum_target_points_) +
        "，粗体素 " + std::to_string(sampling.rough_leaf) +
        " m，重采样 " + std::to_string(sampling.refinements) +
        " 次）；无法可靠精修");
      return false;
    }
    return true;
  }

  bool runNdt(
    ExecutionContext & context, const PointCloudPtr & rough_source,
    const PointCloudPtr & rough_target,
    const Eigen::Matrix4d & rough_map_from_base,
    const Eigen::Matrix4d & latest_odom_from_base,
    Eigen::Matrix4f & best_transform,
    std::vector<std::pair<double, Eigen::Matrix4f>> * alternatives = nullptr)
  {
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputSource(rough_source);
    ndt.setInputTarget(rough_target);
    ndt.setResolution(static_cast<float>(ndt_resolution_));
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    const auto goal = context.goal_handle->get_goal();
    const auto budget = icp::refinementBudget(goal->robust_validation, goal->tracking,
      {ndt_maximum_iterations_, gicp_maximum_iterations_, gicp_maximum_optimizer_iterations_}, initial_budget_);
    ndt.setMaximumIterations(budget.ndt);

    const double rough_yaw = matrixYaw(rough_map_from_base);
    const auto seeds = icp::makeSearchSeeds(
      context.search_xy, context.search_yaw, ndt_yaw_hypotheses_);
    std::vector<Eigen::Matrix4f> seed_transforms;
    std::vector<double> seed_scores;
    pcl::KdTreeFLANN<PointT> seed_tree;
    seed_tree.setInputCloud(rough_target);
    const Eigen::Matrix4d base_from_odom = latest_odom_from_base.inverse();
    // Ranking is bounded to at most 512 measured points per candidate. It
    // only chooses starts for NDT and is never a localization acceptance test.
    const std::size_t stride = std::max<std::size_t>(
      1U, (rough_source->size() + 511U) / 512U);
    std::vector<int> nearest(1);
    std::vector<float> distances(1);
    for (const auto & seed : seeds) {
      if (stopRequested(context)) {return false;}
      const Eigen::Matrix4f transform = (planarPoseMatrix(
          rough_map_from_base(0, 3) + seed.x,
          rough_map_from_base(1, 3) + seed.y,
          rough_map_from_base(2, 3), rough_yaw + seed.yaw) * base_from_odom).cast<float>();
      double sum = 0.0;
      std::size_t count = 0;
      for (std::size_t i = 0; i < rough_source->size(); i += stride) {
        const auto & point = (*rough_source)[i];
        const Eigen::Vector4f mapped = transform * Eigen::Vector4f(point.x, point.y, point.z, 1.0F);
        PointT query;
        query.x = mapped.x(); query.y = mapped.y(); query.z = mapped.z();
        sum += seed_tree.nearestKSearch(query, 1, nearest, distances) > 0 ?
          std::min(4.0, static_cast<double>(distances[0])) : 4.0;
        ++count;
      }
      seed_transforms.push_back(transform);
      seed_scores.push_back(count > 0 ? sum / count : std::numeric_limits<double>::infinity());
    }
    const auto candidates = icp::shortlistSeeds(seed_scores, 9U);
    double best_score = std::numeric_limits<double>::infinity();
    bool converged = false;
    PointCloud aligned;

    RCLCPP_INFO(
      node_.get_logger(),
      "Starting NDT: source=%zu target=%zu hypotheses=%zu shortlisted=%zu "
      "search_xy=%.2f search_yaw_deg=%.1f max_iterations=%d.",
      rough_source->size(), rough_target->size(), seeds.size(), candidates.size(),
      context.search_xy, context.search_yaw * 180.0 / kPi,
      budget.ndt);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (stopRequested(context)) {
        return false;
      }
      const auto & candidate_map_from_odom = seed_transforms[candidates[index]];

      aligned.clear();
      const auto candidate_started = SteadyClock::now();
      ndt.align(aligned, candidate_map_from_odom);
      const double candidate_seconds =
        std::chrono::duration<double>(
        SteadyClock::now() - candidate_started).count();
      RCLCPP_INFO(
        node_.get_logger(),
        "NDT candidate %zu/%zu returned after %.3f s.",
        index + 1U, candidates.size(), candidate_seconds);
      if (stopRequested(context)) {
        return false;
      }
      if (!ndt.hasConverged() || !isFiniteMatrix(ndt.getFinalTransformation())) {
        continue;
      }
      const double score = ndt.getFitnessScore();
      if (!std::isfinite(score) || score > ndt_maximum_fitness_) {
        continue;
      }
      const Eigen::Matrix4d coarse_map_from_base =
        ndt.getFinalTransformation().cast<double>() * latest_odom_from_base;
      const double coarse_xy = std::hypot(
        coarse_map_from_base(0, 3) - rough_map_from_base(0, 3),
        coarse_map_from_base(1, 3) - rough_map_from_base(1, 3));
      const double coarse_yaw = std::abs(normalizeAngle(
          matrixYaw(coarse_map_from_base) - matrixYaw(rough_map_from_base)));
      if (coarse_xy > context.search_xy * 1.5 + 0.05 ||
        coarse_yaw > context.search_yaw * 1.5 + degreesToRadians(1.0))
      {
        continue;
      }
      if (alternatives) {alternatives->emplace_back(score, ndt.getFinalTransformation());}
      if (score < best_score) {
        best_score = score;
        best_transform = ndt.getFinalTransformation();
        converged = true;
      }

      publishFeedback(
        context.goal_handle, "coarse_ndt",
        0.48F + 0.12F *
        static_cast<float>(index + 1U) /
        static_cast<float>(candidates.size()),
        "NDT 粗配准候选 " + std::to_string(index + 1U) + "/" +
        std::to_string(candidates.size()));
    }

    if (!converged) {
      abortGoal(
        context, CODE_NDT_FAILED,
        "NDT 粗配准未收敛或粗匹配分数超限");
      return false;
    }
    RCLCPP_INFO(
      node_.get_logger(), "NDT coarse alignment accepted, score %.6f.",
      best_score);
    return true;
  }

  bool runGicp(
    ExecutionContext & context, const PointCloudPtr & fine_source,
    const PointCloudPtr & fine_target, const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & final_transform, bool report_failure = true)
  {
    if (stopRequested(context)) {
      return false;
    }
    class MeasuredGicp : public pcl::GeneralizedIterativeClosestPoint<PointT, PointT>
    {
    public:
      int iterations() const {return this->nr_iterations_;}
    } gicp;
    const auto goal = context.goal_handle->get_goal();
    const bool robust_initial = goal->robust_validation && !goal->tracking;
    const auto budget = icp::refinementBudget(goal->robust_validation, goal->tracking,
      {ndt_maximum_iterations_, gicp_maximum_iterations_, gicp_maximum_optimizer_iterations_}, initial_budget_);
    gicp.setInputSource(fine_source);
    gicp.setInputTarget(fine_target);
    gicp.setMaximumIterations(budget.gicp);
    gicp.setMaximumOptimizerIterations(budget.optimizer);
    gicp.setMaxCorrespondenceDistance(gicp_maximum_correspondence_distance_);
    gicp.setTransformationEpsilon(gicp_transformation_epsilon_);
    gicp.setEuclideanFitnessEpsilon(gicp_fitness_epsilon_);
    gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);
    gicp.setRANSACIterations(0);

    PointCloud aligned;
    RCLCPP_INFO(
      node_.get_logger(),
      "Starting GICP: source=%zu target=%zu max_iterations=%d "
      "optimizer_iterations=%d.",
      fine_source->size(), fine_target->size(), budget.gicp, budget.optimizer);
    const auto gicp_started = SteadyClock::now();
    gicp.align(aligned, initial_guess);
    const double gicp_seconds =
      std::chrono::duration<double>(
      SteadyClock::now() - gicp_started).count();
    RCLCPP_INFO(
      node_.get_logger(), "GICP returned after %.3f s; iterations=%d/%d; iteration_ceiling=%s.",
      gicp_seconds, gicp.iterations(), budget.gicp,
      icp::iterationCeiling(gicp.hasConverged(), gicp.iterations(), budget.gicp) ? "true" : "false");
    if (stopRequested(context)) {
      return false;
    }
    if (!gicp.hasConverged() || !isFiniteMatrix(gicp.getFinalTransformation()) ||
      (robust_initial && icp::iterationCeiling(gicp.hasConverged(), gicp.iterations(), budget.gicp)))
    {
      context.last_candidate_error = "GICP未稳定收敛或达到迭代上限，未放行该候选";
      if (report_failure) {abortGoal(context, CODE_GICP_FAILED, "GICP 精配准未收敛");}
      return false;
    }
    final_transform = gicp.getFinalTransformation();
    RCLCPP_INFO(
      node_.get_logger(), "GICP converged, PCL fitness score %.6f.",
      gicp.getFitnessScore());
    return true;
  }

  icp::ValidationLimits validationLimitsFor(const ExecutionContext & context) const
  {
    const auto goal = context.goal_handle->get_goal();
    return icp::validationLimits(goal->robust_validation, goal->tracking,
      {maximum_rmse_, maximum_vertical_correction_},
      {tracking_maximum_rmse_, tracking_maximum_vertical_correction_});
  }

  bool validateRegistration(
    ExecutionContext & context, const Eigen::Matrix4d & rough_map_from_base,
    const Eigen::Matrix4d & latest_odom_from_base,
    const Eigen::Matrix4f & final_map_from_odom,
    const PointCloudPtr & aggregate, const PointCloudPtr & fine_target,
    const std::vector<FrameData> & frames,
    Eigen::Matrix4d & refined_map_from_base,
    RegistrationMetrics & metrics, bool report_failure = true)
  {
    const auto limits = validationLimitsFor(context);
    auto reject = [&](ExecutionContext & ctx, uint16_t code, const std::string & message) {
        if (report_failure) {abortGoal(ctx, code, message);}
        else {
          ctx.last_candidate_error = message;
          RCLCPP_WARN(node_.get_logger(), "Rejected candidate: %s", message.c_str());
        }
      };
    refined_map_from_base =
      final_map_from_odom.cast<double>() * latest_odom_from_base;
    const double correction_xy = std::hypot(
      refined_map_from_base(0, 3) - rough_map_from_base(0, 3),
      refined_map_from_base(1, 3) - rough_map_from_base(1, 3));
    const double correction_yaw = std::abs(normalizeAngle(
        matrixYaw(refined_map_from_base) - matrixYaw(rough_map_from_base)));
    context.result->correction_xy = static_cast<float>(correction_xy);
    context.result->correction_yaw = static_cast<float>(correction_yaw);

    if (correction_xy > context.search_xy + 1.0e-3 ||
      correction_yaw > context.search_yaw + degreesToRadians(0.1))
    {
      std::ostringstream message;
      message << "精修改变量超出请求上限：平移 " << correction_xy
              << " m / " << context.search_xy << " m，旋转 "
              << correction_yaw * 180.0 / kPi << "° / "
              << context.search_yaw * 180.0 / kPi << "°";
      reject(context, CODE_VALIDATION_FAILED, message.str());
      return false;
    }

    const double vertical_correction = std::abs(
      refined_map_from_base(2, 3) - rough_map_from_base(2, 3));
    const auto [roll, pitch] = matrixRollPitch(refined_map_from_base);
    if (vertical_correction > limits.vertical ||
      std::abs(roll) > maximum_tilt_ || std::abs(pitch) > maximum_tilt_)
    {
      std::ostringstream message;
      message << "三维配准产生不合理的垂直/倾斜修正（z "
              << vertical_correction << " m，上限 " << limits.vertical << " m，roll "
              << roll * 180.0 / kPi << "°，pitch "
              << pitch * 180.0 / kPi << "°）";
      reject(context, CODE_VALIDATION_FAILED, message.str());
      return false;
    }

    metrics = computeMetrics(
      aggregate, fine_target, final_map_from_odom, metric_maximum_distance_);
    if (stopRequested(context)) {
      return false;
    }
    context.result->fitness = std::isfinite(metrics.rmse) ?
      static_cast<float>(metrics.rmse) : -1.0F;
    context.result->overlap = static_cast<float>(metrics.overlap);
    if (!std::isfinite(metrics.rmse) || metrics.rmse > limits.rmse ||
      metrics.overlap < minimum_overlap_ ||
      metrics.matched_points < minimum_matched_points_ ||
      metrics.planar_eigen_ratio < minimum_planar_eigen_ratio_)
    {
      std::ostringstream message;
      message << "精修质量验证失败：RMSE " << metrics.rmse
              << " m（上限 " << limits.rmse << "），重叠率 "
              << metrics.overlap * 100.0 << "%（下限 "
              << minimum_overlap_ * 100.0 << "%），匹配点 "
              << metrics.matched_points << "，平面特征比 "
              << metrics.planar_eigen_ratio << "（下限 "
              << minimum_planar_eigen_ratio_ << "）";
      reject(context, CODE_VALIDATION_FAILED, message.str());
      return false;
    }

    const double relaxed_rmse = limits.rmse * 1.25;
    const double relaxed_overlap = minimum_overlap_ * 0.8;
    int consistent_frames = 0;
    for (const auto & frame : frames) {
      if (stopRequested(context)) {
        return false;
      }
      const auto frame_cloud =
        limitCloud(frame.cloud, maximum_rough_source_points_);
      const auto frame_metrics = computeMetrics(
        frame_cloud, fine_target, final_map_from_odom,
        metric_maximum_distance_);
      if (std::isfinite(frame_metrics.rmse) &&
        frame_metrics.rmse <= relaxed_rmse &&
        frame_metrics.overlap >= relaxed_overlap &&
        frame_metrics.matched_points >=
        std::min<std::size_t>(minimum_matched_points_ / 2U, frame_cloud->size()))
      {
        ++consistent_frames;
      }
    }
    const int required_consistent_frames =
      std::min(minimum_consistent_frames_, static_cast<int>(frames.size()));
    if (consistent_frames < required_consistent_frames) {
      reject(
        context, CODE_VALIDATION_FAILED,
        "多时间戳一致性验证失败：只有 " +
        std::to_string(consistent_frames) + "/" +
        std::to_string(frames.size()) + " 帧支持该精修结果，至少需要 " +
        std::to_string(required_consistent_frames) + " 帧");
      return false;
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "Relocalization validation passed: RMSE %.3f m, overlap %.1f%%, "
      "correction %.3f m / %.2f deg, planar ratio %.4f, frames %d/%zu.",
      metrics.rmse, metrics.overlap * 100.0, correction_xy,
      correction_yaw * 180.0 / kPi, metrics.planar_eigen_ratio,
      consistent_frames, frames.size());
    return true;
  }

  void fillRefinedPose(
    ExecutionContext & context,
    const builtin_interfaces::msg::Time & cloud_stamp,
    const Eigen::Matrix4d & refined_map_from_base,
    const RegistrationMetrics & metrics)
  {
    auto & refined = context.result->refined_pose;
    refined.header.frame_id = map_frame_;
    // Keeping the exact cloud timestamp avoids asking AMCL to extrapolate a
    // future odom->base transform when the coordinator republishes this pose.
    refined.header.stamp = cloud_stamp;
    refined.pose.pose.position.x = refined_map_from_base(0, 3);
    refined.pose.pose.position.y = refined_map_from_base(1, 3);
    const double yaw = matrixYaw(refined_map_from_base);
    const bool rigid = context.goal_handle->get_goal()->robust_validation;
    const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(refined_map_from_base.block<3, 3>(0, 0)).normalized();
    const auto output = icp::poseOutput(rigid, refined_map_from_base(2, 3),
      {rotation.x(), rotation.y(), rotation.z(), rotation.w()}, yaw);
    refined.pose.pose.position.z = output.z;
    refined.pose.pose.orientation.x = output.rotation[0];
    refined.pose.pose.orientation.y = output.rotation[1];
    refined.pose.pose.orientation.z = output.rotation[2];
    refined.pose.pose.orientation.w = output.rotation[3];
    context.result->transform_model = rigid ? icp::kRigidTransformModel : "legacy_planar_v1";

    refined.pose.covariance.fill(0.0);
    const double xy_stddev = std::clamp(
      std::max(metrics.rmse, minimum_refined_xy_stddev_),
      minimum_refined_xy_stddev_, validationLimitsFor(context).rmse);
    refined.pose.covariance[0] = xy_stddev * xy_stddev;
    refined.pose.covariance[7] = xy_stddev * xy_stddev;
    refined.pose.covariance[14] = rigid ? xy_stddev * xy_stddev : 1.0e6;
    refined.pose.covariance[21] = rigid ? refined_yaw_stddev_ * refined_yaw_stddev_ : 1.0e6;
    refined.pose.covariance[28] = refined.pose.covariance[21];
    refined.pose.covariance[35] =
      refined_yaw_stddev_ * refined_yaw_stddev_;
  }

  void executeGoal(const std::shared_ptr<GoalHandleRelocalize> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Relocalize::Result>();
    result->success = false;
    result->code = CODE_INTERNAL_ERROR;
    result->message = "三维重定位尚未完成";
    result->refined_pose = goal->rough_pose;
    result->fitness = -1.0F;
    result->overlap = 0.0F;
    result->correction_xy = 0.0F;
    result->correction_yaw = 0.0F;

    const double requested_timeout =
      goal->timeout_sec > 0.0F ? static_cast<double>(goal->timeout_sec) :
      default_timeout_sec_;
    const double effective_timeout = std::clamp(
      requested_timeout, 0.1, maximum_timeout_sec_);
    ExecutionContext context{
      goal_handle, result,
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(effective_timeout)),
      0.0, 0.0, effective_timeout};

    publishFeedback(
      goal_handle, "validating_goal", 0.02F,
      "正在校验粗初始位姿和精修范围");
    Eigen::Matrix4d rough_map_from_base = Eigen::Matrix4d::Identity();
    if (!validateGoal(*goal, context, rough_map_from_base) ||
      stopRequested(context))
    {
      return;
    }

    RCLCPP_INFO(node_.get_logger(),
      "REGISTRATION_REQUEST map=%s load_id=%s stamp=%ld.%09u rough_xyz=[%.9f,%.9f,%.9f] "
      "rough_yaw=%.9f robust=%s tracking=%s search_xy=%.3f search_yaw=%.6f timeout=%.3f",
      goal->map_name.c_str(), goal->map_load_id.c_str(),
      static_cast<long>(goal->rough_pose.header.stamp.sec), goal->rough_pose.header.stamp.nanosec,
      rough_map_from_base(0,3), rough_map_from_base(1,3), rough_map_from_base(2,3), matrixYaw(rough_map_from_base),
      goal->robust_validation ? "true" : "false", goal->tracking ? "true" : "false",
      context.search_xy, context.search_yaw, context.timeout_sec);

    uint64_t map_generation = 0;
    if (goal->robust_validation) {
      std::lock_guard<std::mutex> lock(active_map_mutex_);
      map_generation = active_map_generation_;
    }

    publishFeedback(
      goal_handle, "loading_map", 0.08F,
      "正在校验并加载地图 '" + goal->map_name + "' 的定位 PCD");
    LoadedMap loaded_map;
    uint16_t map_error_code = CODE_INTERNAL_ERROR;
    std::string map_error;
    if (!loadMap(goal->map_name, loaded_map, map_error_code, map_error)) {
      abortGoal(context, map_error_code, map_error);
      return;
    }
    if (stopRequested(context)) {
      return;
    }

    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> messages;
    if (!collectClouds(context, messages)) {
      return;
    }

    publishFeedback(
      goal_handle, "preprocessing", 0.38F,
      "正在按时间戳读取 TF，并过滤自体、地面和远距离点");
    std::vector<FrameData> frames;
    PointCloudPtr aggregate;
    if (!prepareFrames(context, messages, frames, aggregate)) {
      return;
    }
    const Eigen::Matrix4d latest_odom_from_base =
      frames.back().odom_from_base;
    if (goal->robust_validation) {
      const double age = (node_.now() - rclcpp::Time(frames.back().message->header.stamp)).seconds();
      const double span = (rclcpp::Time(frames.back().message->header.stamp) -
        rclcpp::Time(frames.front().message->header.stamp)).seconds();
      if (age < -0.1 || age > 0.5 || span <= 0.0 || span > 1.0) {
        abortGoal(context, CODE_CLOUD_INVALID, "三维定位点云时间戳或帧间隔异常");
        return;
      }
    }
    if (goal->tracking) {
      if (!goal->robust_validation || context.search_xy > 0.251 ||
        context.search_yaw > degreesToRadians(8.1))
      {
        abortGoal(context, CODE_INVALID_GOAL, "行进匹配只能使用受限局部搜索");
        return;
      }
      try {
        const auto reference = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, rclcpp::Time(goal->rough_pose.header.stamp));
        // Predict to the last measured cloud; never compare different-time robot poses.
        rough_map_from_base = rough_map_from_base *
          transformToMatrix(reference).inverse() * latest_odom_from_base;
      } catch (const tf2::TransformException & error) {
        abortGoal(context, CODE_TF_UNAVAILABLE, error.what());
        return;
      }
    }

    PointCloudPtr rough_target;
    PointCloudPtr fine_target;
    if (!prepareMapRoi(
        context, loaded_map, rough_map_from_base,
        rough_target, fine_target))
    {
      return;
    }
    double rough_source_leaf = rough_leaf_size_;
    auto rough_source = limitCloud(
      voxelFilter(aggregate, rough_source_leaf),
      maximum_rough_source_points_);
    if (
      rough_source->size() < minimum_rough_source_points_ &&
      rough_source_leaf > fine_leaf_size_ + 1.0e-6)
    {
      const std::size_t first_count = rough_source->size();
      rough_source_leaf = std::max(
        fine_leaf_size_, rough_leaf_size_ * 0.75);
      rough_source = limitCloud(
        voxelFilter(aggregate, rough_source_leaf),
        maximum_rough_source_points_);
      RCLCPP_WARN(
        node_.get_logger(),
        "Sparse rough source at %.3f m (%zu points); retried at %.3f m "
        "and obtained %zu points.",
        rough_leaf_size_, first_count, rough_source_leaf,
        rough_source->size());
    }
    RCLCPP_INFO(
      node_.get_logger(),
      "Prepared registration clouds: rough source=%zu at %.3f m "
      "(minimum=%zu), fine source=%zu.",
      rough_source->size(), rough_source_leaf,
      minimum_rough_source_points_, aggregate->size());
    if (rough_source->size() < minimum_rough_source_points_) {
      abortGoal(
        context, CODE_CLOUD_INVALID,
        "粗配准点云降采样后只有 " +
        std::to_string(rough_source->size()) + " 点，少于最低要求 " +
        std::to_string(minimum_rough_source_points_) +
        "；请检查雷达遮挡、点云范围和 TF");
      return;
    }

    publishFeedback(
      goal_handle, "coarse_ndt", 0.48F,
      "开始低分辨率 NDT 粗配准");
    std::vector<std::pair<double, Eigen::Matrix4f>> alternatives;
    Eigen::Matrix4f coarse_map_from_odom = Eigen::Matrix4f::Identity();
    if (!runNdt(
        context, rough_source, rough_target, rough_map_from_base,
        latest_odom_from_base, coarse_map_from_odom,
        goal->robust_validation ? &alternatives : nullptr))
    {
      return;
    }

    publishFeedback(goal_handle, "fine_gicp", 0.64F,
      "正在执行 GICP，并验证三维候选及实际输出变换");
    Eigen::Matrix4f final_map_from_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4d refined_map_from_base = Eigen::Matrix4d::Identity();
    RegistrationMetrics metrics;
    if (goal->robust_validation) {
      std::stable_sort(alternatives.begin(), alternatives.end(),
        [](const auto & a, const auto & b) {return a.first < b.first;});
      struct ValidCandidate {Eigen::Matrix4f transform; Eigen::Matrix4d pose; RegistrationMetrics metrics;};
      std::vector<ValidCandidate> valid;
      std::vector<Eigen::Matrix4d> attempted;
      std::vector<std::string> rejected_reasons;
      // Bounded work: up to three distinct coarse basins, not three copies of one basin.
      for (const auto & candidate : alternatives) {
        if (stopRequested(context)) {return;}
        const Eigen::Matrix4d coarse_pose = candidate.second.cast<double>() * latest_odom_from_base;
        const auto [coarse_roll, coarse_pitch] = matrixRollPitch(coarse_pose);
        bool duplicate = false;
        for (const auto & previous : attempted) {
          const auto [previous_roll, previous_pitch] = matrixRollPitch(previous);
          if (std::hypot(previous(0, 3)-coarse_pose(0, 3), previous(1, 3)-coarse_pose(1, 3)) < 0.15 &&
            std::abs(previous(2, 3)-coarse_pose(2, 3)) < 0.15 &&
            std::hypot(normalizeAngle(previous_roll-coarse_roll),
              normalizeAngle(previous_pitch-coarse_pitch)) < degreesToRadians(3.0) &&
            std::abs(normalizeAngle(matrixYaw(previous)-matrixYaw(coarse_pose))) < degreesToRadians(5.0))
          {duplicate = true; break;}
        }
        if (duplicate) {continue;}
        if (attempted.size() >= 3U) {break;}
        attempted.push_back(coarse_pose);
        context.last_candidate_error.clear();
        Eigen::Matrix4f transform;
        Eigen::Matrix4d pose;
        RegistrationMetrics candidate_metrics;
        if (!runGicp(context, aggregate, fine_target, candidate.second, transform, false)) {
          if (stopRequested(context)) {return;}
          rejected_reasons.push_back(context.last_candidate_error);
          continue;
        }
        const Eigen::Matrix4d measured_pose = transform.cast<double>() * latest_odom_from_base;
        const auto [measured_roll, measured_pitch] = matrixRollPitch(measured_pose);
        RCLCPP_INFO(node_.get_logger(),
          "REGISTRATION_CANDIDATE index=%zu coarse_xyz=[%.6f,%.6f,%.6f] coarse_yaw=%.9f "
          "fine_xyz=[%.6f,%.6f,%.6f] fine_rpy=[%.9f,%.9f,%.9f]",
          attempted.size(), coarse_pose(0,3), coarse_pose(1,3), coarse_pose(2,3), matrixYaw(coarse_pose),
          measured_pose(0,3), measured_pose(1,3), measured_pose(2,3),
          measured_roll, measured_pitch, matrixYaw(measured_pose));
        if (validateRegistration(context, rough_map_from_base, latest_odom_from_base,
            transform, aggregate, fine_target, frames, pose, candidate_metrics, false))
        {
          // The authority publishes this same verified SE(3) map->odom.
          // Nav2 reads XY/yaw from it; cloud quality must not be evaluated
          // against a different zero-height/yaw-only transform.
          valid.push_back({transform, pose, candidate_metrics});
        }
        if (!context.last_candidate_error.empty()) {
          rejected_reasons.push_back(context.last_candidate_error);
        }
      }
      result->candidates_checked = static_cast<uint32_t>(attempted.size());
      if (stopRequested(context)) {return;}
      if (valid.empty()) {
        std::string reason = "全部三维候选均未通过校验";
        for (std::size_t i = 0; i < rejected_reasons.size(); ++i) {
          reason += "；候选" + std::to_string(i+1) + ": " + rejected_reasons[i];
        }
        abortGoal(context, CODE_VALIDATION_FAILED, reason);
        return;
      }
      std::stable_sort(valid.begin(), valid.end(), [](const auto & a, const auto & b) {
          return icp::candidateCost(a.metrics.rmse, a.metrics.overlap) <
                 icp::candidateCost(b.metrics.rmse, b.metrics.overlap);
        });
      const auto & best = valid.front();
      const auto [best_roll, best_pitch] = matrixRollPitch(best.pose);
      for (std::size_t i = 1; i < valid.size(); ++i) {
        const auto & other = valid[i];
        const auto [other_roll, other_pitch] = matrixRollPitch(other.pose);
        if (icp::ambiguousCandidate(
            icp::candidateCost(best.metrics.rmse, best.metrics.overlap),
            icp::candidateCost(other.metrics.rmse, other.metrics.overlap),
            std::hypot(best.pose(0, 3)-other.pose(0, 3), best.pose(1, 3)-other.pose(1, 3)),
            std::abs(normalizeAngle(matrixYaw(best.pose)-matrixYaw(other.pose)))) ||
          icp::ambiguousRigidCandidate(
            icp::candidateCost(best.metrics.rmse, best.metrics.overlap),
            icp::candidateCost(other.metrics.rmse, other.metrics.overlap),
            std::abs(best.pose(2, 3)-other.pose(2, 3)),
            std::hypot(normalizeAngle(best_roll-other_roll),
              normalizeAngle(best_pitch-other_pitch))))
        {
          abortGoal(context, CODE_VALIDATION_FAILED, "多个空间分离候选质量接近，无法排除重复结构误匹配；请重新选初始点");
          return;
        }
      }
      final_map_from_odom = best.transform;
      refined_map_from_base = best.pose;
      metrics = best.metrics;
      result->fitness = static_cast<float>(metrics.rmse);
      result->overlap = static_cast<float>(metrics.overlap);
      result->candidate_verified = true;
      result->correction_xy = static_cast<float>(std::hypot(
        best.pose(0, 3)-rough_map_from_base(0, 3), best.pose(1, 3)-rough_map_from_base(1, 3)));
      result->correction_yaw = static_cast<float>(std::abs(normalizeAngle(
        matrixYaw(best.pose)-matrixYaw(rough_map_from_base))));
      std::lock_guard<std::mutex> lock(active_map_mutex_);
      if (goal->map_name != active_map_name_ || goal->map_load_id != active_map_load_id_ ||
        map_generation != active_map_generation_) {
        abortGoal(context, CODE_VALIDATION_FAILED, "定位期间地图已切换，结果作废");
        return;
      }
    } else {
      if (!runGicp(context, aggregate, fine_target, coarse_map_from_odom, final_map_from_odom) ||
        !validateRegistration(context, rough_map_from_base, latest_odom_from_base,
          final_map_from_odom, aggregate, fine_target, frames, refined_map_from_base, metrics))
      {return;}
    }
    if (stopRequested(context)) {return;}

    publishFeedback(goal_handle, "validating", 0.90F, "配准质量与最终输出变换验证通过");
    fillRefinedPose(
      context, frames.back().message->header.stamp,
      refined_map_from_base, metrics);
    result->success = true;
    result->code = CODE_OK;
    std::ostringstream success_message;
    success_message << "NDT/GICP 精修成功：RMSE "
                    << std::fixed << std::setprecision(3) << metrics.rmse
                    << " m，重叠率 " << std::setprecision(1)
                    << metrics.overlap * 100.0 << "%，修正 "
                    << std::setprecision(3) << result->correction_xy
                    << " m / " << std::setprecision(1)
                    << result->correction_yaw * 180.0 / kPi << "°";
    result->message = success_message.str();
    publishFeedback(goal_handle, "succeeded", 1.0F, result->message);
    goal_handle->succeed(result);
  }

  IcpNode & node_;

  std::string pointcloud_topic_;
  std::string pcd_directory_;
  std::string global_map_topic_;
  double global_map_publish_period_sec_{3.0};
  std::size_t global_map_maximum_points_{120000U};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    global_map_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    global_map_status_publisher_;
  rclcpp::Subscription<lzp_interface::msg::CurrentMap>::SharedPtr
    current_map_subscription_;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  std::string active_map_name_;
  std::string active_map_load_id_;
  uint64_t active_map_generation_{0};
  std::mutex active_map_mutex_;
  std::string published_map_name_;
  std::mutex published_map_mutex_;
  std::mutex map_cache_mutex_;
  std::string metadata_schema_;
  int metadata_version_{1};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  double default_search_xy_{3.0};
  double default_search_yaw_{degreesToRadians(90.0)};
  double hard_max_search_xy_{3.0};
  double hard_max_search_yaw_{degreesToRadians(90.0)};
  double default_timeout_sec_{30.0};
  double maximum_timeout_sec_{30.0};
  int accumulation_frames_{5};
  int minimum_consistent_frames_{5};
  double cloud_wait_timeout_sec_{3.0};
  double tf_lookup_retry_sec_{0.25};

  double source_min_range_{0.55};
  double source_max_range_{12.0};
  double source_min_z_{-0.15};
  double source_max_z_{2.5};
  double source_self_radius_{0.55};
  double source_self_min_z_{-0.5};
  double source_self_max_z_{0.7};
  double rough_leaf_size_{0.45};
  double fine_leaf_size_{0.15};
  std::size_t minimum_points_per_frame_{300U};
  std::size_t minimum_source_points_{900U};
  std::size_t minimum_rough_source_points_{300U};
  std::size_t minimum_target_points_{1500U};
  std::size_t maximum_raw_points_per_frame_{120000U};
  std::size_t maximum_rough_source_points_{6000U};
  std::size_t maximum_fine_source_points_{12000U};
  std::size_t maximum_rough_target_points_{30000U};
  std::size_t maximum_fine_target_points_{60000U};
  std::size_t maximum_map_points_{1000000U};
  uintmax_t maximum_pcd_bytes_{64U * 1024U * 1024U};

  double roi_radius_{12.0};
  double roi_min_z_{-1.5};
  double roi_max_z_{3.0};
  double ndt_resolution_{1.0};
  double ndt_step_size_{0.2};
  double ndt_transformation_epsilon_{0.01};
  int ndt_maximum_iterations_{12};
  int ndt_yaw_hypotheses_{5};
  double ndt_maximum_fitness_{2.0};
  int gicp_maximum_iterations_{15};
  int gicp_maximum_optimizer_iterations_{8};
  icp::RefinementBudget initial_budget_{20, 40, 12};
  double gicp_maximum_correspondence_distance_{0.6};
  double gicp_transformation_epsilon_{0.001};
  double gicp_fitness_epsilon_{0.0001};
  int gicp_correspondence_randomness_{20};

  double metric_maximum_distance_{0.4};
  double maximum_rmse_{0.20};
  double minimum_overlap_{0.55};
  double minimum_planar_eigen_ratio_{0.01};
  std::size_t minimum_matched_points_{500U};
  double maximum_vertical_correction_{0.35};
  double tracking_maximum_rmse_{0.22};
  double tracking_maximum_vertical_correction_{0.45};
  double maximum_tilt_{degreesToRadians(8.0)};
  double refined_yaw_stddev_{degreesToRadians(5.0)};
  double minimum_refined_xy_stddev_{0.05};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_subscription_;
  rclcpp_action::Server<Relocalize>::SharedPtr action_server_;

  std::atomic_bool active_{false};
  std::atomic_bool shutting_down_{false};
  std::mutex worker_mutex_;
  std::thread worker_thread_;

  std::mutex cloud_mutex_;
  std::condition_variable cloud_condition_;
  bool collecting_clouds_{false};
  std::string rejected_cloud_frame_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> collected_clouds_;

  LoadedMap map_cache_;
};

IcpNode::IcpNode(const rclcpp::NodeOptions & options)
: Node("icp_registration", options),
  impl_(std::make_unique<Impl>(*this))
{
}

IcpNode::~IcpNode() = default;

}  // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
