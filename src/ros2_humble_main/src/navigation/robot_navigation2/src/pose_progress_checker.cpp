#include "robot_navigation2/pose_progress_checker.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_navigation2
{

void PoseProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PoseProgressChecker parent node is unavailable");
  }

  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".required_movement_radius", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".required_movement_angle", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".movement_time_allowance", rclcpp::ParameterValue(15.0));

  node->get_parameter(plugin_name_ + ".required_movement_radius", required_movement_radius_);
  node->get_parameter(plugin_name_ + ".required_movement_angle", required_movement_angle_);
  double time_allowance_seconds = 15.0;
  node->get_parameter(plugin_name_ + ".movement_time_allowance", time_allowance_seconds);
  movement_time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_seconds);

  if (required_movement_radius_ < 0.0 || required_movement_angle_ < 0.0 ||
    time_allowance_seconds <= 0.0)
  {
    throw std::runtime_error(
            "PoseProgressChecker radius/angle must be non-negative and allowance positive");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Pose progress checker: %.3f m or %.3f rad within %.1f s",
    required_movement_radius_, required_movement_angle_, time_allowance_seconds);
}

bool PoseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto pose = toPose2D(current_pose);
  if (!baseline_pose_set_ || isRobotMovedEnough(pose)) {
    resetBaselinePose(pose);
    return true;
  }

  return (clock_->now() - baseline_time_) <= movement_time_allowance_;
}

void PoseProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

bool PoseProgressChecker::isRobotMovedEnough(
  const geometry_msgs::msg::Pose2D & pose) const
{
  return poseDistance(pose, baseline_pose_) > required_movement_radius_ ||
         angularDistance(pose.theta, baseline_pose_.theta) > required_movement_angle_;
}

void PoseProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

geometry_msgs::msg::Pose2D PoseProgressChecker::toPose2D(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::Pose2D result;
  result.x = pose.pose.position.x;
  result.y = pose.pose.position.y;

  const auto & orientation = pose.pose.orientation;
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  result.theta = std::atan2(sin_yaw, cos_yaw);
  return result;
}

double PoseProgressChecker::poseDistance(
  const geometry_msgs::msg::Pose2D & first,
  const geometry_msgs::msg::Pose2D & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

double PoseProgressChecker::angularDistance(double first, double second)
{
  const double delta = first - second;
  return std::abs(std::atan2(std::sin(delta), std::cos(delta)));
}

}  // namespace robot_navigation2

PLUGINLIB_EXPORT_CLASS(
  robot_navigation2::PoseProgressChecker,
  nav2_core::ProgressChecker)
