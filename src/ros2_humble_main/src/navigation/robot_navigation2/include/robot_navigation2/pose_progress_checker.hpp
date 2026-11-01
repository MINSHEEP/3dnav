#ifndef ROBOT_NAVIGATION2__POSE_PROGRESS_CHECKER_HPP_
#define ROBOT_NAVIGATION2__POSE_PROGRESS_CHECKER_HPP_

#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_navigation2
{

/**
 * @brief Humble-compatible progress checker that accepts translation or rotation.
 *
 * Nav2 Humble's SimpleProgressChecker only measures x/y displacement. A robot
 * legitimately turning in place can therefore be reported as stuck. This
 * checker resets its progress baseline after either sufficient translation or
 * sufficient heading change.
 */
class PoseProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

protected:
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose) const;
  void resetBaselinePose(const geometry_msgs::msg::Pose2D & pose);

  static geometry_msgs::msg::Pose2D toPose2D(
    const geometry_msgs::msg::PoseStamped & pose);
  static double poseDistance(
    const geometry_msgs::msg::Pose2D & first,
    const geometry_msgs::msg::Pose2D & second);
  static double angularDistance(double first, double second);

  rclcpp::Clock::SharedPtr clock_;
  double required_movement_radius_{0.10};
  double required_movement_angle_{0.20};
  rclcpp::Duration movement_time_allowance_{0, 0};
  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};
  std::string plugin_name_;
};

}  // namespace robot_navigation2

#endif  // ROBOT_NAVIGATION2__POSE_PROGRESS_CHECKER_HPP_
