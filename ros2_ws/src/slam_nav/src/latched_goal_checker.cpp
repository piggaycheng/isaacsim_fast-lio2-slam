// Goal checker whose xy latch survives Nav2 replanning.
//
// Humble's controller_server calls GoalChecker::reset() from setPlannerPath(),
// which runs on every replanned path (1 Hz here). SimpleGoalChecker's
// "stateful" xy latch is therefore cleared every second, and the goal only
// succeeds when position and heading happen to be within tolerance at the same
// time. This checker keeps the xy latch while the goal pose is unchanged and
// the robot stays within latch_release_distance of it.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace slam_nav
{

class LatchedGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".latch_release_distance", rclcpp::ParameterValue(0.5));
    node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
    node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter(plugin_name + ".latch_release_distance", latch_release_distance_);
    logger_ = node->get_logger();
  }

  // Replanning calls reset() for the same goal; goal changes are detected below.
  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    if (!has_goal_ || goalChanged(goal_pose)) {
      goal_ = goal_pose;
      has_goal_ = true;
      xy_latched_ = false;
    }

    const double distance = std::hypot(
      goal_pose.position.x - query_pose.position.x,
      goal_pose.position.y - query_pose.position.y);
    if (xy_latched_ && distance > std::max(latch_release_distance_, xy_goal_tolerance_)) {
      RCLCPP_WARN(logger_, "Robot drifted %.3f m from the goal; releasing xy latch", distance);
      xy_latched_ = false;
    }
    if (!xy_latched_) {
      if (distance > xy_goal_tolerance_) {
        return false;
      }
      xy_latched_ = true;
    }

    const double heading_error = std::remainder(
      tf2::getYaw(goal_pose.orientation) - tf2::getYaw(query_pose.orientation), 2.0 * M_PI);
    return std::abs(heading_error) <= yaw_goal_tolerance_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    const double invalid = std::numeric_limits<double>::lowest();
    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid;
    tf2::Quaternion yaw_tolerance;
    yaw_tolerance.setRPY(0.0, 0.0, yaw_goal_tolerance_);
    pose_tolerance.orientation.x = yaw_tolerance.x();
    pose_tolerance.orientation.y = yaw_tolerance.y();
    pose_tolerance.orientation.z = yaw_tolerance.z();
    pose_tolerance.orientation.w = yaw_tolerance.w();
    vel_tolerance.linear.x = invalid;
    vel_tolerance.linear.y = invalid;
    vel_tolerance.linear.z = invalid;
    vel_tolerance.angular.x = invalid;
    vel_tolerance.angular.y = invalid;
    vel_tolerance.angular.z = invalid;
    return true;
  }

private:
  bool goalChanged(const geometry_msgs::msg::Pose & goal) const
  {
    // The goal is compared in the costmap frame, so map -> odom corrections
    // move it slightly between checks; only a real new goal exceeds this.
    const double moved = std::hypot(
      goal.position.x - goal_.position.x, goal.position.y - goal_.position.y);
    const double turned = std::abs(std::remainder(
      tf2::getYaw(goal.orientation) - tf2::getYaw(goal_.orientation), 2.0 * M_PI));
    return moved > 0.1 || turned > 0.1;
  }

  rclcpp::Logger logger_{rclcpp::get_logger("LatchedGoalChecker")};
  geometry_msgs::msg::Pose goal_;
  bool has_goal_{false};
  bool xy_latched_{false};
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.25};
  double latch_release_distance_{0.5};
};

}  // namespace slam_nav

PLUGINLIB_EXPORT_CLASS(slam_nav::LatchedGoalChecker, nav2_core::GoalChecker)
