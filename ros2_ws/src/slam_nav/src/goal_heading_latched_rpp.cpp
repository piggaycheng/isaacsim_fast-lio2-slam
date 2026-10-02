// Regulated Pure Pursuit with a latched final in-place rotation.
//
// Humble's RPP re-evaluates "within xy_goal_tolerance" every cycle, and it
// stops translating exactly at that boundary. Small base_link drift while
// rotating in place then flips it back to path tracking, so the robot keeps
// switching rotation targets until the progress checker aborts the goal.
// Once the goal position is reached, this controller keeps rotating toward the
// goal heading until the goal checker succeeds, a new goal arrives, or the
// robot drifts beyond latch_release_distance.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace slam_nav
{

class GoalHeadingLatchedRPP
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".latch_release_distance", rclcpp::ParameterValue(0.5));
    node->get_parameter(name + ".latch_release_distance", latch_release_distance_);
    latched_ = false;
  }

  void activate() override
  {
    RegulatedPurePursuitController::activate();
    latched_ = false;
  }

  void deactivate() override
  {
    RegulatedPurePursuitController::deactivate();
    latched_ = false;
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (latched_ && (path.poses.empty() || goalChanged(path.poses.back()))) {
      RCLCPP_INFO(logger_, "New goal received; releasing final heading latch");
      latched_ = false;
    }
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    RegulatedPurePursuitController::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    if (!use_rotate_to_heading_ || goal_.header.frame_id.empty()) {
      return RegulatedPurePursuitController::computeVelocityCommands(
        pose, velocity, goal_checker);
    }

    geometry_msgs::msg::PoseStamped goal;
    if (!transformPose(pose.header.frame_id, goal_, goal)) {
      throw nav2_core::PlannerException("Unable to transform goal pose into robot frame");
    }
    const double distance = std::hypot(
      goal.pose.position.x - pose.pose.position.x,
      goal.pose.position.y - pose.pose.position.y);

    double xy_tolerance = goal_dist_tol_;
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (goal_checker && goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      xy_tolerance = pose_tolerance.position.x;
    }

    if (!latched_ && distance < xy_tolerance) {
      RCLCPP_INFO(
        logger_, "Goal position reached (%.3f m); latching final heading rotation", distance);
      latched_ = true;
    } else if (latched_ && distance > std::max(latch_release_distance_, xy_tolerance)) {
      RCLCPP_WARN(
        logger_, "Robot drifted %.3f m from the goal; releasing final heading latch", distance);
      latched_ = false;
    }

    if (!latched_) {
      return RegulatedPurePursuitController::computeVelocityCommands(
        pose, velocity, goal_checker);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const double heading_error = std::remainder(
      tf2::getYaw(goal.pose.orientation) - tf2::getYaw(pose.pose.orientation), 2.0 * M_PI);
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    rotateToHeading(linear_vel, angular_vel, heading_error, velocity);

    if (use_collision_detection_ &&
      isCollisionImminent(pose, linear_vel, angular_vel, distance))
    {
      throw nav2_core::PlannerException(
        "RegulatedPurePursuitController detected collision ahead!");
    }

    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = linear_vel;
    command.twist.angular.z = angular_vel;
    return command;
  }

private:
  bool goalChanged(const geometry_msgs::msg::PoseStamped & goal) const
  {
    if (goal.header.frame_id != goal_.header.frame_id) {
      return true;
    }
    const double moved = std::hypot(
      goal.pose.position.x - goal_.pose.position.x,
      goal.pose.position.y - goal_.pose.position.y);
    const double turned = std::abs(std::remainder(
      tf2::getYaw(goal.pose.orientation) - tf2::getYaw(goal_.pose.orientation), 2.0 * M_PI));
    // Replanning can quantize the path end to the costmap grid; ignore that jitter.
    return moved > 0.1 || turned > 0.1;
  }

  geometry_msgs::msg::PoseStamped goal_;
  double latch_release_distance_{0.5};
  bool latched_{false};
};

}  // namespace slam_nav

PLUGINLIB_EXPORT_CLASS(slam_nav::GoalHeadingLatchedRPP, nav2_core::Controller)
