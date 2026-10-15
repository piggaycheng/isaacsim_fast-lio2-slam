// Regulated Pure Pursuit with hysteretic path alignment and latched final rotation.
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
#include <stdexcept>
#include <string>
#include <vector>

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
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".path_alignment_tolerance", rclcpp::ParameterValue(5.0 * M_PI / 180.0));
    node->get_parameter(name + ".path_alignment_tolerance", path_alignment_tolerance_);
    if (!validHeadingAngles(path_alignment_tolerance_, rotate_to_heading_min_angle_)) {
      throw std::invalid_argument(
              "Heading thresholds must satisfy 0 < path_alignment_tolerance <= "
              "rotate_to_heading_min_angle <= pi");
    }
    latched_ = false;
    aligning_path_ = true;
  }

  void activate() override
  {
    RegulatedPurePursuitController::activate();
    latched_ = false;
    aligning_path_ = true;
    heading_parameters_handler_ = node_.lock()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        std::lock_guard<std::mutex> lock(mutex_);
        double alignment = path_alignment_tolerance_;
        double moving = rotate_to_heading_min_angle_;
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != plugin_name_ + ".path_alignment_tolerance" &&
            parameter.get_name() != plugin_name_ + ".rotate_to_heading_min_angle")
          {
            continue;
          }
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = "Heading thresholds must be doubles in radians";
            return result;
          }
          if (parameter.get_name() == plugin_name_ + ".path_alignment_tolerance") {
            alignment = parameter.as_double();
          } else {
            moving = parameter.as_double();
          }
        }
        result.successful = validHeadingAngles(alignment, moving);
        if (result.successful) {
          path_alignment_tolerance_ = alignment;
        } else {
          result.reason =
            "Heading thresholds must satisfy 0 < path_alignment_tolerance <= "
            "rotate_to_heading_min_angle <= pi";
        }
        return result;
      });
  }

  void deactivate() override
  {
    heading_parameters_handler_.reset();
    RegulatedPurePursuitController::deactivate();
    latched_ = false;
    aligning_path_ = true;
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.poses.empty() || goalChanged(path.poses.back())) {
      if (latched_) {
        RCLCPP_INFO(logger_, "New goal received; releasing final heading latch");
      }
      latched_ = false;
      aligning_path_ = true;
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
      aligning_path_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
        *costmap_ros_->getCostmap()->getMutex());
      if (latched_) {
        const double heading_error = std::remainder(
          tf2::getYaw(goal.pose.orientation) - tf2::getYaw(pose.pose.orientation),
          2.0 * M_PI);
        return rotationCommand(pose, velocity, heading_error, distance);
      }

      // Humble's shouldRotateToPath() is not virtual; reuse its lookahead geometry
      // to enforce hysteresis without changing the upstream tracking controller.
      auto transformed_plan = transformGlobalPlan(pose);
      auto carrot = getLookAheadPoint(getLookAheadDistance(velocity), transformed_plan);
      const double heading_error = std::atan2(carrot.pose.position.y, carrot.pose.position.x);
      if (aligning_path_) {
        if (std::abs(heading_error) <= path_alignment_tolerance_) {
          aligning_path_ = false;
        }
      } else if (std::abs(heading_error) > rotate_to_heading_min_angle_) {
        aligning_path_ = true;
      }
      if (aligning_path_) {
        carrot_pub_->publish(createCarrotMsg(carrot));
        return rotationCommand(
          pose, velocity, heading_error,
          std::hypot(carrot.pose.position.x, carrot.pose.position.y));
      }
    }

    return RegulatedPurePursuitController::computeVelocityCommands(
      pose, velocity, goal_checker);
  }

private:
  static bool validHeadingAngles(double alignment, double moving)
  {
    return std::isfinite(alignment) && std::isfinite(moving) &&
           alignment > 0.0 && alignment <= moving && moving <= M_PI;
  }

  geometry_msgs::msg::TwistStamped rotationCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, double heading_error, double distance)
  {
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
  double path_alignment_tolerance_{5.0 * M_PI / 180.0};
  bool latched_{false};
  bool aligning_path_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr heading_parameters_handler_;
};

}  // namespace slam_nav

PLUGINLIB_EXPORT_CLASS(slam_nav::GoalHeadingLatchedRPP, nav2_core::Controller)
