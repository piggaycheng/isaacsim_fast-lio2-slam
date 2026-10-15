#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "../src/goal_heading_latched_rpp.cpp"
#include "../src/latched_goal_checker.cpp"

namespace
{

double radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

class HeadingAlignmentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "heading_controller",
      rclcpp::NodeOptions().parameter_overrides({
        {"FollowPath.desired_linear_vel", 0.75},
        {"FollowPath.lookahead_dist", 0.8},
        {"FollowPath.rotate_to_heading_min_angle", radians(45)},
        {"FollowPath.path_alignment_tolerance", radians(5)},
        {"goal_checker.xy_goal_tolerance", 0.15},
      }));
    node_->declare_parameter("controller_frequency", 10.0);
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("heading_costmap");
    costmap_->set_parameters({
      {"plugins", std::vector<std::string>{}},
      {"global_frame", "map"}, {"robot_base_frame", "base_link"},
      {"track_unknown_space", false}, {"rolling_window", false},
      {"width", 10}, {"height", 10}, {"resolution", 0.05},
      {"origin_x", -5.0}, {"origin_y", -5.0},
      {"footprint", "[[0.141, 0.25], [0.141, -0.25], [-0.588, -0.25], [-0.588, 0.25]]"},
    });
    costmap_->configure();
    ASSERT_NE(costmap_->getCostmap(), nullptr);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    setRobotPose(0.0, 0.0, 0.0);
    controller_.configure(node_, "FollowPath", tf_, costmap_);
    goal_checker_.initialize(node_, "goal_checker", costmap_);
    controller_.activate();
  }

  void TearDown() override
  {
    controller_.deactivate();
    controller_.cleanup();
    costmap_->cleanup();
  }

  void setPath(double degrees, double goal_x = 2.0, double goal_yaw = 0.0)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 10; ++i) {
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      point.pose.position.x = i * 0.1 * std::cos(radians(degrees));
      point.pose.position.y = i * 0.1 * std::sin(radians(degrees));
      point.pose.orientation.w = 1.0;
      path.poses.push_back(point);
    }
    auto goal = path.poses.back();
    goal.pose.position.x = goal_x;
    goal.pose.position.y = 0.0;
    goal.pose.orientation.z = std::sin(goal_yaw / 2.0);
    goal.pose.orientation.w = std::cos(goal_yaw / 2.0);
    path.poses.push_back(goal);
    controller_.setPlan(path);
  }

  void setRobotPose(double x, double y, double yaw)
  {
    robot_pose_.header.frame_id = "map";
    robot_pose_.pose.position.x = x;
    robot_pose_.pose.position.y = y;
    robot_pose_.pose.orientation.z = std::sin(yaw / 2.0);
    robot_pose_.pose.orientation.w = std::cos(yaw / 2.0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation = robot_pose_.pose.orientation;
    ASSERT_TRUE(tf_->setTransform(transform, "heading_test", true));
  }

  geometry_msgs::msg::Twist command(
    double degrees, double goal_x = 2.0, double goal_yaw = 0.0)
  {
    setPath(degrees, goal_x, goal_yaw);
    return controller_.computeVelocityCommands(
      robot_pose_, geometry_msgs::msg::Twist(), &goal_checker_).twist;
  }

  void expectRotation(const geometry_msgs::msg::Twist & velocity, double sign)
  {
    EXPECT_DOUBLE_EQ(velocity.linear.x, 0.0);
    EXPECT_GT(velocity.angular.z * sign, 0.0);
  }

  void blockCurrentFootprint()
  {
    for (double wx = -0.7; wx <= 0.2; wx += 0.025) {
      for (double wy = -0.35; wy <= 0.35; wy += 0.025) {
        unsigned int x, y;
        ASSERT_TRUE(costmap_->getCostmap()->worldToMap(wx, wy, x, y));
        costmap_->getCostmap()->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  slam_nav::GoalHeadingLatchedRPP controller_;
  slam_nav::LatchedGoalChecker goal_checker_;
};

TEST_F(HeadingAlignmentTest, AlignsBeforeStartingInBothDirections)
{
  expectRotation(command(5.001), 1.0);
  expectRotation(command(-5.001), -1.0);
  EXPECT_GT(command(-4.999).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, TracksUntilFortyFiveThenRealignsToFive)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  EXPECT_GT(command(30.0).linear.x, 0.0);
  EXPECT_GT(command(44.999).linear.x, 0.0);
  expectRotation(command(45.001), 1.0);
  expectRotation(command(30.0), 1.0);
  expectRotation(command(5.001), 1.0);
  EXPECT_GT(command(4.999).linear.x, 0.0);
  EXPECT_GT(command(-44.999).linear.x, 0.0);
  expectRotation(command(-45.001), -1.0);
  expectRotation(command(-30.0), -1.0);
  EXPECT_GT(command(-4.999).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, ReplanningAndZeroMeasuredSpeedDoNotResetTracking)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_GT(command(20.0).linear.x, 0.0);
  }
  EXPECT_GT(command(20.0, 2.04).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, UsesRobotRelativeRatherThanGlobalPathHeading)
{
  setRobotPose(0.0, 0.0, radians(90));
  EXPECT_GT(command(90.0).linear.x, 0.0);
  EXPECT_GT(command(120.0).linear.x, 0.0);
  expectRotation(command(136.0), 1.0);
  expectRotation(command(120.0), 1.0);
  EXPECT_GT(command(94.9).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, NewPositionGoalRequiresInitialAlignment)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  expectRotation(command(20.0, 3.0), 1.0);
}

TEST_F(HeadingAlignmentTest, NewGoalOrientationRequiresInitialAlignment)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  expectRotation(command(20.0, 2.0, 0.2), 1.0);
}

TEST_F(HeadingAlignmentTest, EmptyPlanResetsInitialAlignment)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  nav_msgs::msg::Path empty;
  controller_.setPlan(empty);
  expectRotation(command(20.0), 1.0);
}

TEST_F(HeadingAlignmentTest, ReactivationRequiresInitialAlignment)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  controller_.deactivate();
  controller_.activate();
  expectRotation(command(20.0), 1.0);
}

TEST_F(HeadingAlignmentTest, DisabledRotationKeepsUpstreamTracking)
{
  ASSERT_TRUE(node_->set_parameter({"FollowPath.use_rotate_to_heading", false}).successful);
  EXPECT_GT(command(30.0).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, FinalHeadingLatchSurvivesReplanningAndSmallDrift)
{
  expectRotation(command(-30.0, 0.05, 1.0), 1.0);
  setRobotPose(-0.2, 0.0, 0.0);
  expectRotation(command(-30.0, 0.05, 1.0), 1.0);
}

TEST_F(HeadingAlignmentTest, LeavingFinalHeadingLatchRequiresInitialAlignmentAgain)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  setRobotPose(1.95, 0.0, -0.5);
  expectRotation(command(0.0), 1.0);
  setRobotPose(1.4, 0.5, 0.0);
  expectRotation(command(0.0), -1.0);
}

TEST_F(HeadingAlignmentTest, PathAlignmentStillChecksCollisions)
{
  blockCurrentFootprint();
  EXPECT_THROW(command(30.0), nav2_core::PlannerException);
}

TEST_F(HeadingAlignmentTest, TrackingStillChecksCollisions)
{
  EXPECT_GT(command(0.0).linear.x, 0.0);
  blockCurrentFootprint();
  EXPECT_THROW(command(20.0), nav2_core::PlannerException);
}

TEST_F(HeadingAlignmentTest, FinalHeadingStillChecksCollisions)
{
  blockCurrentFootprint();
  EXPECT_THROW(command(0.0, 0.05, 1.0), nav2_core::PlannerException);
}

TEST_F(HeadingAlignmentTest, RejectsInvalidDynamicThresholdsWithoutChangingBehavior)
{
  for (double value : {0.0, -0.1, radians(46), std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(node_->set_parameter({"FollowPath.path_alignment_tolerance", value}).successful);
  }
  for (double value : {radians(4), M_PI + 0.1, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(node_->set_parameter({"FollowPath.rotate_to_heading_min_angle", value}).successful);
  }
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("FollowPath.rotate_to_heading_min_angle").as_double(), radians(45));
  expectRotation(command(5.001), 1.0);
  EXPECT_GT(command(4.999).linear.x, 0.0);
  EXPECT_GT(command(30.0).linear.x, 0.0);
}

TEST_F(HeadingAlignmentTest, ValidDynamicUpdatesChangeBothThresholds)
{
  ASSERT_TRUE(node_->set_parameters_atomically({
    {"FollowPath.path_alignment_tolerance", radians(3)},
    {"FollowPath.rotate_to_heading_min_angle", radians(60)},
  }).successful);
  expectRotation(command(4.0), 1.0);
  EXPECT_GT(command(2.9).linear.x, 0.0);
  EXPECT_GT(command(50.0).linear.x, 0.0);
  expectRotation(command(60.1), 1.0);
  expectRotation(command(4.0), 1.0);
}

TEST_F(HeadingAlignmentTest, RejectsInvalidThresholdsAtConfigure)
{
  auto invalid_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "invalid_heading_controller", rclcpp::NodeOptions().parameter_overrides({
      {"FollowPath.path_alignment_tolerance", radians(50)},
      {"FollowPath.rotate_to_heading_min_angle", radians(45)},
    }));
  slam_nav::GoalHeadingLatchedRPP invalid_controller;
  EXPECT_THROW(
    invalid_controller.configure(invalid_node, "FollowPath", tf_, costmap_),
    std::invalid_argument);
}

}  // namespace
