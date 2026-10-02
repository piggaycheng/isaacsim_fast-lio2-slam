#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double yawToQuaternionZ(double yaw)
{
  return std::sin(0.5 * yaw);
}

double yawToQuaternionW(double yaw)
{
  return std::cos(0.5 * yaw);
}
}  // namespace

class WheelEncoderOdometry final : public rclcpp::Node
{
public:
  WheelEncoderOdometry()
  : Node("wheel_encoder_odometry"),
    random_engine_(static_cast<unsigned int>(declare_parameter<int>("random_seed", 42)))
  {
    joint_topic_ = declare_parameter<std::string>("joint_topic", "/isaac/joint_states");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odom");
    left_joint_ = declare_parameter<std::string>("left_joint", "joint_wheel_right");
    right_joint_ = declare_parameter<std::string>("right_joint", "joint_wheel_left");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.14);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.4132);
    encoder_ticks_per_revolution_ =
      declare_parameter<int>("encoder_ticks_per_revolution", 2048);
    left_distance_scale_ = declare_parameter<double>("left_distance_scale", 1.002);
    right_distance_scale_ = declare_parameter<double>("right_distance_scale", 0.998);
    left_direction_ = declare_parameter<double>("left_direction", -1.0);
    right_direction_ = declare_parameter<double>("right_direction", -1.0);
    distance_noise_ratio_ = declare_parameter<double>("distance_noise_ratio", 0.005);
    // Increment error model: Var(ds) = k_s|ds|, Var(dyaw) = k_yd|ds| + k_yy|dyaw|.
    distance_variance_per_meter_ =
      declare_parameter<double>("distance_variance_per_meter", 1.0e-4);
    yaw_variance_per_meter_ = declare_parameter<double>("yaw_variance_per_meter", 1.0e-4);
    yaw_variance_per_radian_ = declare_parameter<double>("yaw_variance_per_radian", 1.0e-3);
    position_variance_per_radian_ =
      declare_parameter<double>("position_variance_per_radian", 1.0e-5);
    min_linear_velocity_variance_ =
      declare_parameter<double>("min_linear_velocity_variance", 1.0e-4);
    min_angular_velocity_variance_ =
      declare_parameter<double>("min_angular_velocity_variance", 1.0e-4);
    min_position_variance_ = declare_parameter<double>("min_position_variance", 1.0e-4);
    min_yaw_variance_ = declare_parameter<double>("min_yaw_variance", 1.0e-4);

    if (wheel_radius_ <= 0.0 || wheel_base_ <= 0.0 ||
      encoder_ticks_per_revolution_ <= 0 || distance_noise_ratio_ < 0.0)
    {
      throw std::invalid_argument("Wheel odometry dimensions and encoder resolution must be valid");
    }
    for (const double value : {
        distance_variance_per_meter_, yaw_variance_per_meter_, yaw_variance_per_radian_,
        position_variance_per_radian_})
    {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("Wheel odometry variance growth rates must be nonnegative");
      }
    }
    for (const double value : {
        min_linear_velocity_variance_, min_angular_velocity_variance_,
        min_position_variance_, min_yaw_variance_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Wheel odometry minimum variances must be positive");
      }
    }

    tick_angle_ = kTwoPi / static_cast<double>(encoder_ticks_per_revolution_);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelEncoderOdometry::jointStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Encoder odometry: %s/%s -> %s (%.0f ticks/rev, radius %.5f m, base %.4f m)",
      left_joint_.c_str(), right_joint_.c_str(), odom_topic_.c_str(),
      static_cast<double>(encoder_ticks_per_revolution_), wheel_radius_, wheel_base_);
  }

private:
  static size_t findJoint(
    const sensor_msgs::msg::JointState & message, const std::string & joint_name)
  {
    const auto iterator = std::find(message.name.begin(), message.name.end(), joint_name);
    if (iterator == message.name.end()) {
      return std::numeric_limits<size_t>::max();
    }
    return static_cast<size_t>(std::distance(message.name.begin(), iterator));
  }

  double quantizeAngle(double angle) const
  {
    return std::round(angle / tick_angle_) * tick_angle_;
  }

  double noisyDistance(double distance)
  {
    if (distance == 0.0 || distance_noise_ratio_ == 0.0) {
      return distance;
    }
    std::normal_distribution<double> distribution(
      0.0, std::abs(distance) * distance_noise_ratio_);
    return distance + distribution(random_engine_);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const size_t left_index = findJoint(*message, left_joint_);
    const size_t right_index = findJoint(*message, right_joint_);
    if (left_index >= message->position.size() || right_index >= message->position.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "JointState does not contain wheel joints '%s' and '%s'",
        left_joint_.c_str(), right_joint_.c_str());
      return;
    }

    const double left_angle = quantizeAngle(message->position[left_index]);
    const double right_angle = quantizeAngle(message->position[right_index]);
    const rclcpp::Time stamp(message->header.stamp);

    if (!initialized_ || stamp <= previous_stamp_) {
      const bool time_reset = initialized_ && stamp < previous_stamp_;
      previous_left_angle_ = left_angle;
      previous_right_angle_ = right_angle;
      previous_stamp_ = stamp;
      initialized_ = true;
      if (time_reset) {
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
        position_variance_ = 0.0;
        yaw_variance_ = 0.0;
      }
      return;
    }

    const double dt = (stamp - previous_stamp_).seconds();
    const double left_delta = std::remainder(left_angle - previous_left_angle_, kTwoPi);
    const double right_delta = std::remainder(right_angle - previous_right_angle_, kTwoPi);
    previous_left_angle_ = left_angle;
    previous_right_angle_ = right_angle;
    previous_stamp_ = stamp;

    double left_distance =
      left_direction_ * left_delta * wheel_radius_ * left_distance_scale_;
    double right_distance =
      right_direction_ * right_delta * wheel_radius_ * right_distance_scale_;
    left_distance = noisyDistance(left_distance);
    right_distance = noisyDistance(right_distance);

    const double distance = 0.5 * (left_distance + right_distance);
    const double delta_yaw = (right_distance - left_distance) / wheel_base_;
    const double midpoint_yaw = yaw_ + 0.5 * delta_yaw;
    x_ += distance * std::cos(midpoint_yaw);
    y_ += distance * std::sin(midpoint_yaw);
    yaw_ = std::remainder(yaw_ + delta_yaw, kTwoPi);

    // Integrated pose error grows only while the wheels move, so a parked robot
    // keeps its current heading confidence instead of drifting toward either sensor.
    // In-place turns still scrub the wheels, so translation uncertainty grows with rotation.
    const double distance_variance = distance_variance_per_meter_ * std::abs(distance) +
      position_variance_per_radian_ * std::abs(delta_yaw);
    const double step_yaw_variance =
      yaw_variance_per_meter_ * std::abs(distance) +
      yaw_variance_per_radian_ * std::abs(delta_yaw);
    position_variance_ += distance_variance + distance * distance * yaw_variance_;
    yaw_variance_ += step_yaw_variance;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = message->header.stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.z = yawToQuaternionZ(yaw_);
    odometry.pose.pose.orientation.w = yawToQuaternionW(yaw_);
    odometry.twist.twist.linear.x = distance / dt;
    odometry.twist.twist.angular.z = delta_yaw / dt;

    // White-noise equivalent of this step's increment variance.
    const double dt_squared = dt * dt;
    odometry.pose.covariance[0] = min_position_variance_ + position_variance_;
    odometry.pose.covariance[7] = min_position_variance_ + position_variance_;
    odometry.pose.covariance[14] = 1.0e6;
    odometry.pose.covariance[21] = 1.0e6;
    odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = min_yaw_variance_ + yaw_variance_;
    odometry.twist.covariance[0] =
      min_linear_velocity_variance_ + distance_variance / dt_squared;
    odometry.twist.covariance[7] = 1.0e6;
    odometry.twist.covariance[14] = 1.0e6;
    odometry.twist.covariance[21] = 1.0e6;
    odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] =
      min_angular_velocity_variance_ + step_yaw_variance / dt_squared;
    publisher_->publish(odometry);
  }

  std::string joint_topic_;
  std::string odom_topic_;
  std::string left_joint_;
  std::string right_joint_;
  std::string odom_frame_;
  std::string base_frame_;
  double wheel_radius_;
  double wheel_base_;
  int encoder_ticks_per_revolution_;
  double left_distance_scale_;
  double right_distance_scale_;
  double left_direction_;
  double right_direction_;
  double distance_noise_ratio_;
  double distance_variance_per_meter_;
  double yaw_variance_per_meter_;
  double yaw_variance_per_radian_;
  double position_variance_per_radian_;
  double min_linear_velocity_variance_;
  double min_angular_velocity_variance_;
  double min_position_variance_;
  double min_yaw_variance_;
  double tick_angle_;
  double position_variance_{0.0};
  double yaw_variance_{0.0};

  bool initialized_{false};
  double previous_left_angle_{0.0};
  double previous_right_angle_{0.0};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  std::mt19937 random_engine_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEncoderOdometry>());
  rclcpp::shutdown();
  return 0;
}
