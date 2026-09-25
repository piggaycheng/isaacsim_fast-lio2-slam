#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuScaleAdapter final : public rclcpp::Node
{
public:
  ImuScaleAdapter()
  : Node("imu_scale_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/isaac/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/imu");
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.1);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.01);
    angular_velocity_variance_ =
      declare_parameter<double>("angular_velocity_variance", 0.0025);
    linear_acceleration_variance_ =
      declare_parameter<double>("linear_acceleration_variance", 0.04);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuScaleAdapter::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Scaling %s acceleration by %.3f and publishing to %s",
      input_topic_.c_str(), acceleration_scale_, output_topic_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    sensor_msgs::msg::Imu output = *message;
    output.linear_acceleration.x *= acceleration_scale_;
    output.linear_acceleration.y *= acceleration_scale_;
    output.linear_acceleration.z *= acceleration_scale_;
    output.orientation_covariance[0] = orientation_variance_;
    output.orientation_covariance[4] = orientation_variance_;
    output.orientation_covariance[8] = orientation_variance_;
    output.angular_velocity_covariance[0] = angular_velocity_variance_;
    output.angular_velocity_covariance[4] = angular_velocity_variance_;
    output.angular_velocity_covariance[8] = angular_velocity_variance_;
    output.linear_acceleration_covariance[0] = linear_acceleration_variance_;
    output.linear_acceleration_covariance[4] = linear_acceleration_variance_;
    output.linear_acceleration_covariance[8] = linear_acceleration_variance_;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double acceleration_scale_;
  double orientation_variance_;
  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuScaleAdapter>());
  rclcpp::shutdown();
  return 0;
}
