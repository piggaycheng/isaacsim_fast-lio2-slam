#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovarianceAdapter final : public rclcpp::Node
{
public:
  ImuCovarianceAdapter()
  : Node("imu_covariance_adapter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/isaac/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/nav/imu");
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.01);
    angular_velocity_variance_ =
      declare_parameter<double>("angular_velocity_variance", 0.0025);
    linear_acceleration_variance_ =
      declare_parameter<double>("linear_acceleration_variance", 0.04);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        sensor_msgs::msg::Imu output = *message;
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
      });
  }

private:
  double orientation_variance_;
  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovarianceAdapter>());
  rclcpp::shutdown();
  return 0;
}
