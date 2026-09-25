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
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double acceleration_scale_;
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
