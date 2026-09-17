#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
constexpr uint64_t kScanPeriodNanoseconds = 100000000;
constexpr uint8_t kDefaultReflectivity = 100;
}

class PointCloud2ToLivox final : public rclcpp::Node
{
public:
  PointCloud2ToLivox()
  : Node("pointcloud2_to_livox")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/isaac/lidar_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
    lidar_id_ = static_cast<uint8_t>(declare_parameter<int>("lidar_id", 1));

    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2ToLivox::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Converting %s [PointCloud2] to %s [livox CustomMsg]",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    livox_ros_driver2::msg::CustomMsg output;
    output.header = message->header;
    output.timebase =
      static_cast<uint64_t>(message->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(message->header.stamp.nanosec);
    output.lidar_id = lidar_id_;
    output.rsvd = {0, 0, 0};
    output.points.reserve(message->width * message->height);

    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");

    const size_t point_count = message->width * message->height;
    for (size_t index = 0; index < point_count; ++index, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }

      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = point_count > 1
        ? static_cast<uint32_t>((kScanPeriodNanoseconds * index) / (point_count - 1))
        : 0;
      point.x = *x;
      point.y = *y;
      point.z = *z;
      point.reflectivity = kDefaultReflectivity;
      point.tag = 0;
      point.line = static_cast<uint8_t>(index % 4);
      output.points.push_back(point);
    }

    output.point_num = static_cast<uint32_t>(output.points.size());
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  uint8_t lidar_id_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2ToLivox>());
  rclcpp::shutdown();
  return 0;
}
