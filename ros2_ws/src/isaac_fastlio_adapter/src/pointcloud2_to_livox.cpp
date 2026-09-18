#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
constexpr uint8_t kFastlioLineCount = 4;

struct LidarPoint
{
  float x;
  float y;
  float z;
  float intensity;
  uint64_t timestamp;
  uint32_t emitter_id;
  uint32_t channel_id;
};

bool hasField(const sensor_msgs::msg::PointCloud2 & message, const std::string & name)
{
  return std::any_of(
    message.fields.begin(), message.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
}
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
    constexpr const char * required_fields[] = {
      "x", "y", "z", "intensity", "timestamp", "emitter_id", "channel_id"
    };
    for (const char * field : required_fields) {
      if (!hasField(*message, field)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Input PointCloud2 is missing required RTX metadata field '%s'", field);
        return;
      }
    }

    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(*message, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> timestamp(*message, "timestamp");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> emitter_id(*message, "emitter_id");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> channel_id(*message, "channel_id");

    std::vector<LidarPoint> points;
    points.reserve(message->width * message->height);
    uint64_t scan_start_timestamp = std::numeric_limits<uint64_t>::max();

    const size_t point_count = message->width * message->height;
    for (size_t index = 0; index < point_count;
      ++index, ++x, ++y, ++z, ++intensity, ++timestamp, ++emitter_id, ++channel_id)
    {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }

      const uint64_t point_timestamp =
        (static_cast<uint64_t>(timestamp[1]) << 32) |
        static_cast<uint64_t>(timestamp[0]);
      scan_start_timestamp = std::min(scan_start_timestamp, point_timestamp);
      points.push_back(
        {*x, *y, *z, *intensity, point_timestamp, *emitter_id, *channel_id});
    }

    if (points.empty()) {
      return;
    }

    livox_ros_driver2::msg::CustomMsg output;
    output.header = message->header;
    output.header.stamp.sec = static_cast<int32_t>(scan_start_timestamp / 1000000000ULL);
    output.header.stamp.nanosec =
      static_cast<uint32_t>(scan_start_timestamp % 1000000000ULL);
    output.timebase = scan_start_timestamp;
    output.lidar_id = lidar_id_;
    output.rsvd = {0, 0, 0};
    output.points.reserve(points.size());

    for (const LidarPoint & input : points) {
      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = static_cast<uint32_t>(
        std::min<uint64_t>(
          input.timestamp - scan_start_timestamp,
          std::numeric_limits<uint32_t>::max()));
      point.x = input.x;
      point.y = input.y;
      point.z = input.z;
      point.reflectivity = static_cast<uint8_t>(
        std::clamp(input.intensity * 255.0F, 0.0F, 255.0F));
      point.tag = 0;
      point.line = static_cast<uint8_t>(input.channel_id % kFastlioLineCount);
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
