#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class GroundObstacleFilter final : public rclcpp::Node
{
public:
  GroundObstacleFilter()
  : Node("ground_obstacle_filter")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/isaac/lidar_points");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/obstacles");
    const auto self_filtered_topic = declare_parameter<std::string>(
      "self_filtered_topic", "/perception/self_filtered_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    min_range_ = declare_parameter<double>("min_range", 0.0);
    self_filter_bounds_ = declare_parameter<std::vector<double>>(
      "self_filter_bounds", {-0.20, 0.65, -0.32, 0.32});
    max_range_ = declare_parameter<double>("max_range", 20.0);
    ground_search_height_ = declare_parameter<double>("ground_search_height", 0.25);
    ground_distance_ = declare_parameter<double>("ground_distance", 0.04);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.06);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 2.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.08);
    min_ground_points_ = declare_parameter<int>("min_ground_points", 50);
    if (base_frame_.empty() || min_range_ < 0 || max_range_ <= min_range_ ||
      ground_search_height_ <= 0 || ground_distance_ <= 0 ||
      min_obstacle_height_ <= ground_distance_ ||
      max_obstacle_height_ <= min_obstacle_height_ || voxel_size_ <= 0 ||
      min_ground_points_ < 3)
    {
      throw std::invalid_argument("Invalid ground obstacle filter dimensions");
    }
    if (self_filter_bounds_.size() != 4 ||
      !std::isfinite(self_filter_bounds_[0]) || !std::isfinite(self_filter_bounds_[1]) ||
      !std::isfinite(self_filter_bounds_[2]) || !std::isfinite(self_filter_bounds_[3]) ||
      self_filter_bounds_[0] >= self_filter_bounds_[1] ||
      self_filter_bounds_[2] >= self_filter_bounds_[3])
    {
      throw std::invalid_argument("self_filter_bounds must be [min_x, max_x, min_y, max_y]");
    }

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    self_filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      self_filtered_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&GroundObstacleFilter::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Filtering %s into %s (%s frame)",
      input_topic.c_str(), output_topic.c_str(), base_frame_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    for (const auto & name : {"x", "y", "z"}) {
      bool found = false;
      for (const auto & field : message->fields) {
        if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Obstacle cloud needs FLOAT32 %s", name);
        return;
      }
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = buffer_->lookupTransform(
        base_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot transform obstacle cloud: %s",
        error.what());
      return;
    }
    sensor_msgs::msg::PointCloud2 transformed;
    tf2::doTransform(*message, transformed, transform);

    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(transformed, input);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(input.size());
    const double max_range_squared = max_range_ * max_range_;
    const double min_range_squared = min_range_ * min_range_;
    for (const auto & point : input) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double distance_squared = point.x * point.x + point.y * point.y;
      constexpr double boundary_tolerance = 1e-6;
      const bool inside_body =
        point.x >= self_filter_bounds_[0] - boundary_tolerance &&
        point.x <= self_filter_bounds_[1] + boundary_tolerance &&
        point.y >= self_filter_bounds_[2] - boundary_tolerance &&
        point.y <= self_filter_bounds_[3] + boundary_tolerance;
      if (!inside_body && distance_squared >= min_range_squared &&
        distance_squared <= max_range_squared)
      {
        cloud.push_back(point);
      }
    }
    // Keep ground returns for /scan, even when plane fitting cannot produce obstacles.
    sensor_msgs::msg::PointCloud2 self_filtered;
    pcl::toROSMsg(cloud, self_filtered);
    self_filtered.header = message->header;
    self_filtered.header.frame_id = base_frame_;
    self_filtered_publisher_->publish(self_filtered);

    auto ground_candidates = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ground_candidates->reserve(cloud.size());
    for (const auto & point : cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const double distance_squared = point.x * point.x + point.y * point.y;
      if (distance_squared >= min_range_squared &&
        distance_squared <= max_range_squared &&
        std::abs(point.z) <= ground_search_height_)
      {
        ground_candidates->push_back(point);
      }
    }
    if (ground_candidates->size() < static_cast<size_t>(min_ground_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No ground candidates; withholding obstacle cloud");
      return;
    }

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(0.262);
    segmentation.setDistanceThreshold(ground_distance_);
    segmentation.setMaxIterations(100);
    segmentation.setInputCloud(ground_candidates);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients plane;
    segmentation.segment(inliers, plane);
    if (inliers.indices.size() < static_cast<size_t>(min_ground_points_) ||
      plane.values.size() != 4)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No reliable ground plane; withholding obstacle cloud");
      return;
    }

    const auto & c = plane.values;
    const double normal_length = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    if (!std::isfinite(normal_length) || normal_length == 0 ||
      !std::isfinite(c[3]) || std::abs(c[3]) / normal_length > ground_distance_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ground plane is not near base_link z=0");
      return;
    }
    const double orientation = c[2] < 0 ? -1.0 : 1.0;
    auto obstacles = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    obstacles->reserve(cloud.size());
    for (const auto & point : cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const double distance_squared = point.x * point.x + point.y * point.y;
      const double height =
        orientation * (c[0] * point.x + c[1] * point.y + c[2] * point.z + c[3]) /
        normal_length;
      if (distance_squared >= min_range_squared &&
        distance_squared <= max_range_squared &&
        height >= min_obstacle_height_ && height <= max_obstacle_height_)
      {
        obstacles->push_back(point);
      }
    }

    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(obstacles);
    downsample.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    pcl::PointCloud<pcl::PointXYZ> filtered;
    downsample.filter(filtered);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(filtered, output);
    output.header = message->header;
    output.header.frame_id = base_frame_;
    publisher_->publish(output);
  }

  std::string base_frame_;
  std::vector<double> self_filter_bounds_;
  double min_range_;
  double max_range_;
  double ground_search_height_;
  double ground_distance_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double voxel_size_;
  int min_ground_points_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr self_filtered_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
