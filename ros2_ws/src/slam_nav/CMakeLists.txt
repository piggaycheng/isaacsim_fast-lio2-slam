cmake_minimum_required(VERSION 3.8)
project(slam_nav)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(wheel_encoder_odometry src/wheel_encoder_odometry.cpp)
ament_target_dependencies(wheel_encoder_odometry
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(imu_covariance_adapter src/imu_covariance_adapter.cpp)
ament_target_dependencies(imu_covariance_adapter
  rclcpp
  sensor_msgs
)

add_executable(ground_obstacle_filter src/ground_obstacle_filter.cpp)
target_include_directories(ground_obstacle_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(ground_obstacle_filter ${PCL_LIBRARIES})
ament_target_dependencies(ground_obstacle_filter
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

add_library(slam_nav_plugins SHARED
  src/goal_heading_latched_rpp.cpp
  src/latched_goal_checker.cpp
)
ament_target_dependencies(slam_nav_plugins
  geometry_msgs
  nav2_core
  nav2_regulated_pure_pursuit_controller
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2
)
pluginlib_export_plugin_description_file(nav2_core nav2_plugins.xml)

install(TARGETS wheel_encoder_odometry imu_covariance_adapter ground_obstacle_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS slam_nav_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
