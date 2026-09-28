cmake_minimum_required(VERSION 3.8)
project(isaac_nav)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
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

install(TARGETS wheel_encoder_odometry imu_covariance_adapter ground_obstacle_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
