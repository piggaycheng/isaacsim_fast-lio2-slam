cmake_minimum_required(VERSION 3.8)
project(isaac_nav)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

install(TARGETS wheel_encoder_odometry imu_covariance_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
