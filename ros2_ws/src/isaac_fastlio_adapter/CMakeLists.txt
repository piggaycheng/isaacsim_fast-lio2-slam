cmake_minimum_required(VERSION 3.8)
project(isaac_fastlio_adapter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud2_to_livox src/pointcloud2_to_livox.cpp)
ament_target_dependencies(pointcloud2_to_livox
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(imu_scale_adapter src/imu_scale_adapter.cpp)
ament_target_dependencies(imu_scale_adapter
  rclcpp
  sensor_msgs
)

install(TARGETS pointcloud2_to_livox imu_scale_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
