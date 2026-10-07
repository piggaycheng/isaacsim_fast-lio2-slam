cmake_minimum_required(VERSION 3.8)
project(slam_sensor_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GimbalMove.action"
  "action/GimbalRotate.action"
  DEPENDENCIES action_msgs
)

install(PROGRAMS scripts/gimbal_action_server.py DESTINATION lib/${PROJECT_NAME})

ament_package()
