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

install(PROGRAMS scripts/gimbal_action_server.py scripts/gimbal_mqtt_bridge.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/gimbal_protocol.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
