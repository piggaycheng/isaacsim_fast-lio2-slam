cmake_minimum_required(VERSION 3.8)
project(slam_fleet_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/fleet_bridge_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/task_bt.py scripts/fleet_pose.py DESTINATION lib/${PROJECT_NAME})

ament_package()
