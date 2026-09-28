cmake_minimum_required(VERSION 3.8)
project(isaac_localization_3d)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_path(NAV2_COSTMAP_INCLUDE_DIR nav2_costmap_2d/costmap_2d_ros.hpp
  PATH_SUFFIXES include/nav2_costmap_2d REQUIRED)
find_path(NAV2_UTIL_INCLUDE_DIR nav2_util/lifecycle_node.hpp
  PATH_SUFFIXES include REQUIRED)
find_library(NAV2_COSTMAP_LIBRARY nav2_costmap_2d_core REQUIRED)
find_library(NAV2_UTIL_LIBRARY nav2_util_core REQUIRED)

add_executable(costmap_observer src/costmap_observer.cpp)
target_include_directories(costmap_observer PRIVATE
  ${NAV2_COSTMAP_INCLUDE_DIR}
  ${NAV2_UTIL_INCLUDE_DIR}
  ${NAV2_UTIL_INCLUDE_DIR}/bondcpp
  ${NAV2_UTIL_INCLUDE_DIR}/bond
  ${NAV2_UTIL_INCLUDE_DIR}/smclib
)
target_link_libraries(costmap_observer ${NAV2_COSTMAP_LIBRARY} ${NAV2_UTIL_LIBRARY})
ament_target_dependencies(costmap_observer
  geometry_msgs map_msgs nav_msgs nav2_msgs pluginlib rclcpp rclcpp_lifecycle
  tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS costmap_observer DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/localization_3d_pose.py
  scripts/global_pose_adapter.py scripts/global_tf_gate.py
  scripts/wait_for_costmap_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
