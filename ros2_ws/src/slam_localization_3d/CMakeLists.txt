cmake_minimum_required(VERSION 3.8)
project(slam_localization_3d)

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
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(fleet_panel SHARED src/fleet_panel.cpp)
target_link_libraries(fleet_panel Qt5::Widgets)
ament_target_dependencies(fleet_panel rviz_common pluginlib)
pluginlib_export_plugin_description_file(rviz_common fleet_plugins.xml)
install(TARGETS fleet_panel LIBRARY DESTINATION lib)
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
  scripts/global_localization_xyz.py
  scripts/wait_for_costmap_tf.py
  scripts/cmd_vel_safety.py
  scripts/adaptive_surround.py
  scripts/wait_for_navigation.py
  scripts/costmap_filter_editor.py
  scripts/covariance_calibration.py
  scripts/covariance_drive.py
  scripts/fleet_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/xyz_cloud.py scripts/costmap_zone_geometry.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Qt5 REQUIRED COMPONENTS Test)
  add_executable(test_fleet_panel test/test_fleet_panel.cpp)
  target_link_libraries(test_fleet_panel Qt5::Widgets Qt5::Test)
  ament_target_dependencies(test_fleet_panel rviz_common rclcpp geometry_msgs)
endif()

ament_package()
