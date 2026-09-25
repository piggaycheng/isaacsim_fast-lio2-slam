cmake_minimum_required(VERSION 3.8)
project(isaac_localization_3d)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/localization_3d_pose.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
