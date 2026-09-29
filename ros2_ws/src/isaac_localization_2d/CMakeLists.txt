cmake_minimum_required(VERSION 3.8)
project(isaac_localization_2d)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
