#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sdk_source="$workspace_dir/src/Livox-SDK2"
sdk_build="$workspace_dir/livox_sdk_build"
sdk_install="$workspace_dir/livox_sdk_install"
sophus_source="$workspace_dir/src/Sophus"
sophus_build="$workspace_dir/sophus_build"
sophus_install="$workspace_dir/sophus_install"
livox_driver="$workspace_dir/src/livox_ros_driver2"

source /opt/ros/humble/setup.bash
set -u

cmake -S "$sdk_source" -B "$sdk_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$sdk_install"
cmake --build "$sdk_build" -j2
cmake --install "$sdk_build"

cmake -S "$sophus_source" -B "$sophus_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$sophus_install" \
  -DSOPHUS_USE_BASIC_LOGGING=ON \
  -DBUILD_SOPHUS_TESTS=OFF
cmake --build "$sophus_build" -j2
cmake --install "$sophus_build"

cp "$livox_driver/package_ROS2.xml" "$livox_driver/package.xml"
rm -rf "$livox_driver/launch"
cp -r "$livox_driver/launch_ROS2" "$livox_driver/launch"

rm -rf \
  "$workspace_dir/build/livox_ros_driver2" \
  "$workspace_dir/build/fastlio2" \
  "$workspace_dir/build/isaac_fastlio_adapter"

cd "$workspace_dir"
colcon build \
  --packages-up-to fastlio2 isaac_fastlio_adapter \
  --cmake-args \
  -DROS_EDITION=ROS2 \
  -DDISTRO_ROS=humble \
  -DCMAKE_PREFIX_PATH="$sophus_install" \
  -DCMAKE_CXX_FLAGS="-I$sophus_install/include" \
  -DCMAKE_LIBRARY_PATH="$sdk_install/lib" \
  -DCMAKE_INCLUDE_PATH="$sdk_install/include"

rm -rf "$livox_driver/launch"
rm -f "$livox_driver/package.xml"
