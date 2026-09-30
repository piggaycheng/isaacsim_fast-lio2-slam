#!/usr/bin/env bash
# Builds ros2_ws inside the ROS container. Run it with:
#   docker compose run --rm ros build [extra colcon build arguments]
set -eo pipefail

workspace_dir=/workspace/ros2_ws
livox_driver="$workspace_dir/src/livox_ros_driver2"
fastlio_source="$workspace_dir/src/FASTLIO2_ROS2"
pgo_patch=/workspace/patches/fastlio2-pgo-sync.patch
pgo_patch_applied=false

if [[ ! -f /.dockerenv ]]; then
  echo "Run this inside the ROS container: docker compose run --rm ros build" >&2
  exit 1
fi
if [[ ! -f "$livox_driver/package_ROS2.xml" || ! -d "$fastlio_source/pgo" ||
      ! -f "$workspace_dir/src/FAST_LIO_LOCALIZATION2/package.xml" ]]; then
  echo "Submodules are missing. Run: git submodule update --init --recursive" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

cleanup() {
  rm -rf "$livox_driver/launch"
  rm -f "$livox_driver/package.xml"
  if [[ "$pgo_patch_applied" == true ]]; then
    git -C "$fastlio_source" apply --reverse "$pgo_patch"
  fi
}
trap cleanup EXIT

cp "$livox_driver/package_ROS2.xml" "$livox_driver/package.xml"
rm -rf "$livox_driver/launch"
cp -r "$livox_driver/launch_ROS2" "$livox_driver/launch"

if git -C "$fastlio_source" apply --reverse --check "$pgo_patch" 2>/dev/null; then
  echo "FASTLIO2 PGO synchronization patch is already applied"
elif git -C "$fastlio_source" apply --check "$pgo_patch"; then
  git -C "$fastlio_source" apply "$pgo_patch"
  pgo_patch_applied=true
else
  echo "FASTLIO2 PGO synchronization patch does not apply cleanly" >&2
  exit 1
fi

# These packages depend on the temporary Livox manifest and PGO patch, so
# rebuild them from scratch to avoid stale CMake state.
rm -rf \
  "$workspace_dir/build/livox_ros_driver2" \
  "$workspace_dir/build/interface" \
  "$workspace_dir/build/fastlio2" \
  "$workspace_dir/build/pgo"

cd "$workspace_dir"
colcon build \
  --packages-up-to fastlio2 pgo isaac_fastlio_adapter isaac_nav \
    isaac_localization_2d fast_lio_localization isaac_localization_3d \
  "$@" \
  --cmake-args \
  -DROS_EDITION=ROS2 \
  -DDISTRO_ROS=humble
