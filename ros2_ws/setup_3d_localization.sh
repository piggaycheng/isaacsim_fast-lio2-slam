#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname "$workspace_dir")"
base="$workspace_dir/localization_3d_install"
prefix="$base/debs/opt/ros/humble"

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Build the existing ROS workspace with ros2_ws/build_workspace.sh first." >&2
  exit 1
fi

git -C "$project_dir" submodule update --init --recursive ros2_ws/src/FAST_LIO_LOCALIZATION2
mkdir -p "$base/packages" "$base/debs"
for package in ros-humble-pcl-ros ros-humble-tf-transformations; do
  if ! find "$base/packages" -maxdepth 1 -name "${package}_*.deb" -print -quit | grep -q .; then
    (cd "$base/packages" && apt download "$package")
  fi
done
for deb in "$base"/packages/*.deb; do
  dpkg-deb -x "$deb" "$base/debs"
done

if [[ ! -f "$base/venv/bin/activate" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$base/venv"
fi
"$base/venv/bin/pip" install 'numpy==1.23.5' 'open3d==0.18.0' \
  transforms3d pybase64 transformations
"$base/venv/bin/pip" install --no-deps 'ros2-numpy==0.0.5'

source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
colcon --log-base "$base/log" build \
  --base-paths "$workspace_dir/src/FAST_LIO_LOCALIZATION2" "$workspace_dir/src/isaac_localization_3d" \
  --packages-select fast_lio_localization isaac_localization_3d \
  --build-base "$base/build" --install-base "$base/install"

echo "3D localization installed in $base"
