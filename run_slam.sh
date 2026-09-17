#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$project_dir/ros2_ws"

source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$workspace_dir/livox_sdk_install/lib:${LD_LIBRARY_PATH:-}"

ros2 launch isaac_fastlio_adapter fastlio.launch.py &
ros_pid=$!

cleanup() {
  kill "$ros_pid" 2>/dev/null || true
  wait "$ros_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$project_dir/standalone.py"
