#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
set -u

require_ros_workspace
start_ros slam
echo "Mapping is initially idle. In another terminal: ./scripts/slam_session.sh start FLOOR"
echo "Before changing floor: ./scripts/slam_session.sh save-stop"

"$project_dir/scripts/standalone.py" --dynamic-elevator-doors --elevator-door-mode manual
