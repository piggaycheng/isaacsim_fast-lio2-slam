#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$project_dir/docker/ros_compose.sh"
set -u

require_ros_workspace
start_ros slam

"$project_dir/standalone.py"
