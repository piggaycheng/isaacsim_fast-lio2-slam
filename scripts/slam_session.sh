#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
set -u

if [[ $# == 1 && ( "$1" == status || "$1" == save-stop ) ]] ||
   [[ $# == 2 && "$1" == start ]]; then
  if [[ -z "$(ros_compose ps --status running --quiet ros)" ]]; then
    echo "ROS container is not running. Start ./scripts/run_slam.sh first." >&2
    exit 1
  fi
  ros_compose exec -T ros /workspace/docker/entrypoint.sh \
    python3 /workspace/scripts/mapping_manager.py "$@"
elif [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
  echo "Usage: $0 start FLOOR | save-stop | status"
  echo "Start ./scripts/run_slam.sh first. start always creates a new map; existing directories are rejected."
else
  echo "Usage: $0 start FLOOR | save-stop | status" >&2
  exit 2
fi
