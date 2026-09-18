#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$project_dir/ros2_ws"
output_dir="${1:-$project_dir/maps/office}"
save_patches="${2:-true}"

if [[ "$save_patches" != "true" && "$save_patches" != "false" ]]; then
  echo "Usage: $0 [output_directory] [true|false]" >&2
  exit 2
fi

if [[ "$output_dir" == *"'"* ]]; then
  echo "Output directory must not contain a single quote." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u

mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"

if [[ "$(ros2 service type /pgo/save_maps 2>/dev/null || true)" != "interface/srv/SaveMaps" ]]; then
  echo "/pgo/save_maps is unavailable. Start ./run_slam.sh before saving." >&2
  exit 1
fi

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$output_dir', save_patches: $save_patches}"
