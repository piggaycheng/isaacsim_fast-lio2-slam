#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$project_dir/ros2_ws"
output_dir="$project_dir/maps/office"
save_patches="true"
save_map_resolution=""

usage() {
  cat <<EOF
Usage: $0 [OPTIONS]

Save the current optimized PGO map as a binary PCD.

Options:
  -o, --output-dir PATH       Output directory (default: maps/office)
  -p, --save-patches BOOL     Save keyframe patches: true or false (default: true)
  -v, --voxel-size METERS    Final map voxel size; 0 disables downsampling
  -h, --help                 Show this help message

Examples:
  $0
  $0 --voxel-size 0.05
  $0 --output-dir maps/office --save-patches false --voxel-size 0.1
EOF
}

require_value() {
  if [[ $# -lt 2 || -z "$2" ]]; then
    echo "Option $1 requires a value." >&2
    usage >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output-dir)
      require_value "$@"
      output_dir="$2"
      shift 2
      ;;
    -p|--save-patches)
      require_value "$@"
      save_patches="$2"
      shift 2
      ;;
    -v|--voxel-size)
      require_value "$@"
      save_map_resolution="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$save_patches" != "true" && "$save_patches" != "false" ]]; then
  echo "--save-patches must be true or false." >&2
  exit 2
fi

if [[ -n "$save_map_resolution" && ! "$save_map_resolution" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--voxel-size must be a non-negative number in meters." >&2
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

if [[ -n "$save_map_resolution" ]]; then
  if [[ "$save_map_resolution" != *.* ]]; then
    save_map_resolution="${save_map_resolution}.0"
  fi
  ros2 param set /pgo/pgo_node save_map_resolution "$save_map_resolution"
fi

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$output_dir', save_patches: $save_patches}"
