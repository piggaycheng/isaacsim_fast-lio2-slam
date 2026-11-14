#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
free_fly=false
headless=false
auto_jog=false

usage() {
  cat <<'EOF'
Usage: ./scripts/run_slam.sh [OPTIONS]
       ./scripts/run_mapping.sh [OPTIONS]

Launch the FAST-LIO2 SLAM mapping service and Isaac Sim Office environment.

Options:
      --free-fly       Spawn a collision-free LiDAR+IMU sensor pod and fly in 3D
                       with W/S/A/D and Q/E for multi-floor mapping in one pass.
      --headless       Run Isaac Sim without its GUI.
      --auto-jog       Drive/fly the robot automatically for headless SLAM tests.
  -h, --help           Show this help.

Examples:
  ./scripts/run_slam.sh
  ./scripts/run_slam.sh --free-fly
  ./scripts/run_mapping.sh --free-fly
EOF
}

while (($# > 0)); do
  case "$1" in
    --free-fly)
      free_fly=true
      shift
      ;;
    --headless)
      headless=true
      shift
      ;;
    --auto-jog)
      auto_jog=true
      shift
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

set -u

require_ros_workspace
start_ros slam
echo "Mapping is initially idle. In another terminal: ./scripts/slam_session.sh start FLOOR"
echo "Before changing floor: ./scripts/slam_session.sh save-stop"

isaac_args=(--dynamic-elevator-doors --elevator-door-mode manual)
if [[ "$free_fly" == true ]]; then
  isaac_args+=(--free-fly)
fi
if [[ "$headless" == true ]]; then
  isaac_args+=(--headless)
fi
if [[ "$auto_jog" == true ]]; then
  isaac_args+=(--auto-jog)
fi

"$(isaac_python_path)" "$project_dir/scripts/standalone.py" "${isaac_args[@]}"
