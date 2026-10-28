#!/usr/bin/env bash
# Isaac Sim check of the 3D odom -> base_link (local_odometry_3d) on a ramp; no policy needed.
# Usage: ISAAC_PYTHON=/path/to/isaacsim/python.sh tests/run_3d_odometry_validation.sh [probe args]
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_python="${ISAAC_PYTHON:-/home/yu/isaacsim-6.1.0/python.sh}"
directory="$project_dir/ros2_ws/log/3d_odometry/$(date +%Y%m%d_%H%M%S)"
export ROS_DOMAIN_ID="${VALIDATION_ROS_DOMAIN_ID:-188}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
ramp="${RAMP:-1.0,0.0,0.15,6.0,1.5,1.6}"
inputs="${LOCAL_INPUTS:-wheel,imu,lio}"
sim_pid=""

compose() {
  docker compose --project-directory "$project_dir" \
    -f "$project_dir/docker-compose.yml" -p isaacsim-3d-odometry "$@"
}

if [[ -n "$(docker compose --project-directory "$project_dir" ps --status running -q)" ]] ||
   [[ -n "$(compose ps --status running -q)" ]] ||
   pgrep -f '[/]standalone.py([[:space:]]|$)' >/dev/null; then
  echo "Stop the existing simulation/navigation before running this validation." >&2
  exit 1
fi
if [[ ! -f "$project_dir/ros2_ws/install/setup.bash" ]]; then
  echo "Build the ROS workspace before running this validation." >&2
  exit 1
fi

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    while read -r child; do
      kill -TERM "$child" 2>/dev/null || true
    done < <(pgrep -P "$sim_pid" || true)
    kill -TERM "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  fi
  compose down --timeout 20 >/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

IFS=, read -r ramp_x _ ramp_rise ramp_slope ramp_plateau _ <<< "$ramp"
mkdir -p "$directory"
echo "Validation evidence: $directory"
export ROS_COMMAND=launch
export ROS_LAUNCH_ARGS="$(printf '%s\n' \
  slam_localization_3d global_fusion.launch.py \
  map_pcd:=/workspace/maps/office/map.pcd \
  map_pgm:=/workspace/maps/office/map_2d.yaml \
  rviz:=false auto_initial_pose:=true obstacle_cloud:=true costmaps:=true \
  local_odometry_inputs:="$inputs" local_odometry_3d:=true)"
compose up -d ros
"$isaac_python" "$project_dir/scripts/standalone.py" --headless --ros-cmd-vel \
  --lidar-motion-compensation noncompensated --ramp "$ramp" \
  > "$directory/isaac.log" 2>&1 &
sim_pid=$!
relative="${directory#"$project_dir"/}"
status=0
compose exec -T ros /workspace/docker/entrypoint.sh \
  python3 /workspace/tests/validate_3d_odometry.py \
  --output "/workspace/$relative/results.json" --ramp-foot "$ramp_x" \
  --rise "$ramp_rise" --slope-deg "$ramp_slope" --plateau "$ramp_plateau" "$@" \
  2>&1 | tee "$directory/probe.log" || status=$?
compose logs --no-color ros > "$directory/ros.log"
exit "$status"
