#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_python="${ISAAC_PYTHON:-/home/yu/isaacsim-6.1.0/python.sh}"
directory="$project_dir/ros2_ws/log/navigation_environment/$(date +%Y%m%d_%H%M%S)"
export ROS_DOMAIN_ID="${VALIDATION_ROS_DOMAIN_ID:-189}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
sim_pid=""
probe=validate_navigation_environment.py
if [[ "${1:-}" == --braking ]]; then
  probe=validate_braking.py
  shift
fi

compose() {
  docker compose --project-directory "$project_dir" \
    -f "$project_dir/docker-compose.yml" -p isaacsim-nav-validation "$@"
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
    # Isaac's python.sh is a shell wrapper; terminate its Python child as well.
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

mkdir -p "$directory"
echo "Validation evidence: $directory"
"$isaac_python" "$project_dir/tests/validate_carter_geometry.py" \
  --output "$directory/geometry.json" > "$directory/geometry.log" 2>&1
export ROS_COMMAND=launch
export ROS_LAUNCH_ARGS="$(printf '%s\n' \
  slam_localization_3d global_fusion.launch.py \
  map_pcd:=/workspace/maps/office/map.pcd \
  map_pgm:=/workspace/maps/office/map_2d.yaml \
  rviz:=false auto_initial_pose:=true obstacle_cloud:=true costmaps:=true navigate:=true)"
compose up -d ros
"$isaac_python" "$project_dir/scripts/standalone.py" --headless --ros-cmd-vel \
  --lidar-motion-compensation noncompensated --validation-control-dir "$directory" \
  > "$directory/isaac.log" 2>&1 &
sim_pid=$!
relative="${directory#"$project_dir"/}"
status=0
probe_args=()
if [[ "$probe" == validate_navigation_environment.py ]]; then
  probe_args+=(--planning-config /workspace/ros2_ws/install/slam_localization_3d/share/slam_localization_3d/config/observation_costmaps.yaml)
fi
compose exec -T ros /workspace/docker/entrypoint.sh \
  python3 "/workspace/tests/$probe" \
  --control-dir "/workspace/$relative" --geometry "/workspace/$relative/geometry.json" \
  --output "/workspace/$relative/results.json" \
  "${probe_args[@]}" "$@" \
  2>&1 | tee "$directory/probe.log" || status=$?
compose logs --no-color ros > "$directory/ros.log"
exit "$status"
