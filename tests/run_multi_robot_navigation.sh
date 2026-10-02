#!/usr/bin/env bash
# Headless two-robot end-to-end check: one ROS container per robot (as on real
# robots) localizes its namespaced stack and reaches concurrent goals.
# The fleet relay must merge both TF trees. Evidence goes to ros2_ws/log/multi_robot_navigation/.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_python="${ISAAC_PYTHON:-/home/yu/isaacsim-6.1.0/python.sh}"
directory="$project_dir/ros2_ws/log/multi_robot_navigation/$(date +%Y%m%d_%H%M%S)"
export ROS_DOMAIN_ID="${VALIDATION_ROS_DOMAIN_ID:-188}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROBOT_PROJECT_PREFIX=isaacsim-multi-validation
sim_pid=""
robots=("carter1@0,0,0" "carter2:carter_v1@3.5,0,0")
# Map X,Y,YAW goals in the free room around the Office spawn.
goals=("0.5,2.5,1.5707963" "3.7,-2.5,-1.5707963")

source "$project_dir/docker/ros_compose.sh"

if [[ -n "$(docker ps -q --filter "label=com.docker.compose.project.working_dir=$project_dir")" ]] ||
   pgrep -f '[/]standalone.py([[:space:]]|$)' >/dev/null; then
  echo "Stop the existing simulation/navigation containers before running this validation." >&2
  exit 1
fi
require_ros_workspace

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    while read -r child; do
      kill -TERM "$child" 2>/dev/null || true
    done < <(pgrep -P "$sim_pid" || true)
    kill -TERM "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  fi
  stop_ros_containers
}

mkdir -p "$directory"
echo "Validation evidence: $directory"
names=()
for robot in "${robots[@]}"; do
  name="${robot%%@*}"
  name="${name%%:*}"
  names+=("$name")
  start_ros_container "$name" launch slam_localization_3d robot.launch.py "robot:=$robot" \
    map_pcd:=/workspace/maps/office/map.pcd map_pgm:=/workspace/maps/office/map_2d.yaml \
    rviz:=false > "$directory/ros_$name.log" 2>&1
done
start_ros_container fleet-rviz launch slam_localization_3d fleet_rviz.launch.py \
  "robots:=$(IFS=';'; printf '%s' "${names[*]}")" rviz:=false > "$directory/fleet_relay.log" 2>&1
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

isaac_args=(--headless --ros-cmd-vel --lidar-motion-compensation noncompensated)
for robot in "${robots[@]}"; do isaac_args+=(--robot "$robot"); done
"$isaac_python" "$project_dir/scripts/standalone.py" "${isaac_args[@]}" > "$directory/isaac.log" 2>&1 &
sim_pid=$!

relative="${directory#"$project_dir"/}"
probe_args=(--output "/workspace/$relative/results.json")
for index in "${!robots[@]}"; do
  probe_args+=(--robot "${robots[$index]}" --goal "${goals[$index]}")
done
# The probe is a separate client container, like an operator station.
ros_compose -p "$ROBOT_PROJECT_PREFIX-probe" run --rm --no-deps ros \
  python3 /workspace/tests/validate_multi_robot_navigation.py "${probe_args[@]}" \
  2>&1 | tee "$directory/probe.log"
