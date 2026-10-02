#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_pgm="$project_dir/maps/office/map_2d.yaml"
map_pcd="$project_dir/maps/office/map.pcd"
headless=false
rviz=true
auto_initial_pose=true
robots=()
boxes=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_multi_nav.sh [OPTIONS]

Launch Isaac Sim with several robots. Each robot runs its own 3D localization
and Nav2 stack in its own container (Compose project isaacsim-fastlio2-NAME),
under /NAME (topics, actions and the TF tree /NAME/tf), like separate robots.
One fleet RViz (container isaacsim-fastlio2-fleet-rviz) shows all robots.

Options:
      --robot NAME[:TYPE]@X,Y[,YAW]
                       Spawn a robot of TYPE (config/robots/TYPE.yaml, default
                       nova_carter) at Isaac world X,Y (m), YAW (rad) of the
                       robot prim (world is aligned with the Office map).
                       Repeat per robot. Default: carter1@0,0 and carter2@3.5,0.
  -m, --map FILE       Nav2 map YAML file (default: maps/office/map_2d.yaml).
      --pcd FILE       PCD map (default: maps/office/map.pcd).
      --manual-initial-pose
                       Set each initial pose in RViz instead of the spawn pose.
      --no-rviz        Do not start the fleet RViz.
      --headless       Run Isaac Sim without its GUI.
      --box X,Y[,SX,SY,SZ]
                       Place a static box obstacle at Office map X,Y (m).
  -h, --help           Show this help.

In RViz's Fleet Control panel, select a robot, click "Set navigation goal",
then click and drag on the map. The shared toolbar pose tools use that robot too.
Or send a goal directly:
  ros2 action send_goal /carter2/navigate_to_pose nav2_msgs/action/NavigateToPose ...

Examples:
  ./scripts/run_multi_nav.sh
  ./scripts/run_multi_nav.sh --robot a@0,0 --robot b@3.5,-2,1.57 --robot c@1,2
EOF
}

value() {
  if (($# < 2)) || [[ -z "$2" ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

while (($# > 0)); do
  case "$1" in
    --robot) value "$@"; robots+=("$2"); shift 2 ;;
    -m|--map) value "$@"; map_pgm="$2"; shift 2 ;;
    --pcd) value "$@"; map_pcd="$2"; shift 2 ;;
    --manual-initial-pose) auto_initial_pose=false; shift ;;
    --no-rviz) rviz=false; shift ;;
    --headless) headless=true; shift ;;
    --box) value "$@"; boxes+=(--box "$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ((${#robots[@]} == 0)); then robots=("carter1@0,0,0" "carter2@3.5,0,0"); fi
spec="$(IFS=';'; printf '%s' "${robots[*]}")"
# Fail before starting anything; robot.launch.py and standalone.py check again.
if ! python3 - "$project_dir" "$spec" <<'EOF'
import sys
sys.path.insert(0, sys.argv[1] + "/ros2_ws/src/slam_localization_3d/launch")
try:
    import robot_fleet
except ImportError:  # host without PyYAML
    sys.exit(0)
try:
    specs = robot_fleet.parse_robot_specs(sys.argv[2])
    for spec in specs:
        robot_fleet.load_robot_profile(spec.robot_type)
except ValueError as error:
    sys.exit(f"{error}")
EOF
then
  exit 2
fi

source "$project_dir/docker/ros_compose.sh"

if [[ "$map_pcd" != /* ]]; then map_pcd="$project_dir/$map_pcd"; fi
if [[ "$map_pgm" != /* ]]; then map_pgm="$project_dir/$map_pgm"; fi
if [[ ! -f "$map_pcd" || "$map_pcd" != *.pcd ]]; then
  echo "PCD map does not exist or is not a .pcd file: $map_pcd" >&2
  exit 1
fi
if [[ ! -f "$map_pgm" ]]; then
  echo "PGM map YAML does not exist: $map_pgm" >&2
  exit 1
fi
if [[ "$map_pcd" != "$project_dir/maps/office/map.pcd" ||
      "$map_pgm" != "$project_dir/maps/office/map_2d.yaml" ]]; then
  # Spawn poses equal map poses only for the Office maps.
  auto_initial_pose=false
fi
require_ros_workspace
set -u

names=()
for robot in "${robots[@]}"; do
  name="${robot%%@*}"
  name="${name%%:*}"
  names+=("$name")
  start_ros_container "$name" launch slam_localization_3d robot.launch.py "robot:=$robot" \
    map_pcd:="$(container_path "$map_pcd")" map_pgm:="$(container_path "$map_pgm")" \
    rviz:=false auto_initial_pose:="$auto_initial_pose"
done
if [[ "$rviz" == true ]]; then
  start_ros_container fleet-rviz launch slam_localization_3d fleet_rviz.launch.py \
    "robots:=$(IFS=';'; printf '%s' "${names[*]}")"
fi

isaac_args=(--ros-cmd-vel --lidar-motion-compensation noncompensated)
if [[ "$headless" == true ]]; then isaac_args+=(--headless); fi
for robot in "${robots[@]}"; do isaac_args+=(--robot "$robot"); done
isaac_args+=("${boxes[@]}")
"$project_dir/scripts/standalone.py" "${isaac_args[@]}"
