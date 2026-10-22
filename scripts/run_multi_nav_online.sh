#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_pgm="$project_dir/maps/1F/map_2d.yaml"
map_pcd=""
headless=true
rviz=true
auto_initial_pose=true
robots=()
boxes=()
filter_editor=false
filter_state_dir=""
camera_args=()
mqtt_host="${MQTT_HOST:-localhost}"
mqtt_port="${MQTT_PORT:-1883}"
mqtt_topic="${MQTT_TOPIC:-slam/cameras}"
mqtt_username=""
mqtt_password=""
ceiling_cameras=true
elevator_lift=""

usage() {
  cat <<'EOF'
Usage: ./scripts/run_multi_nav_online.sh [OPTIONS]

Launch Isaac Sim with several robots. Each robot runs its own 3D localization
and Nav2 stack in its own container (Compose project isaacsim-fastlio2-NAME),
under /NAME (topics, actions and the TF tree /NAME/tf), like separate robots.
One fleet RViz (container isaacsim-fastlio2-fleet-rviz) shows all robots.
Its floor selector loads the 1F and B1 PGM maps by default, initially displaying 1F.

Same as run_multi_nav.sh, plus the outbound connections: the camera list is published
to MQTT, cameras are switched on/off over MQTT, and RTSP streams go to MediaMTX.

Options:
      --robot NAME[:TYPE][#FLEET]@X,Y[,YAW]
                       Spawn a robot of TYPE (config/robots/TYPE.yaml, default
                       nova_carter) at Isaac world X,Y (m), YAW (rad) of the
                       robot prim (world is aligned with the Office map).
                       Repeat per robot. Default: carter1:nova_carter#fleet1@0,0 and
                       carter2:carter_v1#fleet2@3.5,0.
  -m, --map FILE       Nav2 map YAML file (default: maps/1F/map_2d.yaml).
      --pcd FILE       PCD map (default: map.pcd beside the map YAML).
      --manual-initial-pose
                       Disable automatic spawn/session-derived initial poses;
                       show reference arrows until each manual pose is set.
      --no-rviz        Do not start the fleet RViz.
      --filter-editor  Shared Keepout/Speed zones per floor: annotate them in the
                       fleet RViz; every robot applies the zones of its current
                       floor and floor switches wait for that floor's zones. With
                       --no-rviz the saved zones are still served (no editing).
      --no-filter-editor
                       Disable shared zones (default unless run via
                       run_multi_nav_lift_online.sh).
      --filter-state-dir DIR
                       Zone state directory, one FLOOR.json per floor
                       (default: maps/costmap_filters/fleet).
      --headless       Run Isaac Sim without its GUI (default).
      --gui            Run Isaac Sim with its GUI.
      --camera [ROBOT=]CAMERA
                       Offer a robot camera as an RTSP stream
                       rtsp://HOST:8554/[ROBOT/]camera/NAME, rendered only after
                       true is published to MQTT <topic>/[ROBOT/]camera/NAME/enable.
                       CAMERA is a prim path relative to the robot (e.g.
                       chassis_link/sensors/front_owl/camera). Repeatable.
      --no-ceiling-cameras
                       Do not offer the ceiling cameras
                       (rtsp://HOST:8554/ceiling_cams/ceiling_cam_N). Each sits on a
                       pan/tilt gimbal: MQTT slam/gimbal/ceiling_cams/ceiling_cam_N/command.
      --camera-resolution WxH, --camera-fps N
                       Camera image size (default 640x480) and rate (default 15).
      --mqtt-host HOST, --mqtt-port N, --mqtt-topic T, --mqtt-username U, --mqtt-password P
                       MQTT broker (default $MQTT_HOST/localhost, $MQTT_PORT/1883) and
                       topic (default $MQTT_TOPIC/slam/cameras). The camera list is a
                       retained JSON message on T, updated when a camera is switched;
                       {"online":false} on shutdown. The broker may start later.
      --box X,Y[,SX,SY,SZ]
                       Place a static box obstacle at Office map X,Y (m).
      --elevator-lift NAME
                       Enable the moving Office elevator and let a lift controller
                       (Open-RMF's mqtt_lift_adapter) drive it over MQTT
                       rmf/lift/NAME/request and rmf/lift/NAME/state, landings 1F and B1.
  -h, --help           Show this help.

In RViz's Fleet Control panel, select a robot, click "Set navigation goal",
then click and drag on the map. The shared toolbar pose tools use that robot too.
Or send a goal directly:
  ros2 action send_goal /carter2/navigate_to_pose nav2_msgs/action/NavigateToPose ...

Examples:
  ./scripts/run_multi_nav_online.sh
  ./scripts/run_multi_nav_online.sh --robot a@0,0 --robot b@3.5,-2,1.57 --robot c@1,2
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
    --filter-editor) filter_editor=true; shift ;;
    --no-filter-editor) filter_editor=false; shift ;;
    --filter-state-dir) value "$@"; filter_state_dir="$2"; shift 2 ;;
    --headless) headless=true; shift ;;
    --gui) headless=false; shift ;;
    --no-ceiling-cameras) ceiling_cameras=false; shift ;;
    --ceiling-cameras) camera_args+=("$1"); shift ;;
    --mqtt-host) value "$@"; mqtt_host="$2"; shift 2 ;;
    --mqtt-port) value "$@"; mqtt_port="$2"; shift 2 ;;
    --mqtt-topic) value "$@"; mqtt_topic="$2"; shift 2 ;;
    --mqtt-username) value "$@"; mqtt_username="$2"; shift 2 ;;
    --mqtt-password) value "$@"; mqtt_password="$2"; shift 2 ;;
    --camera|--camera-resolution|--camera-fps)
      value "$@"; camera_args+=("$1" "$2"); shift 2 ;;
    --box) value "$@"; boxes+=(--box "$2"); shift 2 ;;
    --elevator-lift) value "$@"; elevator_lift="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ((${#robots[@]} == 0)); then robots=("carter1#fleet1@0,0,0" "carter2:carter_v1#fleet2@3.5,0,0"); fi
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

if [[ "$map_pgm" != /* ]]; then map_pgm="$project_dir/$map_pgm"; fi
if [[ -z "$map_pcd" ]]; then map_pcd="$(dirname "$map_pgm")/map.pcd"; fi
if [[ "$map_pcd" != /* ]]; then map_pcd="$project_dir/$map_pcd"; fi
if [[ ! -f "$map_pcd" || "$map_pcd" != *.pcd ]]; then
  echo "PCD map does not exist or is not a .pcd file: $map_pcd" >&2
  exit 1
fi
if [[ ! -f "$map_pgm" ]]; then
  echo "PGM map YAML does not exist: $map_pgm" >&2
  exit 1
fi
if [[ "$auto_initial_pose" == true && "$(basename "$(dirname "$map_pgm")")" != office &&
      ! -f "$(dirname "$map_pgm")/session.json" ]]; then
  echo "Automatic initial poses require $(dirname "$map_pgm")/session.json; use --manual-initial-pose." >&2
  exit 1
fi
if [[ -n "$filter_state_dir" && "$filter_editor" != true ]]; then
  echo "--filter-state-dir requires --filter-editor" >&2
  exit 2
fi
if [[ -z "$filter_state_dir" ]]; then filter_state_dir="$project_dir/maps/costmap_filters/fleet"; fi
if [[ "$filter_state_dir" != /* ]]; then filter_state_dir="$project_dir/$filter_state_dir"; fi
filter_state_dir="$(container_path "$filter_state_dir")"
simulation_reference=false
if [[ "$auto_initial_pose" == false ]]; then simulation_reference=true; fi
require_ros_workspace
set -u

pose_mqtt_args=(mqtt_host:="$mqtt_host" mqtt_port:="$mqtt_port")
if [[ -n "$mqtt_username" ]]; then
  pose_mqtt_args+=(mqtt_username:="$mqtt_username" mqtt_password:="$mqtt_password")
fi
names=()
for robot in "${robots[@]}"; do
  name="${robot%%@*}"
  name="${name%%#*}"
  name="${name%%:*}"
  names+=("$name")
  start_ros_container "$name" launch slam_bringup robot.launch.py "robot:=$robot" \
    map_pcd:="$(container_path "$map_pcd")" map_pgm:="$(container_path "$map_pgm")" \
    rviz:=false auto_initial_pose:="$auto_initial_pose" fleet_filters:="$filter_editor" \
    "${pose_mqtt_args[@]}"
done
fleet_args=(rviz:="$rviz" filter_editor:="$filter_editor")
if [[ "$filter_editor" == true ]]; then
  fleet_args+=(filter_state_dir:="$filter_state_dir")
fi
# Zone editors run beside the fleet RViz; robots wait for their masks.
if [[ "$rviz" == true || "$filter_editor" == true ]]; then
  start_ros_container fleet-rviz launch slam_localization_3d fleet_rviz.launch.py \
    "${fleet_args[@]}" \
    "robots:=$(IFS=';'; printf '%s' "${robots[*]}")" \
    maps_root:="$(container_path "$(dirname "$(dirname "$map_pgm")")")" \
    "floors:=1F;B1" \
    simulation_reference:="$simulation_reference" \
    initial_floor:="$(basename "$(dirname "$map_pgm")")"
fi
if [[ "$ceiling_cameras" == true ]]; then
  # One gimbal action server + MQTT bridge per ceiling camera.
  start_ros_container ceiling-gimbals launch slam_bringup ceiling_cameras.launch.py \
    "${pose_mqtt_args[@]}"
fi

isaac_args=(--ros-cmd-vel --lidar-motion-compensation noncompensated)
if [[ "$headless" == true ]]; then isaac_args+=(--headless); fi
for robot in "${robots[@]}"; do isaac_args+=(--robot "$robot"); done
isaac_args+=("${boxes[@]}" "${camera_args[@]}" --mqtt-host "$mqtt_host" --mqtt-port "$mqtt_port" --mqtt-topic "$mqtt_topic")
if [[ -n "$mqtt_username" ]]; then isaac_args+=(--mqtt-username "$mqtt_username" --mqtt-password "$mqtt_password"); fi
if [[ "$ceiling_cameras" == true ]]; then isaac_args+=(--ceiling-cameras); fi
if [[ -n "$elevator_lift" ]]; then
  isaac_args+=(--dynamic-elevator-doors --elevator-mqtt-lift "$elevator_lift")
fi
"$project_dir/scripts/standalone.py" "${isaac_args[@]}"
