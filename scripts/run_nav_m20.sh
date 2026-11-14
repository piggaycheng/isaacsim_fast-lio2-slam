#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$project_dir/docker/ros_compose.sh"

map_pgm="$project_dir/maps/1F/map_2d.yaml"
map_pcd=""
policy=""
policy_device=cpu
free_fly=false
build=false
headless=false
rviz=true
auto_initial_pose=false
direction_zones=true
filter_editor=false
filter_state="$project_dir/maps/costmap_filters/editor.json"
filter_state_set=false
flat_height_scan="${FLAT_HEIGHT_SCAN:-false}"
mqtt_host=""
mqtt_port=1883
boxes=()
sim_args=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_nav_m20.sh (--policy /path/to/policy.pt | --free-fly) [OPTIONS] [-- M20_SIM_ARGS...]

Launch Isaac Sim with the Deep Robotics M20 wheel-legged robot (or a terrain-
following free-fly M20 sensor rig when --free-fly is passed), 3D FAST-LIO2
localization + Nav2 navigation (in the `ros` container), and GPU elevation
mapping (+ RSL-RL locomotion policy inference when --policy is used, in the
`height_map` container).

Nav2 publishes velocity commands on /cmd_vel, which either m20_policy_node
(with --policy) or the terrain-following free-fly sensor rig (with --free-fly)
follows across floors and stairs.

Options:
      --policy FILE        TorchScript policy.pt exported by `isaaclab play`
                           (e.g. logs/rsl_rl/m20_rough/<run>/exported/policy.pt).
      --free-fly           Policy-less mode: spawn a vehicle-less M20 sensor rig
                           at M20 standing height (0.49 m) that follows ground/
                           stair height automatically and drives from /cmd_vel
                           (plus optional W/S/A/D + Q/E keyboard override).
      --policy-device DEV  Device for m20_policy_node inference: cpu (default)
                           or cuda.
      --build              Build ros2_ws in the height_map container before starting.
  -m, --map FILE           Nav2 map YAML file (default: maps/1F/map_2d.yaml).
      --pcd FILE           PCD map used by 3D ICP (default: map.pcd beside the map YAML).
      --manual-initial-pose
                           Wait for RViz's 2D Pose Estimate instead of using the
                           Office spawn near (0, 0). Required for non-Office maps.
      --static-zones       Keep all collision zones active in every direction
                           (default switches forward/reverse/rotate zone sets).
      --filter-editor      Annotate Keepout/Speed polygons live in RViz.
      --filter-state FILE  Persistent editor JSON (default: maps/costmap_filters/editor.json).
      --mqtt-host HOST     Switch floors by MQTT change_floor on
                           rmf/manual/robot/carter/command (see docs/nav.md).
      --mqtt-port PORT     MQTT broker port (default: 1883).
      --flat-height-scan   Debug: ignore the elevation map and feed flat ground
                           to the policy.
      --headless           Run Isaac Sim without its GUI.
      --no-rviz            Do not start RViz.
      --box X,Y[,SX,SY,SZ] Place a static box obstacle in the Isaac Sim scene at
                           Office map coordinates X,Y (m). Default size
                           0.6x0.6x1.0 m. Repeat for more boxes.
  -h, --help               Show this help.

Examples:
  ./scripts/run_nav_m20.sh --policy /path/to/policy.pt
  ./scripts/run_nav_m20.sh --free-fly
  ./scripts/run_nav_m20.sh --policy /path/to/policy.pt --headless --no-rviz
  ./scripts/run_nav_m20.sh --policy /path/to/policy.pt --box 2.0,0.0
EOF
}

while (($# > 0)); do
  case "$1" in
    --policy)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for --policy" >&2
        exit 2
      fi
      policy="$2"
      shift 2
      ;;
    --free-fly)
      free_fly=true
      shift
      ;;
    --policy-device)
      if (($# < 2)) || [[ "$2" != cpu && "$2" != cuda ]]; then
        echo "--policy-device requires cpu or cuda" >&2
        exit 2
      fi
      policy_device="$2"
      shift 2
      ;;
    --build) build=true; shift ;;
    -m|--map|--pgm)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      map_pgm="$2"
      shift 2
      ;;
    --pcd)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for --pcd" >&2
        exit 2
      fi
      map_pcd="$2"
      shift 2
      ;;
    --manual-initial-pose) auto_initial_pose=false; shift ;;
    --static-zones) direction_zones=false; shift ;;
    --filter-editor) filter_editor=true; shift ;;
    --filter-state)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      filter_state="$2"
      filter_state_set=true
      shift 2
      ;;
    --mqtt-host|--mqtt-port)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      if [[ "$1" == --mqtt-host ]]; then mqtt_host="$2"; else mqtt_port="$2"; fi
      shift 2
      ;;
    --flat-height-scan) flat_height_scan=true; shift ;;
    --headless) headless=true; shift ;;
    --no-rviz) rviz=false; shift ;;
    --box)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for --box" >&2
        exit 2
      fi
      boxes+=(--box "$2")
      shift 2
      ;;
    --)
      shift
      sim_args+=("$@")
      break
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$free_fly" == true && -n "$policy" ]]; then
  echo "--free-fly and --policy cannot be combined" >&2
  exit 2
fi
if [[ "$free_fly" != true ]]; then
  if [[ -z "$policy" ]]; then
    echo "--policy /path/to/policy.pt (or --free-fly) is required" >&2
    usage >&2
    exit 2
  fi
  if [[ ! -f "$policy" ]]; then
    echo "Policy file does not exist: $policy" >&2
    exit 1
  fi
fi
if [[ "$filter_state_set" == true && "$filter_editor" != true ]]; then
  echo "--filter-state requires --filter-editor" >&2
  exit 2
fi
if [[ ! "$mqtt_port" =~ ^[0-9]+$ ]] || ((mqtt_port < 1 || mqtt_port > 65535)); then
  echo "--mqtt-port must be 1-65535" >&2
  exit 2
fi

if [[ "$map_pgm" != /* ]]; then map_pgm="$project_dir/$map_pgm"; fi
if [[ -z "$map_pcd" ]]; then map_pcd="$(dirname "$map_pgm")/map.pcd"; fi
if [[ "$map_pcd" != /* ]]; then map_pcd="$project_dir/$map_pcd"; fi
if [[ ! -f "$map_pgm" || ( "$map_pgm" != *.yaml && "$map_pgm" != *.yml ) ]]; then
  echo "PGM map YAML does not exist: $map_pgm" >&2
  exit 1
fi
if [[ ! -f "$map_pcd" || "$map_pcd" != *.pcd ]]; then
  echo "PCD map does not exist or is not a .pcd file: $map_pcd" >&2
  exit 1
fi
map_image="$(sed -n 's/^[[:space:]]*image:[[:space:]]*//p' "$map_pgm" | head -1)"
map_image="${map_image%\"}"
map_image="${map_image#\"}"
map_image="${map_image%\'}"
map_image="${map_image#\'}"
if [[ -z "$map_image" ]]; then
  echo "Map YAML has no image entry: $map_pgm" >&2
  exit 1
fi
if [[ "$map_image" != /* ]]; then map_image="$(dirname "$map_pgm")/$map_image"; fi
if [[ ! -f "$map_image" ]]; then
  echo "PGM referenced by map YAML does not exist: $map_image" >&2
  exit 1
fi

container_pcd="$(container_path "$map_pcd")"
container_pgm="$(container_path "$map_pgm")"
container_path "$map_image" >/dev/null
filter_args=()
if [[ "$filter_editor" == true ]]; then
  if [[ "$filter_state" != /* ]]; then filter_state="$project_dir/$filter_state"; fi
  filter_args+=(filter_editor:=true "filter_state:=$(container_path "$filter_state")")
fi

if [[ "$free_fly" != true ]]; then
  policy="$(readlink -f "$policy")"
  export POLICY_DIR="$(dirname "$policy")"
fi
export ELEVATION_CONFIG="/workspace/ros2_ws/src/slam_localization_3d/config/elevation_m20_nav.yaml"

if [[ "$build" == true ]]; then
  ros_compose --profile height_map run --rm height_map build
fi
require_height_map_workspace
set -u

if [[ "$rviz" == true && -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

launch_args=(
  robot_type:=m20
  map_pcd:="$container_pcd"
  map_pgm:="$container_pgm"
  rviz:="$rviz"
  auto_initial_pose:="$auto_initial_pose"
  obstacle_cloud:=true
  costmaps:=true
  navigate:=true
  direction_zones:="$direction_zones"
  "${filter_args[@]}"
)
if [[ -n "$mqtt_host" ]]; then
  launch_args+=(floor_mqtt_host:="$mqtt_host" floor_mqtt_port:="$mqtt_port")
fi

start_ros launch slam_localization_3d global_fusion.launch.py "${launch_args[@]}"
if [[ "$free_fly" == true ]]; then
  start_height_map launch isaac_fastlio_adapter height_map.launch.py \
    use_sim_time:=true \
    fastlio:=false \
    "rviz:=$rviz" \
    "elevation_config:=$ELEVATION_CONFIG"
  isaac_args=(--free-fly --no-cmd-vel --no-tf)
else
  start_height_map launch m20_policy_node m20_policy.launch.py \
    "policy_path:=/policies/$(basename "$policy")" \
    "device:=$policy_device" \
    use_sim_time:=true \
    fastlio:=false \
    "rviz:=$rviz" \
    "elevation_config:=$ELEVATION_CONFIG" \
    "flat_height_scan:=$flat_height_scan"
  isaac_args=(--external --no-cmd-vel --no-tf)
fi
if [[ "$headless" == true ]]; then isaac_args+=(--headless); fi
isaac_args+=("${boxes[@]}" "${sim_args[@]}")

isaac_python="$(isaac_python_path)"
isaacsim_dir="${ISAACSIM_DIR:-$(dirname "$isaac_python")}"
ros_distro="${ISAAC_ROS_DISTRO:-humble}"
sim_ld_path="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | { grep -v '^/opt/ros' || true; } | paste -sd:)"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
  ROS_DISTRO="$ros_distro" RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="$sim_ld_path:$isaacsim_dir/exts/isaacsim.ros2.core/$ros_distro/lib" \
  "$isaac_python" "$project_dir/scripts/m20_standalone.py" "${isaac_args[@]}"
