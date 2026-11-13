#!/usr/bin/env bash
# Starts the m20_policy_node + elevation_mapping_cupy + FAST-LIO2 in the height_map container
# and Isaac Sim m20_standalone.py --external on the host (manual jog control, no Nav2).
#   scripts/run_m20_policy.sh --policy /path/to/policy.pt [--policy-device cpu|cuda] [--build] [--no-rviz] [--no-jog] [m20_standalone.py args, e.g. --headless --device cpu]
# The policy is TorchScript exported by `isaaclab play` (logs/rsl_rl/m20_rough/<run>/exported/policy.pt).
# The Qt jog panel (m20_jog) starts too and provides /cmd_vel; --no-jog skips it (the sim then publishes a zero /cmd_vel).
# Stops everything on Ctrl+C.
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$project_dir"

export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/docker/fastdds_udp.xml"

build=false
rviz=true
jog=true
policy=""
policy_device=cpu
sim_args=()
while (($# > 0)); do
  case "$1" in
    --policy) policy="${2:?--policy needs a path}"; shift 2 ;;
    --build) build=true; shift ;;
    --no-rviz) rviz=false; shift ;;
    --no-jog) jog=false; shift ;;
    --policy-device) policy_device="${2:?--policy-device needs cpu or cuda}"; shift 2 ;;
    *) sim_args+=("$1"); shift ;;
  esac
done

if [[ -z "$policy" || ! -f "$policy" ]]; then
  echo "Usage: $0 --policy /path/to/policy.pt [--policy-device cpu|cuda] [--build] [--no-rviz] [m20_standalone.py args...]" >&2
  exit 2
fi
policy="$(readlink -f "$policy")"
# the policy directory is mounted read-only at /policies in the container
export POLICY_DIR="$(dirname "$policy")"
export ELEVATION_CONFIG="${ELEVATION_CONFIG:-/workspace/config/elevation_isaac.yaml}"
export HEIGHT_MAP_RVIZ_CONFIG="${HEIGHT_MAP_RVIZ_CONFIG:-/workspace/config/height_map_jog.rviz}"

if [[ "$build" == true || ! -f ros2_ws/install/m20_policy_node/share/m20_policy_node/package.xml || ! -f ros2_ws/install/m20_jog/share/m20_jog/package.xml ]]; then
  docker compose --profile height_map run --rm height_map build
fi

if [[ ( "$rviz" == true || "$jog" == true ) && -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

cleanup() {
  trap - EXIT INT TERM
  [[ -n "${sim_pid:-}" ]] && kill "$sim_pid" 2>/dev/null || true
  docker compose --profile height_map down --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

HEIGHT_MAP_COMMAND=launch \
HEIGHT_MAP_LAUNCH_ARGS=$'m20_policy_node\nm20_policy.launch.py\npolicy_path:=/policies/'"$(basename "$policy")"$'\ndevice:='"$policy_device"$'\nuse_sim_time:=true\nrviz:='"$rviz"$'\nrviz_config:='"$HEIGHT_MAP_RVIZ_CONFIG"$'\nflat_height_scan:='"${FLAT_HEIGHT_SCAN:-false}" \
  docker compose --profile height_map up -d --force-recreate height_map

if [[ "$jog" == true ]]; then
  docker compose --profile height_map exec -d height_map /workspace/docker/entrypoint.sh ros2 run m20_jog m20_jog
  sim_args+=(--no-cmd-vel)
fi

docker compose --profile height_map logs -f --no-log-prefix height_map &

isaacsim_dir="${ISAACSIM_DIR:-$HOME/isaacsim-6.2.0}"
ros_distro="${ISAAC_ROS_DISTRO:-humble}"
sim_ld_path="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | { grep -v '^/opt/ros' || true; } | paste -sd:)"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
  ROS_DISTRO="$ros_distro" RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="$sim_ld_path:$isaacsim_dir/exts/isaacsim.ros2.core/$ros_distro/lib" \
  "$isaacsim_dir/python.sh" "$project_dir/scripts/m20_standalone.py" --external "${sim_args[@]}" &
sim_pid=$!
wait "$sim_pid"
