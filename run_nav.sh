#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$project_dir/ros2_ws"
nav_prefix="$workspace_dir/nav_install/opt/ros/humble"
nav_root="$workspace_dir/nav_install"
map_file="$project_dir/maps/office/map_2d.yaml"
headless=false
rviz=true
auto_jog=false

usage() {
  cat <<'EOF'
Usage: ./run_nav.sh [OPTIONS]

Launch Isaac Sim and the 2D localization stack.

Options:
  -m, --map FILE       Nav2 map YAML file. The YAML selects its PGM image.
                       Default: maps/office/map_2d.yaml
      --headless       Run Isaac Sim without its GUI.
      --auto-jog       Drive Carter automatically for localization testing.
      --no-rviz        Do not start RViz.
  -h, --help           Show this help.

Examples:
  ./run_nav.sh
  ./run_nav.sh --map maps/warehouse/map.yaml
  ./run_nav.sh --headless --auto-jog --no-rviz
EOF
}

while (($# > 0)); do
  case "$1" in
    -m|--map)
      if (($# < 2)); then
        echo "Missing value for $1" >&2
        usage >&2
        exit 2
      fi
      map_file="$2"
      shift 2
      ;;
    --headless)
      headless=true
      shift
      ;;
    --auto-jog)
      auto_jog=true
      shift
      ;;
    --no-rviz)
      rviz=false
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

if [[ "$map_file" != /* ]]; then
  map_file="$project_dir/$map_file"
fi
if [[ ! -f "$map_file" ]]; then
  echo "Map YAML does not exist: $map_file" >&2
  exit 1
fi
if [[ "$map_file" != *.yaml && "$map_file" != *.yml ]]; then
  echo "--map must point to a Nav2 YAML file, not directly to a PGM: $map_file" >&2
  exit 1
fi

map_image="$(sed -n 's/^[[:space:]]*image:[[:space:]]*//p' "$map_file" | head -1)"
map_image="${map_image%\"}"
map_image="${map_image#\"}"
map_image="${map_image%\'}"
map_image="${map_image#\'}"
if [[ -z "$map_image" ]]; then
  echo "Map YAML has no image entry: $map_file" >&2
  exit 1
fi
if [[ "$map_image" != /* ]]; then
  map_image="$(dirname "$map_file")/$map_image"
fi
if [[ ! -f "$map_image" ]]; then
  echo "PGM referenced by the map YAML does not exist: $map_image" >&2
  exit 1
fi
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "ROS workspace is not built. Run ros2_ws/build_workspace.sh first." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -d "$nav_prefix" ]]; then
  export AMENT_PREFIX_PATH="$nav_prefix:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$nav_prefix:${CMAKE_PREFIX_PATH:-}"
  export PATH="$nav_prefix/bin:$nav_prefix/lib/nav2_amcl:$nav_prefix/lib/nav2_lifecycle_manager:$nav_prefix/lib/nav2_map_server:$nav_prefix/lib/pointcloud_to_laserscan:$nav_prefix/lib/robot_localization:$PATH"
  export LD_LIBRARY_PATH="$nav_prefix/lib:$nav_prefix/lib/x86_64-linux-gnu:$nav_root/usr/lib:$nav_root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$nav_prefix/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
source "$workspace_dir/install/setup.bash"
set -u

required_packages=(
  isaac_fastlio_adapter
  nav2_amcl
  nav2_lifecycle_manager
  nav2_map_server
  pointcloud_to_laserscan
  robot_localization
)
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Missing ROS package: $package" >&2
    echo "Run ros2_ws/install_nav_dependencies.sh and rebuild the workspace." >&2
    exit 1
  fi
done

ros2 launch isaac_fastlio_adapter localization_2d.launch.py \
  map:="$map_file" \
  rviz:="$rviz" &
ros_pid=$!

cleanup() {
  kill "$ros_pid" 2>/dev/null || true
  wait "$ros_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

isaac_args=(--lidar-motion-compensation compensated)
if [[ "$headless" == true ]]; then
  isaac_args+=(--headless)
fi
if [[ "$auto_jog" == true ]]; then
  isaac_args+=(--auto-jog)
fi

"$project_dir/standalone.py" "${isaac_args[@]}"
