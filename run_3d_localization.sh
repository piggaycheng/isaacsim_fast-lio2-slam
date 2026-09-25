#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$project_dir/ros2_ws"
base="$workspace_dir/localization_3d_install"
nav_prefix="$workspace_dir/nav_install/opt/ros/humble"
nav_root="$workspace_dir/nav_install"
map_pcd="$project_dir/maps/office/map.pcd"
map_pgm="$project_dir/maps/office/map_2d.yaml"
headless=false
rviz=true
auto_jog=false
auto_initial_pose=true

usage() {
  cat <<'EOF'
Usage: ./run_3d_localization.sh [OPTIONS]

Run Isaac Sim Office and isolated FAST_LIO_LOCALIZATION2 with a PGM map in RViz.
Do not run this alongside run_nav.sh.

Options:
      --pcd FILE      PCD map used by 3D ICP (default: maps/office/map.pcd).
      --pgm FILE      Nav2 YAML for the RViz PGM map (default: maps/office/map_2d.yaml).
      --headless      Run Isaac Sim without its GUI.
      --auto-jog      Move Carter automatically instead of using W/S/A/D.
      --manual-initial-pose
                      Wait for RViz's 2D Pose Estimate instead of using the
                      Office spawn near (0, 0). Required for other maps.
      --no-rviz       Run without RViz.
  -h, --help          Show this help.
EOF
}

while (($# > 0)); do
  case "$1" in
    --pcd|--pgm)
      if (($# < 2)); then
        echo "Missing value for $1" >&2
        exit 2
      fi
      if [[ "$1" == --pcd ]]; then map_pcd="$2"; else map_pgm="$2"; fi
      shift 2
      ;;
    --headless) headless=true; shift ;;
    --auto-jog) auto_jog=true; shift ;;
    --manual-initial-pose) auto_initial_pose=false; shift ;;
    --no-rviz) rviz=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$map_pcd" != /* ]]; then map_pcd="$project_dir/$map_pcd"; fi
if [[ "$map_pgm" != /* ]]; then map_pgm="$project_dir/$map_pgm"; fi
if [[ "$map_pcd" != "$project_dir/maps/office/map.pcd" ||
      "$map_pgm" != "$project_dir/maps/office/map_2d.yaml" ]]; then
  auto_initial_pose=false
fi
if [[ ! -f "$map_pcd" || "$map_pcd" != *.pcd ]]; then
  echo "PCD map does not exist or is not a .pcd file: $map_pcd" >&2
  exit 1
fi
if [[ ! -f "$map_pgm" || ( "$map_pgm" != *.yaml && "$map_pgm" != *.yml ) ]]; then
  echo "PGM map YAML does not exist: $map_pgm" >&2
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
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Build the ROS workspace with ros2_ws/build_workspace.sh first." >&2
  exit 1
fi
if [[ ! -f "$base/install/setup.bash" || ! -x "$base/venv/bin/python" ||
      ! -d "$base/debs/opt/ros/humble" ]]; then
  echo "3D localization dependencies missing; run ros2_ws/setup_3d_localization.sh." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -d "$nav_prefix" ]]; then
  export AMENT_PREFIX_PATH="$nav_prefix:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$nav_prefix:${CMAKE_PREFIX_PATH:-}"
  export PATH="$nav_prefix/bin:$nav_prefix/lib/nav2_map_server:$nav_prefix/lib/nav2_lifecycle_manager:$PATH"
  export LD_LIBRARY_PATH="$nav_prefix/lib:$nav_prefix/lib/x86_64-linux-gnu:$nav_root/usr/lib:$nav_root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$nav_prefix/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
source "$workspace_dir/install/setup.bash"
source "$base/install/setup.bash"
export PATH="$base/venv/bin:$PATH"
export PYTHONPATH="$base/debs/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$base/debs/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
set -u

for package in isaac_localization_3d fast_lio_localization isaac_fastlio_adapter nav2_map_server nav2_lifecycle_manager; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Missing ROS package: $package. Run ros2_ws/install_nav_dependencies.sh and ros2_ws/setup_3d_localization.sh." >&2
    exit 1
  fi
done
if ! "$base/venv/bin/python" -c 'import open3d, ros2_numpy, tf_transformations'; then
  echo "3D Python dependencies unavailable; run ros2_ws/setup_3d_localization.sh." >&2
  exit 1
fi

ros2 launch isaac_localization_3d localization_3d.launch.py \
  map_pcd:="$map_pcd" map_pgm:="$map_pgm" rviz:="$rviz" \
  auto_initial_pose:="$auto_initial_pose" &
ros_pid=$!

cleanup() {
  kill "$ros_pid" 2>/dev/null || true
  wait "$ros_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

isaac_args=(--lidar-motion-compensation noncompensated)
if [[ "$headless" == true ]]; then isaac_args+=(--headless); fi
if [[ "$auto_jog" == true ]]; then isaac_args+=(--auto-jog); fi
"$project_dir/standalone.py" "${isaac_args[@]}"
