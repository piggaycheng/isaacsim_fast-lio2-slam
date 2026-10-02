#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
map_pcd="$project_dir/maps/office/map.pcd"
map_pgm="$project_dir/maps/office/map_2d.yaml"
headless=false
rviz=true
auto_jog=false
auto_initial_pose=true
global_fusion=false
obstacle_cloud=false
costmaps=false
navigate=false
adaptive_surround=false
filter_editor=false
filter_state="$project_dir/maps/costmap_filters/editor.json"
filter_state_set=false
ros_cmd_vel=false
boxes=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_3d_localization.sh [OPTIONS]

Run Isaac Sim Office and isolated FAST_LIO_LOCALIZATION2 with a PGM map in RViz.
Do not run this alongside scripts/run_nav.sh.

Options:
      --pcd FILE      PCD map used by 3D ICP (default: maps/office/map.pcd).
      --pgm FILE      Nav2 YAML for the RViz PGM map (default: maps/office/map_2d.yaml).
      --headless      Run Isaac Sim without its GUI.
      --auto-jog      Move Carter automatically instead of using W/S/A/D.
      --manual-initial-pose
                      Wait for RViz's 2D Pose Estimate instead of using the
                      Office spawn near (0, 0). Required for other maps.
      --no-rviz       Run without RViz.
      --global-fusion Run isolated PCD + wheel/IMU dual-EKF fusion instead of
                      the original 3D-only visualization (no AMCL or Nav2).
      --obstacle-cloud
                      With --global-fusion, filter ground from the 3D LiDAR
                      and publish /perception/obstacles (not a Nav2 costmap).
      --costmaps      With --global-fusion, observe PGM-based global and
                      3D-obstacle local Nav2 costmaps (no autonomous driving).
      --navigate      With --global-fusion, start low-speed Nav2 navigation
                      from RViz goals and accept /cmd_vel (no automatic goal).
      --filter-editor Annotate Keepout/Speed polygons live in RViz;
                      requires fusion and costmaps.
      --filter-state FILE
                      Persistent editor JSON inside the project (default:
                      maps/costmap_filters/editor.json).
      --adaptive-surround
                      Experimental low-speed Surround; requires --global-fusion --navigate.
      --ros-cmd-vel   Drive Carter from ROS 2 /cmd_vel without Nav2, e.g. for
                      covariance_drive.py calibration runs.
      --box X,Y[,SX,SY,SZ]
                      Place a static box obstacle in the scene at Office map
                      X,Y (m); default size 0.6x0.6x1.0 m. Repeatable.
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
    --auto-jog) auto_jog=true; shift ;;
    --manual-initial-pose) auto_initial_pose=false; shift ;;
    --no-rviz) rviz=false; shift ;;
    --global-fusion) global_fusion=true; shift ;;
    --obstacle-cloud) obstacle_cloud=true; shift ;;
    --costmaps) costmaps=true; obstacle_cloud=true; shift ;;
    --navigate) navigate=true; costmaps=true; obstacle_cloud=true; shift ;;
    --adaptive-surround) adaptive_surround=true; shift ;;
    --ros-cmd-vel) ros_cmd_vel=true; shift ;;
    --box)
      if (($# < 2)); then
        echo "Missing value for --box" >&2
        exit 2
      fi
      boxes+=(--box "$2")
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ "$adaptive_surround" == true &&
      ( "$global_fusion" != true || "$navigate" != true ) ]]; then
  echo "--adaptive-surround requires --global-fusion --navigate" >&2
  exit 2
fi
if [[ "$filter_editor" == true &&
      ( "$global_fusion" != true || "$costmaps" != true ) ]]; then
  echo "Costmap filters require --global-fusion and --costmaps or --navigate" >&2
  exit 2
fi
if [[ "$filter_state_set" == true && "$filter_editor" != true ]]; then
  echo "--filter-state requires --filter-editor" >&2
  exit 2
fi
if [[ "$obstacle_cloud" == true && "$global_fusion" != true ]]; then
  echo "--obstacle-cloud, --costmaps and --navigate require --global-fusion" >&2
  exit 2
fi
if [[ ( "$navigate" == true || "$ros_cmd_vel" == true ) && "$auto_jog" == true ]]; then
  echo "--auto-jog cannot be combined with --navigate or --ros-cmd-vel" >&2
  exit 2
fi

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
container_pcd="$(container_path "$map_pcd")"
container_pgm="$(container_path "$map_pgm")"
container_path "$map_image" >/dev/null
filter_args=()
if [[ "$filter_editor" == true ]]; then
  if [[ "$filter_state" != /* ]]; then filter_state="$project_dir/$filter_state"; fi
  filter_args+=(filter_editor:=true "filter_state:=$(container_path "$filter_state")")
fi
require_ros_workspace
set -u

launch_file=localization_3d.launch.py
launch_args=(map_pcd:="$container_pcd" map_pgm:="$container_pgm" rviz:="$rviz"
  auto_initial_pose:="$auto_initial_pose")
if [[ "$global_fusion" == true ]]; then
  launch_file=global_fusion.launch.py
  launch_args+=(obstacle_cloud:="$obstacle_cloud" costmaps:="$costmaps"
    navigate:="$navigate" adaptive_surround:="$adaptive_surround" "${filter_args[@]}")
fi
start_ros launch slam_localization_3d "$launch_file" "${launch_args[@]}"

isaac_args=(--lidar-motion-compensation noncompensated)
if [[ "$headless" == true ]]; then isaac_args+=(--headless); fi
if [[ "$auto_jog" == true ]]; then isaac_args+=(--auto-jog); fi
if [[ "$navigate" == true || "$ros_cmd_vel" == true ]]; then isaac_args+=(--ros-cmd-vel); fi
isaac_args+=("${boxes[@]}")
"$project_dir/scripts/standalone.py" "${isaac_args[@]}"
