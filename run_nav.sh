#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode=""
map_file="$project_dir/maps/office/map_2d.yaml"
map_pcd=""
headless=false
auto_jog=false
rviz=true
manual_initial_pose=false

usage() {
  cat <<'EOF'
Usage: ./run_nav.sh --mode {2d|3d} [OPTIONS]

Launch Isaac Sim with exactly one localization mode.

Options:
      --mode MODE      Required: 2d (AMCL), or 3d (PCD fusion and costmaps).
  -m, --map FILE       Nav2 map YAML file (default: maps/office/map_2d.yaml).
      --pcd FILE       3d only: PCD map (default: maps/office/map.pcd).
      --manual-initial-pose
                       3d only: set initial pose in RViz instead of Office spawn.
      --headless       Run Isaac Sim without its GUI.
      --auto-jog       Drive Carter automatically for localization testing.
      --no-rviz        Do not start RViz.
  -h, --help           Show this help.

Examples:
  ./run_nav.sh --mode 2d
  ./run_nav.sh --mode 3d --headless --no-rviz
EOF
}

while (($# > 0)); do
  case "$1" in
    --mode)
      if (($# < 2)) || [[ -n "$mode" ]]; then
        echo "--mode requires one value and may only be specified once" >&2
        exit 2
      fi
      mode="$2"
      shift 2
      ;;
    -m|--map)
      if (($# < 2)); then
        echo "Missing value for $1" >&2
        exit 2
      fi
      map_file="$2"
      shift 2
      ;;
    --pcd)
      if (($# < 2)); then
        echo "Missing value for --pcd" >&2
        exit 2
      fi
      map_pcd="$2"
      shift 2
      ;;
    --manual-initial-pose) manual_initial_pose=true; shift ;;
    --headless) headless=true; shift ;;
    --auto-jog) auto_jog=true; shift ;;
    --no-rviz) rviz=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$mode" != 2d && "$mode" != 3d ]]; then
  echo "Specify --mode 2d or --mode 3d" >&2
  usage >&2
  exit 2
fi
args=()
if [[ "$headless" == true ]]; then args+=(--headless); fi
if [[ "$auto_jog" == true ]]; then args+=(--auto-jog); fi
if [[ "$rviz" == false ]]; then args+=(--no-rviz); fi
if [[ "$mode" == 2d ]]; then
  if [[ -n "$map_pcd" || "$manual_initial_pose" == true ]]; then
    echo "--pcd and --manual-initial-pose require --mode 3d" >&2
    exit 2
  fi
  exec "$project_dir/run_2d_localization.sh" --map "$map_file" "${args[@]}"
fi
if [[ -n "$map_pcd" ]]; then args+=(--pcd "$map_pcd"); fi
if [[ "$manual_initial_pose" == true ]]; then args+=(--manual-initial-pose); fi
exec "$project_dir/run_3d_localization.sh" --global-fusion --costmaps \
  --pgm "$map_file" "${args[@]}"
