#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode=""
map_file="$project_dir/maps/office/map_2d.yaml"
map_pcd=""
headless=false
auto_jog=false
rviz=true
manual_initial_pose=false
navigate=false
adaptive_surround=false
static_zones=false
filter_editor=false
filter_state=""
boxes=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_nav.sh --mode {2d|3d} [OPTIONS]

Launch Isaac Sim with exactly one localization mode.

Options:
      --mode MODE      Required: 2d (AMCL), or 3d (PCD fusion and costmaps).
  -m, --map FILE       Nav2 map YAML file (default: maps/office/map_2d.yaml).
      --pcd FILE       3d only: PCD map (default: maps/office/map.pcd).
      --manual-initial-pose
                       3d only: set initial pose in RViz instead of Office spawn.
      --navigate       3d only: enable low-speed Nav2 navigation from RViz goals.
      --adaptive-surround
                       Experimental speed-adaptive Surround; requires --navigate.
      --static-zones   Keep all collision zones active in every direction
                       (default switches forward/reverse/rotate zone sets).
      --filter-editor  3d only: annotate Keepout/Speed polygons live in RViz.
      --filter-state FILE
                       Persistent editor JSON (default: maps/costmap_filters/editor.json).
      --headless       Run Isaac Sim without its GUI.
      --auto-jog       Drive Carter automatically for localization testing.
      --no-rviz        Do not start RViz.
      --box X,Y[,SX,SY,SZ]
                       Place a static box obstacle in the Isaac Sim scene at
                       Office map coordinates X,Y (m). Default size
                       0.6x0.6x1.0 m. Repeat for more boxes.
  -h, --help           Show this help.

Examples:
  ./scripts/run_nav.sh --mode 2d
  ./scripts/run_nav.sh --mode 3d --headless --no-rviz
  ./scripts/run_nav.sh --mode 3d --navigate --box 2.0,0.0
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
    --filter-editor) filter_editor=true; shift ;;
    --filter-state)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      filter_state="$2"
      shift 2
      ;;
    --navigate) navigate=true; shift ;;
    --adaptive-surround) adaptive_surround=true; shift ;;
    --static-zones) static_zones=true; shift ;;
    --headless) headless=true; shift ;;
    --auto-jog) auto_jog=true; shift ;;
    --no-rviz) rviz=false; shift ;;
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

if [[ "$mode" != 2d && "$mode" != 3d ]]; then
  echo "Specify --mode 2d or --mode 3d" >&2
  usage >&2
  exit 2
fi
if [[ "$adaptive_surround" == true && ( "$mode" != 3d || "$navigate" != true ) ]]; then
  echo "--adaptive-surround requires --mode 3d --navigate" >&2
  exit 2
fi
args=()
if [[ "$headless" == true ]]; then args+=(--headless); fi
if [[ "$auto_jog" == true ]]; then args+=(--auto-jog); fi
if [[ "$rviz" == false ]]; then args+=(--no-rviz); fi
args+=("${boxes[@]}")
if [[ "$mode" == 2d ]]; then
  if [[ -n "$map_pcd" ||
        -n "$filter_state" || "$filter_editor" == true ||
        "$manual_initial_pose" == true || "$navigate" == true ||
        "$static_zones" == true ]]; then
    echo "--pcd, --manual-initial-pose, --navigate, --static-zones and costmap filters require --mode 3d" >&2
    exit 2
  fi
  exec "$project_dir/scripts/run_2d_localization.sh" --map "$map_file" "${args[@]}"
fi
if [[ -n "$map_pcd" ]]; then args+=(--pcd "$map_pcd"); fi
if [[ "$manual_initial_pose" == true ]]; then args+=(--manual-initial-pose); fi
if [[ "$navigate" == true ]]; then args+=(--navigate); fi
if [[ "$adaptive_surround" == true ]]; then args+=(--adaptive-surround); fi
if [[ "$static_zones" == true ]]; then args+=(--static-zones); fi
if [[ "$filter_editor" == true ]]; then args+=(--filter-editor); fi
if [[ -n "$filter_state" ]]; then args+=(--filter-state "$filter_state"); fi
exec "$project_dir/scripts/run_3d_localization.sh" --global-fusion --costmaps \
  --pgm "$map_file" "${args[@]}"
