#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
map_file="$project_dir/maps/office/map_2d.yaml"
headless=false
rviz=true
auto_jog=false
boxes=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_2d_localization.sh [OPTIONS]

Launch Isaac Sim and the 2D AMCL localization stack.

Options:
  -m, --map FILE       Nav2 map YAML file. The YAML selects its PGM image.
                       Default: maps/office/map_2d.yaml
      --headless       Run Isaac Sim without its GUI.
      --auto-jog       Drive Carter automatically for localization testing.
      --no-rviz        Do not start RViz.
      --box X,Y[,SX,SY,SZ]
                       Place a static box obstacle in the scene at Office map
                       X,Y (m); default size 0.6x0.6x1.0 m. Repeatable.
  -h, --help           Show this help.

Examples:
  ./scripts/run_2d_localization.sh
  ./scripts/run_2d_localization.sh --map maps/warehouse/map.yaml
  ./scripts/run_2d_localization.sh --headless --auto-jog --no-rviz
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
    --box)
      if (($# < 2)); then
        echo "Missing value for --box" >&2
        usage >&2
        exit 2
      fi
      boxes+=(--box "$2")
      shift 2
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
container_map="$(container_path "$map_file")"
container_path "$map_image" >/dev/null
require_ros_workspace
set -u

start_ros launch slam_localization_2d localization_2d.launch.py \
  map:="$container_map" \
  rviz:="$rviz"

isaac_args=(--lidar-motion-compensation compensated)
if [[ "$headless" == true ]]; then
  isaac_args+=(--headless)
fi
if [[ "$auto_jog" == true ]]; then
  isaac_args+=(--auto-jog)
fi
isaac_args+=("${boxes[@]}")

"$project_dir/scripts/standalone.py" "${isaac_args[@]}"
