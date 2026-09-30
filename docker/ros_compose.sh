# Shared helpers for the host-side run scripts. Source after setting
# project_dir. The ROS 2 stack runs in the `ros` Docker Compose service while
# Isaac Sim (standalone.py) runs on the host.

project_dir="$(cd "$project_dir" && pwd -P)"
workspace_dir="$project_dir/ros2_ws"
ros_logs_pid=""

ros_compose() {
  HOST_UID="$(id -u)" HOST_GID="$(id -g)" \
    docker compose --project-directory "$project_dir" \
    -f "$project_dir/docker-compose.yml" "$@"
}

require_ros_workspace() {
  if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
    echo "ROS workspace is not built. Run: docker compose run --rm ros build" >&2
    exit 1
  fi
}

# Prints the container path (/workspace/...) of a host file in the project.
container_path() {
  local path
  path="$(realpath -m "$1")"
  if [[ "$path" != "$project_dir"/* ]]; then
    echo "Path must be inside $project_dir to be visible in the ROS container: $1" >&2
    return 1
  fi
  printf '/workspace/%s\n' "${path#"$project_dir"/}"
}

stop_ros() {
  trap - EXIT INT TERM
  ros_compose down --remove-orphans >/dev/null 2>&1 || true
  if [[ -n "$ros_logs_pid" ]]; then
    kill "$ros_logs_pid" 2>/dev/null || true
    wait "$ros_logs_pid" 2>/dev/null || true
  fi
}

# start_ros COMMAND [LAUNCH_ARGS...]
# Starts the ROS container in the background, follows its logs, and stops it
# when the calling script exits.
start_ros() {
  export ROS_COMMAND="$1"
  shift
  ROS_LAUNCH_ARGS="$(printf '%s\n' "$@")"
  export ROS_LAUNCH_ARGS
  ros_compose up -d --force-recreate ros
  trap stop_ros EXIT INT TERM
  ros_compose logs -f --no-log-prefix ros &
  ros_logs_pid=$!
}
