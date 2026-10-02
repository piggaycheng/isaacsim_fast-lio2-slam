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

# One container per robot (as on real robots) plus e.g. one fleet RViz: each
# is its own Compose project, so containers, logs and lifecycles are independent.
ros_projects=()
ros_log_pids=()

ros_project() {
  printf '%s-%s\n' "${ROBOT_PROJECT_PREFIX:-isaacsim-fastlio2}" "${1,,}"
}

stop_ros_containers() {
  trap - EXIT INT TERM
  local project pid down_pids=()
  for project in "${ros_projects[@]}"; do
    ros_compose -p "$project" down --remove-orphans >/dev/null 2>&1 &
    down_pids+=("$!")
  done
  for pid in "${down_pids[@]}"; do wait "$pid" 2>/dev/null || true; done
  for pid in "${ros_log_pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}

# start_ros_container NAME COMMAND [LAUNCH_ARGS...]
# Starts container NAME (project <prefix>-NAME) in the background with its logs
# prefixed by [NAME]. All of them stop when the calling script exits.
start_ros_container() {
  local name="$1" command="$2" project
  shift 2
  project="$(ros_project "$name")"
  ros_projects+=("$project")
  trap stop_ros_containers EXIT INT TERM
  ROS_COMMAND="$command" ROS_LAUNCH_ARGS="$(printf '%s\n' "$@")" \
    ros_compose -p "$project" up -d --force-recreate ros
  ros_compose -p "$project" logs -f --no-log-prefix ros 2>&1 |
    sed -u "s/^/[$name] /" &
  ros_log_pids+=("$!")
}
