#!/usr/bin/env bash
# Container entrypoint: sources ROS 2 and the workspace, then runs a command.
#   build        Build ros2_ws (extra arguments are passed to colcon build).
#   slam         Start FASTLIO2/PGO and RViz.
#   launch       Run `ros2 launch` with newline-separated $ROS_LAUNCH_ARGS.
#   idle         Keep the container running for `docker compose exec`.
#   <other>      Execute the command in the sourced environment.
set -eo pipefail

workspace_dir=/workspace/ros2_ws

source /opt/ros/humble/setup.bash
if [[ -f "$workspace_dir/install/setup.bash" ]]; then
  source "$workspace_dir/install/setup.bash"
fi

require_workspace() {
  if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
    echo "ROS workspace is not built. Run: docker compose run --rm ros build" >&2
    exit 1
  fi
}

command="${1:-idle}"
if (($# > 0)); then
  shift
fi

case "$command" in
  build)
    exec /workspace/docker/build_workspace.sh "$@"
    ;;
  idle)
    exec sleep infinity
    ;;
  launch)
    require_workspace
    mapfile -t launch_args < <(printf '%s' "${ROS_LAUNCH_ARGS:-}")
    if ((${#launch_args[@]} == 0)); then
      echo "ROS_LAUNCH_ARGS must contain the launch package and file." >&2
      exit 2
    fi
    exec ros2 launch "${launch_args[@]}"
    ;;
  slam)
    require_workspace
    ros2 launch isaac_fastlio_adapter fastlio.launch.py &
    ros_pid=$!
    rviz2 -d "$workspace_dir/src/isaac_fastlio_adapter/config/fastlio.rviz" &
    rviz_pid=$!
    # Background jobs of a non-interactive shell ignore SIGINT, so forward TERM.
    trap 'kill -TERM "$ros_pid" "$rviz_pid" 2>/dev/null || true' INT TERM
    wait "$ros_pid" || true
    kill -TERM "$ros_pid" "$rviz_pid" 2>/dev/null || true
    wait || true
    ;;
  *)
    exec "$command" "$@"
    ;;
esac
