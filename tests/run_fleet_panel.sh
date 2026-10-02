#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/docker/ros_compose.sh"
require_ros_workspace
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-94}"
export ROS_LOCALHOST_ONLY=1

# Requires a graphical display and a workspace built with BUILD_TESTING enabled.
ros_compose -p isaacsim-fleet-panel-test run --rm --no-deps -T --entrypoint bash ros -lc \
  'source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && python3 -' <<'PY'
import pathlib
import subprocess
import sys
import tempfile

import yaml

sys.path.insert(0, "ros2_ws/src/isaac_localization_3d/launch")
from robot_fleet import fleet_rviz

executable = pathlib.Path("ros2_ws/build/isaac_localization_3d/test_fleet_panel")
if not executable.is_file():
    raise SystemExit("Build the workspace with BUILD_TESTING enabled before running this test.")
with tempfile.TemporaryDirectory(prefix="fleet_panel_") as directory:
    config = pathlib.Path(directory) / "fleet.rviz"
    config.write_text(yaml.safe_dump(fleet_rviz(["carter1", "carter2"]), sort_keys=False))
    result = subprocess.run([str(executable), str(config)], timeout=90, check=False)
    sys.exit(result.returncode)
PY
