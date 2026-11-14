import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ScriptEntrypointsTest(unittest.TestCase):
    def test_ros_helper_exports_transport_profile_to_simulator_children(self):
        result = subprocess.run(
            ["bash", "-c",
             'project_dir="$1"; source "$project_dir/docker/ros_compose.sh"; '
             'bash -c \'test -r "$FASTRTPS_DEFAULT_PROFILES_FILE" && '
             'printf "%s" "$FASTRTPS_DEFAULT_PROFILES_FILE"\'',
             "bash", str(ROOT)],
            cwd="/tmp", capture_output=True, text=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, str(ROOT / "docker/fastdds_udp.xml"))

    def test_ros_helper_defaults_to_nonzero_ros_domain_and_keeps_override(self):
        for env_domain, expected in ((None, "10"), ("42", "42")):
            with self.subTest(env_domain=env_domain):
                env = {k: v for k, v in os.environ.items() if k != "ROS_DOMAIN_ID"}
                if env_domain:
                    env["ROS_DOMAIN_ID"] = env_domain
                result = subprocess.run(
                    ["bash", "-c",
                     'project_dir="$1"; source "$project_dir/docker/ros_compose.sh" '
                     '2>/dev/null; bash -c \'printf "%s" "$ROS_DOMAIN_ID"\'',
                     "bash", str(ROOT)],
                    capture_output=True, text=True, check=False, env=env,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, expected)

    def test_ros_helper_warns_when_kernel_caps_dds_udp_buffers(self):
        with tempfile.TemporaryDirectory() as directory:
            limits = Path(directory)
            (limits / "wmem_max").write_text("8388608\n")
            for rmem_max, warned in (("212992", True), ("8388608", False)):
                with self.subTest(rmem_max=rmem_max):
                    (limits / "rmem_max").write_text(rmem_max + "\n")
                    result = subprocess.run(
                        ["bash", "-c",
                         'project_dir="$1"; source "$project_dir/docker/ros_compose.sh" '
                         '2>/dev/null; warn_small_dds_buffers "$2"',
                         "bash", str(ROOT), directory],
                        capture_output=True, text=True, check=False,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual("net.core.rmem_max=212992" in result.stderr, warned)
                    self.assertEqual(
                        "sysctl -w net.core.rmem_max=8388608" in result.stderr, warned
                    )

    def test_session_cli_rejects_invalid_usage_and_missing_container(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "scripts").mkdir()
            (root / "docker").mkdir()
            script = root / "scripts/slam_session.sh"
            shutil.copy2(ROOT / "scripts/slam_session.sh", script)
            (root / "docker/ros_compose.sh").write_text("ros_compose() { return 0; }\n")
            for args in ((), ("start",), ("stop",), ("start", "floor", "extra")):
                with self.subTest(args=args):
                    result = subprocess.run([str(script), *args], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn("Usage:", result.stderr)
            result = subprocess.run([str(script), "status"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn("container is not running", result.stderr)

    def test_isaac_python_path_prefers_isaac_python_over_isaacsim_dir(self):
        cases = (
            ({}, "/tmp/test_home/isaacsim-6.2.0/python.sh"),
            ({"ISAACSIM_DIR": "/opt/isaac"}, "/opt/isaac/python.sh"),
            ({"ISAAC_PYTHON": "/custom/python.sh"}, "/custom/python.sh"),
            (
                {"ISAAC_PYTHON": "/custom/python.sh", "ISAACSIM_DIR": "/opt/isaac"},
                "/custom/python.sh",
            ),
        )
        for extra_env, expected in cases:
            with self.subTest(extra_env=extra_env):
                env = {
                    k: v for k, v in os.environ.items()
                    if k not in ("ISAAC_PYTHON", "ISAACSIM_DIR")
                }
                env["HOME"] = "/tmp/test_home"
                env.update(extra_env)
                result = subprocess.run(
                    ["bash", "-c",
                     'project_dir="$1"; source "$project_dir/docker/ros_compose.sh" '
                     '2>/dev/null; isaac_python_path',
                     "bash", str(ROOT)],
                    capture_output=True, text=True, check=False, env=env,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), expected)

    def test_launchers_use_repository_resources_and_moved_simulator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / "scripts"
            scripts.mkdir()
            for script in (ROOT / "scripts").glob("*.sh"):
                shutil.copy2(script, scripts / script.name)
            (scripts / "standalone.py").touch()
            fake_isaac = root / "fake_isaac"
            fake_isaac.mkdir()
            fake_python = fake_isaac / "python.sh"
            fake_python.write_text(
                '#!/usr/bin/env bash\n'
                'script="$1"; shift\n'
                'if [[ "$(basename "$script")" == "m20_standalone.py" ]]; then\n'
                '  printf "M20_SIMULATOR %s %s\\n" "$(basename "$script")" "$*"\n'
                'else\n'
                '  printf "SIMULATOR %s\\n" "$*"\n'
                'fi\n',
            )
            fake_python.chmod(0o755)
            override_python = root / "override_isaac/python.sh"
            override_python.parent.mkdir()
            override_python.write_text(
                '#!/usr/bin/env bash\n'
                'printf "OVERRIDE_PYTHON %s | %s\\n" "$LD_LIBRARY_PATH" "$*"\n',
            )
            override_python.chmod(0o755)
            fake_policy = root / "policy.pt"
            fake_policy.touch()
            maps = root / "maps/office"
            maps.mkdir(parents=True)
            (maps / "map.pcd").touch()
            (maps / "map_2d.pgm").touch()
            (maps / "map_2d.yaml").write_text("image: map_2d.pgm\n")
            shutil.copytree(maps, root / "maps/1F")
            (root / "maps/1F/session.json").write_text("{}")
            docker = root / "docker"
            docker.mkdir()
            (docker / "ros_compose.sh").write_text(
                'isaac_python_path() { printf "%s\\n" "${ISAAC_PYTHON:-${ISAACSIM_DIR:-$HOME/isaacsim-6.2.0}/python.sh}"; }\n'
                'require_ros_workspace() { printf "ROOT %s\\n" "$project_dir"; }\n'
                'require_height_map_workspace() { printf "ROOT %s\\n" "$project_dir"; }\n'
                'container_path() { printf "/workspace/%s\\n" "${1#"$project_dir"/}"; }\n'
                'start_ros() { printf "ROS %s\\n" "$*"; }\n'
                'start_height_map() { printf "HEIGHT_MAP %s\\n" "$*"; }\n'
                'start_ros_container() { printf "ROS %s\\n" "$*"; }\n'
                'ros_compose() {\n'
                '  if [[ "$1" == ps ]]; then\n'
                '    echo running\n'
                '  elif [[ "$*" == *"service type"* ]]; then\n'
                '    echo interface/srv/SaveMaps\n'
                '  else\n'
                '    printf "ROS %s\\n" "$*"\n'
                '  fi\n'
                '}\n',
            )
            cases = (
                ("run_slam.sh", (), ("ROOT", "ROS slam",
                 "SIMULATOR --dynamic-elevator-doors --elevator-door-mode manual\n")),
                ("run_slam.sh", ("--free-fly",), ("ROOT", "ROS slam",
                 "SIMULATOR --dynamic-elevator-doors --elevator-door-mode manual --free-fly\n")),
                ("run_mapping.sh", ("--free-fly",), ("ROOT", "ROS slam",
                 "SIMULATOR --dynamic-elevator-doors --elevator-door-mode manual --free-fly\n")),
                ("slam_session.sh", ("start", "floor_ground"),
                 ("python3 /workspace/scripts/mapping_manager.py start floor_ground",)),
                ("slam_session.sh", ("save-stop",),
                 ("python3 /workspace/scripts/mapping_manager.py save-stop",)),
                ("slam_session.sh", ("status",),
                 ("python3 /workspace/scripts/mapping_manager.py status",)),
                ("run_2d_localization.sh", ("--headless", "--no-rviz"),
                 ("ROOT", "ROS launch slam_localization_2d",
                  "map:=/workspace/maps/1F/map_2d.yaml",
                  "rviz:=false", "SIMULATOR", "--headless")),
                ("run_3d_localization.sh", ("--headless", "--no-rviz"),
                 ("ROOT", "ROS launch slam_localization_3d",
                  "map_pcd:=/workspace/maps/1F/map.pcd",
                  "map_pgm:=/workspace/maps/1F/map_2d.yaml",
                  "rviz:=false", "SIMULATOR", "--headless")),
                ("run_3d_localization.sh",
                 ("--dynamic-elevator-doors", "--elevator-door-mode", "manual"),
                 ("SIMULATOR", "--dynamic-elevator-doors --elevator-door-mode manual")),
                ("run_3d_localization.sh", ("--headless", "--dynamic-elevator-doors"),
                 ("SIMULATOR", "--headless",
                  "--dynamic-elevator-doors --elevator-door-mode cycle")),
                ("run_nav.sh", ("--mode", "2d"),
                 ("ROOT", "localization_2d.launch.py", "SIMULATOR")),
                ("run_nav.sh", ("--mode", "2d", "--headless", "--no-rviz"),
                 ("ROOT", "localization_2d.launch.py", "SIMULATOR", "--headless")),
                ("run_nav.sh", (),
                 ("ROOT", "global_fusion.launch.py", "costmaps:=true",
                  "navigate:=true", "rviz:=true", "SIMULATOR",
                  "--ros-cmd-vel", "--dynamic-elevator-doors --elevator-door-mode manual")),
                ("run_nav.sh", ("--mode", "3d", "--no-rviz"),
                 ("global_fusion.launch.py", "rviz:=false", "--ros-cmd-vel",
                  "--dynamic-elevator-doors --elevator-door-mode manual")),
                ("run_nav.sh", ("--mode", "3d", "--headless", "--no-rviz"),
                 ("ROOT", "global_fusion.launch.py", "costmaps:=true",
                  "navigate:=true", "rviz:=false", "SIMULATOR", "--headless",
                  "--ros-cmd-vel")),
                ("run_nav.sh", ("--navigate", "--manual-initial-pose"),
                 ("global_fusion.launch.py", "navigate:=true",
                  "auto_initial_pose:=false", "--ros-cmd-vel",
                  "--dynamic-elevator-doors --elevator-door-mode manual")),
                ("run_nav.sh", ("--mqtt-host", "localhost", "--mqtt-port", "1884"),
                 ("global_fusion.launch.py", "navigate:=true",
                  "floor_mqtt_host:=localhost", "floor_mqtt_port:=1884")),
                ("run_nav_m20.sh",
                 ("--policy", str(fake_policy), "--headless", "--no-rviz",
                  "--policy-device", "cuda"),
                 ("ROOT", "ROS launch slam_localization_3d global_fusion.launch.py",
                  "robot_type:=m20", "costmaps:=true", "navigate:=true", "rviz:=false",
                  "HEIGHT_MAP launch m20_policy_node m20_policy.launch.py",
                  "policy_path:=/policies/policy.pt", "device:=cuda", "fastlio:=false",
                  "M20_SIMULATOR", "m20_standalone.py --external --no-cmd-vel --no-tf --headless")),
                ("run_multi_nav.sh", ("--headless", "--no-rviz"),
                 ("ROOT", "launch slam_bringup robot.launch.py",
                  "robot:=carter1@0,0,0", "robot:=carter2:carter_v1@3.5,0,0",
                  "SIMULATOR", "--headless", "--robot carter1@0,0,0",
                  "--robot carter2:carter_v1@3.5,0,0")),
                ("run_multi_nav.sh", ("--headless",),
                 ("fleet_rviz.launch.py", "robots:=carter1@0,0,0;carter2:carter_v1@3.5,0,0",
                  "maps_root:=/workspace/maps", "floors:=1F;B1", "initial_floor:=1F",
                  "auto_initial_pose:=true", "simulation_reference:=false")),
                ("run_multi_nav_online.sh", ("--no-ceiling-cameras", "--mqtt-host", "localhost"),
                 ("fleet_rviz.launch.py", "robots:=carter1#fleet1@0,0,0;carter2:carter_v1#fleet2@3.5,0,0",
                  "maps_root:=/workspace/maps", "floors:=1F;B1", "initial_floor:=1F",
                  "mqtt_host:=localhost", "auto_initial_pose:=true", "simulation_reference:=false")),
                ("run_multi_nav.sh", ("--headless", "--no-rviz", "--filter-editor"),
                 ("robot.launch.py robot:=carter1@0,0,0", "fleet_filters:=true",
                  "fleet_rviz.launch.py rviz:=false filter_editor:=true "
                  "filter_state_dir:=/workspace/maps/costmap_filters/fleet")),
                ("run_multi_nav_online.sh",
                 ("--no-ceiling-cameras", "--mqtt-host", "localhost",
                  "--filter-editor", "--filter-state-dir", "zones"),
                 ("fleet_filters:=true", "fleet_rviz.launch.py rviz:=true filter_editor:=true "
                  "filter_state_dir:=/workspace/zones")),
                ("run_multi_nav.sh", ("--manual-initial-pose",),
                 ("auto_initial_pose:=false", "simulation_reference:=true")),
                ("run_multi_nav_online.sh", ("--manual-initial-pose", "--no-ceiling-cameras"),
                 ("auto_initial_pose:=false", "simulation_reference:=true")),
                ("run_multi_nav_lift_online.sh", ("--no-ceiling-cameras", "--mqtt-host", "localhost"),
                 ("fleet_rviz.launch.py", "mqtt_host:=localhost", "fleet_filters:=true",
                  "filter_editor:=true", "--dynamic-elevator-doors --elevator-mqtt-lift lift_1")),
                ("run_multi_nav_lift_online.sh",
                 ("--no-ceiling-cameras", "--mqtt-host", "localhost", "--no-filter-editor"),
                 ("fleet_filters:=false", "filter_editor:=false")),
                ("save_map.sh", ("--voxel-size", "0.05"),
                 ("save_map_resolution 0.05",
                  "file_path: '/workspace/maps/office'")),
            )
            for name, args, expected in cases:
                with self.subTest(script=name, args=args):
                    env = {
                        **{k: v for k, v in os.environ.items() if k != "ISAAC_PYTHON"},
                        "ISAACSIM_DIR": str(fake_isaac),
                    }
                    result = subprocess.run(
                        [str(scripts / name), *args], cwd=maps, env=env,
                        capture_output=True, text=True, timeout=10, check=False,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    for text in expected:
                        self.assertIn(text, result.stdout)
                    if (name == "run_nav.sh" and
                            ("--headless" in args or "--mode 2d" in " ".join(args))):
                        self.assertNotIn("--dynamic-elevator-doors", result.stdout)
                        self.assertNotIn("--elevator-door-mode", result.stdout)
                    if "--mqtt-host" not in args:
                        self.assertNotIn("floor_mqtt_host", result.stdout)
                    if (name.startswith("run_multi_nav") and "--filter-editor" not in args
                            and (name != "run_multi_nav_lift_online.sh"
                                 or "--no-filter-editor" in args)):
                        self.assertNotIn("fleet_filters:=true", result.stdout)
                        if "--no-rviz" in args:
                            self.assertNotIn("fleet_rviz.launch.py", result.stdout)
                    if name == "run_3d_localization.sh" and "--dynamic-elevator-doors" not in args:
                        self.assertNotIn("--dynamic-elevator-doors", result.stdout)
                    if "ROOT" in expected:
                        self.assertIn(f"ROOT {root}\n", result.stdout)
                    if name in ("run_nav.sh", "run_nav_m20.sh", "run_3d_localization.sh", "run_multi_nav.sh"):
                        self.assertIn("/workspace/maps/1F/map_2d.yaml", result.stdout)
                        if name != "run_nav.sh" or "--mode 2d" not in " ".join(args):
                            self.assertIn("/workspace/maps/1F/map.pcd", result.stdout)
                            expected_auto = ("true" if name == "run_multi_nav.sh"
                                             and "--manual-initial-pose" not in args else "false")
                            self.assertIn(f"auto_initial_pose:={expected_auto}", result.stdout)
                        else:
                            self.assertNotIn("global_fusion.launch.py", result.stdout)
                            self.assertNotIn("navigate:=true", result.stdout)
                            self.assertNotIn("--ros-cmd-vel", result.stdout)

            for name, args, expected_ld in (
                ("run_slam.sh", (), ""),
                ("run_nav_m20.sh", ("--policy", str(fake_policy), "--headless", "--no-rviz"),
                 f"{override_python.parent}/exts/isaacsim.ros2.core/humble/lib"),
            ):
                with self.subTest(isaac_python_override=name):
                    env = {
                        k: v for k, v in os.environ.items()
                        if k != "ISAACSIM_DIR"
                    }
                    env["ISAAC_PYTHON"] = str(override_python)
                    result = subprocess.run(
                        [str(scripts / name), *args], cwd=maps, env=env,
                        capture_output=True, text=True, timeout=10, check=False,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn("OVERRIDE_PYTHON ", result.stdout)
                    if expected_ld:
                        self.assertIn(expected_ld, result.stdout)

            (root / "maps/1F/session.json").unlink()
            for name in ("run_multi_nav.sh", "run_multi_nav_online.sh"):
                result = subprocess.run([str(scripts / name)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("use --manual-initial-pose", result.stderr)
                self.assertNotIn("ROOT", result.stdout)


if __name__ == "__main__":
    unittest.main()
