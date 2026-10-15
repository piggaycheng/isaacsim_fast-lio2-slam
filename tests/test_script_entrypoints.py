import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ScriptEntrypointsTest(unittest.TestCase):
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

    def test_launchers_use_repository_resources_and_moved_simulator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / "scripts"
            scripts.mkdir()
            for script in (ROOT / "scripts").glob("*.sh"):
                shutil.copy2(script, scripts / script.name)
            simulator = scripts / "standalone.py"
            simulator.write_text(
                '#!/usr/bin/env bash\nprintf "SIMULATOR %s\\n" "$*"\n',
            )
            simulator.chmod(0o755)
            maps = root / "maps/office"
            maps.mkdir(parents=True)
            (maps / "map.pcd").touch()
            (maps / "map_2d.pgm").touch()
            (maps / "map_2d.yaml").write_text("image: map_2d.pgm\n")
            shutil.copytree(maps, root / "maps/1F")
            docker = root / "docker"
            docker.mkdir()
            (docker / "ros_compose.sh").write_text(
                'require_ros_workspace() { printf "ROOT %s\\n" "$project_dir"; }\n'
                'container_path() { printf "/workspace/%s\\n" "${1#"$project_dir"/}"; }\n'
                'start_ros() { printf "ROS %s\\n" "$*"; }\n'
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
                ("run_nav.sh", ("--mode", "2d", "--headless", "--no-rviz"),
                 ("ROOT", "localization_2d.launch.py", "SIMULATOR", "--headless")),
                ("run_nav.sh", (),
                 ("ROOT", "global_fusion.launch.py", "costmaps:=true",
                  "navigate:=true", "rviz:=true", "SIMULATOR",
                  "--ros-cmd-vel")),
                ("run_nav.sh", ("--mode", "3d", "--headless", "--no-rviz"),
                 ("ROOT", "global_fusion.launch.py", "costmaps:=true",
                  "navigate:=true", "rviz:=false", "SIMULATOR", "--headless",
                  "--ros-cmd-vel")),
                ("run_nav.sh", ("--navigate", "--manual-initial-pose"),
                 ("global_fusion.launch.py", "navigate:=true",
                  "auto_initial_pose:=false", "--ros-cmd-vel")),
                ("run_multi_nav.sh", ("--headless", "--no-rviz"),
                 ("ROOT", "launch slam_bringup robot.launch.py",
                  "robot:=carter1@0,0,0", "robot:=carter2:carter_v1@3.5,0,0",
                  "SIMULATOR", "--headless", "--robot carter1@0,0,0",
                  "--robot carter2:carter_v1@3.5,0,0")),
                ("save_map.sh", ("--voxel-size", "0.05"),
                 ("save_map_resolution 0.05",
                  "file_path: '/workspace/maps/office'")),
            )
            for name, args, expected in cases:
                with self.subTest(script=name, args=args):
                    result = subprocess.run(
                        [str(scripts / name), *args], cwd=maps,
                        capture_output=True, text=True, timeout=10, check=False,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    for text in expected:
                        self.assertIn(text, result.stdout)
                    if "ROOT" in expected:
                        self.assertIn(f"ROOT {root}\n", result.stdout)
                    if name in ("run_nav.sh", "run_3d_localization.sh", "run_multi_nav.sh"):
                        self.assertIn("/workspace/maps/1F/map_2d.yaml", result.stdout)
                        if name != "run_nav.sh" or "--mode 2d" not in " ".join(args):
                            self.assertIn("/workspace/maps/1F/map.pcd", result.stdout)
                            self.assertIn("auto_initial_pose:=false", result.stdout)
                        else:
                            self.assertNotIn("global_fusion.launch.py", result.stdout)
                            self.assertNotIn("navigate:=true", result.stdout)
                            self.assertNotIn("--ros-cmd-vel", result.stdout)


if __name__ == "__main__":
    unittest.main()
