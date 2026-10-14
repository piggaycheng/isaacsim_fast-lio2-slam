"""Run in the sourced ROS container on an isolated ROS_DOMAIN_ID.

Mapping nodes are real; RViz is replaced by a headless lifetime stub.
"""

import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField

ROOT = Path(__file__).resolve().parents[1]
MANAGER = ROOT / "scripts/mapping_manager.py"
sys.path.insert(0, str(ROOT / "scripts"))
from pcd2pgm import load_xyz


def validate():
    rclpy.init()
    node = rclpy.create_node("mapping_validation_sensors")
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    imu_pub = node.create_publisher(Imu, "/isaac/imu", 20)
    lidar_pub = node.create_publisher(PointCloud2, "/isaac/lidar_points", 10)
    truth_pub = node.create_publisher(Odometry, "/isaac/ground_truth/odom", 20)
    running = threading.Event()
    running.set()
    height = [0.0]
    room_scale = [1.0]
    ticks = [0]
    walls = []
    for side in (-4.0, 4.0):
        for horizontal in np.linspace(-4, 4, 60):
            for vertical in np.linspace(-0.5, 2.5, 20):
                walls.extend(((side, horizontal, vertical), (horizontal, side, vertical)))
    for x in np.linspace(-4, 4, 40):
        for y in np.linspace(-4, 4, 40):
            walls.append((x, y, -0.5))

    def publish():
        while running.is_set():
            ticks[0] += 1
            stamp_ns = 10_000_000_000 + ticks[0] * 10_000_000
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
            clock_pub.publish(clock)
            imu = Imu()
            imu.header.stamp = clock.clock
            imu.header.frame_id = "imu_link"
            imu.linear_acceleration.z = 9.81
            imu.orientation.w = 1.0
            imu_pub.publish(imu)
            truth = Odometry()
            truth.header.stamp = clock.clock
            truth.header.frame_id = "isaac_world"
            truth.child_frame_id = "chassis_link"
            truth.pose.pose.position.z = height[0]
            truth.pose.pose.orientation.w = 1.0
            truth_pub.publish(truth)
            if ticks[0] % 10 == 0:
                cloud = PointCloud2()
                cloud.header.stamp = clock.clock
                cloud.header.frame_id = "lidar_link"
                cloud.height, cloud.width = 1, len(walls)
                cloud.fields = [
                    PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
                ] + [
                    PointField(name="timestamp", offset=16, datatype=PointField.UINT32, count=2),
                    PointField(name="emitter_id", offset=24, datatype=PointField.UINT32, count=1),
                    PointField(name="channel_id", offset=28, datatype=PointField.UINT32, count=1),
                ]
                cloud.point_step = 32
                cloud.row_step = cloud.point_step * cloud.width
                cloud.is_dense = True
                chunks = []
                for index, (x, y, z) in enumerate(walls):
                    timestamp = stamp_ns - 100_000_000 + index * 99_000_000 // len(walls)
                    chunks.append(struct.pack(
                        "<ffffQII", x * room_scale[0], y * room_scale[0], z,
                        0.5, timestamp, index % 32, index % 32,
                    ))
                cloud.data = b"".join(chunks)
                lidar_pub.publish(cloud)
            time.sleep(0.01)

    thread = threading.Thread(target=publish)
    thread.start()
    created = []
    server = None
    with tempfile.TemporaryDirectory() as temporary:
        log_path = Path(temporary) / "manager.log"
        rviz_pid_path = Path(temporary) / "rviz.pid"
        rviz_args_path = Path(temporary) / "rviz.args"
        shell_init = Path(temporary) / "headless_rviz.sh"
        shell_init.write_text(
            'rviz2() { echo "$BASHPID" > "$RVIZ_VALIDATION_PID"; '
            'printf "%s\\n" "$@" > "$RVIZ_VALIDATION_ARGS"; exec sleep 120; }\n',
        )
        try:
            with log_path.open("w") as log:
                server = subprocess.Popen(
                    [str(ROOT / "docker/entrypoint.sh"), "slam"],
                    env={**os.environ, "BASH_ENV": str(shell_init),
                         "RVIZ_VALIDATION_PID": str(rviz_pid_path),
                         "RVIZ_VALIDATION_ARGS": str(rviz_args_path)},
                    stdout=log, stderr=subprocess.STDOUT,
                )

            def command(*args, success=True):
                result = subprocess.run(
                    [sys.executable, str(MANAGER), *args], capture_output=True, text=True, timeout=120,
                )
                assert (result.returncode == 0) == success, (args, result.stdout, result.stderr)
                response = json.loads(result.stdout)
                assert response["ok"] == success
                return response

            for _ in range(50):
                check = subprocess.run(
                    [sys.executable, str(MANAGER), "status"], capture_output=True, text=True, timeout=5,
                )
                if check.returncode == 0:
                    break
                if server.poll() is not None:
                    raise RuntimeError(log_path.read_text())
                time.sleep(0.1)
            else:
                raise RuntimeError("Manager socket did not become ready")
            assert command("status")["state"] == "idle"
            assert rviz_args_path.read_text().splitlines()[-3:] == [
                "--ros-args", "-p", "use_sim_time:=true",
            ]
            command("save-stop", success=False)
            identifier = uuid.uuid4().hex[:12]
            for floor, elevation in (("ground", 0.0), ("lower", -3.3)):
                name = f"validation_{identifier}_{floor}"
                directory = ROOT / "maps" / name
                assert not directory.exists()
                created.append(directory)
                height[0] = elevation
                room_scale[0] = 1.0 if floor == "ground" else 1.4
                start = command("start", name)
                assert start["state"] == "mapping"
                reference = start["session"]["reference"]
                assert reference["isaac_chassis_pose"]["position"][2] == elevation
                assert np.linalg.norm(reference["lio_initial_pose"]["position"]) < 0.1
                command("start", name + "_extra", success=False)
                time.sleep(1)
                saved = command("save-stop")
                assert saved["state"] == "idle"
                assert saved["last_session"]["point_count"] > 100
                metadata = json.loads((directory / "session.json").read_text())
                assert metadata["state"] == "saved_stopped"
                assert metadata["reference"]["isaac_chassis_pose"]["position"][2] == elevation
                assert (directory / "patches/0.pcd").exists()
                points = load_xyz(directory / "map.pcd")
                half_width = np.max(np.abs(points[:, :2]))
                assert abs(half_width - 4.0 * room_scale[0]) < 0.1, half_width
                command("start", name, success=False)
                before = ticks[0]
                time.sleep(0.2)
                assert ticks[0] > before, "Sensors stopped when mapping stopped"
                assert server.poll() is None, "Container entrypoint exited when mapping stopped"
                os.kill(int(rviz_pid_path.read_text()), 0)
                print(f"Mapping session passed: {floor}, points={metadata['point_count']}, "
                      f"simulator reference z={elevation}", flush=True)
            print("Real FAST-LIO2/PGO start, save, stop, independent restart, CLI and "
                  "entrypoint/headless RViz lifetime passed", flush=True)
        except (AssertionError, OSError, RuntimeError, ValueError, subprocess.TimeoutExpired):
            print(log_path.read_text(), file=sys.stderr)
            for directory in created:
                if (directory / "slam.log").exists():
                    print((directory / "slam.log").read_text()[-12000:], file=sys.stderr)
            raise
        finally:
            if server is not None:
                server.send_signal(signal.SIGTERM)
                server.wait(timeout=25)
            running.clear()
            thread.join(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            for directory in created:
                if directory.exists():
                    shutil.rmtree(directory)


if __name__ == "__main__":
    validate()
