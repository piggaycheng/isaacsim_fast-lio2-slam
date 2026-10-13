"""Container-side, single-robot mapping sessions and a local terminal client."""

import argparse
from collections import deque
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import stat
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SOCKET_PATH = Path("/tmp/isaacsim-mapping-manager.sock")
MAPPING_NODES = {"/fastlio2/lio_node", "/pgo/pgo_node",
                 "/pointcloud2_to_livox", "/imu_scale_adapter"}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def validate_map(directory):
    path = directory / "map.pcd"
    with path.open("rb") as stream:
        fields, sizes, counts, points = [], [], [], 0
        for _ in range(100):
            line = stream.readline(4096)
            if line.startswith(b"FIELDS "):
                fields = line.split()[1:]
            elif line.startswith(b"SIZE "):
                sizes = [int(v) for v in line.split()[1:]]
            elif line.startswith(b"COUNT "):
                counts = [int(v) for v in line.split()[1:]]
            elif line.startswith(b"POINTS "):
                points = int(line.split()[1])
            if line.startswith(b"DATA "):
                counts = counts or [1] * len(fields)
                if (line.split()[1:] != [b"binary"] or points <= 0
                        or not fields or len(fields) != len(sizes) or len(sizes) != len(counts)
                        or min(sizes + counts) <= 0):
                    raise RuntimeError("Saved binary PCD header is invalid")
                expected = points * sum(size * count for size, count in zip(sizes, counts))
                if path.stat().st_size - stream.tell() != expected:
                    raise RuntimeError("Saved PCD point data is truncated or invalid")
                break
        else:
            raise RuntimeError("Saved PCD header is invalid")
    if not (directory / "poses.txt").read_text().strip():
        raise RuntimeError("Saved poses.txt is empty")
    if not any((directory / "patches").glob("*.pcd")):
        raise RuntimeError("Saved keyframe patches are missing")
    return points


class MappingProcess:
    def __init__(self):
        self.process = None

    def start(self, directory):
        if self.process is not None:
            raise RuntimeError("Previous mapping process has not been cleaned up")
        with (directory / "slam.log").open("ab") as stream:
            self.process = subprocess.Popen(
                ["ros2", "launch", "isaac_fastlio_adapter", "fastlio.launch.py"],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
            )

    def running(self):
        return self.process is not None and self.process.poll() is None

    def stop(self):
        if self.process is None:
            return
        process = self.process
        for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            deadline = time.monotonic() + timeout
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                process.wait(timeout=2)
                self.process = None
                return
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print(f"Mapping shutdown exceeded {timeout}s; escalating signal", file=sys.stderr, flush=True)
            while time.monotonic() < deadline:
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    self.process = None
                    return
                time.sleep(0.05)
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            self.process = None
            return
        raise RuntimeError("Mapping process group is still alive after shutdown")


class MappingManager:
    def __init__(self, maps, backend, process):
        self.maps = Path(maps)
        self.backend = backend
        self.process = process
        self.session = None
        self.last_session = None
        self.last_error = None
        self.state = "idle"

    def persist(self):
        temporary = Path(self.session["output_dir"]) / "session.json.tmp"
        temporary.write_text(json.dumps(self.session, indent=2, allow_nan=False) + "\n")
        temporary.replace(temporary.with_suffix(""))

    def status(self):
        if self.session and self.state == "mapping" and not self.process.running():
            self.state = "error"
            self.last_error = "Mapping launch exited unexpectedly; inspect slam.log"
            self.session["state"] = "error"
            self.session["error"] = self.last_error
            self.persist()
            self.process.stop()
            self.last_session = self.session
            self.session = None
        return {"state": self.state, "session": self.session,
                "last_session": self.last_session, "last_error": self.last_error}

    def start(self, floor):
        if not isinstance(floor, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", floor):
            raise ValueError("Floor name must be 1-64 letters, digits, underscores or hyphens")
        if self.session is not None:
            raise RuntimeError("A session already exists; save-stop it before starting another")
        self.backend.ensure_stopped()
        directory = self.maps / floor
        directory.mkdir(parents=True, exist_ok=False)
        self.state = "starting"
        self.session = {"floor": floor, "output_dir": str(directory), "started_at": utc_now(),
                        "state": self.state, "reference": None}
        self.last_error = None
        try:
            self.persist()
            self.backend.begin()
            self.process.start(directory)
            self.session["reference"] = self.backend.wait_ready(self.process)
            self.state = self.session["state"] = "mapping"
            self.persist()
        except (OSError, RuntimeError, ValueError, TimeoutError) as error:
            self.process.stop()
            self.session.update(state="start_failed", error=str(error))
            try:
                self.persist()
            finally:
                self.last_session = self.session
                self.session = None
                self.state = "idle"
            raise
        return self.status()

    def save_stop(self):
        if self.session is None:
            raise RuntimeError("No active mapping session")
        directory = Path(self.session["output_dir"])
        if self.session["state"] not in ("saved", "saved_stopped"):
            if not self.process.running():
                raise RuntimeError("Mapping process is not running; inspect slam.log")
            self.state = "saving"
            try:
                self.backend.save(directory)
                points = validate_map(directory)
                self.session.update(state="saved", saved_at=utc_now(), point_count=points)
                self.persist()
            except (OSError, RuntimeError, ValueError, TimeoutError):
                self.state = "mapping"
                self.session["state"] = "mapping"
                raise
        self.state = "stopping"
        self.process.stop()
        self.backend.ensure_stopped()
        self.backend.clear_display()
        self.session["state"] = "saved_stopped"
        self.session["stopped_at"] = utc_now()
        self.persist()
        self.last_session = self.session
        self.session = None
        self.state = "idle"
        self.last_error = None
        return self.status()

    def close(self):
        if self.session:
            print(f"WARNING: closing mapping manager with session {self.session['floor']}; "
                  "unsaved work is not automatically saved. Use save-stop before exiting.",
                  file=sys.stderr, flush=True)
        self.process.stop()
        if self.session:
            saved = self.session["state"] in ("saved", "saved_stopped")
            self.session["state"] = "saved_stopped" if saved else "interrupted"
            self.session["stopped_at"] = utc_now()
            self.persist()


class RosBackend:
    def __init__(self, stopping):
        import rclpy
        from interface.srv import SaveMaps
        from nav_msgs.msg import Odometry
        from rclpy.parameter import Parameter
        from visualization_msgs.msg import MarkerArray

        self.rclpy = rclpy
        self.stopping = stopping
        sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_localization_3d/launch"))
        from robot_fleet import load_robot_profile

        self.sensor_frames = load_robot_profile("nova_carter")["sensor_frames"]
        self.node = rclpy.create_node(
            "mapping_manager", parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.client = self.node.create_client(SaveMaps, "/pgo/save_maps")
        self.ground_truth = deque(maxlen=200)
        self.first_odom = None
        self.initial_reference = None
        self.collecting = False
        self.odom_subscription = None
        self.Odometry = Odometry
        self.save_future = None
        self.save_directory = None
        self.node.create_subscription(Odometry, "/isaac/ground_truth/odom", self.on_truth, 20)
        self.markers = self.node.create_publisher(MarkerArray, "/pgo/loop_markers", 10)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-12):
            raise ValueError("Mapping reference pose is non-finite or has an invalid quaternion")
        return {"stamp": RosBackend.stamp(message), "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "position": [p.x, p.y, p.z], "orientation_xyzw": [q.x, q.y, q.z, q.w]}

    def on_truth(self, message):
        self.ground_truth.append(message)

    def on_odom(self, message):
        if self.collecting and self.first_odom is None:
            self.first_odom = message

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if self.stopping():
                raise RuntimeError("Mapping manager is shutting down")
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            self.spin()

    def ensure_stopped(self):
        self.wait(
            lambda: not any(
                f"{namespace.rstrip('/')}/{name}" in MAPPING_NODES
                for name, namespace in self.node.get_node_names_and_namespaces()
            ),
            10, "Existing mapping nodes are still running; stop the other SLAM launcher first",
        )

    def begin(self):
        if self.odom_subscription is not None:
            self.node.destroy_subscription(self.odom_subscription)
        self.odom_subscription = self.node.create_subscription(
            self.Odometry, "/fastlio2/lio_odom", self.on_odom, 20,
        )
        self.ground_truth.clear()
        self.first_odom = None
        self.initial_reference = None
        self.collecting = True
        self.save_future = None
        self.save_directory = None
        self.clear_display()

    def reference(self):
        if self.initial_reference is not None:
            return self.initial_reference
        if self.first_odom is None or not self.ground_truth:
            return None
        truth = min(self.ground_truth, key=lambda msg: abs(self.stamp(msg) - self.stamp(self.first_odom)))
        delta = abs(self.stamp(truth) - self.stamp(self.first_odom))
        if delta > 0.1:
            return None
        self.initial_reference = {
            "lio_initial_pose": self.pose(self.first_odom),
            "isaac_chassis_pose": self.pose(truth), "stamp_difference_seconds": delta,
            "robot_type": "nova_carter", "sensor_frames": self.sensor_frames,
            "registration": "Reference poses only; not an optimized map-to-world transform",
        }
        return self.initial_reference

    def wait_ready(self, process):
        def ready():
            if not process.running():
                raise RuntimeError("Mapping launch exited during startup; inspect slam.log")
            reference = self.reference()
            return self.client.service_is_ready() and reference is not None

        self.wait(ready, 30, "No initialized LIO/PGO with matching simulator truth within 30s. "
                  "Keep Isaac Sim playing and the robot stationary; inspect slam.log")
        self.collecting = False
        return self.reference()

    def save(self, directory):
        from interface.srv import SaveMaps

        if self.save_future is None:
            self.wait(self.client.service_is_ready, 10, "/pgo/save_maps is unavailable")
            request = SaveMaps.Request()
            request.file_path = str(directory)
            request.save_patches = True
            self.save_directory = directory
            self.save_future = self.client.call_async(request)
        if self.save_directory != directory:
            raise RuntimeError("A save request for a different session is pending")
        self.wait(self.save_future.done, 60, "Save request timed out; mapping is still running. "
                  "Retry save-stop to wait for the same request")
        response = self.save_future.result()
        self.save_future = None
        if response is None or not response.success:
            raise RuntimeError(f"PGO refused to save: {response.message if response else 'no response'}")

    def clear_display(self):
        from visualization_msgs.msg import Marker, MarkerArray

        marker = Marker()
        marker.header.frame_id = "lidar"
        marker.action = Marker.DELETEALL
        self.markers.publish(MarkerArray(markers=[marker]))


def receive(connection):
    data = b""
    while b"\n" not in data:
        chunk = connection.recv(4096)
        if not chunk:
            raise ValueError("Incomplete mapping manager message")
        data += chunk
        if len(data) > 65536:
            raise ValueError("Mapping manager message is too large")
    return json.loads(data.split(b"\n", 1)[0])


def serve():
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    stopping = False

    def request_shutdown(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    with SOCKET_PATH.with_suffix(".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if SOCKET_PATH.exists():
            if not stat.S_ISSOCK(SOCKET_PATH.stat().st_mode):
                raise RuntimeError(f"Not a socket: {SOCKET_PATH}")
            SOCKET_PATH.unlink()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        backend = RosBackend(lambda: stopping)
        manager = MappingManager(ROOT / "maps", backend, MappingProcess())
        try:
            with socket.socket(socket.AF_UNIX) as server:
                server.bind(str(SOCKET_PATH))
                os.chmod(SOCKET_PATH, 0o600)
                server.listen(4)
                server.settimeout(0.1)
                print("Mapping manager ready (idle). Use scripts/slam_session.sh start FLOOR", flush=True)
                while not stopping:
                    backend.spin()
                    manager.status()
                    try:
                        connection, _ = server.accept()
                    except socket.timeout:
                        continue
                    with connection:
                        connection.settimeout(5)
                        try:
                            request = receive(connection)
                            if not isinstance(request, dict):
                                raise ValueError("Expected a command object")
                            command = request.get("command")
                            if command == "start":
                                result = manager.start(request.get("floor"))
                            elif command == "save-stop":
                                result = manager.save_stop()
                            elif command == "status":
                                result = manager.status()
                            else:
                                raise ValueError(f"Unknown mapping command: {command}")
                            reply = {"ok": True, **result}
                        except (OSError, RuntimeError, ValueError, TimeoutError) as error:
                            manager.last_error = str(error)
                            print(f"Mapping command failed: {error}", file=sys.stderr, flush=True)
                            reply = {"ok": False, "error": str(error), **manager.status()}
                        try:
                            connection.sendall((json.dumps(reply, allow_nan=False) + "\n").encode())
                        except (BrokenPipeError, ConnectionResetError):
                            print("Mapping client disconnected; command outcome remains in status",
                                  file=sys.stderr, flush=True)
        finally:
            try:
                manager.close()
            finally:
                SOCKET_PATH.unlink(missing_ok=True)
                backend.node.destroy_node()
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("serve")
    subparsers.add_parser("status")
    subparsers.add_parser("save-stop")
    subparsers.add_parser("start").add_argument("floor")
    args = parser.parse_args()
    try:
        if args.command == "serve":
            serve()
            return 0
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(120)
            connection.connect(str(SOCKET_PATH))
            connection.sendall((json.dumps(vars(args)) + "\n").encode())
            response = receive(connection)
        print(json.dumps(response, indent=2, ensure_ascii=False))
        return 0 if response["ok"] else 1
    except (OSError, RuntimeError, ValueError, TimeoutError) as error:
        print(f"Mapping manager error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
