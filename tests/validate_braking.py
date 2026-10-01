"""ROS-side physical braking probe; run in Docker with the validation scene active."""

import argparse
import json
import math
import os
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros2_ws/src/isaac_localization_3d/config"


def rectangle(bounds, position, yaw):
    x0, x1, y0, y1 = bounds
    points = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + position


def separation(first, second):
    """Signed SAT gap: a conservative lower bound on rectangle clearance."""
    gaps = []
    for polygon in (first, second):
        for edge in np.roll(polygon, -1, axis=0) - polygon:
            axis = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
            a, b = first @ axis, second @ axis
            gaps.append(max(b.min() - a.max(), a.min() - b.max()))
    return float(max(gaps))


class BrakingProbe(Node):
    def __init__(self, directory, bounds, footprint_bounds):
        super().__init__("braking_validation", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True),
        ])
        self.directory = directory
        self.bounds = bounds
        self.footprint_bounds = footprint_bounds
        self.directory.mkdir(parents=True, exist_ok=True)
        self.latest = {}
        self.history = []
        self.command = Twist()
        self.sequence = time.time_ns()
        self.paused = set()
        self.publisher = self.create_publisher(Twist, "/nav2/cmd_vel_nav", 10)
        self.create_timer(0.05, lambda: self.publisher.publish(self.command))
        for topic, kind, key in [
            ("/isaac/ground_truth/odom", Odometry, "truth"),
            ("/cmd_vel", Twist, "final"),
            ("/nav2/cmd_vel_monitored", Twist, "monitored"),
            ("/scan", LaserScan, "scan"),
            ("/perception/obstacles", PointCloud2, "cloud"),
            ("/localization_3d/accepted_correction", Header, "correction"),
        ]:
            self.create_subscription(
                kind, topic, lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data,
            )

    def receive(self, key, message):
        self.latest[key] = message
        if key in ("truth", "final", "monitored"):
            self.history.append((self.now(), key, message))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise TimeoutError("Braking condition not reached")

    def hold(self, seconds):
        end = self.now() + seconds
        self.wait(lambda: self.now() >= end, timeout=max(30, seconds * 10))

    def control(self, action, **kwargs):
        self.sequence += 1
        request = {"sequence": self.sequence, "action": action, **kwargs}
        temp = self.directory / "request.tmp"
        temp.write_text(json.dumps(request))
        temp.replace(self.directory / "request.json")
        response_path = self.directory / "response.json"

        def acknowledged():
            return response_path.exists() and (
                json.loads(response_path.read_text())["sequence"] == self.sequence
            )

        self.wait(acknowledged)
        return json.loads(response_path.read_text())

    def moving_speed(self, angular):
        velocity = self.latest["truth"].twist.twist
        return abs(velocity.angular.z) if angular else math.hypot(
            velocity.linear.x, velocity.linear.y,
        )

    @staticmethod
    def pose(message):
        position, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # Simulator truth uses chassis_link; ROS base_link faces chassis -x.
        return np.array([position.x, position.y]), yaw + math.pi

    def stop(self):
        self.command = Twist()
        self.hold(0.7)

    def case(self, direction, requested, repeat):
        self.stop()
        self.control("hide")
        self.hold(0.5)
        angular = direction in ("left", "right")
        self.command = Twist()
        if angular:
            self.command.angular.z = requested if direction == "left" else -requested
        else:
            self.command.linear.x = requested if direction == "front" else -requested
        requested_twist = [self.command.linear.x, self.command.angular.z]
        steady_since = None

        def cruising():
            nonlocal steady_since
            minimum = (0.85 if angular else 0.95) * requested
            if not minimum <= self.moving_speed(angular) <= 1.1 * requested:
                steady_since = None
                return False
            if steady_since is None:
                steady_since = self.now()
            return self.now() - steady_since >= 0.2

        self.wait(cruising)
        cruise = self.moving_speed(angular)
        origin, yaw = self.pose(self.latest["truth"])
        self.history.clear()
        trigger_time = self.now()
        response = None
        if direction == "watchdog":
            matches = []
            for path in Path("/proc").glob("[0-9]*/cmdline"):
                try:
                    args = path.read_bytes().split(b"\0")
                except (FileNotFoundError, ProcessLookupError):
                    continue
                if args and Path(os.fsdecode(args[0])).name == "ground_obstacle_filter":
                    matches.append(int(path.parent.name))
            if len(matches) != 1:
                raise RuntimeError(f"Expected one ground_obstacle_filter, got {matches}")
            self.paused.add(matches[0])
            os.kill(matches[0], signal.SIGSTOP)
        else:
            monitor = yaml.safe_load((CONFIG / "collision_monitor.yaml").read_text())
            params = monitor["collision_monitor"]["ros__parameters"]
            polygon = np.array(params[
                "PolygonStop" if direction == "front" else "PolygonSurround"
            ]["points"]).reshape(-1, 2)
            if direction == "front":
                offset, sizes = [float(polygon[:, 0].max()) + 0.09, 0.0], [0.2, 0.6, 1.2]
            elif direction == "rear":
                offset, sizes = [float(polygon[:, 0].min()) - 0.09, 0.0], [0.2, 0.6, 1.2]
            else:
                sign = 1 if direction == "left" else -1
                offset = [0.50, sign * (float(abs(polygon[:, 1]).max()) + 0.09)]
                sizes = [0.2, 0.2, 1.2]
            response = self.control("place", offset=offset, sizes=sizes)
            # The scene must use the same forward frame as simulator ground truth.
            yaw_error = math.atan2(
                math.sin(response["robot_yaw"] - yaw), math.cos(response["robot_yaw"] - yaw),
            )
            if abs(yaw_error) > 0.25:
                raise AssertionError(f"Scene/truth frame mismatch: {yaw_error}")
        failure = None
        try:
            self.wait(
                lambda: self.latest["final"] == Twist() and self.moving_speed(angular) < 0.02,
                timeout=5,
            )
        except TimeoutError:
            failure = "No confirmed safety stop within 5 wall seconds"
            self.stop()
        stopped_time = self.now()
        self.hold(0.3)
        truth = [(t, msg) for t, key, msg in self.history if key == "truth"]
        zeros = [t for t, key, msg in self.history if key == "final" and msg == Twist()]
        zero_time = zeros[0] if zeros and failure is None else None
        path = [origin] + [self.pose(msg)[0] for _, msg in truth]
        distance = sum(float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:]))
        angle = yaw
        rotation = 0.0
        for _, msg in truth:
            next_angle = self.pose(msg)[1]
            rotation += abs(math.atan2(math.sin(next_angle - angle), math.cos(next_angle - angle)))
            angle = next_angle
        after_zero = [self.pose(msg)[0] for t, msg in truth if zero_time is not None and t >= zero_time]
        post_zero = sum(
            float(np.linalg.norm(b - a)) for a, b in zip(after_zero, after_zero[1:])
        )
        clearance = None
        footprint_clearance = None
        if response is not None:
            sx, sy, _ = response["sizes"]
            box = rectangle(
                [-sx / 2, sx / 2, -sy / 2, sy / 2],
                response["center"][:2], response["yaw"],
            )
            clearance = min(separation(
                rectangle(self.bounds, *self.pose(msg)), box,
            ) for _, msg in truth)
            footprint_clearance = min(separation(
                rectangle(self.footprint_bounds, *self.pose(msg)), box,
            ) for _, msg in truth)
        result = {
            "direction": direction, "requested_speed": requested, "repeat": repeat,
            "requested_twist": requested_twist,
            "measured_cruise_speed": cruise,
            "zero_command_delay_s": None if zero_time is None else zero_time - trigger_time,
            "stopped_after_s": stopped_time - trigger_time,
            "travel_to_stop_m": distance, "rotation_to_stop_rad": rotation,
            "travel_after_zero_m": post_zero, "min_clearance_lower_bound_m": clearance,
            "min_padded_footprint_clearance_m": footprint_clearance,
            "failure": failure,
            "pass": failure is None and (
                clearance is None or (clearance >= 0.02 and footprint_clearance >= 0.02)
            ),
        }
        print("BRAKING_RESULT " + json.dumps(result), flush=True)
        self.stop()
        for pid in tuple(self.paused):
            os.kill(pid, signal.SIGCONT)
            self.paused.remove(pid)
        self.control("hide")
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--linear-speeds", type=float, nargs="+", default=[0.25, 0.5, 0.75])
    parser.add_argument("--angular-speeds", type=float, nargs="+", default=[0.35, 0.7])
    args = parser.parse_args()
    if args.repeats < 1 or any(not 0 < v <= 0.75 for v in args.linear_speeds):
        parser.error("Use positive repeats and linear speeds in (0, 0.75]")
    if any(not 0 < v <= 0.7 for v in args.angular_speeds):
        parser.error("Use angular speeds in (0, 0.7]")
    rclpy.init()
    def interrupted(signum, frame):
        raise RuntimeError(f"Braking probe interrupted by signal {signum}")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    geometry = json.loads(args.geometry.read_text())
    minima = np.minimum(geometry["visible"]["min"], geometry["collision"]["min"])
    maxima = np.maximum(geometry["visible"]["max"], geometry["collision"]["max"])
    footprint = np.asarray(geometry["footprint"])
    fmin = footprint.min(axis=0) - geometry["footprint_padding"]
    fmax = footprint.max(axis=0) + geometry["footprint_padding"]
    probe = BrakingProbe(
        args.control_dir, [minima[0], maxima[0], minima[1], maxima[1]],
        [fmin[0], fmax[0], fmin[1], fmax[1]],
    )
    results = []
    try:
        probe.wait(lambda: all(
            key in probe.latest for key in ("truth", "final", "scan", "cloud", "correction")
        ))
        probe.control("hide")
        # Move away from the Office wall so its approach zone cannot limit cruise speed.
        probe.command.linear.x = -0.25
        probe.hold(3.0)
        probe.stop()
        for repeat in range(args.repeats):
            for speed in args.linear_speeds:
                for direction in ("rear", "front"):
                    results.append(probe.case(direction, speed, repeat))
                    args.output.write_text(json.dumps(results, indent=2) + "\n")
            for speed in args.angular_speeds:
                for direction in ("left", "right"):
                    results.append(probe.case(direction, speed, repeat))
                    args.output.write_text(json.dumps(results, indent=2) + "\n")
        results.append(probe.case("watchdog", max(args.linear_speeds), 0))
        args.output.write_text(json.dumps(results, indent=2) + "\n")
        if not all(result["pass"] for result in results):
            raise AssertionError("Insufficient physical clearance in braking results")
        print(f"PASS: {len(results) - 1} physical obstacle cases and 1 sensor-loss case", flush=True)
    finally:
        probe.command = Twist()
        probe.publisher.publish(probe.command)
        for pid in tuple(probe.paused):
            os.kill(pid, signal.SIGCONT)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
