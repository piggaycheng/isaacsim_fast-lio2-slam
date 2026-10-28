"""ROS-side check of a 3D odom -> base_link (local_odometry_3d) against Isaac Sim truth.

Drives the robot with /cmd_vel up a ramp spawned by standalone.py --ramp, holds on the
plateau and reverses back, sampling odom -> base_link, odom -> lidar_link and
map -> odom/base_link alongside /isaac/ground_truth/odom (never fused). Needs no
locomotion policy: the wheeled robot only has to move the base in 3D.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class Probe(Node):
    def __init__(self):
        super().__init__("validate_3d_odometry")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.truth = None
        self.create_subscription(Odometry, "/isaac/ground_truth/odom", self.on_truth,
                                 qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_truth(self, message):
        pose = message.pose.pose
        self.truth = ((pose.position.x, pose.position.y, pose.position.z),
                      (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                       pose.orientation.w))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def transform(self, parent, child):
        try:
            t = self.buffer.lookup_transform(parent, child, Time()).transform
        except Exception:  # noqa: BLE001 - not yet available
            return None
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def drive(self, speed):
        message = Twist()
        message.linear.x = float(speed)
        self.publisher.publish(message)


def sample(probe, start, phase):
    odom = probe.transform("odom", "base_link")
    lidar = probe.transform("odom", "lidar_link")
    map_odom = probe.transform("map", "odom")
    map_base = probe.transform("map", "base_link")
    truth = probe.truth
    if None in (odom, lidar, map_odom, map_base, truth):
        return None
    truth_roll, truth_pitch, _ = rpy(truth[1])
    roll, pitch, _ = rpy(odom[1])
    map_roll, map_pitch, _ = rpy(map_odom[1])
    mount = rotate(truth[1], start["lidar_mount"])
    return {
        "phase": phase,
        "truth_x": truth[0][0] - start["truth"][0][0],
        "truth_dz": truth[0][2] - start["truth"][0][2],
        "odom_dz": odom[0][2] - start["odom"][0][2],
        "truth_roll": truth_roll, "truth_pitch": truth_pitch,
        "odom_roll": roll, "odom_pitch": pitch,
        # Elevation mapping needs the sensor height: odom -> lidar_link vs truth.
        "lidar_dz": lidar[0][2] - start["lidar"][0][2],
        "truth_lidar_dz": truth[0][2] + mount[2] - start["truth_lidar_z"],
        "map_odom_z": map_odom[0][2], "map_odom_roll": map_roll, "map_odom_pitch": map_pitch,
        "map_base_minus_odom_z": map_base[0][2] - odom[0][2],
    }


def summarize(samples, rise, max_local_drift=0.05):
    def worst(key_a, key_b):
        return max(abs(s[key_a] - s[key_b]) for s in samples)

    def local_drift(window):
        """Worst change of the z error between samples within `window` m of each other."""
        worst_change = 0.0
        for index, first in enumerate(samples):
            error = first["odom_dz"] - first["truth_dz"]
            for second in samples[index + 1:]:
                if abs(second["truth_x"] - first["truth_x"]) <= window:
                    worst_change = max(worst_change,
                                       abs(second["odom_dz"] - second["truth_dz"] - error))
        return worst_change

    path = sum(abs(second["truth_x"] - first["truth_x"])
               for first, second in zip(samples, samples[1:]))
    plateau = [s for s in samples if s["phase"] == "plateau"]
    final = [s for s in samples if s["phase"] == "back"]
    result = {
        "samples": len(samples),
        "ramp_rise": rise,
        "max_truth_dz": max(s["truth_dz"] for s in samples),
        "max_odom_dz": max(s["odom_dz"] for s in samples),
        "max_truth_pitch_deg": math.degrees(max(abs(s["truth_pitch"]) for s in samples)),
        "max_odom_pitch_deg": math.degrees(max(abs(s["odom_pitch"]) for s in samples)),
        "path_length": path,
        "max_dz_error": worst("odom_dz", "truth_dz"),
        # A height scan compares base and terrain heights within about a metre.
        "max_local_dz_drift_1m": local_drift(1.0),
        "max_lidar_dz_error": worst("lidar_dz", "truth_lidar_dz"),
        "max_pitch_error_deg": math.degrees(worst("odom_pitch", "truth_pitch")),
        "max_roll_error_deg": math.degrees(worst("odom_roll", "truth_roll")),
        "plateau_odom_dz": sum(s["odom_dz"] for s in plateau) / max(len(plateau), 1),
        "plateau_truth_dz": sum(s["truth_dz"] for s in plateau) / max(len(plateau), 1),
        "final_odom_dz": final[-1]["odom_dz"] if final else None,
        "final_truth_dz": final[-1]["truth_dz"] if final else None,
        "max_map_odom_z": max(abs(s["map_odom_z"]) for s in samples),
        "max_map_odom_tilt_deg": math.degrees(max(
            max(abs(s["map_odom_roll"]), abs(s["map_odom_pitch"])) for s in samples)),
        "max_map_base_minus_odom_z": max(abs(s["map_base_minus_odom_z"]) for s in samples),
    }
    checks = {
        "climbed_the_ramp": result["max_truth_dz"] > 0.7 * rise,
        # Absolute drift budget: 3 cm or 1 % of the travelled path.
        "odom_follows_height": result["max_dz_error"] < max(0.03, 0.01 * path),
        "lidar_height_follows": result["max_lidar_dz_error"] < max(0.03, 0.01 * path),
        "odom_height_locally_consistent": result["max_local_dz_drift_1m"] < max_local_drift,
        "odom_follows_pitch": result["max_pitch_error_deg"] < 1.0,
        "odom_follows_roll": result["max_roll_error_deg"] < 1.0,
        "map_to_odom_stays_planar": result["max_map_odom_z"] < 1e-6
        and result["max_map_odom_tilt_deg"] < 1e-4,
        "map_base_keeps_odom_height": result["max_map_base_minus_odom_z"] < 1e-3,
    }
    result["checks"] = checks
    result["passed"] = all(checks.values())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ramp-foot", type=float, default=1.0,
                        help="Ramp foot distance ahead of the start (m).")
    parser.add_argument("--rise", type=float, default=0.15)
    parser.add_argument("--slope-deg", type=float, default=6.0)
    parser.add_argument("--plateau", type=float, default=1.5)
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--settle", type=float, default=20.0,
                        help="Seconds to wait after TF appears (FAST-LIO and ICP start-up).")
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument(
        "--max-local-drift", type=float, default=0.05,
        help="Allowed change of the odom z error within 1 m of travel (m); the default is half "
             "of Isaac Lab's default +-0.1 m height-scan observation noise.")
    parser.add_argument("--lidar-mount", type=float, nargs=3, default=(-0.213, 0.009, 0.526))
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and (
            probe.truth is None or probe.transform("map", "base_link") is None
            or probe.transform("odom", "lidar_link") is None):
        probe.spin_for(0.5)
    if probe.truth is None:
        raise SystemExit("No ground truth or TF: is the simulation running?")
    print(f"TF and ground truth available; settling {args.settle:.0f} s", flush=True)
    probe.spin_for(args.settle)

    start = {"odom": probe.transform("odom", "base_link"),
             "lidar": probe.transform("odom", "lidar_link"),
             "truth": probe.truth, "lidar_mount": tuple(args.lidar_mount)}
    start["truth_lidar_z"] = start["truth"][0][2] + rotate(start["truth"][1], start["lidar_mount"])[2]
    run = args.rise / math.tan(math.radians(args.slope_deg))
    plateau_target = args.ramp_foot + run + 0.6 * args.plateau
    samples = []

    def collect(phase, until, speed, seconds_limit):
        end = time.monotonic() + seconds_limit
        next_sample = 0.0
        while time.monotonic() < end and not until():
            probe.drive(speed)
            probe.spin_for(0.02)
            if time.monotonic() >= next_sample:
                next_sample = time.monotonic() + 0.05
                value = sample(probe, start, phase)
                if value is not None:
                    samples.append(value)
        probe.drive(0.0)

    distance = lambda: probe.truth[0][0] - start["truth"][0][0]  # noqa: E731
    collect("flat", lambda: False, 0.0, 3.0)
    collect("up", lambda: distance() >= plateau_target, args.speed, 120.0)
    reached = distance()
    collect("plateau", lambda: False, 0.0, 4.0)
    collect("down", lambda: distance() <= 0.2, -args.speed, 120.0)
    collect("back", lambda: False, 0.0, 4.0)
    probe.drive(0.0)
    result = summarize(samples, args.rise, args.max_local_drift)
    result["reached_distance"] = reached
    args.output.write_text(json.dumps({"summary": result, "samples": samples}, indent=1))
    print(json.dumps(result, indent=1), flush=True)
    probe.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
