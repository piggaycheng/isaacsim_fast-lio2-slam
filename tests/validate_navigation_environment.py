"""ROS-side Nav2 validation with real Isaac Sim LiDAR and physical test obstacles."""

import argparse
import hashlib
import json
import math
import signal
import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformListener

from validate_braking import BrakingProbe, rectangle, separation


CASES = (
    "baseline", "crossing_slow", "crossing_fast", "blocked_then_clear",
    "blocked_then_clear_open",
    "corridor_1.8", "corridor_1.6", "corridor_1.4", "corridor_0.6",
)


def walls(width):
    return [
        {"offset": [2.0, sign * (width / 2 + 0.1)], "sizes": [2.4, 0.2, 1.2]}
        for sign in (-1, 1)
    ]


PLANNING_CONFIG = Path(__file__).resolve().parents[1] / (
    "ros2_ws/src/slam_localization_3d/config/observation_costmaps.yaml"
)


def planning_corridor_width(config_path=PLANNING_CONFIG):
    config = yaml.safe_load(Path(config_path).read_text())
    params = config["global_costmap"]["global_costmap"]["ros__parameters"]
    footprint = np.asarray(json.loads(params["footprint"]))
    return float(np.ptp(footprint[:, 1]) + 2 * params["footprint_padding"])


def configured_corridor_width(config_path=PLANNING_CONFIG, adaptive_surround=False):
    """Nominal aligned safety width, not a guarantee of traversability."""
    if adaptive_surround:
        profile = yaml.safe_load(Path(config_path).with_name("adaptive_surround.yaml").read_text())
        points = np.asarray(profile["adaptive_surround"]["ros__parameters"]["crawl_points"])
    else:
        monitor_path = Path(config_path).with_name("collision_monitor.yaml")
        monitor = yaml.safe_load(monitor_path.read_text())
        points = np.asarray(monitor["collision_monitor"]["ros__parameters"]["PolygonSurround"]["points"])
    return max(planning_corridor_width(config_path), float(np.ptp(points.reshape(-1, 2)[:, 1])))


def scenario(name, corridor_width=None):
    if name not in CASES:
        raise ValueError(f"Unknown navigation validation case: {name}")
    if name == "baseline":
        return [], 2.5, False
    if name.startswith("corridor_"):
        width = float(name.split("_")[1])
        if corridor_width is None:
            corridor_width = configured_corridor_width()
        return walls(width), 3.5, width <= corridor_width
    if name == "blocked_then_clear_open":
        return [{
            "offset": [1.05, 0.0], "sizes": [0.2, 2.0, 1.2],
            "velocity": [0.0, 0.8], "duration": 3.25,
        }], 3.5, False
    if name == "blocked_then_clear":
        return [*walls(1.8), {
            "offset": [2.0, 0.0], "sizes": [0.3, 1.8, 1.2],
            "velocity": [0.0, 0.8], "duration": 3.5,
        }], 3.5, False
    speed = 0.35 if name == "crossing_slow" else 0.65
    side = 1 if name == "crossing_slow" else -1
    start = 1.0 if name == "crossing_slow" else 1.8
    return [{
        "offset": [2.0, side * start], "sizes": [0.4, 0.4, 1.2],
        "velocity": [0.0, -side * speed], "duration": (start + 1.8) / speed,
    }], 3.5, False


def obstacle_polygon(obstacle, stamp, motion_started):
    elapsed = 0.0 if motion_started is None else min(
        max(stamp - motion_started, 0.0), obstacle["duration"],
    )
    center = np.asarray(obstacle["center"][:2]) + elapsed * np.asarray(obstacle["velocity"])
    sx, sy, _ = obstacle["sizes"]
    return rectangle([-sx / 2, sx / 2, -sy / 2, sy / 2], center, obstacle["yaw"])


def local_position(position, anchor):
    delta = np.asarray(position) - np.asarray(anchor[:2])
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])


def clearance_metrics(samples, obstacles, motion_started, bounds, footprint_bounds):
    if len(samples) < 2:
        raise ValueError("At least two ground-truth samples are required")
    gaps = np.diff([row["stamp"] for row in samples])
    if (gaps <= 0).any():
        raise ValueError("Ground truth must have increasing simulation timestamps")
    minimum, padded = math.inf, math.inf
    for row in samples:
        position, yaw = row["position"], row["yaw"]
        for obstacle in obstacles:
            polygon = obstacle_polygon(obstacle, row["stamp"], motion_started)
            minimum = min(minimum, separation(rectangle(bounds, position, yaw), polygon))
            padded = min(padded, separation(rectangle(footprint_bounds, position, yaw), polygon))
    # Account for motion between 60 Hz samples, not just disjoint sampled rectangles.
    radius = math.hypot(max(abs(bounds[0]), abs(bounds[1])),
                        max(abs(bounds[2]), abs(bounds[3])))
    obstacle_speed = max((math.hypot(*o["velocity"]) for o in obstacles), default=0.0)
    allowance = float(max(gaps)) / 2 * (1.0 + 0.75 * radius + obstacle_speed)
    return {
        "truth_samples": len(samples), "max_truth_gap_s": float(max(gaps)),
        "min_body_separation_m": None if not obstacles else minimum,
        "min_padded_footprint_separation_m": None if not obstacles else padded,
        "between_samples_motion_allowance_m": allowance,
        "min_continuous_clearance_lower_bound_m": None if not obstacles else minimum - allowance,
        "geometric_overlap": bool(obstacles and minimum <= 0),
    }


def classify_clearance_event(samples, commands, obstacles, motion_started, bounds,
                             allowance, actor_motion_verified, actor_stamps=()):
    """Diagnose the first clearance breach; never waive collision-free acceptance."""
    result = {
        "clearance_classification": "no_clearance_breach",
        "stop_before_intrusion_verified": None,
        "first_clearance_breach_sim_s": None,
        "classification_reason": None,
    }
    event = None
    breached = []
    for row in samples:
        breached = [obstacle for obstacle in obstacles if separation(
            rectangle(bounds, row["position"], row["yaw"]),
            obstacle_polygon(obstacle, row["stamp"], motion_started),
        ) - allowance < 0.02]
        if breached:
            event = row
            break
    if event is None:
        return result
    result.update(
        clearance_classification="clearance_breach_unverified",
        stop_before_intrusion_verified=False,
        first_clearance_breach_sim_s=event["stamp"],
    )

    def unverified(reason):
        result["classification_reason"] = reason
        return result

    start = event["stamp"] - 0.25
    before = [row for row in samples if row["stamp"] <= start]
    if not before:
        return unverified("Insufficient pre-breach stationary history")
    window = [row for row in samples if before[-1]["stamp"] <= row["stamp"] <= event["stamp"]]
    if max(np.diff([row["stamp"] for row in window])) > 0.15:
        return unverified("Ground-truth sampling gap exceeds 0.15 s")
    if any("speed" not in row or "angular_speed" not in row for row in window):
        return unverified("Missing linear or angular ground-truth velocity")
    if any(not np.isfinite([row["speed"], row["angular_speed"], row["yaw"],
                            *row["position"]]).all() for row in window):
        return unverified("Non-finite ground-truth stationary evidence")
    if any(row["speed"] >= 0.03 or row["angular_speed"] >= 0.03 for row in window):
        result["clearance_classification"] = (
            "robot_motion_clearance_breach"
            if event["speed"] >= 0.03 or event["angular_speed"] >= 0.03
            else "insufficient_stationary_dwell"
        )
        return unverified("Robot was not stationary throughout the preceding 0.25 s")
    initial = window[0]
    if any(np.linalg.norm(np.asarray(row["position"]) - initial["position"]) > 0.01
           or abs(math.atan2(math.sin(row["yaw"] - initial["yaw"]),
                             math.cos(row["yaw"] - initial["yaw"]))) > 0.01
           for row in window):
        return unverified("Stationary pose drift exceeds 1 cm or 0.01 rad")
    final = [row for row in commands if row["source"] == "final" and row["stamp"] <= event["stamp"]]
    prior = [row for row in final if row["stamp"] <= start]
    if not prior:
        return unverified("No final command evidence before the stationary window")
    final = [row for row in final if row["stamp"] >= prior[-1]["stamp"]]
    stamps = [row["stamp"] for row in final] + [event["stamp"]]
    if start - stamps[0] > 0.15 or max(np.diff(stamps)) > 0.15:
        return unverified("Final command evidence is stale or has gaps")
    if any(not np.isfinite([row["linear"], row["angular"]]).all()
           or abs(row["linear"]) > 1e-6 or abs(row["angular"]) > 1e-6 for row in final):
        return unverified("Final linear and angular commands were not continuously zero")
    if not actor_motion_verified:
        return unverified("Actual moving-actor telemetry is not verified")
    actor_window = [stamp for stamp in actor_stamps if stamp <= event["stamp"]]
    prior_actor = [stamp for stamp in actor_window if stamp <= initial["stamp"]]
    if not prior_actor:
        return unverified("No actor telemetry before the stationary window")
    actor_window = [stamp for stamp in actor_window if stamp >= prior_actor[-1]]
    if (initial["stamp"] - actor_window[0] > 0.15
            or max(np.diff(actor_window + [event["stamp"]])) > 0.15):
        return unverified("Actor telemetry is stale or has gaps during the stationary window")
    for obstacle in breached:
        if (motion_started is None or math.hypot(*obstacle["velocity"]) <= 0
                or not motion_started < event["stamp"] < motion_started + obstacle["duration"]):
            return unverified("A breached obstacle was not actively moving")
        frozen_actor = obstacle_polygon(obstacle, initial["stamp"], motion_started)
        if any(separation(rectangle(bounds, row["position"], row["yaw"]), frozen_actor)
               - allowance < 0.02 for row in window):
            return unverified("Robot motion alone could breach clearance against the frozen actor")
        frozen_robot = rectangle(bounds, initial["position"], initial["yaw"])
        if separation(frozen_robot, obstacle_polygon(
                obstacle, event["stamp"], motion_started)) - allowance >= 0.02:
            return unverified("Actor motion alone does not explain the breach")
    result.update(
        clearance_classification="stationary_moving_actor_intrusion",
        stop_before_intrusion_verified=True,
        classification_reason="Verified stationary pose and zero commands for at least 0.25 s; actor motion causes intrusion",
    )
    return result


class EnvironmentProbe(Node):
    def __init__(self, directory, geometry, planning_config=PLANNING_CONFIG, adaptive_surround=False):
        super().__init__("navigation_environment_validation", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True),
        ])
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.sequence = time.time_ns()
        self.planning_config = Path(planning_config)
        self.planning_width = planning_corridor_width(self.planning_config)
        self.adaptive_surround = adaptive_surround
        self.corridor_width = configured_corridor_width(self.planning_config, adaptive_surround)
        self.adaptive_limits = []
        minima = np.minimum(geometry["visible"]["min"], geometry["collision"]["min"])
        maxima = np.maximum(geometry["visible"]["max"], geometry["collision"]["max"])
        self.bounds = [minima[0], maxima[0], minima[1], maxima[1]]
        footprint = np.asarray(geometry["footprint"])
        fmin = footprint.min(axis=0) - geometry["footprint_padding"]
        fmax = footprint.max(axis=0) + geometry["footprint_padding"]
        self.footprint_bounds = [fmin[0], fmax[0], fmin[1], fmax[1]]
        self.latest = {}
        self.samples = None
        self.commands = []
        self.goal = None
        self.result = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.emergency = self.create_publisher(Bool, "/navigation/emergency_stop", 10)
        for topic, kind, key in [
            ("/isaac/ground_truth/odom", Odometry, "truth"),
            ("/cmd_vel", Twist, "final"),
            ("/nav2/cmd_vel_nav", Twist, "raw"),
            ("/navigation/adaptive_surround_limits", TwistStamped, "adaptive_limits"),
            ("/perception/obstacles", PointCloud2, "cloud"),
            ("/localization_3d/accepted_correction", Header, "correction"),
            ("/global_costmap/costmap", OccupancyGrid, "global_costmap"),
            ("/local_costmap/costmap", OccupancyGrid, "local_costmap"),
        ]:
            self.create_subscription(
                kind, topic, lambda msg, key=key: self.receive(key, msg),
                qos_profile_sensor_data,
            )

    def verify_costmap_geometry(self):
        config = yaml.safe_load(self.planning_config.read_text())
        for name in ("global_costmap", "local_costmap"):
            expected = config[name][name]["ros__parameters"]
            client = self.create_client(GetParameters, f"/{name}/{name}/get_parameters")
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f"{name} parameter service is unavailable")
            future = client.call_async(GetParameters.Request(names=["footprint", "footprint_padding"]))
            self.wait(future.done, 30)
            values = future.result().values
            if (len(values) != 2 or values[0].type != ParameterType.PARAMETER_STRING
                    or values[1].type != ParameterType.PARAMETER_DOUBLE
                    or not np.allclose(json.loads(values[0].string_value),
                                       json.loads(expected["footprint"]), atol=1e-9, rtol=0)
                    or abs(values[1].double_value - expected["footprint_padding"]) > 1e-9):
                raise AssertionError(f"Live {name} footprint does not match validation configuration")
            print(f"VERIFIED_GEOMETRY {name}: {values[0].string_value}, "
                  f"padding={values[1].double_value}", flush=True)
        if self.adaptive_surround:
            client = self.create_client(GetParameters, "/cmd_vel_safety/get_parameters")
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError("Adaptive final gate parameter service unavailable")
            future = client.call_async(GetParameters.Request(names=["require_adaptive_limits"]))
            self.wait(future.done, 30)
            values = future.result().values
            if (len(values) != 1 or values[0].type != ParameterType.PARAMETER_BOOL
                    or not values[0].bool_value):
                raise AssertionError("Adaptive physical validation requires mandatory final-gate limits")
            self.wait(lambda: "adaptive_limits" in self.latest, 30)

    def receive(self, key, message):
        self.latest[key] = message
        if self.samples is not None and key == "truth":
            position, yaw = BrakingProbe.pose(message)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if not self.samples or stamp > self.samples[-1]["stamp"]:
                self.samples.append({
                    "stamp": stamp, "position": position.tolist(), "yaw": yaw,
                    "speed": math.hypot(message.twist.twist.linear.x,
                                        message.twist.twist.linear.y),
                    "angular_speed": abs(message.twist.twist.angular.z),
                })
        if self.samples is not None and key in ("raw", "final"):
            self.commands.append({
                "stamp": self.now(), "source": key,
                "linear": message.linear.x, "angular": message.angular.z,
            })
        if self.samples is not None and key == "adaptive_limits":
            self.adaptive_limits.append({
                "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                "linear": message.twist.linear.x, "angular": message.twist.angular.z,
            })

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, predicate, timeout=90):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise TimeoutError("Navigation validation condition not reached")

    def hold(self, seconds):
        deadline = self.now() + seconds
        self.wait(lambda: self.now() >= deadline, max(30, seconds * 12))

    def control(self, action, **kwargs):
        self.sequence += 1
        temp = self.directory / "request.tmp"
        temp.write_text(json.dumps({"sequence": self.sequence, "action": action, **kwargs}))
        temp.replace(self.directory / "request.json")
        path = self.directory / "response.json"
        self.wait(lambda: path.exists() and json.loads(path.read_text())["sequence"] == self.sequence)
        response = json.loads(path.read_text())
        if action == "start_motion" and abs(response["sim_time"] - self.now()) > 0.15:
            raise RuntimeError("Obstacle motion clock does not match ROS simulation time")
        return response

    def map_pose(self):
        pose = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        q = pose.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return [pose.translation.x, pose.translation.y, yaw]

    def navigate(self, target):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = target[:2]
        message.pose.orientation.z, message.pose.orientation.w = (
            math.sin(target[2] / 2), math.cos(target[2] / 2),
        )
        future = self.client.send_goal_async(NavigateToPose.Goal(pose=message))
        self.wait(future.done)
        self.goal = future.result()
        if not self.goal.accepted:
            raise RuntimeError("Nav2 rejected validation goal")
        self.result = self.goal.get_result_async()

    def stop(self):
        if self.goal is not None and self.result is not None and not self.result.done():
            future = self.goal.cancel_goal_async()
            self.wait(future.done)
            future.result()
            self.wait(self.result.done)
        self.wait(lambda: math.hypot(
            self.latest["truth"].twist.twist.linear.x,
            self.latest["truth"].twist.twist.linear.y,
        ) < 0.02 and self.latest.get("final") == Twist())
        self.hold(0.5)

    def return_home(self, home):
        self.samples = None
        self.stop()
        self.control("hide")
        self.hold(3.0)
        self.navigate(home)
        self.wait(self.result.done, 240)
        if self.result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("Could not safely return to validation start")
        self.stop()
        self.hold(1.0)

    def case(self, name, repeat):
        print(f"ENVIRONMENT_CASE {name} repeat={repeat}", flush=True)
        definitions, distance, blocked = scenario(name, self.corridor_width)
        start_map = self.map_pose()
        start_truth, yaw = BrakingProbe.pose(self.latest["truth"])
        anchor = [*start_truth.tolist(), yaw]
        response = None
        if definitions:
            response = self.control("configure", obstacles=definitions, record_telemetry=True)
            if (np.linalg.norm(np.asarray(response["anchor"][:2]) - start_truth) > 0.05
                    or abs(math.atan2(math.sin(response["anchor"][2] - yaw),
                                     math.cos(response["anchor"][2] - yaw))) > 0.05):
                raise AssertionError("Stage and ground-truth obstacle frames do not match")
            anchor = response["anchor"]
        target = [
            start_map[0] + distance * math.cos(start_map[2]),
            start_map[1] + distance * math.sin(start_map[2]), start_map[2],
        ]
        self.hold(1.0)
        self.samples, self.commands, self.adaptive_limits = [], [], []
        self.navigate(target)
        began, wall_began = self.now(), time.monotonic()
        motion_started = None
        still_since = None
        max_stop = 0.0
        failure = None
        emergency_stop_requested = False
        obstacles = [] if response is None else response["obstacles"]
        actor_samples = []
        max_actor_model_error = 0.0
        deadline = 12.0 if blocked else 75.0
        while not self.result.done() and self.now() - began < deadline:
            if time.monotonic() - wall_began > 300:
                failure = "Wall-clock timeout or stalled simulation"
                break
            rclpy.spin_once(self, timeout_sec=0.02)
            telemetry_path = self.directory / "telemetry.json"
            if obstacles and telemetry_path.exists():
                telemetry = json.loads(telemetry_path.read_text())
                if telemetry["sequence"] == self.sequence and (
                        not actor_samples or telemetry["sim_time"] > actor_samples[-1]["sim_time"]):
                    if len(telemetry["centers"]) != len(obstacles):
                        raise AssertionError("Scene telemetry is missing obstacle poses")
                    actor_samples.append(telemetry)
                    for obstacle, center in zip(obstacles, telemetry["centers"]):
                        expected = obstacle_polygon(
                            obstacle, telemetry["sim_time"], telemetry["motion_started"],
                        ).mean(axis=0)
                        max_actor_model_error = max(
                            max_actor_model_error, float(np.linalg.norm(expected - center[:2])),
                        )
                    if max_actor_model_error > 0.02:
                        raise AssertionError("Actual obstacle poses differ from the clearance model")
            position, current_yaw = BrakingProbe.pose(self.latest["truth"])
            forward, _ = local_position(position, anchor)
            speed = math.hypot(self.latest["truth"].twist.twist.linear.x,
                               self.latest["truth"].twist.twist.linear.y)
            if speed < 0.03 and abs(self.latest["truth"].twist.twist.angular.z) < 0.03:
                if still_since is None:
                    still_since = self.now()
                max_stop = max(max_stop, self.now() - still_since)
            else:
                still_since = None
            if motion_started is None and name.startswith("crossing_") and forward >= 0.35:
                motion_started = self.control("start_motion")["sim_time"]
            if (motion_started is None and name.startswith("blocked_then_clear")
                    and still_since is not None
                    and self.now() - still_since >= 2.0):
                motion_started = self.control("start_motion")["sim_time"]
            for obstacle in obstacles:
                gap = separation(rectangle(self.bounds, position, current_yaw),
                                 obstacle_polygon(obstacle, self.now(), motion_started))
                if gap < 0.02:
                    failure = "Physical clearance below 2 cm; emergency stop requested"
                    self.emergency.publish(Bool(data=True))
                    emergency_stop_requested = True
                    break
            if failure:
                break
        status = self.result.result().status if self.result.done() else None
        elapsed, wall_elapsed = self.now() - began, time.monotonic() - wall_began
        if status is None and not blocked and failure is None:
            failure = "Navigation simulation-time deadline exceeded"
        self.stop()
        samples = self.samples
        self.samples = None
        metrics = clearance_metrics(
            samples, obstacles, motion_started, self.bounds, self.footprint_bounds,
        )
        if self.adaptive_surround:
            if not any(row["linear"] == 0.1 and row["angular"] == 0.2 for row in self.adaptive_limits):
                failure = failure or "Adaptive crawl profile was never confirmed during trial"
        classification = classify_clearance_event(
            samples, self.commands, obstacles, motion_started, self.bounds,
            metrics["between_samples_motion_allowance_m"],
            actor_motion_verified=len(actor_samples) >= 2 and max_actor_model_error <= 0.02,
            actor_stamps=[row["sim_time"] for row in actor_samples],
        )
        positions = np.array([local_position(row["position"], anchor) for row in samples])
        endpoint = positions[-1]
        endpoint_error = float(np.linalg.norm(endpoint - [distance, 0.0]))
        endpoint_yaw_error = abs(math.atan2(
            math.sin(samples[-1]["yaw"] - anchor[2]), math.cos(samples[-1]["yaw"] - anchor[2]),
        ))
        width = float(name.split("_")[1]) if name.startswith("corridor_") else (
            1.8 if name == "blocked_then_clear" else None
        )
        entered = positions[:, 0] >= 0.8
        max_lateral = float(max(abs(positions[entered, 1]))) if entered.any() else None
        crossed_entrance = bool(entered.any() and abs(positions[entered][0, 1]) < width / 2) \
            if width is not None else None
        safe = (
            metrics["max_truth_gap_s"] <= 0.15 and not metrics["geometric_overlap"]
            and (not obstacles or len(actor_samples) >= 2)
            and (not obstacles or metrics["min_continuous_clearance_lower_bound_m"] >= 0.02)
        )
        if blocked:
            behavior = status != GoalStatus.STATUS_SUCCEEDED and not entered.any()
        else:
            behavior = (
                status == GoalStatus.STATUS_SUCCEEDED and endpoint_error <= 0.30
                and endpoint_yaw_error <= 0.35
            )
            if width is not None:
                behavior = behavior and crossed_entrance and max_lateral <= width / 2
            if name.startswith(("crossing_", "blocked_then_clear")):
                behavior = behavior and motion_started is not None
        crossing_observed = None
        if name.startswith("crossing_"):
            crossing_time = None if motion_started is None else motion_started + abs(
                definitions[0]["offset"][1] / definitions[0]["velocity"][1],
            )
            crossing_observed = (
                crossing_time is not None and began <= crossing_time <= began + elapsed
            )
            behavior = behavior and crossing_observed
        moving_actor_observed = None
        if name.startswith(("crossing_", "blocked_then_clear")):
            moving_index = len(obstacles) - 1 if name.startswith("blocked_then_clear") else 0
            moving_centers = [row["centers"][moving_index][:2] for row in actor_samples]
            moving_actor_observed = (
                len(moving_centers) >= 2
                and np.linalg.norm(np.asarray(moving_centers[-1]) - moving_centers[0]) >= 0.1
            )
            behavior = behavior and moving_actor_observed
        if name.startswith("blocked_then_clear"):
            behavior = behavior and max_stop >= 2.0
        if failure is None and not safe:
            failure = "Clearance, overlap, or ground-truth sampling criterion failed"
        if failure is None and not behavior:
            failure = "Navigation completion, corridor traversal, or dynamic exposure criterion failed"
        result = {
            "case": name, "repeat": repeat,
            "expected": "safe_stop_or_rejection" if blocked else "traversal",
            "pass": bool(safe and behavior and failure is None), "failure": failure,
            "collision_free_pass": bool(safe),
            "emergency_stop_requested": emergency_stop_requested,
            "planning_corridor_min_width_m": self.planning_width,
            "configured_corridor_min_width_m": self.corridor_width,
            **classification,
            "navigation_status_before_cancel": status, "elapsed_sim_s": elapsed,
            "elapsed_wall_s": wall_elapsed, "endpoint_error_m": endpoint_error,
            "endpoint_yaw_error_rad": endpoint_yaw_error,
            "corridor_width_m": width, "crossed_corridor_entrance": crossed_entrance,
            "max_corridor_lateral_offset_m": max_lateral,
            "max_forward_progress_m": float(max(positions[:, 0])),
            "longest_physical_stop_s": max_stop, "motion_started_sim_s": motion_started,
            "actor_crossed_route_during_navigation": crossing_observed,
            "moving_actor_observed": None if moving_actor_observed is None else bool(moving_actor_observed),
            "max_actor_pose_model_error_m": max_actor_model_error,
            "max_raw_linear_command_mps": max(
                (abs(row["linear"]) for row in self.commands if row["source"] == "raw"),
                default=0.0,
            ),
            "max_final_linear_command_mps": max(
                (abs(row["linear"]) for row in self.commands if row["source"] == "final"),
                default=0.0,
            ),
            **metrics, "anchor_world": anchor, "target_map": target, "obstacles": obstacles,
            "trajectory": samples, "commands": self.commands, "actor_samples": actor_samples,
            "adaptive_limits": self.adaptive_limits,
        }
        print("ENVIRONMENT_RESULT " + json.dumps(
            {key: value for key, value in result.items()
             if key not in ("trajectory", "commands", "actor_samples", "adaptive_limits")},
        ), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-dir", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, default=PLANNING_CONFIG)
    parser.add_argument("--adaptive-surround", action="store_true")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()

    def interrupted(signum, frame):
        raise RuntimeError(f"Environment probe interrupted by signal {signum}")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    probe = EnvironmentProbe(args.control_dir, json.loads(args.geometry.read_text()),
                             args.planning_config, args.adaptive_surround)
    results = []
    try:
        probe.wait(lambda: all(
            key in probe.latest for key in ("truth", "final", "cloud", "correction",
                                           "global_costmap", "local_costmap")
        ) and probe.buffer.can_transform("map", "base_link", rclpy.time.Time()), 180)
        if not probe.client.wait_for_server(timeout_sec=30):
            raise RuntimeError("NavigateToPose action server unavailable")
        probe.verify_costmap_geometry()
        probe.control("hide")
        probe.hold(2.0)
        # Align with the open +map-x aisle before placing robot-relative obstacles.
        home = [0.4, 0.0, 0.0]
        probe.navigate(home)
        probe.wait(probe.result.done, 180)
        if probe.result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("Could not align Carter with the clear Office aisle")
        probe.stop()
        probe.hold(1.0)
        for repeat in range(args.repeats):
            for name in args.cases:
                results.append(probe.case(name, repeat))
                args.output.write_text(json.dumps({
                    "criteria": {
                        "continuous_body_clearance_min_m": 0.02, "truth_gap_max_s": 0.15,
                        "endpoint_error_max_m": 0.30, "endpoint_yaw_error_max_rad": 0.35,
                        "navigation_success_status": GoalStatus.STATUS_SUCCEEDED,
                        "stationary_window_s": 0.25, "stationary_linear_speed_max_mps": 0.03,
                        "stationary_angular_speed_max_radps": 0.03,
                        "stationary_pose_drift_max_m": 0.01,
                        "stationary_yaw_drift_max_rad": 0.01,
                        "overall_collision_clearance_is_not_waived": True,
                    },
                    "planning": {
                        "config": str(args.planning_config),
                        "sha256": hashlib.sha256(args.planning_config.read_bytes()).hexdigest(),
                        "minimum_corridor_width_m": probe.planning_width,
                        "configured_corridor_min_width_m": probe.corridor_width,
                        "adaptive_surround": args.adaptive_surround,
                        "adaptive_profile_sha256": hashlib.sha256(
                            args.planning_config.with_name("adaptive_surround.yaml").read_bytes()
                        ).hexdigest() if args.adaptive_surround else None,
                    },
                    "results": results,
                }, indent=2) + "\n")
                if results[-1]["emergency_stop_requested"]:
                    raise AssertionError("Unsafe clearance; stopping suite with emergency stop latched")
                probe.return_home(home)
        if not all(row["pass"] for row in results):
            raise AssertionError("Navigation environment acceptance criteria not met; inspect result JSON")
        print(f"PASS: {len(results)} navigation environment trials", flush=True)
    finally:
        try:
            if "truth" in probe.latest:
                probe.stop()
                probe.control("hide")
        finally:
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
