"""Two-robot Nav2 validation: namespaced localization and concurrent NavigateToPose goals.

Run in its own ROS container while standalone.py --robot ..., one
robot.launch.py container per robot and the fleet relay are running
(see tests/run_multi_robot_navigation.sh).
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/launch"
))
from robot_fleet import (  # noqa: E402
    MAP_REFERENCE, body_pose, load_robot_profile, parse_robot_spec,
)


def yaw_of(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Robot:
    def __init__(self, node, spec, goal):
        self.spec = spec
        self.name = spec.name
        self.goal = goal
        self.profile = load_robot_profile(spec.robot_type)
        self.latest = {}
        self.client = ActionClient(node, NavigateToPose, f"/{self.name}/navigate_to_pose")
        for topic, kind, key in (
            ("isaac/ground_truth/odom", Odometry, "truth"),
            ("odometry/global", Odometry, "estimate"),
            ("localization_3d/accepted_correction", Header, "correction"),
        ):
            node.create_subscription(
                kind, f"/{self.name}/{topic}",
                lambda message, key=key: self.latest.__setitem__(key, message),
                qos_profile_sensor_data,
            )
        self.handle = None
        self.result = None

    def true_map_pose(self, reference):
        """base_link pose in map from simulator truth.

        IsaacComputeOdometry reports the chassis pose relative to its start
        (spawn) pose, so compose it with the spawn first.
        """
        truth = self.latest["truth"].pose.pose
        sx, sy, syaw = self.spec.x, self.spec.y, self.spec.yaw
        rx, ry = truth.position.x, truth.position.y
        wx = sx + math.cos(syaw) * rx - math.sin(syaw) * ry
        wy = sy + math.sin(syaw) * rx + math.cos(syaw) * ry
        forward_yaw = math.pi if float(self.profile["simulation"]["forward_sign"]) < 0 else 0.0
        mx, my, _, myaw = reference
        dx, dy = wx - mx, wy - my
        cos, sin = math.cos(myaw), math.sin(myaw)
        return (cos * dx + sin * dy, -sin * dx + cos * dy,
                wrap(syaw + yaw_of(truth.orientation) + forward_yaw - myaw))

    def estimated_map_pose(self):
        pose = self.latest["estimate"].pose.pose
        return pose.position.x, pose.position.y, yaw_of(pose.orientation)


class Probe(Node):
    def __init__(self, specs, goals):
        super().__init__("multi_robot_navigation_validation", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True),
        ])
        self.robots = [Robot(self, spec, goal) for spec, goal in zip(specs, goals)]
        self.reference = body_pose(MAP_REFERENCE, load_robot_profile(MAP_REFERENCE.robot_type))
        self.fleet_frames = set()
        self.create_subscription(TFMessage, "/fleet/tf", lambda message: self.fleet_frames.update(
            (t.header.frame_id, t.child_frame_id) for t in message.transforms), 100)

    def wait(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"Timed out waiting for {what}")

    def wait_active(self, name, timeout):
        client = self.create_client(Trigger, f"/{name}/lifecycle_manager_navigation/is_active")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(Trigger.Request())
                self.wait(future.done, 10, f"{name} navigation state")
                if future.result().success:
                    return
            self.spin_for(1.0)
        raise TimeoutError(f"{name} navigation did not become active")

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def localization_errors(self):
        errors = {}
        for robot in self.robots:
            tx, ty, tyaw = robot.true_map_pose(self.reference)
            ex, ey, eyaw = robot.estimated_map_pose()
            errors[robot.name] = {
                "true": [tx, ty, tyaw], "estimate": [ex, ey, eyaw],
                "position": math.hypot(ex - tx, ey - ty), "yaw": abs(wrap(eyaw - tyaw)),
            }
        return errors

    def send(self, robot):
        x, y, yaw = robot.goal
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = robot.client.send_goal_async(NavigateToPose.Goal(pose=message))
        self.wait(future.done, 30, f"{robot.name} goal response")
        robot.handle = future.result()
        if not robot.handle.accepted:
            raise RuntimeError(f"{robot.name} rejected its goal")
        robot.result = robot.handle.get_result_async()


def parse_goal(text):
    values = [float(value) for value in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("goal must be X,Y,YAW")
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", action="append", required=True, help="NAME[:TYPE]@X,Y[,YAW]")
    parser.add_argument("--goal", action="append", required=True, type=parse_goal,
                        help="Map X,Y,YAW per --robot, same order")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--position-tolerance", type=float, default=0.35)
    parser.add_argument("--yaw-tolerance", type=float, default=0.15)
    parser.add_argument("--goal-tolerance", type=float, default=0.5)
    args = parser.parse_args()
    if len(args.robot) != len(args.goal):
        parser.error("Give one --goal per --robot")
    specs = [parse_robot_spec(text) for text in args.robot]
    rclpy.init()
    probe = Probe(specs, args.goal)
    report = {"robots": [text for text in args.robot], "goals": args.goal}
    status = 1
    try:
        required = {"truth", "estimate", "correction"}
        try:
            probe.wait(lambda: all(required <= set(robot.latest) for robot in probe.robots),
                       240, "namespaced truth, global odometry and accepted ICP corrections")
        except TimeoutError as error:
            missing = {robot.name: sorted(required - set(robot.latest)) for robot in probe.robots}
            raise TimeoutError(f"{error}; missing {missing}") from None
        # One RViz sees every robot: map -> <ns>/odom -> <ns>/base_link on /fleet/tf.
        expected = {edge for robot in probe.robots for edge in (
            ("map", f"{robot.name}/odom"), (f"{robot.name}/odom", f"{robot.name}/base_link"))}
        try:
            probe.wait(lambda: expected <= probe.fleet_frames, 60, "merged fleet TF")
        except TimeoutError as error:
            raise TimeoutError(f"{error}; missing {sorted(expected - probe.fleet_frames)}") from None
        report["fleet_tf"] = sorted(probe.fleet_frames)
        for robot in probe.robots:
            if not robot.client.wait_for_server(timeout_sec=120):
                raise RuntimeError(f"/{robot.name}/navigate_to_pose unavailable")
        # The action server exists from configure; goals are accepted once active.
        for robot in probe.robots:
            probe.wait_active(robot.name, 180)
        start = time.monotonic()
        while True:
            errors = probe.localization_errors()
            if all(e["position"] <= args.position_tolerance and e["yaw"] <= args.yaw_tolerance
                   for e in errors.values()):
                break
            if time.monotonic() - start > 60:
                raise AssertionError(f"Localization does not match simulator truth: {errors}")
            probe.spin_for(1.0)
        report["localization"] = errors
        print("LOCALIZATION " + json.dumps(errors), flush=True)
        for robot in probe.robots:
            probe.send(robot)
        print("GOALS_SENT", flush=True)
        probe.wait(lambda: all(robot.result.done() for robot in probe.robots), 300,
                   "both navigation results")
        outcomes = {}
        for robot in probe.robots:
            final = robot.true_map_pose(probe.reference)
            outcomes[robot.name] = {
                "status": robot.result.result().status, "final": final,
                "goal_distance": math.hypot(final[0] - robot.goal[0], final[1] - robot.goal[1]),
            }
        report["navigation"] = outcomes
        print("NAVIGATION " + json.dumps(outcomes), flush=True)
        if any(o["status"] != GoalStatus.STATUS_SUCCEEDED or
               o["goal_distance"] > args.goal_tolerance for o in outcomes.values()):
            raise AssertionError("A robot did not reach its goal")
        status = 0
        print("MULTI_ROBOT_NAVIGATION PASSED", flush=True)
    except Exception as error:  # noqa: BLE001 - record any failure in the report
        report["error"] = repr(error)
        print(f"MULTI_ROBOT_NAVIGATION FAILED: {error}", flush=True)
    finally:
        args.output.write_text(json.dumps(report, indent=2))
        probe.destroy_node()
        rclpy.shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
