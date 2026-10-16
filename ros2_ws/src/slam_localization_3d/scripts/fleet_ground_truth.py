#!/usr/bin/env python3
"""Approximate simulator reference markers; never an input to localization."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from floor_maps import validate_initial_pose
from localization_3d_pose import body_to_base, compose_pose, pose_values, rotate_vector, valid_pose


def inverse_pose(pose):
    position, quaternion = pose
    inverse = tuple(-value for value in quaternion[:3]) + (quaternion[3],)
    return rotate_vector(inverse, tuple(-value for value in position)), inverse


def reference_pose(data):
    position = tuple(float(value) for value in data["position"])
    quaternion = tuple(float(value) for value in data["orientation_xyzw"])
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError("Reference pose needs position[3] and orientation_xyzw[4]")
    if not all(math.isfinite(value) for value in (*position, *quaternion)):
        raise ValueError("Reference pose must be finite")
    norm = sum(value * value for value in quaternion)
    if not math.isclose(norm, 1.0, abs_tol=1e-3):
        raise ValueError("Reference orientation must be a unit quaternion")
    return position, tuple(value / math.sqrt(norm) for value in quaternion)


@dataclass(frozen=True)
class FloorReference:
    name: str
    map_from_world: tuple
    world_height: float

    @classmethod
    def load(cls, name, path):
        with Path(path).open(encoding="utf-8") as stream:
            session = json.load(stream)
        if session["floor"] != name:
            raise ValueError(f"session floor does not match {name!r}")
        reference = session["reference"]
        if not isinstance(reference, dict):
            raise ValueError("session has no mapping reference")
        lio = reference["lio_initial_pose"]
        chassis = reference["isaac_chassis_pose"]
        if chassis["frame_id"] != "isaac_world":
            raise ValueError("Reference chassis pose must be in isaac_world")
        if lio["frame_id"] != "lidar" or lio["child_frame_id"] != "body":
            raise ValueError("Reference LIO pose must describe lidar -> body")
        mount = reference["sensor_frames"]["imu_link"]
        body_base = body_to_base([float(mount[key]) for key in
                                  ("x", "y", "z", "roll", "pitch", "yaw")])
        world_base = reference_pose(chassis)
        map_base = compose_pose(reference_pose(lio), body_base)
        return cls(name, compose_pose(map_base, inverse_pose(world_base)), world_base[0][2])

    def project(self, pose):
        return compose_pose(self.map_from_world, pose)

    def initial_body_pose(self, world_body):
        x, y, z, yaw = world_body
        position, quaternion = self.project(
            ((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))
        qx, qy, qz, qw = quaternion
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return (*position, yaw)


def floor_at_height(references, height, tolerance):
    matches = [reference for reference in references
               if abs(reference.world_height - height) <= tolerance]
    return matches[0] if len(matches) == 1 else None


def reference_markers(robot, pose, stamp):
    position, quaternion = pose
    x, y, z, w = quaternion
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    markers = []
    for index, kind in enumerate((Marker.ARROW, Marker.TEXT_VIEW_FACING)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = f"simulation_reference/{robot}"
        marker.id, marker.type, marker.action = index, kind, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = float(position[0]), float(position[1])
        marker.pose.position.z = 0.1 if index == 0 else 0.5
        marker.pose.orientation.w = 1.0
        if index == 0:
            marker.pose.orientation.z = math.sin(yaw / 2)
            marker.pose.orientation.w = math.cos(yaw / 2)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.9, 0.14, 0.14
        else:
            marker.text = f"{robot} [SIM REF ~]"
            marker.scale.z = 0.24
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.9, 0.1, 0.9
        marker.lifetime.sec = 1
        markers.append(marker)
    return markers


def delete_markers(robot):
    markers = []
    for index in range(2):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns, marker.id, marker.action = f"simulation_reference/{robot}", index, Marker.DELETE
        markers.append(marker)
    return markers


class FleetGroundTruth(Node):
    def __init__(self, **kwargs):
        super().__init__("fleet_ground_truth", **kwargs)
        self.robots = list(self.declare_parameter("robots", ["carter1"]).value)
        floors = list(self.declare_parameter("floors", ["1F", "B1"]).value)
        sessions = list(self.declare_parameter("session_files", ["", ""]).value)
        if (not self.robots or not all(self.robots) or len(set(self.robots)) != len(self.robots)
                or not floors or not all(floors) or len(set(floors)) != len(floors)
                or len(floors) != len(sessions)):
            raise ValueError("Unique robots/floors and one session_files entry per floor are required")
        spawn_robots = list(self.declare_parameter("spawn_robots", [""]).value)
        spawn_poses = list(self.declare_parameter("spawn_poses", [0.0, 0.0, 0.0]).value)
        if spawn_robots == [""]:
            spawn_robots, spawn_poses = [], []
        if (len(set(spawn_robots)) != len(spawn_robots)
                or not set(spawn_robots).issubset(self.robots)
                or len(spawn_poses) != 3 * len(spawn_robots)
                or not all(math.isfinite(value) for value in spawn_poses)):
            raise ValueError("spawn_poses needs finite X,Y,YAW for each unique spawn_robots entry")
        self.world_from_spawn = {}
        for index, robot in enumerate(spawn_robots):
            x, y, yaw = spawn_poses[3 * index:3 * index + 3]
            self.world_from_spawn[robot] = (
                (x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        for robot in self.robots:
            if robot not in self.world_from_spawn:
                self.get_logger().warning(
                    f"No spawn pose for {robot}; simulation reference disabled. "
                    "Launch Fleet RViz with full NAME[:TYPE][#FLEET]@X,Y,YAW robot specs.")
        self.height_tolerance = float(self.declare_parameter("height_tolerance", 0.75).value)
        self.stale_after = float(self.declare_parameter("stale_after", 2.0).value)
        if not all(math.isfinite(value) and value > 0
                   for value in (self.height_tolerance, self.stale_after)):
            raise ValueError("height_tolerance and stale_after must be finite and positive")
        self.references = {}
        self.marker_publishers = {}
        self.truth = {}
        self.initialized_robots = set()
        for index, (floor, path) in enumerate(zip(floors, sessions)):
            self.marker_publishers[floor] = self.create_publisher(
                MarkerArray, f"/fleet/maps/floor_{index}/simulation_reference",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            try:
                self.references[floor] = FloorReference.load(floor, path)
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.get_logger().warning(
                    f"No simulation reference for {floor}: {path}: {error}; markers disabled on this floor")
        for robot in self.robots:
            self.create_subscription(
                Odometry, f"/{robot}/isaac/ground_truth/odom",
                lambda message, robot=robot: self.on_truth(robot, message),
                qos_profile_sensor_data)
            self.create_subscription(
                PoseWithCovarianceStamped, f"/{robot}/initialpose",
                lambda message, robot=robot: self.on_initial_pose(robot, message), 10)
        self.create_timer(0.2, self.publish_references, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            "Yellow SIM REF markers are approximate session-based projections, not localization. "
            f"References available: {', '.join(self.references) or 'none'}. Waiting for simulator truth.")

    def on_truth(self, robot, message):
        if robot not in self.world_from_spawn:
            return
        if message.header.frame_id != "isaac_world" or not valid_pose(message.pose.pose):
            self.truth.pop(robot, None)
            self.get_logger().warning(
                f"Ignoring invalid simulator truth for {robot}: expected a valid isaac_world pose",
                throttle_duration_sec=5.0)
            return
        self.truth[robot] = (message, time.monotonic())

    def on_initial_pose(self, robot, message):
        try:
            validate_initial_pose(message)
        except ValueError as error:
            self.get_logger().warning(f"Not clearing simulation reference for {robot}: {error}")
            return
        self.initialized_robots.add(robot)
        self.publish_references()

    def publish_references(self):
        snapshots = {floor: [] for floor in self.marker_publishers}
        now, stamp = time.monotonic(), self.get_clock().now().to_msg()
        for robot in self.robots:
            sample = self.truth.get(robot)
            reference = None
            if (robot not in self.initialized_robots and sample is not None
                    and now - sample[1] < self.stale_after):
                reference = floor_at_height(
                    self.references.values(), sample[0].pose.pose.position.z, self.height_tolerance)
            for floor, markers in snapshots.items():
                if reference is not None and floor == reference.name:
                    # Isaac odometry is relative to each robot's spawn, despite its frame label.
                    world_pose = compose_pose(
                        self.world_from_spawn[robot], pose_values(sample[0].pose.pose))
                    markers.extend(reference_markers(
                        robot, reference.project(world_pose), stamp))
                else:
                    markers.extend(delete_markers(robot))
        for floor, markers in snapshots.items():
            self.marker_publishers[floor].publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = FleetGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
