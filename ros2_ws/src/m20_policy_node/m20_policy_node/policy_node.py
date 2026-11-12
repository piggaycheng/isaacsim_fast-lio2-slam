#!/usr/bin/env python3
"""ROS 2 node running the M20 locomotion policy outside the simulator / robot process.

Subscribes (all topic names are parameters):
  /m20/joint_states    sensor_msgs/JointState   position + velocity of the 16 M20 joints, by name.
                                                 Inference runs on a timer (control_rate_hz, default 50) with the latest
                                                 state; with control_rate_hz=0 every message triggers one inference.
  /m20/imu             sensor_msgs/Imu          orientation (base -> world) and angular velocity in the base frame
  /m20/base_velocity   geometry_msgs/TwistStamped  base linear velocity in the base frame
  /cmd_vel             geometry_msgs/Twist      velocity command (vx, vy, yaw rate)
  /elevation_mapping_node/elevation_map_filter  grid_map_msgs/GridMap   terrain for the height scan (use_height_scan)
  tf                   map_frame -> base_frame, the base pose in the elevation map frame (use_height_scan)
Publishes:
  /m20/joint_commands  sensor_msgs/JointState   name = 16 joints; position = leg position targets (rad),
                                                 velocity = wheel velocity targets (rad/s); NaN where unused.
                                                 header.stamp is copied from the joint_states used.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from tf2_ros import Buffer, TransformListener

from . import m20_policy as m20


class M20PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("m20_policy_node")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("joint_states_topic", "/m20/joint_states")
        self.declare_parameter("imu_topic", "/m20/imu")
        self.declare_parameter("base_velocity_topic", "/m20/base_velocity")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_commands_topic", "/m20/joint_commands")
        # best effort (depth 1) for the state inputs and the command output, as sensor drivers use; false = reliable
        self.declare_parameter("best_effort_qos", True)
        # inference runs on a timer at this rate using the latest state; 0 = one inference per joint_states message
        self.declare_parameter("control_rate_hz", 50.0)
        # timer mode: joint states / IMU / base velocity older than this (node clock) skip the inference (0 disables)
        self.declare_parameter("state_timeout_s", 0.1)
        # a command older than this is treated as zero (0 disables the check)
        self.declare_parameter("command_timeout_s", 0.5)
        # the rough-terrain policy observes a height scan sampled from an elevation_mapping_cupy map
        self.declare_parameter("use_height_scan", True)
        self.declare_parameter("elevation_map_topic", "/elevation_mapping_node/elevation_map_filter")
        self.declare_parameter("height_layer", "inpaint")
        self.declare_parameter("base_frame", "base_link")
        # a map older than this (node clock) is not used; the robot then waits instead of walking blind
        self.declare_parameter("map_timeout_s", 2.0)
        self.declare_parameter("drift_free_height", True)
        self.declare_parameter("flat_height_scan", False)  # debug: ignore the map and observe flat ground

        policy_path = self.get_parameter("policy_path").value
        if not policy_path:
            raise RuntimeError("Parameter 'policy_path' (TorchScript policy.pt) is required")
        self._policy = m20.M20Policy(policy_path, self.get_parameter("device").value)
        self._command_timeout = float(self.get_parameter("command_timeout_s").value)

        self._use_height_scan = bool(self.get_parameter("use_height_scan").value)
        self._height_layer = self.get_parameter("height_layer").value
        self._base_frame = self.get_parameter("base_frame").value
        self._map_timeout = float(self.get_parameter("map_timeout_s").value)
        self._drift_free = bool(self.get_parameter("drift_free_height").value)
        self._flat_scan = bool(self.get_parameter("flat_height_scan").value)
        self._map = None
        self._map_frame_id = ""
        self._map_time = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._orientation_wxyz = None
        self._ang_vel = None
        self._lin_vel = None
        self._command = np.zeros(3)
        self._command_time = None
        self._warned_missing = False
        self._last_stamp_ns = None
        self._joint_msg = None
        self._joint_time = self._imu_time = self._vel_time = None
        rate = float(self.get_parameter("control_rate_hz").value)
        self._control_period = 1.0 / rate if rate > 0.0 else 0.0
        self._state_timeout = float(self.get_parameter("state_timeout_s").value)

        param = self.get_parameter
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT if param("best_effort_qos").value else ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(JointState, param("joint_commands_topic").value, qos)
        self.create_subscription(Imu, param("imu_topic").value, self._on_imu, qos)
        self.create_subscription(TwistStamped, param("base_velocity_topic").value, self._on_base_velocity, qos)
        self.create_subscription(Twist, param("cmd_vel_topic").value, self._on_cmd_vel, 1)
        self.create_subscription(JointState, param("joint_states_topic").value, self._on_joint_states, qos)
        if self._use_height_scan:
            self.create_subscription(GridMap, param("elevation_map_topic").value, self._on_elevation_map, 1)
        if self._control_period > 0.0:
            self.create_timer(self._control_period, self._on_timer)  # node clock (/clock with use_sim_time)
        self.get_logger().info(f"Loaded policy {policy_path}")

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._orientation_wxyz = np.array([q.w, q.x, q.y, q.z])
        w = msg.angular_velocity
        self._ang_vel = np.array([w.x, w.y, w.z])
        self._imu_time = self.get_clock().now()

    def _on_base_velocity(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._lin_vel = np.array([v.x, v.y, v.z])
        self._vel_time = self.get_clock().now()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self._command_time = self.get_clock().now()

    def _on_elevation_map(self, msg: GridMap) -> None:
        if self._height_layer not in msg.layers:
            self.get_logger().warn(
                f"Layer '{self._height_layer}' not in elevation map {list(msg.layers)}", throttle_duration_sec=5.0
            )
            return
        layer = msg.data[msg.layers.index(self._height_layer)]
        rows, cols = layer.layout.dim[0].size, layer.layout.dim[1].size
        z = np.array(layer.data, dtype=np.float32).reshape(cols, rows).T  # grid_map data is column-major
        info = msg.info
        self._map = m20.ElevationMap(
            z,
            np.array([info.pose.position.x, info.pose.position.y]),
            np.array([info.length_x, info.length_y]),
            info.resolution,
        )
        self._map_frame_id = msg.header.frame_id
        self._map_time = self.get_clock().now()

    def _height_scan(self) -> np.ndarray | None:
        """Height scan observation, or None (with a warning) while the map or the base pose is unavailable."""
        if self._flat_scan:
            return np.full(m20.SCAN_POINTS.shape[0], m20.STANDING_HEIGHT - m20.SCAN_OFFSET, dtype=np.float32)
        if self._map is None:
            self.get_logger().warn("Waiting for the elevation map", throttle_duration_sec=5.0)
            return None
        age = (self.get_clock().now() - self._map_time).nanoseconds * 1e-9
        if self._map_timeout > 0.0 and not 0.0 <= age <= self._map_timeout:
            self.get_logger().warn(f"Elevation map is stale ({age:.1f} s old)", throttle_duration_sec=5.0)
            return None
        if not self._tf_buffer.can_transform(self._map_frame_id, self._base_frame, rclpy.time.Time()):
            self.get_logger().warn(
                f"Waiting for {self._map_frame_id} -> {self._base_frame} transform",
                throttle_duration_sec=5.0,
            )
            return None
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame_id, self._base_frame, rclpy.time.Time())
        except Exception as error:  # noqa: BLE001 - tf2 raises several exception types
            self.get_logger().warn(f"No {self._map_frame_id} -> {self._base_frame} transform: {error}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = m20.yaw_from_quat(np.array([q.w, q.x, q.y, q.z]))
        return m20.height_scan_observation(self._map.sample, np.array([t.x, t.y, t.z]), yaw, self._drift_free)

    def _active_command(self) -> np.ndarray:
        if self._command_timeout <= 0.0:
            return self._command
        if self._command_time is None:
            return np.zeros(3)
        # uses the node clock (/clock with use_sim_time); a negative age means the clock was reset
        age = (self.get_clock().now() - self._command_time).nanoseconds * 1e-9
        return self._command if 0.0 <= age <= self._command_timeout else np.zeros(3)

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_msg = msg
        self._joint_time = self.get_clock().now()
        if self._control_period <= 0.0:
            self._infer(msg)

    def _on_timer(self) -> None:
        msg = self._joint_msg
        if msg is None:
            self.get_logger().warn("Waiting for joint states", throttle_duration_sec=5.0)
            return
        if self._state_timeout > 0.0:
            now = self.get_clock().now()
            for name, received in (("joint states", self._joint_time), ("IMU", self._imu_time), ("base velocity", self._vel_time)):
                age = float("inf") if received is None else (now - received).nanoseconds * 1e-9
                if not 0.0 <= age <= self._state_timeout:
                    self.get_logger().warn(f"Stale {name} ({age:.2f} s old); skipping inference", throttle_duration_sec=2.0)
                    return
        self._infer(msg)

    def _infer(self, msg: JointState) -> None:
        if self._orientation_wxyz is None or self._lin_vel is None:
            if not self._warned_missing:
                self.get_logger().warn("Waiting for IMU and base velocity before running the policy")
                self._warned_missing = True
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            self._policy.reset()  # the state source restarted
        self._last_stamp_ns = stamp_ns

        index = {name: i for i, name in enumerate(msg.name)}
        missing = [name for name in m20.JOINT_NAMES if name not in index]
        if missing or len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            self.get_logger().error(f"Invalid joint state (missing joints: {missing}); skipping inference")
            return
        order = [index[name] for name in m20.JOINT_NAMES]
        pos = np.asarray(msg.position)[order]
        vel = np.asarray(msg.velocity)[order]

        height_scan = None
        if self._use_height_scan:
            height_scan = self._height_scan()
            if height_scan is None:
                # no terrain knowledge yet: hold the default pose instead of letting the policy walk blind
                self._policy.reset()
                self._publish_targets(msg.header.stamp, *m20.action_to_targets(self._policy.last_action))
                return
        obs = m20.build_observation(
            self._lin_vel,
            self._ang_vel,
            self._orientation_wxyz,
            self._active_command(),
            pos[:12],
            vel[:12],
            vel[12:],
            self._policy.last_action,
            height_scan,
        )
        if not np.all(np.isfinite(obs)):
            self.get_logger().error("Non-finite observation; skipping inference")
            return
        leg_targets, wheel_targets = m20.action_to_targets(self._policy.infer(obs))
        self._publish_targets(msg.header.stamp, leg_targets, wheel_targets)

    def _publish_targets(self, stamp, leg_targets: np.ndarray, wheel_targets: np.ndarray) -> None:
        out = JointState()
        out.header.stamp = stamp
        out.name = list(m20.JOINT_NAMES)
        out.position = [float(v) for v in leg_targets] + [float("nan")] * 4
        out.velocity = [float("nan")] * 12 + [float(v) for v in wheel_targets]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M20PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
