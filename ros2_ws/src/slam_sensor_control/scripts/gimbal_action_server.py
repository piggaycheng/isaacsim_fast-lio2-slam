#!/usr/bin/env python3
"""Action server that rotates a robot's pan/tilt gimbal and reports success on arrival.

Run it in the robot's namespace. Per goal it publishes the target pose as a
sensor_msgs/JointState on `gimbal/joint_command` and finishes with success only once
the pose read from `gimbal/joint_states` matches the goal within tolerance.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from slam_sensor_control.action import GimbalMove

PAN_JOINT = "gimbal_pan_joint"
TILT_JOINT = "gimbal_tilt_joint"


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GimbalActionServer(Node):
    def __init__(self):
        super().__init__("gimbal_action_server")
        self.pan_tolerance = math.radians(self.declare_parameter("pan_tolerance_deg", 1.0).value)
        self.tilt_tolerance = math.radians(self.declare_parameter("tilt_tolerance_deg", 1.0).value)
        self.timeout = self.declare_parameter("timeout", 30.0).value
        self.state_timeout = self.declare_parameter("state_timeout", 1.0).value
        limits = self.declare_parameter("tilt_limits_deg", [-90.0, 90.0]).value
        self.tilt_limits = (math.radians(limits[0]), math.radians(limits[1]))
        self.rate = self.declare_parameter("rate", 20.0).value

        self.lock = threading.RLock()
        self.pose = None
        self.pose_time = 0.0
        self.active_goal = None
        self.command_pub = self.create_publisher(JointState, "gimbal/joint_command", 10)
        self.create_subscription(JointState, "gimbal/joint_states", self.on_joint_states, 10)
        self.server = ActionServer(
            self, GimbalMove, "gimbal/move",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def on_joint_states(self, message):
        names = list(message.name)
        if PAN_JOINT in names and TILT_JOINT in names and len(message.position) == len(names):
            pan = message.position[names.index(PAN_JOINT)]
            tilt = message.position[names.index(TILT_JOINT)]
            with self.lock:
                self.pose = (pan, tilt)
                self.pose_time = time.monotonic()

    def current_pose(self):
        """Latest (pan, tilt), or None when no fresh gimbal state has arrived."""
        with self.lock:
            if self.pose is None or time.monotonic() - self.pose_time > self.state_timeout:
                return None
            return self.pose

    def on_goal(self, goal):
        values = (goal.pan, goal.tilt)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejected gimbal goal: non-finite angle")
            return GoalResponse.REJECT
        if not self.tilt_limits[0] <= goal.tilt <= self.tilt_limits[1]:
            self.get_logger().warning(f"Rejected gimbal goal: tilt {goal.tilt:.3f} rad out of limits")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def publish_command(self, goal):
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = [PAN_JOINT, TILT_JOINT]
        command.position = [float(goal.pan), float(goal.tilt)]
        self.command_pub.publish(command)

    def finish(self, handle, status, success, message):
        pose = self.current_pose() or (math.nan, math.nan)
        result = GimbalMove.Result(success=success, message=message, pan=pose[0], tilt=pose[1])
        getattr(handle, status)()
        self.get_logger().info(f"Gimbal goal {status}: {message}")
        return result

    def execute(self, handle):
        goal = handle.request
        # A newer goal takes over the gimbal; the older one aborts.
        with self.lock:
            self.active_goal = handle
        started = time.monotonic()
        period = 1.0 / self.rate
        while rclpy.ok():
            with self.lock:
                if self.active_goal is not handle:
                    return self.finish(handle, "abort", False, "Superseded by a newer goal")
            if handle.is_cancel_requested:
                return self.finish(handle, "canceled", False, "Canceled")
            elapsed = time.monotonic() - started
            # Command is repeated so a lost message or late-starting simulator cannot stall it.
            self.publish_command(goal)
            pose = self.current_pose()
            if pose is None:
                if elapsed > self.state_timeout + 1.0:
                    return self.finish(handle, "abort", False, "No fresh gimbal joint state")
            else:
                pan_error = wrap_to_pi(goal.pan - pose[0])
                tilt_error = goal.tilt - pose[1]
                handle.publish_feedback(GimbalMove.Feedback(
                    pan=pose[0], tilt=pose[1], pan_error=pan_error, tilt_error=tilt_error))
                if abs(pan_error) <= self.pan_tolerance and abs(tilt_error) <= self.tilt_tolerance:
                    return self.finish(handle, "succeed", True, "Reached target pose")
            if elapsed > self.timeout:
                return self.finish(handle, "abort", False, "Timed out before reaching target pose")
            time.sleep(period)
        return self.finish(handle, "abort", False, "Shutting down")


def main():
    rclpy.init()
    node = GimbalActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
