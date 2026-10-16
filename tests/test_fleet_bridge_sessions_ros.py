"""Session fencing against a real ROS action server, without a simulator or MQTT broker."""

import json
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from fleet_bridge_node import FleetBridge  # noqa: E402


@unittest.skipUnless(
    os.environ.get("ROS_DOMAIN_ID") not in (None, "", "0"),
    "Run action integration tests in an isolated nonzero ROS_DOMAIN_ID",
)
class FleetBridgeSessionsRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.bridge = None
        self.stopping = threading.Event()
        self.goals = []
        self.server_node = Node("fake_nav2", namespace="/fleet_session_probe")
        self.server = ActionServer(
            self.server_node, NavigateToPose, "navigate_to_pose", self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.action_client = ActionClient(self.server_node, NavigateToPose, "navigate_to_pose")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))
        self.mqtt_patch = patch("fleet_bridge_node.mqtt.Client")
        self.client = self.mqtt_patch.start().return_value
        self.client.is_connected.return_value = True

    def tearDown(self):
        self.stopping.set()
        if self.bridge is not None:
            self.bridge.close()
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        if self.bridge is not None:
            self.bridge.destroy_node()
        self.action_client.destroy()
        self.server.destroy()
        self.server_node.destroy_node()
        self.mqtt_patch.stop()

    def execute(self, goal):
        self.goals.append(goal)
        while not self.stopping.is_set() and not goal.is_cancel_requested:
            time.sleep(0.01)
        if goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        return NavigateToPose.Result()

    def wait_until(self, predicate, message):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail(message)

    def start_bridge(self):
        self.bridge = FleetBridge(
            namespace="/fleet_session_probe",
            parameter_overrides=[
                Parameter("fleet_name", value="session_test"),
                Parameter("footprint_radius", value=0.35),
                Parameter("max_linear_velocity", value=1.0),
                Parameter("max_angular_velocity", value=1.0),
            ],
        )
        self.executor.add_node(self.bridge)
        self.wait_until(
            lambda: not self.bridge.session_reset_pending or self.bridge.session_reset_error,
            "Bridge did not finish stopping old Nav2 goals",
        )
        self.assertEqual(self.bridge.session_reset_error, "")

    def localize_and_ack(self):
        stamp = self.bridge.get_clock().now().to_msg()
        self.bridge.on_correction(Header(frame_id="map", stamp=stamp))
        odometry = Odometry()
        odometry.header.frame_id = "map"
        odometry.header.stamp = stamp
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.orientation.w = 1.0
        self.bridge.on_odometry(odometry)
        self.assertTrue(self.bridge.localization_ready())
        self.bridge.inbox.put((self.bridge.topic["register_ack"], json.dumps({
            "robot_id": self.bridge.robot,
            "session_id": self.bridge.session_id, "status": "success",
        })))
        self.wait_until(lambda: self.bridge.registered, "Matching session ACK was not accepted")

    def test_startup_without_old_goal_can_become_ready(self):
        self.start_bridge()
        self.assertFalse(self.bridge.localization_ready())
        self.localize_and_ack()
        self.bridge.on_heartbeat()
        heartbeats = [
            json.loads(call.args[1]) for call in self.client.publish.call_args_list
            if call.args[0] == self.bridge.topic["heartbeat"]
        ]
        self.assertTrue(heartbeats[-1]["localization_ready"])
        self.assertEqual(heartbeats[-1]["session_id"], self.bridge.session_id)
        self.assertIsNone(heartbeats[-1]["current_cmd_id"])

    def test_startup_cancels_goal_left_by_a_previous_bridge_process(self):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = "map"
        request.pose.pose.orientation.w = 1.0
        future = self.action_client.send_goal_async(request)
        self.wait_until(future.done, "Old Nav2 goal was not accepted")
        self.assertTrue(future.result().accepted)
        self.wait_until(lambda: len(self.goals) == 1, "Old Nav2 goal did not execute")
        result = future.result().get_result_async()
        self.start_bridge()
        self.wait_until(result.done, "Old Nav2 goal did not terminate")
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(self.bridge.runner.active)

    def test_simulator_reset_stops_task_and_fences_old_commands_and_acks(self):
        self.start_bridge()
        self.localize_and_ack()
        old_session = self.bridge.session_id
        command = {
            "robot_id": self.bridge.robot, "session_id": old_session,
            "cmd_id": 1, "action": "navigate", "target": {"x": 1, "y": 0, "yaw": 0},
        }
        self.bridge.inbox.put((self.bridge.topic["command"], json.dumps(command)))
        self.wait_until(lambda: len(self.goals) == 1, "Bridge task did not reach Nav2")
        self.bridge.last_clock = 100.0
        self.bridge.on_clock(Clock(clock=Time(sec=1)))
        new_session = self.bridge.session_id
        self.assertNotEqual(new_session, old_session)
        self.assertFalse(self.bridge.registered)
        self.assertFalse(self.bridge.localization_ready())
        self.wait_until(
            lambda: not self.bridge.session_reset_pending or self.bridge.session_reset_error,
            "Reset did not stop old task",
        )
        self.assertEqual(self.bridge.session_reset_error, "")
        self.assertEqual(self.goals[0].status, GoalStatus.STATUS_CANCELED)
        self.bridge.inbox.put((self.bridge.topic["command"], json.dumps(command)))
        self.bridge.inbox.put((self.bridge.topic["register_ack"], json.dumps({
            "robot_id": self.bridge.robot, "session_id": old_session, "status": "success",
        })))
        self.wait_until(self.bridge.inbox.empty, "Old packets were not processed")
        self.assertFalse(self.bridge.registered)
        self.assertEqual(len(self.goals), 1)
        self.localize_and_ack()
        command["session_id"] = new_session
        self.bridge.inbox.put((self.bridge.topic["command"], json.dumps(command)))
        self.wait_until(lambda: len(self.goals) == 2, "New session could not reuse a command ID")


if __name__ == "__main__":
    unittest.main()
