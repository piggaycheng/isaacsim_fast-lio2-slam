import importlib
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import py_trees
from py_trees.common import Status

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from task_bt import CANCELED, TaskRunner  # noqa: E402


class PendingStep(py_trees.behaviour.Behaviour):
    cancel_error = ""
    outcome = None

    def __init__(self, step):
        super().__init__("navigation")

    def update(self):
        return Status.RUNNING

    def request_cancel(self):
        pass

    def poll_cancel(self):
        return self.outcome


class FleetBridgeCancellationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Exercise bridge logic without requiring a ROS installation or broker.
        modules = {
            "paho": types.ModuleType("paho"),
            "paho.mqtt": types.ModuleType("paho.mqtt"),
            "paho.mqtt.client": types.ModuleType("paho.mqtt.client"),
            "rclpy": types.ModuleType("rclpy"),
            "rclpy.action": types.SimpleNamespace(ActionClient=object),
            "rclpy.executors": types.SimpleNamespace(ExternalShutdownException=Exception),
            "rclpy.node": types.SimpleNamespace(Node=object),
            "nav2_msgs.action": types.SimpleNamespace(NavigateToPose=object),
            "nav_msgs.msg": types.SimpleNamespace(Odometry=object),
            "sensor_msgs.msg": types.SimpleNamespace(BatteryState=object),
            "slam_sensor_control.action": types.SimpleNamespace(GimbalMove=object, GimbalRotate=object),
        }
        with patch.dict(sys.modules, modules):
            cls.bridge_class = importlib.import_module("fleet_bridge_node").FleetBridge
        sys.modules.pop("fleet_bridge_node", None)

    def setUp(self):
        self.messages = []
        self.errors = []
        self.time = 0.0
        self.bridge = self.bridge_class.__new__(self.bridge_class)
        self.bridge.robot = "carter1"
        self.bridge.topic = {"task_state": "task_state", "heartbeat": "heartbeat",
                             "command_result": "command_result"}
        self.bridge.pose = (1.0, 2.0, 0.0)
        self.bridge.battery = 100.0
        self.bridge.registered = True
        self.bridge.cmd_ids = {}
        self.bridge.floor_future = None
        self.bridge.floor_error = False
        self.bridge.level = "1F"
        self.bridge.get_logger = lambda: types.SimpleNamespace(
            info=lambda message: None, warning=lambda message: None, error=self.errors.append)
        self.bridge.client = types.SimpleNamespace(publish=self.publish)
        self.bridge.maybe_register = lambda: None
        self.bridge.runner = TaskRunner(
            PendingStep, self.bridge.on_task_state, clock=lambda: self.time)
        self.command(1, "navigate")
        self.bridge.runner.tick()

    def publish(self, topic, payload, **kwargs):
        self.messages.append((topic, json.loads(payload), kwargs))

    def command(self, cmd_id, action):
        self.bridge.handle_command(json.dumps({
            "robot_id": "carter1", "cmd_id": cmd_id, "action": action,
            "target": {"x": 1.0, "y": 2.0}}))

    def results(self):
        return [(payload["cmd_id"], payload["status"])
                for topic, payload, _ in self.messages if topic == "command_result"]

    def test_stop_result_and_canceled_are_delayed_until_navigation_ends(self):
        self.command(2, "stop")
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [])
        self.bridge.on_heartbeat()
        self.assertEqual(self.messages[-1][1]["status"], "moving")
        self.assertEqual(self.messages[-1][1]["current_cmd_id"], 1)
        self.bridge.runner.cancel_child.outcome = CANCELED
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [(1, "canceled"), (2, "completed")])
        self.bridge.on_heartbeat()
        self.assertEqual(self.messages[-1][1]["status"], "idle")

    def test_replacement_starts_only_after_old_result_is_published(self):
        self.command(2, "navigate")
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [])
        self.assertEqual(self.bridge.runner.task.goal_id, "1")
        self.bridge.runner.cancel_child.outcome = CANCELED
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [(1, "canceled")])
        self.assertEqual(self.messages[-1][1]["cmd_id"], 2)
        self.assertEqual(self.messages[-1][1]["status"], "running")
        self.assertTrue(self.messages[-1][2]["retain"])

    def test_stop_timeout_reports_failed_not_completed(self):
        self.command(2, "stop")
        self.time = 10.0
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [(2, "failed")])
        self.assertTrue(self.errors)
        self.assertTrue(self.bridge.runner.active)
        self.bridge.runner.cancel_child.outcome = CANCELED
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [(2, "failed"), (1, "canceled")])

    def test_pending_replacement_failure_maps_to_command_result(self):
        self.command(2, "navigate")
        self.bridge.runner.cancel_child.cancel_error = "cancel rejected"
        self.bridge.runner.tick()
        self.assertEqual(self.results(), [(2, "failed")])
        self.assertEqual(self.bridge.runner.task.goal_id, "1")


if __name__ == "__main__":
    unittest.main()
