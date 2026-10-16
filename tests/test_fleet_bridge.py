import importlib
import json
import queue
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

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
            "action_msgs.msg": types.SimpleNamespace(GoalStatusArray=object),
            "action_msgs.srv": types.SimpleNamespace(CancelGoal=types.SimpleNamespace(Request=object)),
            "rosgraph_msgs.msg": types.SimpleNamespace(Clock=object),
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
        self.bridge.session_id = "boot-a"
        self.bridge.session_reset_pending = False
        self.bridge.session_reset_error = ""
        self.bridge.task_sessions = {}
        self.bridge.nav_status = {}
        self.bridge.cancel_future = None
        self.bridge.cancel_goals = None
        self.bridge.cancel_started = None
        self.bridge.cancel_timeout = 10.0
        self.bridge.floor_session_id = "boot-a"
        self.bridge.localization_ready = lambda: True
        self.bridge.topic = {"task_state": "task_state", "heartbeat": "heartbeat",
                             "command_result": "command_result", "register_ack": "ack",
                             "command": "command"}
        self.bridge.inbox = queue.Queue()
        self.bridge.pose = (1.0, 2.0, 0.0)
        self.bridge.battery = 100.0
        self.bridge.registered = True
        self.bridge.cmd_ids = {}
        self.bridge.floor_future = None
        self.bridge.floor_error = False
        self.bridge.floor_prepared = False
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
            "session_id": self.bridge.session_id,
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

    def test_prepared_state_blocks_motion_even_without_pending_request(self):
        self.bridge.floor_prepared = True
        self.command(2, "navigate")
        self.assertEqual(self.results(), [(2, "failed")])
        self.assertEqual(self.bridge.runner.task.goal_id, "1")

    def test_late_prepare_response_never_replaces_timeout_with_success(self):
        self.bridge.floor_future = types.SimpleNamespace(
            done=lambda: False, exception=lambda: None,
            result=lambda: types.SimpleNamespace(
                success=True, current_floor="", message="prepared"))
        self.bridge.floor_cmd_id = 11
        self.bridge.floor_action = "prepare_floor_change"
        self.bridge.floor_started = 0.0
        self.bridge.floor_deadline = 90.0
        self.bridge.floor_response_overdue = False
        self.bridge.floor_results = {}
        self.bridge.floor_prepared = True
        with patch("time.monotonic", return_value=91.0):
            self.bridge.poll_floor_switch()
        self.assertEqual(self.results(), [(11, "failed")])
        self.assertIsNotNone(self.bridge.floor_future)
        self.bridge.floor_future.done = lambda: True
        self.bridge.poll_floor_switch()
        self.assertEqual(self.results(), [(11, "failed")])
        self.assertTrue(self.bridge.floor_error)
        self.assertTrue(self.bridge.floor_prepared)


class FleetBridgeSessionTest(FleetBridgeCancellationTest):
    def test_all_task_and_heartbeat_messages_have_session_id(self):
        self.bridge.on_heartbeat()
        self.assertTrue(self.messages)
        self.assertTrue(all(message["session_id"] == "boot-a"
                            for _, message, _ in self.messages))

    def test_foreign_and_missing_session_commands_are_ignored(self):
        before = list(self.messages)
        for session in ("boot-old", None):
            self.bridge.handle_command(json.dumps({
                "robot_id": "carter1", "session_id": session, "cmd_id": 2, "action": "stop"}))
        self.assertEqual(self.messages, before)
        self.assertEqual(self.bridge.runner.task.goal_id, "1")

    def test_foreign_ack_cannot_unregister_current_session(self):
        self.bridge.inbox.put(("ack", json.dumps({
            "robot_id": "carter1", "session_id": "boot-old", "status": "require_register"})))
        self.bridge.on_tick()
        self.assertTrue(self.bridge.registered)

    def test_reconnect_preserves_session_and_running_task(self):
        self.bridge.client.subscribe = Mock()
        self.bridge.on_connect(self.bridge.client, None, {}, 0, None)
        self.assertEqual(self.bridge.session_id, "boot-a")
        self.assertTrue(self.bridge.registered)
        self.assertEqual(self.bridge.runner.task.goal_id, "1")
        self.assertEqual(self.bridge.runner.state["status"], "running")

    def reset(self):
        self.bridge.client.disconnect = Mock()
        self.bridge.client.loop_stop = Mock()
        self.bridge.mqtt_connection = ("localhost", 1883, "", "")
        self.bridge.connect_mqtt = Mock()
        self.bridge.floor_results = {}
        self.bridge.reset_session("test reboot")

    def test_reset_cancels_work_and_old_results_keep_old_identity(self):
        self.reset()
        self.assertNotEqual(self.bridge.session_id, "boot-a")
        self.assertFalse(self.bridge.registered)
        self.assertTrue(self.bridge.session_reset_pending)
        self.assertIsNone(self.bridge.pose)
        self.command(2, "navigate")
        self.assertEqual(self.bridge.runner.task.goal_id, "1")
        self.bridge.runner.cancel_child.outcome = CANCELED
        self.bridge.runner.tick()
        terminal = [message for topic, message, _ in self.messages
                    if topic == "command_result"]
        self.assertEqual(terminal[-1]["session_id"], "boot-a")
        self.bridge.connect_mqtt.assert_called_once_with("localhost", 1883, "", "")

    def test_reset_waits_for_terminal_nav2_status_and_runner_cancellation(self):
        self.reset()
        goal = bytes([1] * 16)
        self.bridge.cancel_client = Mock()
        self.bridge.cancel_client.call_async.return_value = Mock(
            done=lambda: True, exception=lambda: None,
            result=lambda: types.SimpleNamespace(return_code=0, goals_canceling=[
                types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=list(goal)))]))
        self.bridge.poll_session_reset()
        self.assertTrue(self.bridge.session_reset_pending)
        self.bridge.runner.cancel_child.outcome = CANCELED
        self.bridge.runner.tick()
        self.bridge.nav_status[goal] = 3  # CANCELING is not proof that motion ended.
        self.bridge.poll_session_reset()
        self.assertTrue(self.bridge.session_reset_pending)
        self.bridge.nav_status[goal] = 5
        self.bridge.poll_session_reset()
        self.assertFalse(self.bridge.session_reset_pending)
        self.assertEqual(self.bridge.cmd_ids, {})

    def test_rejected_nav2_cancellation_stays_blocked(self):
        self.reset()
        self.bridge.cancel_client = Mock()
        self.bridge.cancel_client.call_async.return_value = Mock(
            done=lambda: True, exception=lambda: None,
            result=lambda: types.SimpleNamespace(return_code=1, goals_canceling=[]))
        self.bridge.poll_session_reset()
        self.assertTrue(self.bridge.session_reset_pending)
        self.assertIn("Cannot cancel", self.bridge.session_reset_error)
        self.assertTrue(self.errors)

    def test_clock_rewind_rotates_session_but_normal_clock_does_not(self):
        self.bridge.last_clock = None
        self.bridge.reset_session = Mock()
        for value in (10, 11, 11):
            self.bridge.on_clock(types.SimpleNamespace(
                clock=types.SimpleNamespace(sec=value, nanosec=0)))
        self.bridge.reset_session.assert_not_called()
        self.bridge.on_clock(types.SimpleNamespace(
            clock=types.SimpleNamespace(sec=1, nanosec=0)))
        self.bridge.reset_session.assert_called_once_with("Simulation clock reset")

    def test_localization_requires_fresh_accepted_correction_not_just_ekf_pose(self):
        self.bridge.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=10_000_000_000))
        self.bridge.pose_stamp = 10.0
        self.bridge.pose_received = 20.0
        self.bridge.correction_stamp = None
        self.bridge.correction_received = None
        self.bridge.pose_timeout = 1.0
        self.bridge.correction_timeout = 5.0
        with patch("time.monotonic", return_value=20.0):
            ready = self.bridge_class.localization_ready
            self.assertFalse(ready(self.bridge))
            self.bridge.correction_stamp = 9.5
            self.bridge.correction_received = 19.5
            self.assertTrue(ready(self.bridge))
            self.bridge.session_reset_pending = True
            self.assertFalse(ready(self.bridge))
            self.bridge.session_reset_pending = False
        with patch("time.monotonic", return_value=22.0):
            self.assertFalse(ready(self.bridge))  # A paused simulator also goes stale.

    def test_late_floor_result_cannot_enter_new_session_cache(self):
        self.reset()
        self.bridge.floor_result(7, "completed", "old floor request ended")
        self.assertEqual(self.bridge.floor_results, {})
        self.assertEqual(self.messages[-1][1]["session_id"], "boot-a")

    def test_late_floor_response_cannot_restore_pre_reset_localization(self):
        self.bridge.floor_future = Mock(
            done=lambda: True, exception=lambda: None,
            result=lambda: types.SimpleNamespace(success=True, current_floor="B1", message="done"))
        self.bridge.floor_cmd_id = 7
        self.bridge.floor_response_overdue = False
        self.reset()
        self.bridge.poll_floor_switch()
        self.assertIsNone(self.bridge.pose)
        self.assertEqual(self.bridge.level, "1F")
        self.assertEqual(self.bridge.floor_results, {})

    def test_repeated_reset_does_not_orphan_a_pending_cancel_all_request(self):
        future = Mock(done=lambda: False)
        self.bridge.cancel_future = future
        self.reset()
        self.assertIs(self.bridge.cancel_future, future)


if __name__ == "__main__":
    unittest.main()
