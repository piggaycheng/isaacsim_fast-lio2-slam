import json
import math
import sys
import threading
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_sensor_control.action import GimbalMove, GimbalRotate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_sensor_control/scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_fleet_bridge/scripts"))
sys.path.insert(0, str(ROOT / "tests"))
from gimbal_action_server import GimbalActionServer  # noqa: E402
from task_bt import STEP_BEHAVIOURS, TaskRunner  # noqa: E402
from test_gimbal_action_server import FakeGimbal  # noqa: E402


class TaskGimbalRosTest(unittest.TestCase):
    """Runs gimbal steps from the behaviour tree against the real gimbal action server."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.server = GimbalActionServer()
        cls.server.timeout = 1.5
        cls.fake = FakeGimbal()
        cls.node = Node("task_gimbal_test")
        action_types = {"gimbal_move": GimbalMove, "gimbal_rotate": GimbalRotate}
        cls.clients = {kind: ActionClient(cls.node, action_types[kind], STEP_BEHAVIOURS[kind][1])
                       for kind in action_types}
        cls.states = []
        cls.lock = threading.Lock()
        cls.runner = TaskRunner(
            lambda step: STEP_BEHAVIOURS[step["type"]][0](cls.node, cls.clients[step["type"]], step),
            cls.states.append)
        cls.node.create_timer(0.02, cls.tick)
        cls.executor = MultiThreadedExecutor()
        for node in (cls.server, cls.fake, cls.node):
            cls.executor.add_node(node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        assert all(client.wait_for_server(timeout_sec=5.0) for client in cls.clients.values())

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        for node in (cls.server, cls.fake, cls.node):
            node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def tick(cls):
        with cls.lock:
            cls.runner.tick()

    def run_task(self, goal_id, steps):
        with self.lock:
            self.states.clear()
            self.runner.submit(json.dumps({"goal_id": goal_id, "steps": steps}))
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self.lock:
                if not self.runner.active:
                    return list(self.states)
                if self.runner.state["status"] == "waiting":
                    self.runner.proceed(goal_id)
            time.sleep(0.02)
        self.fail("task did not finish")

    def test_move_then_rotate_reaches_pose(self):
        states = self.run_task("1", [
            {"type": "gimbal_move", "pan_deg": 60, "tilt_deg": 10},
            {"type": "gimbal_rotate", "pan_deg": 30, "tilt_deg": -5}])
        self.assertEqual(states[-1]["status"], "succeeded", states)
        self.assertIn(1, [state["step"] for state in states])
        self.assertAlmostEqual(math.degrees(self.fake.pan), 90.0, delta=2.0)
        self.assertAlmostEqual(math.degrees(self.fake.tilt), 5.0, delta=2.0)

    def test_unreachable_pose_aborts_with_gimbal_message(self):
        states = self.run_task("2", [
            {"type": "gimbal_move", "pan_deg": 0, "tilt_deg": 120}])
        self.assertEqual(states[-1]["status"], "aborted")
        self.assertIn("gimbal/move", states[-1]["message"])

    def test_stalled_gimbal_aborts_and_stops_later_steps(self):
        self.fake.frozen = True
        try:
            states = self.run_task("3", [
                {"type": "gimbal_move", "pan_deg": 45, "tilt_deg": 0},
                {"type": "gimbal_rotate", "pan_deg": 10}])
        finally:
            self.fake.frozen = False
        self.assertEqual(states[-1]["status"], "aborted")
        self.assertEqual(states[-1]["step"], 0)
        self.assertIn("Timed out", states[-1]["message"])


if __name__ == "__main__":
    unittest.main()
