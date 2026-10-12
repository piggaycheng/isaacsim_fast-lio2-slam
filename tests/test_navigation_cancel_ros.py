import json
import sys
import threading
import time
import unittest
import uuid
from pathlib import Path

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from task_bt import CANCELED, RUNNING, NavigateToPose as NavigateStep, TaskRunner  # noqa: E402


class NavigationCancelRosTest(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = Node("cancel_test_" + uuid.uuid4().hex, context=self.context)
        self.accept_gate = threading.Event()
        self.accept_gate.set()
        self.finish_gate = threading.Event()
        self.cancel_seen = threading.Event()
        self.goal_seen = threading.Event()
        self.lock = threading.Lock()
        self.states = []
        self.goal_count = 0
        action_name = "/" + self.node.get_name() + "/navigate_to_pose"
        self.server = ActionServer(
            self.node, NavigateToPose, action_name, self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.client = ActionClient(self.node, NavigateToPose, action_name)
        self.runner = TaskRunner(
            lambda step: NavigateStep(self.node, self.client, step), self.states.append)
        self.timer = self.node.create_timer(0.01, self.tick)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))

    def tearDown(self):
        self.accept_gate.set()
        self.finish_gate.set()
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.client.destroy()
        self.server.destroy()
        self.node.destroy_node()
        self.context.shutdown()

    def tick(self):
        with self.lock:
            self.runner.tick()

    def accept(self, request):
        self.goal_seen.set()
        self.accept_gate.wait(timeout=5.0)
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancel_seen.set()
        return CancelResponse.ACCEPT

    def execute(self, handle):
        self.goal_count += 1
        self.finish_gate.wait(timeout=5.0)
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def submit(self, goal_id):
        with self.lock:
            self.runner.submit(json.dumps({
                "goal_id": goal_id, "steps": [{"type": "navigate", "x": 1.0, "y": 2.0}]}))

    def wait_for(self, predicate):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(0.01)
        self.fail("timed out waiting for task state")

    def check_replacement_waits_for_result(self):
        self.submit("2")
        self.assertTrue(self.cancel_seen.wait(timeout=5.0))
        self.wait_for(lambda: self.runner.cancel_child.cancel_future.done())
        self.wait_for(lambda: self.goal_count == 1)
        with self.lock:
            self.assertEqual(self.runner.task.goal_id, "1")
            self.assertNotIn(CANCELED, [state["status"] for state in self.states])
            self.assertEqual(self.goal_count, 1)
        self.finish_gate.set()
        self.wait_for(lambda: self.runner.task.goal_id == "2")
        with self.lock:
            changes = [(state["goal_id"], state["status"]) for state in self.states]
            self.assertLess(changes.index(("1", CANCELED)), changes.index(("2", RUNNING)))

    def test_accepted_cancel_waits_for_terminal_result(self):
        self.submit("1")
        self.wait_for(lambda: self.goal_count == 1)
        self.check_replacement_waits_for_result()

    def test_cancel_while_waiting_for_goal_acceptance(self):
        self.accept_gate.clear()
        self.submit("1")
        self.assertTrue(self.goal_seen.wait(timeout=5.0))
        self.submit("2")
        with self.lock:
            self.assertEqual(self.goal_count, 0)
        self.accept_gate.set()
        self.check_replacement_waits_for_result()


if __name__ == "__main__":
    unittest.main()
