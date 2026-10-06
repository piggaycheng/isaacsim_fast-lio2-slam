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
from sensor_msgs.msg import JointState
from slam_sensor_control.action import GimbalMove

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_sensor_control/scripts"))
from gimbal_action_server import PAN_JOINT, TILT_JOINT, GimbalActionServer, wrap_to_pi


class FakeGimbal(Node):
    """Stands in for the simulator: moves the pan the short way at a fixed speed."""

    def __init__(self, speed=2.0):
        super().__init__("fake_gimbal")
        self.speed = speed
        self.pan = self.tilt = 0.0
        self.goal = None
        self.frozen = False
        self.create_subscription(JointState, "gimbal/joint_command", self.on_command, 10)
        self.pub = self.create_publisher(JointState, "gimbal/joint_states", 10)
        self.create_timer(0.02, self.tick)

    def on_command(self, message):
        self.goal = dict(zip(message.name, message.position))

    def tick(self):
        if self.goal and not self.frozen:
            step = self.speed * 0.02
            self.pan += max(-step, min(step, wrap_to_pi(self.goal[PAN_JOINT] - self.pan)))
            self.tilt += max(-step, min(step, self.goal[TILT_JOINT] - self.tilt))
        message = JointState(name=["wheel", PAN_JOINT, TILT_JOINT], position=[0.0, self.pan, self.tilt])
        self.pub.publish(message)


class TestGimbalActionServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.server = GimbalActionServer()
        cls.server.timeout = 1.5
        cls.fake = FakeGimbal()
        cls.client_node = Node("gimbal_client")
        cls.client = ActionClient(cls.client_node, GimbalMove, "gimbal/move")
        cls.executor = MultiThreadedExecutor()
        for node in (cls.server, cls.fake, cls.client_node):
            cls.executor.add_node(node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        for node in (cls.server, cls.fake, cls.client_node):
            node.destroy_node()
        rclpy.shutdown()

    def send(self, pan, tilt):
        goal = GimbalMove.Goal(pan=pan, tilt=tilt)
        future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result()

    def result_of(self, handle, timeout=10.0):
        future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result().result

    def test_success_only_after_pose_matches(self):
        self.fake.frozen = False
        handle = self.send(math.radians(170.0), 0.3)
        self.assertTrue(handle.accepted)
        result = self.result_of(handle)
        self.assertTrue(result.success)
        self.assertLess(abs(wrap_to_pi(result.pan - math.radians(170.0))), math.radians(1.0))
        self.assertLess(abs(result.tilt - 0.3), math.radians(1.0))

    def test_pan_goes_the_short_way(self):
        self.fake.pan = math.radians(170.0)
        handle = self.send(math.radians(-170.0), 0.3)
        self.assertTrue(self.result_of(handle).success)
        self.assertGreater(self.fake.pan, math.radians(170.0))

    def test_times_out_when_pose_never_matches(self):
        self.fake.frozen = True
        try:
            handle = self.send(1.0, 0.0)
            result = self.result_of(handle)
            self.assertFalse(result.success)
            self.assertIn("Timed out", result.message)
        finally:
            self.fake.frozen = False

    def test_rejects_tilt_outside_limits(self):
        self.assertFalse(self.send(0.0, math.radians(120.0)).accepted)

    def test_newer_goal_supersedes(self):
        self.fake.frozen = True
        first = self.send(1.0, 0.0)
        time.sleep(0.2)
        self.fake.frozen = False
        second = self.send(-1.0, 0.0)
        self.assertFalse(self.result_of(first).success)
        self.assertTrue(self.result_of(second).success)


if __name__ == "__main__":
    unittest.main()
