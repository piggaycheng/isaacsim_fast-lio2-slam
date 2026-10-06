import json
import math
import sys
import unittest
from pathlib import Path

from nav_msgs.msg import Odometry

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from fleet_pose import pose_payload, yaw_from_quaternion  # noqa: E402


class FleetPoseTest(unittest.TestCase):
    def test_payload_has_planar_pose_and_yaw(self):
        message = Odometry()
        message.header.frame_id = "map"
        message.header.stamp.sec = 12
        message.header.stamp.nanosec = 500_000_000
        message.pose.pose.position.x = 1.5
        message.pose.pose.position.y = -2.0
        message.pose.pose.position.z = 0.3
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        payload = json.loads(pose_payload("carter1", message))
        self.assertEqual(payload["robot"], "carter1")
        self.assertEqual(payload["frame_id"], "map")
        self.assertAlmostEqual(payload["stamp"], 12.5)
        self.assertEqual((payload["x"], payload["y"]), (1.5, -2.0))
        self.assertAlmostEqual(payload["yaw"], math.pi / 2)
        self.assertNotIn("z", payload)

    def test_yaw_ignores_roll_and_pitch(self):
        message = Odometry()
        message.pose.pose.orientation.w = 1.0
        self.assertAlmostEqual(yaw_from_quaternion(message.pose.pose.orientation), 0.0)


if __name__ == "__main__":
    unittest.main()
