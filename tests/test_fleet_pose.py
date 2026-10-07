import sys
import unittest
from pathlib import Path

from nav_msgs.msg import Odometry

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from fleet_pose import yaw_from_quaternion  # noqa: E402


class FleetPoseTest(unittest.TestCase):
    def test_yaw_ignores_roll_and_pitch(self):
        message = Odometry()
        message.pose.pose.orientation.w = 1.0
        self.assertAlmostEqual(yaw_from_quaternion(message.pose.pose.orientation), 0.0)


if __name__ == "__main__":
    unittest.main()
