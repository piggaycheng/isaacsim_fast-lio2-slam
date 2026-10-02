import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from global_localization_xyz import XYZGlobalLocalization  # noqa: E402


class InitialPoseSeedTest(unittest.TestCase):
    def test_failed_first_icp_keeps_the_guess_for_retries(self):
        node = XYZGlobalLocalization.__new__(XYZGlobalLocalization)
        node.T_map_to_odom = np.eye(4)
        node.initialized = False
        node.cur_scan = None  # guess arrives before a usable scan
        node.get_logger = Mock()
        message = PoseWithCovarianceStamped()
        message.pose.pose.position.x, message.pose.pose.position.y = 3.7, -0.2
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        node.cb_initialize_pose(message)
        self.assertTrue(node.initialized)
        np.testing.assert_allclose(node.T_map_to_odom[:2, 3], [3.7, -0.2])
        np.testing.assert_allclose(node.T_map_to_odom[:2, :2], [[0, -1], [1, 0]], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
