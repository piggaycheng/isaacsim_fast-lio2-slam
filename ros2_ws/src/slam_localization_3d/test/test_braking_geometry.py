import math
import sys
import unittest
from pathlib import Path

import numpy as np
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tests"))
from validate_braking import BrakingProbe, rectangle, separation


class BrakingGeometryTest(unittest.TestCase):
    def test_aligned_clearance_and_overlap(self):
        body = rectangle([-0.2, 0.65, -0.32, 0.32], [0, 0], 0)
        front = rectangle([-0.1, 0.1, -0.3, 0.3], [1.1, 0], 0)
        rear = rectangle([-0.1, 0.1, -0.3, 0.3], [-0.6, 0], 0)
        self.assertAlmostEqual(separation(body, front), 0.35)
        self.assertAlmostEqual(separation(body, rear), 0.3)
        overlap = rectangle([-0.1, 0.1, -0.3, 0.3], [0.65, 0], 0)
        self.assertLess(separation(body, overlap), 0)

    def test_rigid_rotation_preserves_gap(self):
        bounds = [-0.2, 0.65, -0.32, 0.32]
        for yaw in (0, 0.5, math.pi / 2, math.pi):
            body = rectangle(bounds, [2, 3], yaw)
            offset = np.array([math.cos(yaw), math.sin(yaw)]) * 1.1
            box = rectangle([-0.1, 0.1, -0.3, 0.3], offset + [2, 3], yaw)
            self.assertAlmostEqual(separation(body, box), 0.35)

    def test_truth_chassis_is_rotated_to_ros_forward_frame(self):
        truth = Odometry()
        truth.pose.pose.position.x = 2.0
        truth.pose.pose.orientation.w = 1.0
        position, yaw = BrakingProbe.pose(truth)
        np.testing.assert_allclose(position, [2, 0])
        self.assertAlmostEqual(yaw, math.pi)
