import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import open3d as o3d
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from slam_localization_3d.srv import LoadLocalizationMap

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from global_localization_xyz import XYZGlobalLocalization  # noqa: E402
from localization_3d_pose import body_to_base


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

    def test_switch_translates_base_pose_using_mount_and_nonzero_lio(self):
        node = XYZGlobalLocalization.__new__(XYZGlobalLocalization)
        node.get_logger = Mock()
        node.get_clock = Mock(return_value=Mock(now=Mock(
            return_value=Mock(nanoseconds=10_000_000_000))))
        node.get_parameter = Mock(return_value=Mock(value=0.1))
        node.body_to_base = body_to_base((0.2, -0.1, 0.5, 0.0, 0.0, 0.3))
        node.cur_odom = Odometry()
        node.cur_odom.header.stamp.sec = 10
        node.cur_odom.pose.pose.position.x = 12.0
        node.cur_odom.pose.pose.position.y = -4.0
        node.cur_odom.pose.pose.position.z = -3.0
        node.cur_odom.pose.pose.orientation.z = math.sin(0.4)
        node.cur_odom.pose.pose.orientation.w = math.cos(0.4)
        node.cur_scan = o3d.geometry.PointCloud()
        node.scan_stamp = 10.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.pcd"
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(
                np.random.default_rng(42).normal(size=(200, 3)))
            self.assertTrue(o3d.io.write_point_cloud(str(path), cloud))
            request = LoadLocalizationMap.Request(pcd_path=str(path))
            request.initial_pose.header.frame_id = "map"
            pose = request.initial_pose.pose.pose
            pose.position.x, pose.position.y, pose.position.z = 2.0, 3.0, -0.5
            pose.orientation.z, pose.orientation.w = math.sin(0.7), math.cos(0.7)
            result = node.load_localization_map(request, LoadLocalizationMap.Response())
            self.assertTrue(result.success, result.message)
            mount = np.eye(4)
            from tf_transformations import quaternion_matrix
            mount[:3, :3] = quaternion_matrix(node.body_to_base[1])[:3, :3]
            mount[:3, 3] = node.body_to_base[0]
            reconstructed = node.T_map_to_odom @ node.pose_to_mat(node.cur_odom.pose.pose) @ mount
            np.testing.assert_allclose(reconstructed, node.pose_to_mat(pose), atol=1e-9)
            self.assertIsNone(node.cur_scan)  # ICP must wait for a target-floor scan.
            self.assertTrue(node.initialized)

    def test_bad_pcd_does_not_replace_previous_map_or_seed(self):
        node = XYZGlobalLocalization.__new__(XYZGlobalLocalization)
        node.get_logger = Mock()
        node.get_clock = Mock(return_value=Mock(now=Mock(
            return_value=Mock(nanoseconds=10_000_000_000))))
        node.cur_odom = Odometry()
        node.cur_odom.header.stamp.sec = 10
        node.cur_scan = object()
        node.scan_stamp = 10.0
        node.global_map = object()
        node.T_map_to_odom = np.eye(4)
        old_map = node.global_map
        request = LoadLocalizationMap.Request(pcd_path="/missing.pcd")
        request.initial_pose.header.frame_id = "map"
        request.initial_pose.pose.pose.orientation.w = 1.0
        result = node.load_localization_map(request, LoadLocalizationMap.Response())
        self.assertFalse(result.success)
        self.assertIs(node.global_map, old_map)
        np.testing.assert_array_equal(node.T_map_to_odom, np.eye(4))


if __name__ == "__main__":
    unittest.main()
