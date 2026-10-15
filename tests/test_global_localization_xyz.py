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
from std_srvs.srv import SetBool

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from global_localization_xyz import XYZGlobalLocalization  # noqa: E402
from localization_3d_pose import body_to_base


class InitialPoseSeedTest(unittest.TestCase):
    def make_node(self):
        node = XYZGlobalLocalization.__new__(XYZGlobalLocalization)
        node.registration_hold = False
        node.pending_initial_pose = None
        return node

    def test_rviz_seed_registers_scan_after_nonzero_motion_and_elevator_descent(self):
        node = self.make_node()
        node.get_logger = Mock()
        node.get_parameter = lambda name: Mock(value={
            "scan_voxel_size": 0.1, "map_voxel_size": 0.1, "localization_threshold": 0.8,
        }[name])
        node.publish_odom = Mock()
        node.body_to_base = body_to_base((0.2, -0.1, 0.5, 0.0, 0.0, 0.3))
        node.cur_odom = Odometry()
        node.cur_odom.pose.pose.position.x = 12.0
        node.cur_odom.pose.pose.position.y = -4.0
        node.cur_odom.pose.pose.position.z = -3.3
        node.cur_odom.pose.pose.orientation.z = math.sin(0.4)
        node.cur_odom.pose.pose.orientation.w = math.cos(0.4)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = 2.0
        initial.pose.pose.position.y = 3.0
        initial.pose.pose.position.z = -0.5
        initial.pose.pose.orientation.z = math.sin(0.7)
        initial.pose.pose.orientation.w = math.cos(0.7)
        from tf_transformations import quaternion_matrix
        mount = quaternion_matrix(node.body_to_base[1])
        mount[:3, 3] = node.body_to_base[0]
        expected = (
            node.pose_to_mat(initial.pose.pose)
            @ np.linalg.inv(mount) @ np.linalg.inv(node.pose_to_mat(node.cur_odom.pose.pose))
        )
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.random.default_rng(42).uniform(-4.0, 4.0, size=(1000, 3)))
        node.crop_global_map_in_FOV = Mock(return_value=cloud)
        node.cur_scan = o3d.geometry.PointCloud(cloud)
        node.cur_scan.transform(np.linalg.inv(expected))
        naive = o3d.pipelines.registration.registration_icp(
            node.cur_scan, cloud, 1.0, node.pose_to_mat(initial.pose.pose),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        self.assertLess(naive.fitness, 0.8)
        node.cb_initialize_pose(initial)
        node.publish_odom.assert_called_once()
        np.testing.assert_allclose(
            node.publish_odom.call_args.args[0], expected, atol=1e-3)

    def test_failed_first_icp_keeps_the_guess_for_retries(self):
        node = self.make_node()
        node.T_map_to_odom = np.eye(4)
        node.initialized = False
        node.cur_scan = None  # guess arrives before a usable scan
        node.cur_odom = Odometry()
        node.body_to_base = body_to_base((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        node.get_logger = Mock()
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x, message.pose.pose.position.y = 3.7, -0.2
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        node.cb_initialize_pose(message)
        self.assertTrue(node.initialized)
        np.testing.assert_allclose(node.T_map_to_odom[:2, 3], [3.7, -0.2])
        np.testing.assert_allclose(node.T_map_to_odom[:2, :2], [[0, -1], [1, 0]], atol=1e-9)

    def test_switch_translates_base_pose_using_mount_and_nonzero_lio(self):
        node = self.make_node()
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
            node.registration_hold = True
            result = node.load_localization_map(request, LoadLocalizationMap.Response())
            self.assertTrue(result.success, result.message)
            self.assertTrue(node.registration_hold)
            mount = np.eye(4)
            from tf_transformations import quaternion_matrix
            mount[:3, :3] = quaternion_matrix(node.body_to_base[1])[:3, :3]
            mount[:3, 3] = node.body_to_base[0]
            reconstructed = node.T_map_to_odom @ node.pose_to_mat(node.cur_odom.pose.pose) @ mount
            np.testing.assert_allclose(reconstructed, node.pose_to_mat(pose), atol=1e-9)
            self.assertIsNone(node.cur_scan)  # ICP must wait for a target-floor scan.
            self.assertTrue(node.initialized)
            service_seed = node.T_map_to_odom.copy()
            node.T_map_to_odom = np.eye(4)
            node.registration_hold = False
            node.cb_initialize_pose(request.initial_pose)
            np.testing.assert_allclose(node.T_map_to_odom, service_seed, atol=1e-9)

    def test_rviz_pose_before_odometry_is_converted_when_lio_arrives(self):
        node = self.make_node()
        node.get_logger = Mock()
        node.body_to_base = body_to_base((-0.213, 0.009, 0.526, 0.0, 0.0, 0.0))
        node.cur_odom = None
        node.cur_scan = None
        node.initialized = False
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = 2.0
        message.pose.pose.orientation.w = 1.0
        node.cb_initialize_pose(message)
        self.assertFalse(node.initialized)
        odom = Odometry()
        odom.pose.pose.position.x = 12.0
        odom.pose.pose.position.z = -3.3
        node.cb_save_cur_odom(odom)
        self.assertTrue(node.initialized)
        self.assertIsNone(node.pending_initial_pose)
        expected = node.pose_to_mat(message.pose.pose)
        expected[2, 3] = -0.526
        mount = np.eye(4)
        mount[:3, 3] = node.body_to_base[0]
        np.testing.assert_allclose(
            node.T_map_to_odom @ node.pose_to_mat(odom.pose.pose) @ mount, expected, atol=1e-9)

    def test_invalid_rviz_pose_does_not_replace_seed(self):
        node = self.make_node()
        node.get_logger = Mock()
        node.T_map_to_odom = np.eye(4)
        node.initialized = False
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "odom"
        node.cb_initialize_pose(message)
        np.testing.assert_array_equal(node.T_map_to_odom, np.eye(4))
        self.assertFalse(node.initialized)

    def test_bad_pcd_does_not_replace_previous_map_or_seed(self):
        node = self.make_node()
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

    def test_registration_hold_ignores_rviz_and_icp_but_keeps_odometry(self):
        node = self.make_node()
        node.get_logger = Mock()
        node.T_map_to_odom = np.eye(4)
        node.crop_global_map_in_FOV = Mock()
        response = node.on_registration_hold(
            SetBool.Request(data=True), SetBool.Response())
        self.assertTrue(response.success)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = 20.0
        node.cb_initialize_pose(pose)
        node.global_localization(np.eye(4))
        node.crop_global_map_in_FOV.assert_not_called()
        np.testing.assert_array_equal(node.T_map_to_odom, np.eye(4))
        self.assertIsNone(node.pending_initial_pose)
        odom = Odometry()
        odom.pose.pose.position.z = -3.3
        node.cb_save_cur_odom(odom)
        self.assertIs(node.cur_odom, odom)
        node.on_registration_hold(SetBool.Request(data=False), SetBool.Response())
        self.assertFalse(node.registration_hold)


if __name__ == "__main__":
    unittest.main()
