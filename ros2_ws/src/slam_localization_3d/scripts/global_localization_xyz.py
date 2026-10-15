#!/usr/bin/env python3
"""Run upstream PCD localization with ICP covariance and without redundant scans.

/map_to_odom pose.covariance holds the 6x6 ICP covariance in ROS order
(x, y, z, roll, pitch, yaw) as a left perturbation in the map frame:
T_true = Exp([t, w]) * T_estimate. It is unscaled; global_pose_adapter applies
the calibrated registration_covariance_scale.
"""

import copy
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import numpy as np
import rclpy
import open3d as o3d
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_matrix
from slam_localization_3d.srv import LoadLocalizationMap
from std_srvs.srv import SetBool

from floor_maps import validate_initial_pose
from localization_3d_pose import imu_mount_parameter
from xyz_cloud import make_point_cloud

# Open3D information matrices use (wx, wy, wz, tx, ty, tz); ROS uses translation first.
OPEN3D_TO_ROS = [3, 4, 5, 0, 1, 2]


def icp_covariance(information, inlier_rmse, regularization=1e-9):
    """Return the ROS-ordered 6x6 point-to-point ICP covariance."""
    information = np.asarray(information, dtype=float)
    if information.shape != (6, 6) or not np.all(np.isfinite(information)):
        raise ValueError("ICP information matrix must be finite 6x6")
    # Per-axis residual variance; inlier RMSE is a 3D point distance.
    sigma_squared = max(float(inlier_rmse) ** 2 / 3.0, 1e-12)
    scale = max(np.trace(information) / 6.0, 1.0)
    covariance = sigma_squared * np.linalg.inv(
        information + regularization * scale * np.eye(6)
    )
    covariance = covariance[np.ix_(OPEN3D_TO_ROS, OPEN3D_TO_ROS)]
    return 0.5 * (covariance + covariance.T)


def upstream_localization_class():
    # Upstream installs this node as an executable, not a Python package.
    path = (
        Path(get_package_prefix("fast_lio_localization"))
        / "lib/fast_lio_localization/global_localization.py"
    )
    spec = spec_from_file_location("upstream_global_localization", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load upstream localization node from {path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FastLIOLocalization


class XYZGlobalLocalization(upstream_localization_class()):
    def __init__(self):
        super().__init__()
        if not self.destroy_publisher(self.pub_pc_in_map):
            raise RuntimeError("Could not remove redundant /cur_scan_in_map publisher")
        del self.pub_pc_in_map
        self.registration_covariance = None
        self.scan_stamp = None
        self.body_to_base = imu_mount_parameter(self)
        self.pending_initial_pose = None
        self.registration_hold = False
        self.create_service(
            LoadLocalizationMap, "localization_3d/load_map", self.load_localization_map
        )
        self.create_service(
            SetBool, "localization_3d/set_registration_hold", self.on_registration_hold)

    def on_registration_hold(self, request, response):
        self.registration_hold = request.data
        self.pending_initial_pose = None
        self.registration_covariance = None
        response.success = True
        response.message = "Global ICP paused" if request.data else "Global ICP enabled"
        self.get_logger().info(response.message)
        return response

    def load_localization_map(self, request, response):
        try:
            validate_initial_pose(request.initial_pose)
            if self.cur_odom is None or self.cur_scan is None:
                raise ValueError("Waiting for LIO odometry and scan")
            now = self.get_clock().now().nanoseconds * 1e-9
            stamp = self.cur_odom.header.stamp
            if not 0 <= now - (stamp.sec + stamp.nanosec * 1e-9) < 1.0:
                raise ValueError("LIO odometry is stale")
            if self.scan_stamp is None or not 0 <= now - self.scan_stamp < 1.0:
                raise ValueError("LIO scan is stale")
            path = Path(request.pcd_path)
            if path.suffix != ".pcd" or not path.is_file():
                raise ValueError(f"Missing PCD map: {path}")
            cloud = o3d.io.read_point_cloud(str(path))
            if cloud.is_empty() or not np.all(np.isfinite(np.asarray(cloud.points))):
                raise ValueError(f"PCD map is empty or non-finite: {path}")
            cloud = self.voxel_down_sample(cloud, self.get_parameter("map_voxel_size").value)
            # RMF supplies map -> base_link; ICP expects map -> camera_init.
            seed = self.base_pose_seed(request.initial_pose.pose.pose)
        except (ValueError, OSError, RuntimeError) as error:
            self.get_logger().error(f"PCD map switch rejected: {error}")
            response.success = False
            response.message = str(error)
            return response
        self.global_map = cloud
        self.T_map_to_odom = seed
        self.registration_covariance = None
        self.pending_initial_pose = None
        self.cur_scan = None
        self.initialized = True
        response.success = True
        response.message = f"Loaded {path}; waiting for ICP confirmation"
        self.get_logger().info(response.message)
        return response

    def global_localization(self, pose_estimation):
        if self.registration_hold:
            return
        # Same two ICP calls and threshold as upstream; the fine stage is inlined
        # so its downsampled clouds can also produce the information matrix.
        scan = copy.copy(self.cur_scan)
        submap = self.crop_global_map_in_FOV(pose_estimation)
        self.registration_at_scale(scan, submap, initial=pose_estimation, scale=5)
        scan_down = self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value)
        map_down = self.voxel_down_sample(submap, self.get_parameter("map_voxel_size").value)
        max_distance = 1.0
        result = o3d.pipelines.registration.registration_icp(
            scan_down, map_down, max_distance, pose_estimation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        threshold = self.get_parameter("localization_threshold").value
        if result.fitness <= threshold:
            self.get_logger().warn(
                f"Fitness score {result.fitness} less than localization threshold {threshold}"
            )
            return
        try:
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                scan_down, map_down, max_distance, result.transformation
            )
            self.registration_covariance = icp_covariance(information, result.inlier_rmse)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warn(f"ICP covariance unavailable: {error}")
            self.registration_covariance = None
        self.T_map_to_odom = result.transformation
        self.publish_odom(result.transformation)

    def base_pose_seed(self, pose):
        mount = np.eye(4)
        mount[:3, :3] = self.pose_to_mat(Pose(orientation=Quaternion(
            x=self.body_to_base[1][0], y=self.body_to_base[1][1],
            z=self.body_to_base[1][2], w=self.body_to_base[1][3],
        )))[:3, :3]
        mount[:3, 3] = self.body_to_base[0]
        return (
            self.pose_to_mat(pose) @ self.inverse_se3(mount)
            @ self.inverse_se3(self.pose_to_mat(self.cur_odom.pose.pose))
        )

    def cb_initialize_pose(self, msg):
        if self.registration_hold:
            self.get_logger().info("Global ICP held; initialpose is handled by floor manager")
            return
        try:
            validate_initial_pose(msg)
        except ValueError as error:
            self.get_logger().warning(f"Initial pose rejected: {error}")
            return
        if self.cur_odom is None:
            self.pending_initial_pose = copy.deepcopy(msg)
            self.get_logger().info("Initial pose received; waiting for LIO odometry")
            return
        pose = copy.deepcopy(msg.pose.pose)
        # A 2D RViz click supplies z=0; floor maps are recorded at the initial IMU height.
        if pose.position.z == 0.0:
            pose.position.z = self.body_to_base[0][2]
        self.T_map_to_odom = self.base_pose_seed(pose)
        self.pending_initial_pose = None
        self.registration_covariance = None
        self.initialized = True
        self.get_logger().info("Initial base_link pose received and converted to LIO map seed")
        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)

    def cb_save_cur_odom(self, msg):
        super().cb_save_cur_odom(msg)
        if self.pending_initial_pose is not None:
            self.cb_initialize_pose(self.pending_initial_pose)

    def publish_odom(self, transform):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        x, y, z = (float(value) for value in transform[:3, 3])
        qx, qy, qz, qw = (float(value) for value in quaternion_from_matrix(transform))
        message.pose.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        if self.registration_covariance is not None:
            message.pose.covariance = [
                float(value) for value in self.registration_covariance.reshape(-1)
            ]
        self.pub_map_to_odom.publish(message)

    def cb_save_cur_scan(self, msg):
        points = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(points)
        self.scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def publish_point_cloud(self, publisher, header, points):
        publisher.publish(make_point_cloud(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = XYZGlobalLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
