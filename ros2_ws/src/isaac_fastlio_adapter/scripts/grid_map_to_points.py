#!/usr/bin/env python3
"""Publish a grid_map layer as a PointCloud2 and extract untraversable obstacle points."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

try:
    from grid_map_msgs.msg import GridMap
except ImportError:
    GridMap = None


def extract_obstacle_points(x, y, z, resolution, step_threshold, window_radius):
    """Return (N, 3) points where local height jump within window_radius >= step_threshold."""
    valid = np.isfinite(z)
    if not np.any(valid) or resolution <= 0.0 or step_threshold <= 0.0 or window_radius <= 0.0:
        return np.empty((0, 3), dtype=np.float32)
    rows, cols = z.shape
    radius_cells = max(1, int(round(float(window_radius) / float(resolution))))
    z_inf = np.where(valid, z, np.inf)
    padded = np.pad(z_inf, radius_cells, mode="constant", constant_values=np.inf)
    local_min = z_inf.copy()
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr == 0 and dc == 0:
                continue
            if dr * dr + dc * dc <= r2:
                np.minimum(
                    local_min,
                    padded[
                        radius_cells + dr : radius_cells + dr + rows,
                        radius_cells + dc : radius_cells + dc + cols,
                    ],
                    out=local_min,
                )
    mask = valid & np.isfinite(local_min) & ((z - local_min) >= float(step_threshold))
    if not np.any(mask):
        return np.empty((0, 3), dtype=np.float32)
    return np.stack([x[mask], y[mask], z[mask]], axis=1).astype(np.float32)


def transform_points(points, translation, rotation):
    """Apply a geometry_msgs Transform (translation, rotation) to (N, 3) points."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    rot = np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float32)
    offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
    return (points @ rot.T + offset).astype(np.float32)


class GridMapToPoints(Node):
    def __init__(self, **kwargs):
        super().__init__("grid_map_to_points", **kwargs)
        self.layer = self.declare_parameter("layer", "elevation").value
        self.publisher = self.create_publisher(PointCloud2, "/elevation_points", 1)
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.fill_radius = self.declare_parameter("fill_radius", 0.6).value
        self.fill_height = self.declare_parameter("fill_height_offset", 0.0).value
        self.fill_publisher = self.create_publisher(PointCloud2, "/elevation_points_assumed", 1)
        self.publish_obstacles = bool(self.declare_parameter("publish_obstacles", True).value)
        self.obstacles_topic = self.declare_parameter(
            "obstacles_topic", "/perception/obstacles"
        ).value
        # Single stair riser is ~0.18 m over a ~0.28 m tread; a 0.30 m step within
        # 0.12 m ignores climbable stairs and slopes while marking walls, boxes and columns.
        self.obstacle_step_threshold = float(
            self.declare_parameter("obstacle_step_threshold", 0.30).value
        )
        self.obstacle_window_radius = float(
            self.declare_parameter("obstacle_window_radius", 0.12).value
        )
        self.min_obstacle_height = float(
            self.declare_parameter("min_obstacle_height", -2.0).value
        )
        self.max_obstacle_height = float(
            self.declare_parameter("max_obstacle_height", 0.6).value
        )
        self.obstacles_publisher = self.create_publisher(
            PointCloud2, self.obstacles_topic, qos_profile_sensor_data
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            GridMap, self.declare_parameter("input_topic", "/elevation_mapping_node/elevation_map_filter").value,
            self.callback, 1,
        )

    def callback(self, msg: GridMap) -> None:
        if self.layer not in msg.layers:
            return
        layer = msg.data[msg.layers.index(self.layer)]
        rows, cols = layer.layout.dim[0].size, layer.layout.dim[1].size
        # grid_map data is column-major and index (0, 0) is the +x/+y corner of the map.
        z = np.array(layer.data, dtype=np.float32).reshape(cols, rows).T
        res = msg.info.resolution
        x = msg.info.pose.position.x + msg.info.length_x / 2.0 - res * (np.arange(rows)[:, None] + 0.5)
        y = msg.info.pose.position.y + msg.info.length_y / 2.0 - res * (np.arange(cols)[None, :] + 0.5)
        x, y = np.broadcast_arrays(x, y)
        valid = np.isfinite(z)
        points = np.stack([x[valid], y[valid], z[valid]], axis=1).astype(np.float32)

        self.publisher.publish(self.make_cloud(msg.header, points))
        self.publish_assumed(msg.header, x, y, valid)
        if self.publish_obstacles:
            self.publish_obstacle_cloud(msg.header, x, y, z, res)

    def publish_obstacle_cloud(self, header, x, y, z, resolution) -> None:
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame, header.frame_id, rclpy.time.Time()
            )
            transform = transform_stamped.transform
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        obstacles = extract_obstacle_points(
            x, y, z, resolution, self.obstacle_step_threshold, self.obstacle_window_radius
        )
        in_base = transform_points(obstacles, transform.translation, transform.rotation)
        if len(in_base) > 0:
            height_mask = (
                (in_base[:, 2] >= self.min_obstacle_height)
                & (in_base[:, 2] <= self.max_obstacle_height)
            )
            in_base = in_base[height_mask]
        out_header = Header()
        tf_stamp = transform_stamped.header.stamp
        out_header.stamp = tf_stamp if (tf_stamp.sec or tf_stamp.nanosec) else header.stamp
        out_header.frame_id = self.base_frame
        self.obstacles_publisher.publish(self.make_cloud(out_header, in_base))

    def publish_assumed(self, header, x, y, valid) -> None:
        # Cells under the robot with no measurement are assumed flat at the base_link height.
        # They disappear as soon as the layer has a real value there.
        try:
            t = self.tf_buffer.lookup_transform(header.frame_id, self.base_frame, rclpy.time.Time()).transform.translation
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        empty = ~valid & (np.hypot(x - t.x, y - t.y) <= self.fill_radius)
        z = np.full(int(empty.sum()), t.z + self.fill_height, dtype=np.float32)
        points = np.stack([x[empty], y[empty], z], axis=1).astype(np.float32)
        self.fill_publisher.publish(self.make_cloud(header, points))

    @staticmethod
    def make_cloud(header, points) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, points.shape[0]
        cloud.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")
        ]
        cloud.point_step, cloud.row_step = 12, 12 * points.shape[0]
        cloud.is_dense = True
        cloud.data = points.tobytes()
        return cloud


def main():
    rclpy.init()
    rclpy.spin(GridMapToPoints())


if __name__ == "__main__":
    main()
