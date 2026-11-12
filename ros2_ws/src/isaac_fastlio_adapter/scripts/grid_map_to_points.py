#!/usr/bin/env python3
"""Publish a grid_map layer as a PointCloud2 (one point per valid cell) for point-style RViz display."""

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import PointCloud2, PointField


class GridMapToPoints(Node):
    def __init__(self):
        super().__init__("grid_map_to_points")
        self.layer = self.declare_parameter("layer", "elevation").value
        self.publisher = self.create_publisher(PointCloud2, "/elevation_points", 1)
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.fill_radius = self.declare_parameter("fill_radius", 0.6).value
        self.fill_height = self.declare_parameter("fill_height_offset", 0.0).value
        self.fill_publisher = self.create_publisher(PointCloud2, "/elevation_points_assumed", 1)
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
