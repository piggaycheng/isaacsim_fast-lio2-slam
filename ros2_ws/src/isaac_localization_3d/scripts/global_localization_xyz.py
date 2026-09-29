#!/usr/bin/env python3
"""Run upstream PCD localization without redundant scan publication."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import rclpy
import open3d as o3d
from ament_index_python.packages import get_package_prefix

from xyz_cloud import make_point_cloud


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

    def cb_save_cur_scan(self, msg):
        points = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(points)

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
