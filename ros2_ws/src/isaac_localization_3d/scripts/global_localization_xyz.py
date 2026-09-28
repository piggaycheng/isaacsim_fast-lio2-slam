#!/usr/bin/env python3
"""Run upstream PCD localization with truthful XYZ-only visualization clouds."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import rclpy
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
