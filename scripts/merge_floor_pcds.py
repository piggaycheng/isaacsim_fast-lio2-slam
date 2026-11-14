#!/usr/bin/env python3
"""Merge sibling floor PCD maps into a single 3D PCD aligned in the primary floor's map frame."""

import argparse
from pathlib import Path
import shutil
import sys

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "ros2_ws/src/slam_localization_3d/scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from floor_maps import sibling_floor_pcds  # noqa: E402
from localization_3d_pose import rotate_vector  # noqa: E402


def transform_points(points, transform):
    import numpy as np

    translation, (qx, qy, qz, qw) = transform
    # Rotation matrix from unit quaternion (qx, qy, qz, qw).
    rotation = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def merge_floor_pcds(primary_pcd, output_pcd, voxel_size=0.0):
    import numpy as np
    import open3d as o3d

    primary_path = Path(primary_pcd).resolve(strict=True)
    cloud = o3d.io.read_point_cloud(str(primary_path))
    if cloud.is_empty():
        raise ValueError(f"Primary PCD is empty: {primary_path}")
    merged_floors = [primary_path.parent.name]
    for floor_name, sibling_path, transform in sibling_floor_pcds(primary_path):
        sibling = o3d.io.read_point_cloud(sibling_path)
        if sibling.is_empty():
            continue
        points = transform_points(np.asarray(sibling.points), transform)
        transformed = o3d.geometry.PointCloud()
        transformed.points = o3d.utility.Vector3dVector(points)
        cloud += transformed
        merged_floors.append(floor_name)
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(float(voxel_size))
    output_path = Path(output_pcd)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(output_path), cloud)
    primary_session = primary_path.parent / "session.json"
    output_session = output_path.parent / "session.json"
    if primary_session.is_file() and primary_session.resolve() != output_session.resolve():
        shutil.copy2(primary_session, output_session)
    return merged_floors, len(cloud.points)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--primary-pcd",
        default="maps/1F/map.pcd",
        help="Primary floor map.pcd whose coordinate frame defines the merged map frame.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output merged .pcd file path.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Optional voxel downsample size in meters (0 preserves all points).",
    )
    args = parser.parse_args(argv)
    floors, count = merge_floor_pcds(args.primary_pcd, args.output, args.voxel_size)
    print(f"Merged floors {', '.join(floors)} -> {args.output} ({count} points)")


if __name__ == "__main__":
    main()
