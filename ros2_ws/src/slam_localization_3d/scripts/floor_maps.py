"""Validate floor map pairs and the public base_link initial-pose contract."""

import math
from pathlib import Path
import re

import yaml


def floor_maps(root, floor):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", floor):
        raise ValueError("floor_name must be a simple level name, e.g. 1F or B1")
    root = Path(root).resolve(strict=True)
    directory = (root / floor).resolve(strict=True)
    if directory.parent != root:
        raise ValueError("Floor directory must be inside maps_root")
    map_yaml, pcd = directory / "map_2d.yaml", directory / "map.pcd"
    for path in (map_yaml, pcd):
        if not path.is_file():
            raise ValueError(f"Missing floor map: {path}")
    with map_yaml.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
        raise ValueError(f"Map YAML has no image: {map_yaml}")
    image = directory / metadata["image"]
    if not image.is_file():
        raise ValueError(f"Missing map image: {image}")
    return str(map_yaml), str(pcd)


def validate_initial_pose(message):
    if message.header.frame_id != "map":
        raise ValueError("initial_pose.header.frame_id must be map (target floor)")
    pose = message.pose.pose
    values = [
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("initial_pose must be finite")
    if not math.isclose(sum(value * value for value in values[3:]), 1.0, abs_tol=1e-3):
        raise ValueError("initial_pose orientation must be a unit quaternion")
    covariance = message.pose.covariance
    if not all(math.isfinite(value) for value in covariance):
        raise ValueError("initial_pose covariance must be finite")
    if any(covariance[index] < 0 for index in range(0, 36, 7)):
        raise ValueError("initial_pose covariance diagonal must be nonnegative")
