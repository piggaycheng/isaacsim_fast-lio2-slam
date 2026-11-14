"""Validate floor map pairs and the public base_link initial-pose contract."""

import json
import math
from pathlib import Path
import re

import yaml

from localization_3d_pose import body_to_base, compose_pose, inverse_pose

FILTER_KINDS = ("keepout", "speed")


def fleet_zone_namespace(floor):
    """Namespace of the fleet RViz costmap filter editor serving one floor."""
    # Topic tokens cannot start with a digit or contain '-', hence the prefix and check.
    if not re.fullmatch(r"[A-Za-z0-9_]+", floor or ""):
        raise ValueError(
            f"Fleet costmap filters need floor names of letters, digits and _, got {floor!r}")
    return f"/fleet/zones/floor_{floor}"


def fleet_mask_topics(floor):
    namespace = fleet_zone_namespace(floor)
    return [f"{namespace}/costmap_filters/{kind}_mask" for kind in FILTER_KINDS]


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


def reference_pose(data):
    position = tuple(float(value) for value in data["position"])
    quaternion = tuple(float(value) for value in data["orientation_xyzw"])
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError("Reference pose needs position[3] and orientation_xyzw[4]")
    if not all(math.isfinite(value) for value in (*position, *quaternion)):
        raise ValueError("Reference pose must be finite")
    norm = sum(value * value for value in quaternion)
    if not math.isclose(norm, 1.0, abs_tol=1e-3):
        raise ValueError("Reference orientation must be a unit quaternion")
    return position, tuple(value / math.sqrt(norm) for value in quaternion)


def session_map_from_world(session_path, expected_floor=None):
    with Path(session_path).open(encoding="utf-8") as stream:
        session = json.load(stream)
    if expected_floor is not None and session.get("floor") != expected_floor:
        raise ValueError(f"session floor does not match {expected_floor!r}")
    reference = session.get("reference")
    if not isinstance(reference, dict):
        raise ValueError("session has no mapping reference")
    lio = reference["lio_initial_pose"]
    chassis = reference["isaac_chassis_pose"]
    if chassis["frame_id"] != "isaac_world":
        raise ValueError("Reference chassis pose must be in isaac_world")
    if lio["frame_id"] != "lidar" or lio["child_frame_id"] != "body":
        raise ValueError("Reference LIO pose must describe lidar -> body")
    mount = reference["sensor_frames"]["imu_link"]
    body_base = body_to_base([float(mount[key]) for key in
                              ("x", "y", "z", "roll", "pitch", "yaw")])
    world_base = reference_pose(chassis)
    map_base = compose_pose(reference_pose(lio), body_base)
    return compose_pose(map_base, inverse_pose(world_base)), world_base[0][2]


def sibling_floor_pcds(pcd_path):
    """Find sibling floor PCDs and their SE(3) transforms into pcd_path's map frame."""
    primary = Path(pcd_path)
    primary_dir = primary.parent
    primary_session = primary_dir / "session.json"
    if not primary_session.is_file() or not (primary_dir / "map_2d.yaml").is_file():
        return []
    try:
        primary_from_world, _ = session_map_from_world(primary_session, primary_dir.name)
    except (ValueError, KeyError, OSError):
        return []
    siblings = []
    for session_file in sorted(primary_dir.parent.glob("*/session.json")):
        floor_dir = session_file.parent
        if floor_dir == primary_dir or not (floor_dir / "map_2d.yaml").is_file():
            continue
        sibling_pcd = floor_dir / primary.name
        if not sibling_pcd.is_file():
            continue
        try:
            sibling_from_world, _ = session_map_from_world(session_file, floor_dir.name)
        except (ValueError, KeyError, OSError):
            continue
        transform = compose_pose(primary_from_world, inverse_pose(sibling_from_world))
        siblings.append((floor_dir.name, str(sibling_pcd), transform))
    return siblings
