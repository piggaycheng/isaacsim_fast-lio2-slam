"""Run with Isaac Sim's python.sh; check Carter bounds against navigation geometry."""

import ast
import argparse
import json
import sys
import traceback
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from usd_bbox import box_corners, summarize, transform_points, yaw_rotation
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_localization_3d/launch"))
from robot_fleet import load_robot_profile, merge_overrides


def validate(stage, profile):
    from pxr import Usd, UsdGeom, UsdPhysics

    robot = stage.GetDefaultPrim()
    simulation = profile["simulation"]
    frame = stage.GetPrimAtPath(f"{robot.GetPath()}/{simulation['articulation']}")
    if not frame:
        raise RuntimeError("Carter chassis_link not found")
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(frame).GetInverse()
    clouds = {"collision": [], "visible": []}
    colliders = []
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        imageable = UsdGeom.Imageable(prim)
        collision = (
            prim.HasAPI(UsdPhysics.CollisionAPI)
            and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        )
        visible = (
            imageable.ComputeVisibility() != UsdGeom.Tokens.invisible
            and imageable.ComputePurpose() in ("default", "render")
        )
        if not collision and not visible:
            continue
        if prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points:
                raise RuntimeError(f"No mesh points: {prim.GetPath()}")
            local = np.asarray(points)
        else:
            extent = UsdGeom.Boundable.ComputeExtentFromPlugins(
                UsdGeom.Boundable(prim), Usd.TimeCode.Default(),
            )
            if extent is None or len(extent) != 2:
                raise RuntimeError(f"No collider bounds: {prim.GetPath()}")
            local = box_corners(extent[0], extent[1])
        matrix = cache.GetLocalToWorldTransform(prim) * inverse
        yaw = 180 if simulation["forward_sign"] < 0 else 0
        points = transform_points(local, matrix) @ yaw_rotation(yaw)
        points *= UsdGeom.GetStageMetersPerUnit(stage)
        if collision:
            clouds["collision"].append(points)
            colliders.append(str(prim.GetPath()))
        if visible:
            clouds["visible"].append(points)

    config = ROOT / "ros2_ws/src/slam_localization_3d/config"
    def parameters(path):
        return merge_overrides(yaml.safe_load(path.read_text()), profile["parameter_overrides"])

    costmaps = parameters(config / "observation_costmaps.yaml")
    footprints = [
        ast.literal_eval(costmaps[name][name]["ros__parameters"]["footprint"])
        for name in ("global_costmap", "local_costmap")
    ]
    if footprints[0] != footprints[1]:
        raise AssertionError("Global and local physical footprints differ")
    footprint = np.asarray(footprints[1])
    fmin, fmax = footprint.min(axis=0), footprint.max(axis=0)
    filtering = parameters(
        ROOT / "ros2_ws/src/slam_nav/config/ground_obstacle_filter.yaml"
    )["ground_obstacle_filter"]["ros__parameters"]["self_filter_bounds"]
    np.testing.assert_allclose(filtering, [fmin[0], fmax[0], fmin[1], fmax[1]])
    monitor = parameters(config / "collision_monitor.yaml")
    surround = np.asarray(
        monitor["collision_monitor"]["ros__parameters"]["PolygonSurround"]["points"]
    ).reshape(-1, 2)
    padding = costmaps["local_costmap"]["local_costmap"]["ros__parameters"]["footprint_padding"]
    if padding != costmaps["global_costmap"]["global_costmap"]["ros__parameters"]["footprint_padding"]:
        raise AssertionError("Global and local footprint padding differs")
    report = {"robot_type": profile["robot_type"],
              "collider_count": len(colliders), "footprint": footprints[1],
              "planning_footprint": footprints[0],
              "footprint_padding": padding,
              "self_filter": filtering, "surround": surround.tolist()}
    if profile["robot_type"] == "nova_carter":
        def base_x(name):
            prim = stage.GetPrimAtPath(f"{robot.GetPath()}/{name}")
            if not prim:
                raise RuntimeError(f"Missing Nova Carter wheel: {name}")
            position = (cache.GetLocalToWorldTransform(prim) * inverse).ExtractTranslation()
            return float(position[0]) * simulation["forward_sign"]

        drive_x = [base_x(name) for name in ("wheel_left", "wheel_right")]
        caster_x = [base_x(name) for name in ("caster_wheel_left", "caster_wheel_right")]
        if min(drive_x) <= max(caster_x):
            raise AssertionError("Nova Carter's drive wheels must lead its caster wheels")
        report["drive_wheel_x"] = drive_x
        report["caster_wheel_x"] = caster_x
    for kind, chunks in clouds.items():
        if not chunks:
            raise RuntimeError(f"No {kind} geometry found")
        points = np.vstack(chunks)
        minimum, maximum = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
        margins = np.r_[minimum - fmin, fmax - maximum]
        if (margins < 0).any():
            raise AssertionError(f"Footprint does not enclose {kind}: {margins}")
        report[kind] = summarize(points, decimals=6)
        report[kind]["footprint_margins_rear_right_front_left"] = margins.tolist()
    if (surround.min(axis=0) >= fmin).any() or (surround.max(axis=0) <= fmax).any():
        raise AssertionError("Surround zone must extend outside footprint on every side")
    print("GEOMETRY_RESULT " + json.dumps(report), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--robot-type", default="nova_carter")
    args = parser.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    status = 1
    try:
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Usd

        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac assets root not found")
        profile = load_robot_profile(args.robot_type)
        stage = Usd.Stage.Open(root + profile["simulation"]["asset"])
        if stage is None:
            raise RuntimeError("Cannot load Carter asset")
        report = validate(stage, profile)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
        status = 0
    except (AssertionError, KeyError, OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError):
        traceback.print_exc()
        sys.stderr.flush()
    finally:
        app.close(exit_code=status)
