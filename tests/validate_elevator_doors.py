"""Run with Isaac Sim's python.sh; validate Office door skinning and live collisions."""

import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from elevator_doors import ElevatorDoors


def validate(app):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.utils.stage import is_stage_loading
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
    from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

    asset_root = get_assets_root_path()
    if asset_root is None:
        raise RuntimeError("Isaac assets root not found")
    omni.usd.get_context().open_stage(asset_root + "/Isaac/Environments/Office/office.usd")
    app.update()
    while is_stage_loading():
        app.update()
    stage = omni.usd.get_context().get_stage()
    original = stage.GetRootLayer().ExportToString()
    doors = ElevatorDoors(stage)
    assert len(doors.doors) == 5, "Office door set count changed"
    roots = [
        UsdSkel.Root(p) for p in stage.Traverse()
        if p.IsA(UsdSkel.Root) and p.GetName().startswith("SK_Elevator_doors_")
    ]

    def skinned_points(root):
        cache = UsdSkel.Cache()
        cache.Populate(root, Usd.PrimDefaultPredicate)
        skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath(str(root.GetPath()) + "/Skeleton"))
        mesh = stage.GetPrimAtPath(str(root.GetPath()) + "/Mesh")
        points = Vt.Vec3fArray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
        skin = cache.GetSkinningQuery(mesh)
        assert skin.ComputeSkinnedPoints(
            cache.GetSkelQuery(skeleton).ComputeSkinningTransforms(),
            points, Usd.TimeCode.Default(),
        )
        world = UsdGeom.XformCache().GetLocalToWorldTransform(skeleton.GetPrim())
        return [world.Transform(Gf.Vec3d(p)) for p in points], skin

    closed = [skinned_points(root)[0] for root in roots]
    doors.update(0.0)
    for time, fraction in ((6.0, 0.5), (7.0, 1.0), (13.0, 0.5), (14.0, 0.0)):
        doors.update(time)
        assert doors.fraction == fraction
        for root, before, (_, _, _, panels) in zip(roots, closed, doors.doors):
            after, skin = skinned_points(root)
            indices, _ = skin.ComputeVaryingJointInfluences(len(after))
            count = skin.GetNumInfluencesPerComponent()
            for position, center, travel in panels:
                joint = int(position.GetAttr().GetPrim().GetName().split("_")[-1])
                vertices = [i for i in range(len(after)) if indices[i * count] == joint]
                assert vertices
                for vertex in vertices:
                    error = after[vertex] - before[vertex] - travel * fraction
                    assert error.GetLength() < 1e-5, (root.GetPath(), joint, error)
                assert (position.Get() - center - travel * fraction).GetLength() < 1e-6
    assert stage.GetRootLayer().ExportToString() == original, "Modified original asset layer"
    lidar = Lidar(
        "/World/DoorValidationLidar", positions=[[-14.0, -1.5, 1.2]],
        attributes={"omni:sensor:Core:outputMotionCompensationState": "COMPENSATED"},
    )
    sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    SimulationManager.setup_simulation(dt=1 / 60, device="cpu")
    SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
    omni.timeline.get_timeline_interface().play()
    for _ in range(20):
        app.update()
    scene_query = omni.physx.get_physx_scene_query_interface()
    for time, blocked in ((14.0, True), (21.0, False), (28.0, True)):
        doors.update(time)
        ranges = []
        for frame in range(60):
            app.update()
            data, _ = sensor.get_data("generic-model-output")
            if frame < 30 or data is None:
                continue
            cloud = parse_generic_model_output_data(data)
            if cloud.numElements:
                forward = (np.abs(np.abs(cloud.x) - 180) < 5) & (np.abs(cloud.y) < 5)
                ranges.extend(cloud.z[forward & (cloud.z > 0)].tolist())
        assert ranges, "No RTX LiDAR returns facing the door"
        distance = float(np.median(ranges))
        print(f"Door RTX LiDAR: blocked={blocked}, median range={distance:.3f} m", flush=True)
        if blocked:
            assert 0.9 < distance < 1.1, distance
        else:
            assert distance > 1.5, distance
        for y in (-1.5, -4.5):
            hit = scene_query.raycast_closest((-14.0, y, 1.2), (-1.0, 0.0, 0.0), 1.5)
            assert hit["hit"] == blocked, (time, y, hit)
            if blocked:
                assert "/DynamicElevatorDoors/" in hit["collision"], hit
    print("Office elevator doors: skinning, interpolation, session isolation, "
          "closed/open/closed PhysX clearance and RTX LiDAR passed", flush=True)


if __name__ == "__main__":
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "enable_motion_bvh": True})
    status = 1
    try:
        validate(app)
        status = 0
    except (AssertionError, RuntimeError, TypeError, ValueError):
        traceback.print_exc()
        sys.stderr.flush()
    finally:
        app.close(exit_code=status)
