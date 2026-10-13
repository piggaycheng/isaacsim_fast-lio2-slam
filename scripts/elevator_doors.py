"""Animate the Office asset's skeletal doors with matching kinematic colliders."""

import math


MOVE_SECONDS = 2.0
HOLD_SECONDS = 5.0
DOOR_ROOT = "/World/DynamicElevatorDoors"


class ElevatorDoors:
    def __init__(self, stage, cycle=True):
        from pxr import Usd, UsdSkel

        self.stage = stage
        self.cycle = cycle
        self.fraction = 0.0
        self.target_open = False
        self.started = None
        self.last_time = None
        self.doors = []
        roots = [
            UsdSkel.Root(prim) for prim in stage.Traverse()
            if prim.IsA(UsdSkel.Root) and prim.GetName().startswith("SK_Elevator_doors_")
        ]
        if not roots:
            raise RuntimeError("No Office skeletal elevator doors found")
        if stage.GetPrimAtPath(DOOR_ROOT):
            raise RuntimeError(f"Dynamic door prim already exists: {DOOR_ROOT}")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for root in roots:
                self.doors.append(self._create_door(root, len(self.doors)))
        self._apply()
        print(f"Dynamic elevator doors: {len(self.doors)} door sets, "
              f"mode={'cycle' if cycle else 'manual'}", flush=True)

    def _create_door(self, root, index):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt

        cache = UsdSkel.Cache()
        cache.Populate(root, Usd.PrimDefaultPredicate)
        meshes = [UsdGeom.Mesh(p) for p in Usd.PrimRange(root.GetPrim()) if p.IsA(UsdGeom.Mesh)]
        skeletons = [
            UsdSkel.Skeleton(p) for p in Usd.PrimRange(root.GetPrim())
            if p.IsA(UsdSkel.Skeleton)
        ]
        if len(meshes) != 1 or len(skeletons) != 1:
            raise RuntimeError(f"Expected one door mesh and skeleton at {root.GetPath()}")
        mesh, skeleton = meshes[0], skeletons[0]
        skin = cache.GetSkinningQuery(mesh.GetPrim())
        query = cache.GetSkelQuery(skeleton)
        if not skin or not query:
            raise RuntimeError(f"Missing skinning data at {root.GetPath()}")
        joints = list(skeleton.GetJointsAttr().Get())
        rest = skeleton.GetRestTransformsAttr().Get()
        joint_transforms = query.ComputeJointSkelTransforms(Usd.TimeCode.Default(), True)
        bind_transforms = skeleton.GetBindTransformsAttr().Get()
        points = Vt.Vec3fArray(mesh.GetPointsAttr().Get())
        if not skin.ComputeSkinnedPoints(
            Vt.Matrix4dArray([
                bind.GetInverse() * joint
                for bind, joint in zip(bind_transforms, joint_transforms)
            ]),
            points, Usd.TimeCode.Default(),
        ):
            raise RuntimeError(f"Could not skin door mesh at {root.GetPath()}")
        world = UsdGeom.XformCache().GetLocalToWorldTransform(skeleton.GetPrim())
        world_points = [world.Transform(Gf.Vec3d(point)) for point in points]
        joint_indices, weights = skin.ComputeVaryingJointInfluences(len(points))
        influence_count = skin.GetNumInfluencesPerComponent()
        order = list(skin.GetJointOrder() or joints)
        panel_points = {}
        for vertex, point in enumerate(world_points):
            influences = [
                joint_indices[vertex * influence_count + slot]
                for slot in range(influence_count)
                if weights[vertex * influence_count + slot] > 1e-6
            ]
            if len(influences) != 1:
                raise RuntimeError(f"Door vertices must be rigidly weighted: {mesh.GetPath()}")
            joint = joints.index(order[influences[0]])
            panel_points.setdefault(joint, []).append(point)
        if len(panel_points) != 4:
            raise RuntimeError(f"Expected four telescoping door panels at {root.GetPath()}")
        minimum = [min(p[axis] for p in world_points) for axis in range(3)]
        maximum = [max(p[axis] for p in world_points) for axis in range(3)]
        axis = max((0, 1), key=lambda a: maximum[a] - minimum[a])
        if maximum[2] - minimum[2] < 1.0 or maximum[axis] - minimum[axis] < 0.5:
            raise RuntimeError(f"Unexpected Office door dimensions at {root.GetPath()}")
        middle = (minimum[axis] + maximum[axis]) / 2
        displacements = [Gf.Vec3d(0) for _ in joints]
        panels = []
        for joint, vertices in sorted(panel_points.items()):
            low = Gf.Vec3d(*(min(p[a] for p in vertices) for a in range(3)))
            high = Gf.Vec3d(*(max(p[a] for p in vertices) for a in range(3)))
            center = (low + high) / 2
            travel = Gf.Vec3d(0)
            travel[axis] = (
                minimum[axis] - high[axis] - 0.01 if center[axis] < middle
                else maximum[axis] - low[axis] + 0.01
            )
            displacements[joint] = world.GetInverse().TransformDir(travel)
            cube = UsdGeom.Cube.Define(self.stage, f"{DOOR_ROOT}/Door_{index}/Panel_{joint}")
            cube.CreateSizeAttr(1.0)
            position = cube.AddTranslateOp()
            position.Set(center)
            cube.AddScaleOp().Set(Gf.Vec3d(*(max(high[a] - low[a], 0.025) for a in range(3))))
            cube.MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            body.CreateKinematicEnabledAttr(True)
            panels.append((position, center, travel))
        parents = query.GetTopology().GetParentIndices()
        offsets = []
        for joint, parent in enumerate(parents):
            if parent < 0:
                offsets.append(displacements[joint])
            else:
                # Local translations are expressed in the parent joint's rotated frame.
                offsets.append(joint_transforms[parent].GetInverse().TransformDir(
                    displacements[joint] - displacements[parent],
                ))
        translations, rotations, scales = UsdSkel.DecomposeTransforms(rest)
        animation = UsdSkel.Animation.Define(
            self.stage, f"{DOOR_ROOT}/Door_{index}/Animation",
        )
        animation.CreateJointsAttr(joints)
        animation.CreateRotationsAttr(rotations)
        animation.CreateScalesAttr(scales)
        animation.CreateTranslationsAttr(translations)
        UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets(
            [animation.GetPath()],
        )
        print(f"  Elevator door: {root.GetPath()}", flush=True)
        return animation.GetTranslationsAttr(), translations, offsets, panels

    def _apply(self):
        from pxr import Gf, Usd, Vt

        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for attribute, closed, offsets, panels in self.doors:
                attribute.Set(Vt.Vec3fArray([
                    Gf.Vec3f(Gf.Vec3d(position) + offset * self.fraction)
                    for position, offset in zip(closed, offsets)
                ]))
                for position, center, travel in panels:
                    position.Set(center + travel * self.fraction)

    def toggle(self):
        self.cycle = False
        self.target_open = not self.target_open
        print(f"Elevator doors: {'opening' if self.target_open else 'closing'} "
              "(manual mode)", flush=True)

    def update(self, sim_time):
        if not math.isfinite(sim_time):
            raise ValueError("Door simulation time must be finite")
        if self.started is None or sim_time < self.last_time:
            if self.started is not None:
                self.target_open = False
            self.started = sim_time
            self.last_time = sim_time
            if self.fraction != 0.0:
                self.fraction = 0.0
                self._apply()
        dt = sim_time - self.last_time
        self.last_time = sim_time
        if self.cycle:
            phase = (sim_time - self.started) % (2 * (HOLD_SECONDS + MOVE_SECONDS))
            if phase < HOLD_SECONDS:
                fraction = 0.0
                self.target_open = False
            elif phase < HOLD_SECONDS + MOVE_SECONDS:
                fraction = (phase - HOLD_SECONDS) / MOVE_SECONDS
                self.target_open = True
            elif phase < 2 * HOLD_SECONDS + MOVE_SECONDS:
                fraction = 1.0
                self.target_open = True
            else:
                fraction = 1.0 - (phase - 2 * HOLD_SECONDS - MOVE_SECONDS) / MOVE_SECONDS
                self.target_open = False
        else:
            delta = dt / MOVE_SECONDS
            fraction = min(1.0, self.fraction + delta) if self.target_open else max(
                0.0, self.fraction - delta,
            )
        if fraction != self.fraction:
            self.fraction = fraction
            self._apply()
