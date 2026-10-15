"""Animate the Office asset's skeletal doors with matching kinematic colliders."""

import math


MOVE_SECONDS = 2.0
HOLD_SECONDS = 5.0
LIFT_SECONDS = 6.0
LOWER_LEVEL = -3.3
CABIN_SOURCE = "/Root/BP_Elevator_162/SM_Elevator"
DOOR_ROOT = "/World/DynamicElevatorDoors"
GROUND_PLANE = "/Root/GroundPlane/CollisionPlane"
# Office 6.1's two shafts extend behind the ground-floor doors at x=-15.
SHAFT_HOLES = ((-17.0, -2.5, -15.0, -0.5), (-17.0, -5.5, -15.0, -3.5))


def ground_mesh(bounds, holes):
    """Triangulate a rectangular floor, excluding rectangular shaft openings."""
    xmin, ymin, xmax, ymax = bounds
    if not all(math.isfinite(v) for v in bounds) or not (xmin < xmax and ymin < ymax):
        raise ValueError("Ground bounds must have positive area")
    if not all(xmin <= x0 < x1 <= xmax and ymin <= y0 < y1 <= ymax
               for x0, y0, x1, y1 in holes):
        raise ValueError("Shaft openings must lie inside ground bounds")
    xs = sorted({xmin, xmax, *(x for hole in holes for x in (hole[0], hole[2]))})
    ys = sorted({ymin, ymax, *(y for hole in holes for y in (hole[1], hole[3]))})
    points = [(x, y, 0.0) for y in ys for x in xs]
    indices = []
    for row, (y0, y1) in enumerate(zip(ys, ys[1:])):
        for column, (x0, x1) in enumerate(zip(xs, xs[1:])):
            x, y = (x0 + x1) / 2, (y0 + y1) / 2
            if any(hx0 < x < hx1 and hy0 < y < hy1 for hx0, hy0, hx1, hy1 in holes):
                continue
            lower = row * len(xs) + column
            upper = lower + len(xs)
            indices.extend((lower, lower + 1, upper + 1, lower, upper + 1, upper))
    return points, indices


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
        self.door_levels = []
        self.cabins = []
        self.lift_height = 0.0
        self.lift_target = 0.0
        self.lift_start = 0.0
        self.lift_elapsed = 0.0
        self.lift_pending = False
        roots = [
            UsdSkel.Root(prim) for prim in stage.Traverse()
            if prim.IsA(UsdSkel.Root) and prim.GetName().startswith("SK_Elevator_doors_")
        ]
        if not roots:
            raise RuntimeError("No Office skeletal elevator doors found")
        if stage.GetPrimAtPath(DOOR_ROOT):
            raise RuntimeError(f"Dynamic door prim already exists: {DOOR_ROOT}")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            self._create_cabins()
            roots = [
                UsdSkel.Root(prim) for prim in stage.Traverse()
                if prim.IsA(UsdSkel.Root) and prim.GetName().startswith("SK_Elevator_doors_")
            ]
            for root in roots:
                self.doors.append(self._create_door(root, len(self.doors)))
            self._open_shafts()
        self._apply()
        print(f"Dynamic elevator doors: {len(self.doors)} door sets, "
              f"mode={'cycle' if cycle else 'manual'}", flush=True)

    def _create_cabins(self):
        from pxr import Gf, UsdGeom, UsdPhysics

        source = self.stage.GetPrimAtPath(CABIN_SOURCE)
        if not source:
            raise RuntimeError(f"Office elevator cabin not found: {CABIN_SOURCE}")
        cache = UsdGeom.XformCache()
        world = cache.GetLocalToWorldTransform(source)
        ground = Gf.Matrix4d(world)
        ground.SetTranslateOnly(world.ExtractTranslation() + Gf.Vec3d(0, 0, -LOWER_LEVEL))
        clone = UsdGeom.Xform.Define(self.stage, f"{DOOR_ROOT}/Cabin")
        clone.GetPrim().GetReferences().AddInternalReference(CABIN_SOURCE)
        first = Gf.Matrix4d(ground)
        first.SetTranslateOnly(ground.ExtractTranslation() + Gf.Vec3d(0, 3, 0))
        for prim, matrix in ((clone.GetPrim(), first), (source, ground)):
            parent_inverse = cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse()
            op = UsdGeom.Xformable(prim).MakeMatrixXform()
            op.Set(matrix * parent_inverse)
            body = UsdPhysics.RigidBodyAPI.Apply(prim)
            body.CreateKinematicEnabledAttr(True)
            self.cabins.append((op, matrix, parent_inverse))
        print("Elevator cabins ready at ground level; L switches between z=0 and z=-3.3 m", flush=True)

    def _open_shafts(self):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        plane = self.stage.GetPrimAtPath(GROUND_PLANE)
        mesh_prim = self.stage.GetPrimAtPath("/Root/GroundPlane/CollisionMesh")
        if not plane or not plane.HasAPI(UsdPhysics.CollisionAPI) or not mesh_prim:
            raise RuntimeError("Expected Office ground plane and collision mesh")
        world = UsdGeom.XformCache().GetLocalToWorldTransform(plane)
        if (abs(world.ExtractTranslation()[2]) > 1e-5
                or world.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()[2] < 0.999):
            raise RuntimeError("Expected the Office ground plane at world z=0")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        bounds = cache.ComputeWorldBound(mesh_prim).ComputeAlignedRange()
        lower_points, lower_counts, lower_indices = [], [], []
        transforms = UsdGeom.XformCache()
        # Cover the authored floor tiles as well as the original 50 m ground mesh.
        for prim in Usd.PrimRange.Stage(self.stage, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh) and prim.GetName().startswith("SM_Floor_"):
                floor_bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                bounds.UnionWith(floor_bounds)
                if (abs(floor_bounds.GetMin()[2] - LOWER_LEVEL) < 0.01
                        and abs(floor_bounds.GetMax()[2] - LOWER_LEVEL) < 0.01):
                    floor = UsdGeom.Mesh(prim)
                    matrix = transforms.GetLocalToWorldTransform(prim)
                    offset = len(lower_points)
                    lower_points.extend(matrix.Transform(Gf.Vec3d(p)) for p in floor.GetPointsAttr().Get())
                    lower_counts.extend(floor.GetFaceVertexCountsAttr().Get())
                    lower_indices.extend(offset + i for i in floor.GetFaceVertexIndicesAttr().Get())
        if not lower_points:
            raise RuntimeError("Office lower-floor geometry was not found")
        low, high = bounds.GetMin(), bounds.GetMax()
        points, indices = ground_mesh((low[0], low[1], high[0], high[1]), SHAFT_HOLES)
        mesh = UsdGeom.Mesh.Define(self.stage, f"{DOOR_ROOT}/GroundCollision")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(low[0], low[1], 0), Gf.Vec3f(high[0], high[1], 0)])
        mesh.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
        UsdPhysics.CollisionAPI(plane).CreateCollisionEnabledAttr(False)
        lower = UsdGeom.Mesh.Define(self.stage, f"{DOOR_ROOT}/LowerFloorCollision")
        lower.CreatePointsAttr(lower_points)
        lower.CreateFaceVertexCountsAttr(lower_counts)
        lower.CreateFaceVertexIndicesAttr(lower_indices)
        lower.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lower.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(lower.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(lower.GetPrim()).CreateApproximationAttr("none")
        print("Removed infinite ground support inside the two Office elevator shafts", flush=True)

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
        self.door_levels.append(("inside" in root.GetPrim().GetName(), minimum[2]))
        return animation.GetTranslationsAttr(), translations, offsets, panels

    def _apply(self):
        from pxr import Gf, Usd, Vt

        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for op, ground, parent_inverse in self.cabins:
                matrix = Gf.Matrix4d(ground)
                matrix.SetTranslateOnly(ground.ExtractTranslation() + Gf.Vec3d(0, 0, self.lift_height))
                op.Set(matrix * parent_inverse)
            for (attribute, closed, offsets, panels), (inside, height) in zip(self.doors, self.door_levels):
                fraction = self.fraction if inside or abs(height - self.lift_height) < 0.05 else 0.0
                attribute.Set(Vt.Vec3fArray([
                    Gf.Vec3f(Gf.Vec3d(position) + offset * fraction)
                    for position, offset in zip(closed, offsets)
                ]))
                for position, center, travel in panels:
                    position.Set(center + travel * fraction
                                 + Gf.Vec3d(0, 0, self.lift_height if inside else 0))

    @property
    def moving(self):
        """True while the doors close for a trip or the cabins travel."""
        return self.lift_pending or self.lift_height != self.lift_target

    def toggle(self):
        if not self.set_doors(not self.target_open):
            print("Elevator doors: opening blocked until the lift reaches a landing", flush=True)
            return
        print(f"Elevator doors: {'opening' if self.target_open else 'closing'} "
              "(manual mode)", flush=True)

    def toggle_lift(self):
        if self.moving:
            print("Elevator lift: already moving or closing doors; L ignored", flush=True)
            return
        self.move_to(LOWER_LEVEL if self.lift_height == 0.0 else 0.0)

    def set_doors(self, open_doors):
        """Open or close the landing doors; refused while the cabins travel."""
        if self.moving:
            return False
        self.cycle = False
        self.target_open = bool(open_doors)
        return True

    def move_to(self, height):
        """Close the doors, then move both cabins to a landing height (m)."""
        if not math.isfinite(height):
            raise ValueError("Lift height must be finite")
        if self.moving:
            return height == self.lift_target
        self.cycle = False
        if height == self.lift_height:
            return True
        self.target_open = False
        self.lift_target = height
        self.lift_start = self.lift_height
        self.lift_elapsed = 0.0
        self.lift_pending = True
        print(f"Elevator lift requested: z={self.lift_target:.1f} m; closing doors first", flush=True)
        return True

    def update(self, sim_time):
        if not math.isfinite(sim_time):
            raise ValueError("Door simulation time must be finite")
        previous = (self.fraction, self.lift_height)
        if self.started is None or sim_time < self.last_time:
            if self.started is not None:
                self.target_open = False
                self.lift_height = self.lift_target = self.lift_start = 0.0
                self.lift_elapsed = 0.0
                self.lift_pending = False
            self.started = sim_time
            self.last_time = sim_time
            self.fraction = 0.0
        dt = sim_time - self.last_time
        self.last_time = sim_time
        if self.lift_pending:
            fraction = max(0.0, self.fraction - dt / MOVE_SECONDS)
            if fraction == 0.0:
                self.lift_pending = False
        elif self.lift_height != self.lift_target:
            fraction = 0.0
            self.lift_elapsed = min(LIFT_SECONDS, self.lift_elapsed + dt)
            progress = (1.0 - math.cos(math.pi * self.lift_elapsed / LIFT_SECONDS)) / 2.0
            self.lift_height = self.lift_start + (self.lift_target - self.lift_start) * progress
            if self.lift_elapsed == LIFT_SECONDS:
                self.lift_height = self.lift_target
                print(f"Elevator lift arrived: z={self.lift_height:.1f} m", flush=True)
        elif self.cycle:
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
        self.fraction = fraction
        if previous != (self.fraction, self.lift_height):
            self._apply()
