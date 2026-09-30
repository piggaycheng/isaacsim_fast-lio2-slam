#!/home/yu/isaacsim-6.1.0/python.sh
"""Measure the tight bounding box and 2D footprint of any USD asset or prim.

Mesh vertices are transformed into a reference frame, so the box is not
inflated by the rotated local bounds of each part. Other boundable prims
(cubes, cylinders, point instancers, ...) fall back to the corners of their
local extent.

Examples:
  # Nova Carter footprint in ROS base_link (chassis_link turned 180 deg, since
  # Carter drives toward USD -x), padded by 6 cm:
  ./usd_bbox.py /Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd \
      --frame chassis_link --yaw-deg 180 --padding 0.06

  # Any local file, whole default prim in its own frame:
  ./usd_bbox.py path/to/asset.usd
"""

import argparse
import json
import math
import sys
import traceback

import numpy as np


def yaw_rotation(yaw_deg):
    """Row-vector matrix that expresses reference-frame points in a frame yawed by yaw_deg."""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    # p_out = Rz(-yaw) p_frame; with row vectors that is p_frame @ Rz(-yaw).T = p_frame @ Rz(yaw).
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def transform_points(points, matrix):
    """Apply a 4x4 USD (row-vector) transform to Nx3 points."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    result = homogeneous @ np.asarray(matrix, dtype=float)
    return result[:, :3] / result[:, 3:4]


def box_corners(minimum, maximum):
    return np.array([
        [x, y, z]
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    ], dtype=float)


def convex_hull_2d(points):
    """Counter-clockwise convex hull (Andrew's monotone chain) of Nx2 points."""
    unique = sorted(set(map(tuple, np.round(np.asarray(points, dtype=float)[:, :2], 9))))
    if len(unique) <= 2:
        return [list(p) for p in unique]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1] + upper[:-1]]


def outside_distance(polygon, points):
    """Largest distance any point lies outside a convex CCW polygon (0 if all inside)."""
    poly = np.asarray(polygon, dtype=float)
    pts = np.asarray(points, dtype=float)[:, :2]
    a = poly
    edge = np.roll(poly, -1, axis=0) - poly
    length = np.linalg.norm(edge, axis=1)
    keep = length > 0
    a, edge, length = a[keep], edge[keep], length[keep]
    # Positive = right of the edge = outside for a CCW polygon; offsetting every edge
    # by the worst violation puts all points inside.
    signed = (edge[:, 1, None] * (pts[None, :, 0] - a[:, 0, None])
              - edge[:, 0, None] * (pts[None, :, 1] - a[:, 1, None])) / length[:, None]
    return max(0.0, float(signed.max()))


def simplify_convex(polygon, tolerance):
    """Greedily drop hull vertices while every original vertex stays within tolerance outside."""
    original = np.asarray(polygon, dtype=float)
    pts = [list(p) for p in polygon]
    while len(pts) > 3:
        best = None
        for i in range(len(pts)):
            candidate = pts[:i] + pts[i + 1:]
            error = outside_distance(candidate, original)
            if error < tolerance and (best is None or error < best[0]):
                best = (error, i)
        if best is None:
            break
        pts.pop(best[1])
    return pts


def pad_polygon(polygon, padding):
    """Offset a convex CCW polygon outward by padding (vertex moved along its bisector)."""
    if padding == 0 or len(polygon) < 3:
        return [list(p) for p in polygon]
    pts = np.asarray(polygon, dtype=float)
    padded = []
    for i, p in enumerate(pts):
        prev_edge = p - pts[i - 1]
        next_edge = pts[(i + 1) % len(pts)] - p
        # Outward normals of a CCW polygon point to the right of each edge.
        n1 = np.array([prev_edge[1], -prev_edge[0]]) / np.linalg.norm(prev_edge)
        n2 = np.array([next_edge[1], -next_edge[0]]) / np.linalg.norm(next_edge)
        bisector = n1 + n2
        scale = padding / max(np.dot(bisector / np.linalg.norm(bisector), n1), 1e-6)
        padded.append(list(p + bisector / np.linalg.norm(bisector) * scale))
    return padded


def summarize(points, padding=0.0, shape="rect", hull_tolerance=0.02, decimals=3):
    """Bounding box and padded 2D footprint of Nx3 points."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        raise ValueError("No geometry found to measure")
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    if shape == "hull":
        full_hull = convex_hull_2d(points)
        hull = simplify_convex(full_hull, hull_tolerance) if hull_tolerance > 0 else full_hull
        # Grow by the simplification error so the footprint never cuts into the geometry.
        footprint = pad_polygon(hull, padding + outside_distance(hull, full_hull))
    else:
        x0, y0 = minimum[:2] - padding
        x1, y1 = maximum[:2] + padding
        footprint = [[x1, y1], [x1, y0], [x0, y0], [x0, y1]]

    def r(value):
        return round(float(value), decimals) + 0.0

    footprint = [[r(x), r(y)] for x, y in footprint]
    return {
        "min": [r(v) for v in minimum],
        "max": [r(v) for v in maximum],
        "size": [r(v) for v in maximum - minimum],
        "center": [r(v) for v in (minimum + maximum) / 2],
        "padding": r(padding),
        "footprint": footprint,
    }


def nav2_footprint(footprint):
    """Nav2 costmap `footprint` parameter string."""
    return "[" + ", ".join(f"[{x}, {y}]" for x, y in footprint) + "]"


def format_report(result):
    lines = [
        f"USD:        {result['usd']}",
        f"Prim:       {result['prim']}",
        f"Frame:      {result['frame']} (yaw {result['yaw_deg']} deg), meters",
        f"Geometry:   {result['meshes']} meshes, {result['other_prims']} other prims, "
        f"{result['points']} points",
        f"Min xyz:    {result['min']}",
        f"Max xyz:    {result['max']}",
        f"Size xyz:   {result['size']}",
        f"Center xyz: {result['center']}",
        f"Footprint ({result['shape']}, padding {result['padding']} m):",
        f"  footprint: '{nav2_footprint(result['footprint'])}'",
    ]
    return "\n".join(lines)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "usd",
        help="Local USD path or URL, or an Isaac asset path such as /Isaac/Robots/... "
        "(resolved against the Isaac assets root).",
    )
    parser.add_argument("--prim", help="Prim to measure (default: the stage default prim).")
    parser.add_argument(
        "--frame",
        help="Prim whose frame the result is expressed in (default: --prim). "
        "Relative paths are resolved under --prim, e.g. chassis_link.",
    )
    parser.add_argument(
        "--yaw-deg", type=float, default=0.0,
        help="Yaw of the output frame relative to --frame, e.g. 180 when the robot drives "
        "toward the frame's -x axis.",
    )
    parser.add_argument("--padding", type=float, default=0.0, help="Footprint padding (m).")
    parser.add_argument(
        "--shape", choices=("rect", "hull"), default="rect",
        help="Footprint shape: axis-aligned rectangle or 2D convex hull (default: rect).",
    )
    parser.add_argument(
        "--hull-tolerance", type=float, default=0.02,
        help="With --shape hull, drop hull vertices while the geometry sticks out at most this "
        "far (m); the footprint is grown by that error so it never shrinks (default: 0.02, 0 = off).",
    )
    parser.add_argument(
        "--purpose", action="append", choices=("default", "render", "proxy", "guide"),
        help="Geometry purposes to include; repeatable (default: default and render). "
        "Collision meshes are usually 'guide'.",
    )
    parser.add_argument(
        "--exclude", action="append", default=[],
        help="Skip prims whose path contains this text; repeatable.",
    )
    parser.add_argument(
        "--include-invisible", action="store_true", help="Also measure invisible prims."
    )
    parser.add_argument("--json", action="store_true", help="Print JSON instead of text.")
    args = parser.parse_args(argv)
    if args.padding < 0:
        parser.error("--padding must be non-negative")
    args.purpose = args.purpose or ["default", "render"]
    return args


def measure(args):
    """Open the stage and collect geometry points in the output frame (requires Isaac Sim)."""
    from pxr import Usd, UsdGeom

    usd_path = args.usd
    if usd_path.startswith("/Isaac/"):
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac Sim assets root was not found")
        usd_path = root + usd_path

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise FileNotFoundError(f"Cannot open USD stage: {usd_path}")
    prim = stage.GetPrimAtPath(args.prim) if args.prim else stage.GetDefaultPrim()
    if not prim or not prim.IsValid():
        raise ValueError(f"Prim not found: {args.prim or 'default prim (use --prim)'}")

    frame_path = args.frame or str(prim.GetPath())
    if not frame_path.startswith("/"):
        frame_path = f"{prim.GetPath()}/{frame_path}"
    frame = stage.GetPrimAtPath(frame_path)
    if not frame or not frame.IsValid():
        raise ValueError(f"Frame prim not found: {frame_path}")

    time = Usd.TimeCode.Default()
    purposes = list(args.purpose)
    xform_cache = UsdGeom.XformCache(time)
    bbox_cache = UsdGeom.BBoxCache(time, purposes, useExtentsHint=False)
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    world_to_frame = xform_cache.GetLocalToWorldTransform(frame).GetInverse()

    chunks, meshes, others = [], 0, 0
    iterator = iter(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
    for child in iterator:
        if any(text in str(child.GetPath()) for text in args.exclude):
            iterator.PruneChildren()
            continue
        imageable = UsdGeom.Imageable(child)
        if imageable and (
            imageable.ComputePurpose() not in purposes
            or (not args.include_invisible
                and imageable.ComputeVisibility(time) == UsdGeom.Tokens.invisible)
        ):
            iterator.PruneChildren()
            continue
        if child.IsA(UsdGeom.Mesh):
            vertices = UsdGeom.Mesh(child).GetPointsAttr().Get(time)
            if not vertices:
                continue
            local = np.array(vertices, dtype=float)
            meshes += 1
        elif child.IsA(UsdGeom.Gprim) or child.IsA(UsdGeom.PointInstancer):
            bound = bbox_cache.ComputeUntransformedBound(child).ComputeAlignedRange()
            iterator.PruneChildren()
            if bound.IsEmpty():
                continue
            local = box_corners(bound.GetMin(), bound.GetMax())
            others += 1
        else:
            continue
        to_frame = xform_cache.GetLocalToWorldTransform(child) * world_to_frame
        chunks.append(transform_points(local, np.array(to_frame, dtype=float)))

    points = np.vstack(chunks) if chunks else np.empty((0, 3))
    points = points @ yaw_rotation(args.yaw_deg) * meters
    return points, {
        "usd": usd_path,
        "prim": str(prim.GetPath()),
        "frame": frame_path,
        "meshes": meshes,
        "other_prims": others,
    }


def main(argv=None):
    args = parse_args(argv)
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})
    status = 1
    try:
        points, info = measure(args)
        result = {
            **info,
            "yaw_deg": args.yaw_deg,
            "shape": args.shape,
            "points": int(len(points)),
            **summarize(points, args.padding, args.shape, args.hull_tolerance),
        }
        result["nav2_footprint"] = nav2_footprint(result["footprint"])
        # Print before closing: SimulationApp.close() may end the process.
        print("\n" + (json.dumps(result, indent=2) if args.json else format_report(result)),
              flush=True)
        status = 0
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
    except Exception:
        # Print before close(): with fastShutdown the traceback would otherwise be lost.
        traceback.print_exc()
        sys.stderr.flush()
    finally:
        simulation_app.close(exit_code=status)
    return status


if __name__ == "__main__":
    sys.exit(main())
