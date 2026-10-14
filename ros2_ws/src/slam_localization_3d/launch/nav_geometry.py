"""Shared convex geometry and body-derived Nav2 safety zones (metres, base_link)."""

import math

import yaml


def convex_hull_2d(points):
    """Counter-clockwise convex hull using Andrew's monotone chain."""
    unique = sorted({tuple(round(float(v), 9) for v in p[:2]) for p in points})
    if len(unique) <= 2:
        return [list(p) for p in unique]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for chain, vertices in ((lower, unique), (upper, reversed(unique))):
        for p in vertices:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    return [list(p) for p in lower[:-1] + upper[:-1]]


def pad_polygon(polygon, padding):
    """Offset a convex CCW polygon outward, intersecting its offset edge lines."""
    if padding == 0 or len(polygon) < 3:
        return [list(p) for p in polygon]
    padded = []
    for i, p in enumerate(polygon):
        previous, following = polygon[i - 1], polygon[(i + 1) % len(polygon)]
        dx1, dy1 = p[0] - previous[0], p[1] - previous[1]
        dx2, dy2 = following[0] - p[0], following[1] - p[1]
        length1, length2 = math.hypot(dx1, dy1), math.hypot(dx2, dy2)
        nx1, ny1 = dy1 / length1, -dx1 / length1
        nx2, ny2 = dy2 / length2, -dx2 / length2
        bx, by = nx1 + nx2, ny1 + ny2
        scale = padding / (bx * nx1 + by * ny1)
        padded.append([p[0] + bx * scale, p[1] + by * scale])
    return padded


def rectangle(front, rear, left, right):
    return [front, left, front, right, rear, right, rear, left]


def footprint_bounds(costmaps):
    """Require matching rectangular global/local footprints and return padded bounds."""
    configurations = [costmaps[name][name]["ros__parameters"]
                      for name in ("global_costmap", "local_costmap")]
    bounds = []
    for parameters in configurations:
        points = yaml.safe_load(parameters["footprint"])
        if (not isinstance(points, list) or len(points) != 4
                or any(not isinstance(p, list) or len(p) != 2 for p in points)):
            raise ValueError("Costmaps require a four-corner rectangular footprint")
        values = [float(value) for point in points for value in point]
        padding = float(parameters["footprint_padding"])
        if not all(math.isfinite(v) for v in (*values, padding)) or padding < 0:
            raise ValueError("Footprint coordinates and nonnegative padding must be finite")
        xs, ys = values[::2], values[1::2]
        front, rear, left, right = max(xs), min(xs), max(ys), min(ys)
        expected = {(front, left), (front, right), (rear, right), (rear, left)}
        if (set(map(tuple, points)) != expected
                or any(a[0] != b[0] and a[1] != b[1]
                       for a, b in zip(points, points[1:] + points[:1]))
                or not rear < 0 < front or not right < 0 < left):
            raise ValueError("Rectangular footprint must enclose the base_link rotation centre")
        bounds.append((front + padding, rear - padding, left + padding, right - padding))
    if bounds[0] != bounds[1]:
        raise ValueError("Global and local padded footprints must match")
    return bounds[0]


def rotation_radius(costmaps):
    front, rear, left, right = footprint_bounds(costmaps)
    return max(math.hypot(x, y) for x in (front, rear) for y in (left, right))


def safety_geometry(costmaps, speed, settings, slowdown_ratio):
    """Conservative stopping estimate and expanded convex envelope of all stop zones."""
    reaction = float(settings["reaction_time"])
    deceleration = float(settings["effective_deceleration"])
    margin = float(settings["distance_margin"])
    slow_padding = float(settings["slow_padding"])
    if (not all(math.isfinite(v) and v > 0 for v in (speed, reaction, deceleration, margin, slow_padding))
            or not math.isfinite(slowdown_ratio) or not 0 < slowdown_ratio < 1):
        raise ValueError("Safety distance inputs must be finite, positive; slowdown_ratio must be in (0, 1)")
    stop_distance = math.ceil((speed * reaction + speed**2 / (2 * deceleration) + margin) * 100) / 100
    front, rear, left, right = footprint_bounds(costmaps)
    forward = rectangle(front + stop_distance, rear, left, right)
    reverse = rectangle(front, rear - stop_distance, left, right)
    radius = rotation_radius(costmaps)
    # Circumscribed 32-gon never cuts inside the true rotation circle.
    vertex_radius = radius / math.cos(math.pi / 32)
    points = [[vertex_radius * math.cos(2 * math.pi * i / 32),
               vertex_radius * math.sin(2 * math.pi * i / 32)] for i in range(32)]
    points += [list(pair) for zone in (forward, reverse)
               for pair in zip(zone[::2], zone[1::2])]
    slow = pad_polygon(convex_hull_2d(points), slow_padding)
    return {"forward": forward, "reverse": reverse, "radius": radius,
            "slow": [value for point in reversed(slow) for value in point],
            "stop_distance": stop_distance, "slow_padding": slow_padding}
