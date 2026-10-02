import math

import numpy as np


def validate_polygon(points):
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("A zone needs at least three vertices")
    result = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("Each vertex must contain x and y")
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in point):
            raise ValueError("Vertices must be finite numbers")
        result.append([float(value) for value in point])
    if len({tuple(point) for point in result}) != len(result):
        raise ValueError("Duplicate vertices are not allowed")

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(a, b, c):
        return (abs(cross(a, b, c)) < 1e-9
                and min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9
                and min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9)

    size = len(result)
    area = sum(
        result[i][0] * result[(i + 1) % size][1]
        - result[(i + 1) % size][0] * result[i][1] for i in range(size)
    )
    if abs(area) < 1e-9:
        raise ValueError("Polygon area must be nonzero")
    for i in range(size):
        a, b = result[i], result[(i + 1) % size]
        for j in range(i + 1, size):
            if j == i + 1 or (i == 0 and j == size - 1):
                continue
            c, d = result[j], result[(j + 1) % size]
            crossing = (cross(a, b, c) * cross(a, b, d) < 0
                        and cross(c, d, a) * cross(c, d, b) < 0)
            touching = any((on_segment(a, b, c), on_segment(a, b, d),
                            on_segment(c, d, a), on_segment(c, d, b)))
            if crossing or touching:
                raise ValueError("Polygon edges must not intersect")
    return result


def polygon_cells(points, width, height, resolution, origin):
    points = validate_polygon(points)
    ox, oy = origin
    if any(not (ox <= x < ox + width * resolution
                and oy <= y < oy + height * resolution) for x, y in points):
        raise ValueError("Zone vertices must be inside the map")
    x0 = max(0, math.floor((min(p[0] for p in points) - ox) / resolution))
    x1 = min(width, math.ceil((max(p[0] for p in points) - ox) / resolution))
    y0 = max(0, math.floor((min(p[1] for p in points) - oy) / resolution))
    y1 = min(height, math.ceil((max(p[1] for p in points) - oy) / resolution))
    x, y = np.meshgrid(
        ox + (np.arange(x0, x1) + 0.5) * resolution,
        oy + (np.arange(y0, y1) + 0.5) * resolution,
    )
    inside = np.zeros(x.shape, dtype=bool)
    for index, (ax, ay) in enumerate(points):
        bx, by = points[(index + 1) % len(points)]
        if ay != by:
            inside ^= ((ay > y) != (by > y)) & (x < (bx - ax) * (y - ay) / (by - ay) + ax)
    rows, cols = np.nonzero(inside)
    if not len(rows):
        raise ValueError("Zone is too small to cover a map cell center")
    return rows + y0, cols + x0


def rasterize_zones(zones, width, height, resolution, origin):
    keepout = np.zeros((height, width), dtype=np.int8)
    speed = np.zeros_like(keepout)
    identifiers = set()
    for zone in zones:
        identifier = zone["id"]
        if isinstance(identifier, bool) or not isinstance(identifier, int) or identifier < 1:
            raise ValueError("Zone id must be a positive integer")
        if identifier in identifiers:
            raise ValueError("Duplicate zone id")
        identifiers.add(identifier)
        rows, cols = polygon_cells(zone["points"], width, height, resolution, origin)
        if zone["kind"] == "keepout":
            keepout[rows, cols] = 100
        elif zone["kind"] == "speed":
            percent = zone["percent"]
            if isinstance(percent, bool) or not isinstance(percent, int) or not 1 <= percent <= 100:
                raise ValueError("Speed percentage must be an integer in [1, 100]")
            previous = speed[rows, cols]
            speed[rows, cols] = np.where(previous == 0, percent, np.minimum(previous, percent))
        else:
            raise ValueError("Zone kind must be keepout or speed")
    return keepout, speed
