"""Point and polyline maths on the ground plane.

The sim is Y-up: Y is height, X and Z are the ground. Every distance here is
horizontal, ignoring Y, because the road is graded -- a car 4 m up a hill is
not 4 m off its lane.

A point is any mapping with "x"/"y"/"z" keys: the wire's Vec3 shape. "y" is
optional and defaults to 0.0, so plain XZ dicts work too.
"""

import math


def dist2(a, b):
    """Squared horizontal distance between two points. Squared because most
    callers only compare distances, and a square root there is wasted."""
    return (a["x"] - b["x"]) ** 2 + (a["z"] - b["z"]) ** 2


def dist(a, b):
    """Horizontal distance between two points, in metres."""
    return math.sqrt(dist2(a, b))


def arc_lengths(points):
    """Cumulative distance along a polyline, one entry per point (first is
    0.0). Lets a caller address a position by metres travelled instead of by
    point index. Returns [] for no points."""
    if not points:
        return []
    out = [0.0]
    for a, b in zip(points, points[1:]):
        out.append(out[-1] + dist(a, b))
    return out


def nearest_index(points, p):
    """Index of the polyline point closest to `p`. Raises ValueError if the
    polyline is empty."""
    if not points:
        raise ValueError("nearest_index: empty polyline")
    return min(range(len(points)), key=lambda i: dist2(points[i], p))


def project_point(points, p):
    """Project `p` onto a polyline. Returns `(point, index, t, offset)`:

    * `point`  -- the closest point *on* the polyline (interpolated, with Y
                  interpolated too, so it sits on the graded surface)
    * `index`  -- the segment it lies on, points[index] -> points[index + 1]
    * `t`      -- how far along that segment, 0.0 to 1.0
    * `offset` -- horizontal distance from `p` to it, in metres

    Unlike `nearest_index` this looks between the points, so the offset is a
    true perpendicular distance rather than a distance to the nearest vertex.
    Raises ValueError if the polyline is empty. A one-point polyline has no
    segment: it returns `index` 0 and `t` 0.0, so don't index `points[index+1]`
    without checking the length.
    """
    if not points:
        raise ValueError("project_point: empty polyline")
    if len(points) == 1:
        only = points[0]
        return _copy(only), 0, 0.0, dist(only, p)

    best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        ax, az = a["x"], a["z"]
        dx, dz = b["x"] - ax, b["z"] - az
        seg2 = dx * dx + dz * dz
        if seg2 <= 0.0:  # duplicate points: the segment is a single location
            t = 0.0
        else:
            t = ((p["x"] - ax) * dx + (p["z"] - az) * dz) / seg2
            t = min(1.0, max(0.0, t))
        on = _lerp(a, b, t)
        d2 = dist2(on, p)
        if best is None or d2 < best[0]:
            best = (d2, on, i, t)
    d2, on, i, t = best
    return on, i, t, math.sqrt(d2)


def _lerp(a, b, t):
    return {
        "x": a["x"] + (b["x"] - a["x"]) * t,
        "y": a.get("y", 0.0) + (b.get("y", 0.0) - a.get("y", 0.0)) * t,
        "z": a["z"] + (b["z"] - a["z"]) * t,
    }


def _copy(p):
    return {"x": p["x"], "y": p.get("y", 0.0), "z": p["z"]}
