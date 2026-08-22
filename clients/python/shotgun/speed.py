"""Speed profiling: how fast to take a path, corner by corner.

A plan is a list of positions with a speed on each. Stamping one cruise speed
on every waypoint asks the car to take a hairpin as fast as the straight, and
the tyres decide the rest. These functions set a speed per point instead --
slow enough for the corner, and slowing *before* it rather than at it.

Works on any list of points, so it profiles a lane centerline and a
server-returned route alike.
"""

import math

from .geometry import dist

# Lateral acceleration budget, m/s^2. ~0.5 g: half of what a mu=1.0 tyre on
# dry tarmac can hold, leaving the other half for braking, camber and bumps.
A_LAT = 4.9

# Braking deceleration used when planning the slow-down, m/s^2. ~0.3 g --
# comfortably inside the grip the cruise ceiling above leaves spare.
A_BRAKE = 3.0

# Curvature below this counts as straight, 1/m. 1e-6 is a 1000 km radius:
# straighter than any road, and only here to keep the division below finite.
KAPPA_EPS = 1e-6


def curvature(points):
    """Discrete curvature at each point, in 1/m -- one entry per point, so it
    lines up with `points` and with `speed_profile`.

    At an interior point it is the turn angle between the incoming and
    outgoing segments divided by their mean length: for a polyline sampled
    around a circle of radius R this gives ~1/R. The two end points have no
    turn to measure and are 0.0, as are duplicate points.
    """
    n = len(points)
    out = [0.0] * n
    for i in range(1, n - 1):
        a, b, c = points[i - 1], points[i], points[i + 1]
        inx, inz = b["x"] - a["x"], b["z"] - a["z"]
        outx, outz = c["x"] - b["x"], c["z"] - b["z"]
        len_in = math.hypot(inx, inz)
        len_out = math.hypot(outx, outz)
        mean_ds = (len_in + len_out) / 2.0
        if len_in <= 0.0 or len_out <= 0.0 or mean_ds <= 0.0:
            continue  # duplicate points: no direction, so no turn
        cross = inx * outz - inz * outx
        dot = inx * outx + inz * outz
        turn = abs(math.atan2(cross, dot))
        out[i] = turn / mean_ds
    return out


def speed_profile(points, cruise, a_lat=A_LAT, a_brake=A_BRAKE):
    """A speed for every point in `points`, capped by cruise, by cornering
    grip, and by how early braking has to start.

    Three passes:

    1. every point starts at `cruise`;
    2. each point is capped at `sqrt(a_lat / kappa)` -- the fastest a corner
       of that curvature can be held within the lateral budget;
    3. a backward pass caps each point at `sqrt(v_next^2 + 2 * a_brake * ds)`,
       so the slow-down for a corner spreads back up the path ahead of it.
       The lead distance is not a tuned number; it falls out of this.

    `a_lat` and `a_brake` are assumptions about a typical road car, not
    measurements: an agent cannot see its vehicle's mass, tyres or grip, and
    the server never tells it. On ice or in a truck they are optimistic. Pass
    your own if you know better.
    """
    n = len(points)
    if n == 0:
        return []
    speeds = [float(cruise)] * n

    kappa = curvature(points)
    for i, k in enumerate(kappa):
        if k > KAPPA_EPS:
            speeds[i] = min(speeds[i], math.sqrt(a_lat / k))

    for i in range(n - 2, -1, -1):
        ds = dist(points[i], points[i + 1])
        reachable = math.sqrt(speeds[i + 1] ** 2 + 2.0 * a_brake * ds)
        speeds[i] = min(speeds[i], reachable)

    return speeds


def retime(plan, cruise, a_lat=A_LAT, a_brake=A_BRAKE):
    """Re-speed an existing plan: same positions, speeds from
    `speed_profile`. The server's router stamps one flat speed on every
    waypoint of a route -- this is how an agent takes that route and drives it
    at a sane speed for each corner."""
    points = [wp["position"] for wp in plan]
    speeds = speed_profile(points, cruise, a_lat=a_lat, a_brake=a_brake)
    return [
        {"position": dict(wp["position"]), "speed": s} for wp, s in zip(plan, speeds)
    ]
