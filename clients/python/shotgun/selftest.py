"""Self-test for the shotgun toolkit: `python3 -m shotgun.selftest`.

Plain asserts, no test framework, no dependencies -- it runs anywhere python3
does, with nothing else running. It covers the maths that can be quietly
wrong: curvature against a circle of known radius, the two caps in the speed
profile, degenerate paths, and the exact field names of a reflex rule (a
misspelt one is a rule the server silently never fires).

Exits 0 if every check passes, 1 otherwise.
"""

import math
import sys

from . import geometry as g
from . import lanes as ln
from . import reflexes as rx
from . import speed as sp


def p(x, z, y=0.0):
    return {"x": float(x), "y": float(y), "z": float(z)}


def circle(radius, n=128, y=0.0):
    """A closed-ish arc of `n` points sampled around a circle of `radius`."""
    return [
        p(radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n), y)
        for i in range(n)
    ]


def line(length, n, y=0.0):
    """`n` evenly spaced points along +X."""
    return [p(length * i / (n - 1), 0.0, y) for i in range(n)]


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- geometry -------------------------------------------------------------


def test_dist_ignores_height():
    a, b = p(0, 0, y=0.0), p(3, 4, y=100.0)
    assert close(g.dist2(a, b), 25.0), g.dist2(a, b)
    assert close(g.dist(a, b), 5.0), g.dist(a, b)


def test_arc_lengths():
    pts = line(10.0, 6)  # 5 segments of 2 m
    assert g.arc_lengths([]) == []
    assert g.arc_lengths([p(1, 1)]) == [0.0]
    lengths = g.arc_lengths(pts)
    assert len(lengths) == len(pts)
    assert close(lengths[-1], 10.0, 1e-9), lengths
    assert close(lengths[2], 4.0, 1e-9), lengths


def test_nearest_index():
    pts = line(10.0, 6)
    assert g.nearest_index(pts, p(4.1, 3.0)) == 2
    assert g.nearest_index(pts, p(-50.0, 0.0)) == 0
    assert g.nearest_index(pts, p(500.0, 0.0)) == 5
    try:
        g.nearest_index([], p(0, 0))
    except ValueError:
        pass
    else:
        raise AssertionError("nearest_index on an empty polyline must raise")


def test_project_point_finds_the_perpendicular():
    # Two points 10 m apart; the query sits above the middle of the segment,
    # where the nearest *vertex* is 5.83 m away but the polyline is only 3 m.
    pts = [p(0, 0), p(10, 0)]
    point, index, t, offset = g.project_point(pts, p(5, 3))
    assert index == 0 and close(t, 0.5), (index, t)
    assert close(point["x"], 5.0) and close(point["z"], 0.0), point
    assert close(offset, 3.0), offset
    assert offset < g.dist(pts[g.nearest_index(pts, p(5, 3))], p(5, 3))


def test_project_point_interpolates_height():
    pts = [p(0, 0, y=0.0), p(10, 0, y=4.0)]
    point, _, _, _ = g.project_point(pts, p(2.5, 0))
    assert close(point["y"], 1.0), point


def test_project_point_clamps_past_the_ends():
    pts = [p(0, 0), p(10, 0)]
    point, index, t, offset = g.project_point(pts, p(-7, 0))
    assert index == 0 and close(t, 0.0) and close(offset, 7.0), (index, t, offset)
    point, index, t, offset = g.project_point(pts, p(17, 0))
    assert close(t, 1.0) and close(offset, 7.0), (t, offset)


def test_project_point_degenerate():
    point, index, t, offset = g.project_point([p(1, 1)], p(1, 5))
    assert close(offset, 4.0) and index == 0, (offset, index)
    # A polyline of duplicate points has no direction; it must not divide by 0.
    point, _, _, offset = g.project_point([p(2, 2), p(2, 2)], p(2, 5))
    assert close(offset, 3.0), offset
    try:
        g.project_point([], p(0, 0))
    except ValueError:
        pass
    else:
        raise AssertionError("project_point on an empty polyline must raise")


# --- curvature ------------------------------------------------------------


def test_curvature_of_a_known_circle():
    for radius in (10.0, 50.0, 200.0):
        kappa = sp.curvature(circle(radius, n=128))
        for k in kappa[1:-1]:
            # Chord sampling overestimates by (dtheta/2)/sin(dtheta/2), which
            # is under 0.1% at 128 points -- so a 1e-3 relative tolerance is a
            # real check on the formula, not a fudge.
            assert abs(k - 1.0 / radius) / (1.0 / radius) < 1e-3, (radius, k)


def test_curvature_with_uneven_sampling():
    # Real centerlines are not evenly sampled -- an importer emits short
    # segments through a junction and long ones down a straight. The turn
    # angle is shared between the two segments, so it must be divided by their
    # MEAN length; dividing by either one alone is wrong the moment they
    # differ. Alternating 0.02 and 0.06 rad steps around a 50 m circle.
    radius = 50.0
    angles, a = [0.0], 0.0
    for i in range(60):
        a += 0.02 if i % 2 else 0.06
        angles.append(a)
    pts = [p(radius * math.cos(t), radius * math.sin(t)) for t in angles]
    for k in sp.curvature(pts)[1:-1]:
        assert abs(k - 1.0 / radius) / (1.0 / radius) < 1e-3, k


def test_curvature_of_a_straight_line():
    assert all(k == 0.0 for k in sp.curvature(line(100.0, 20)))


def test_curvature_endpoints_and_degenerates():
    assert sp.curvature([]) == []
    assert sp.curvature([p(0, 0)]) == [0.0]
    assert sp.curvature([p(0, 0), p(1, 0)]) == [0.0, 0.0]
    kappa = sp.curvature(circle(10.0, n=64))
    assert kappa[0] == 0.0 and kappa[-1] == 0.0, "ends have no turn to measure"
    # Duplicate points: no direction, so no turn -- and no ZeroDivisionError.
    dup = [p(0, 0), p(1, 0), p(1, 0), p(2, 0)]
    assert sp.curvature(dup) == [0.0, 0.0, 0.0, 0.0]


def test_curvature_right_angle():
    # A 90 deg turn between two 2 m segments: pi/2 over a 2 m mean length.
    kappa = sp.curvature([p(0, 0), p(2, 0), p(2, 2)])
    assert close(kappa[1], (math.pi / 2) / 2.0, 1e-12), kappa


def test_curvature_ignores_turn_direction():
    left = sp.curvature([p(0, 0), p(2, 0), p(2, 2)])
    right = sp.curvature([p(0, 0), p(2, 0), p(2, -2)])
    assert close(left[1], right[1]), (left, right)


# --- speed profile --------------------------------------------------------


def test_profile_of_a_straight_line_is_cruise():
    speeds = sp.speed_profile(line(200.0, 50), 12.0)
    assert len(speeds) == 50
    assert all(close(s, 12.0) for s in speeds), speeds


def test_lateral_limit_binds_on_a_circle():
    radius, cruise = 40.0, 30.0
    speeds = sp.speed_profile(circle(radius, n=256), cruise, a_lat=4.9, a_brake=3.0)
    want = math.sqrt(4.9 * radius)  # v = sqrt(a_lat / kappa), kappa = 1/R
    assert want < cruise, "the test is pointless unless the corner is the limit"
    for s in speeds[1:-1]:
        assert abs(s - want) / want < 1e-3, (s, want)


def test_lateral_limit_scales_with_the_budget():
    grippy = sp.speed_profile(circle(40.0, n=256), 30.0, a_lat=9.8)
    normal = sp.speed_profile(circle(40.0, n=256), 30.0, a_lat=4.9)
    assert close(grippy[64] / normal[64], math.sqrt(2.0), 1e-3)


def test_backward_pass_produces_lead_distance():
    # 200 m straight, then a tight arc. Nothing tells the profiler where to
    # start slowing: the lead distance falls out of the backward pass.
    cruise, a_brake, radius = 20.0, 3.0, 20.0
    straight = [p(-200.0 + 2.0 * i, 0.0) for i in range(100)]  # 2 m spacing
    arc = [
        p(radius * math.sin(a * 0.05), radius - radius * math.cos(a * 0.05))
        for a in range(1, 40)
    ]
    path = straight + arc
    speeds = sp.speed_profile(path, cruise, a_lat=4.9, a_brake=a_brake)

    corner = min(speeds)
    assert corner < cruise, "the arc must be slower than cruise for this to test anything"

    # Speeds along the straight must be non-increasing: the car is already
    # slowing before it reaches the corner, not braking at the apex.
    for a, b in zip(speeds[:99], speeds[1:100]):
        assert b <= a + 1e-9, (a, b)
    assert speeds[0] > speeds[98] + 1.0, "the approach must actually shed speed"

    # Where the slow-down starts is v^2 = u^2 + 2*a*d from the corner speed.
    lengths = g.arc_lengths(path)
    braking = [i for i, s in enumerate(speeds) if s < cruise - 1e-9]
    start = braking[0]
    at_corner = speeds.index(corner)
    want_lead = (cruise ** 2 - corner ** 2) / (2.0 * a_brake)
    got_lead = lengths[at_corner] - lengths[start]
    assert abs(got_lead - want_lead) <= 2.5, (got_lead, want_lead)


def test_backward_pass_respects_the_deceleration_rate():
    # Two profiles over the same corner: half the braking rate must start
    # slowing roughly twice as far out.
    cruise = 20.0
    path = [p(2.0 * i, 0.0) for i in range(80)] + [
        p(160.0 + 20.0 * math.sin(a * 0.05), 20.0 - 20.0 * math.cos(a * 0.05))
        for a in range(1, 30)
    ]
    soft = sp.speed_profile(path, cruise, a_brake=1.5)
    hard = sp.speed_profile(path, cruise, a_brake=3.0)
    lengths = g.arc_lengths(path)

    def lead(speeds):
        start = next(i for i, s in enumerate(speeds) if s < cruise - 1e-9)
        apex = speeds.index(min(speeds))
        return lengths[apex] - lengths[start]

    assert 1.7 < lead(soft) / lead(hard) < 2.3, (lead(soft), lead(hard))


def test_profile_never_exceeds_cruise():
    speeds = sp.speed_profile(circle(500.0, n=64), 8.0)
    assert all(s <= 8.0 + 1e-9 for s in speeds), max(speeds)


def test_profile_degenerate_paths():
    assert sp.speed_profile([], 10.0) == []
    assert sp.speed_profile([p(0, 0)], 10.0) == [10.0]
    assert sp.speed_profile([p(0, 0), p(0, 0)], 10.0) == [10.0, 10.0]
    speeds = sp.speed_profile([p(0, 0), p(0, 0), p(0, 0)], 10.0)
    assert speeds == [10.0, 10.0, 10.0], speeds
    assert all(s == s for s in speeds), "no NaNs"


def test_retime_keeps_positions_and_replaces_speeds():
    pts = circle(30.0, n=64)
    flat = ln.waypoints(pts, 25.0)
    timed = sp.retime(flat, 25.0)
    assert len(timed) == len(flat)
    assert [w["position"] for w in timed] == [w["position"] for w in flat]
    # Every point but the last: the final point of an open path has no
    # successor to brake toward and no measurable turn, so it keeps cruise.
    assert max(w["speed"] for w in timed[:-1]) < 25.0, "a 30 m circle at 25 m/s is not on"
    assert timed[0]["position"] is not flat[0]["position"], "must not alias the input"


def test_profile_endpoints_keep_cruise():
    # Documented consequence of measuring curvature from a turn angle: the two
    # end points have no turn, and the last one has nothing ahead to slow for.
    speeds = sp.speed_profile(circle(30.0, n=64), 25.0)
    assert close(speeds[-1], 25.0), speeds[-1]
    assert speeds[0] < 25.0, "the first point still brakes for the corner after it"


# --- lanes ----------------------------------------------------------------


def fake_map():
    fwd = {
        "id": 7,
        "kind": "driving",
        "direction": "forward",
        "width": 3.5,
        "centerline": [p(0, 0, y=0.0), p(10, 0, y=1.0), p(20, 0, y=2.0)],
        "successors": [8],
        "predecessors": [],
        "neighbors": [9],
    }
    back = dict(fwd, id=9, direction="backward", successors=[], neighbors=[7],
                centerline=[p(20, 4), p(10, 4), p(0, 4)])
    onward = dict(fwd, id=8, successors=[], neighbors=[],
                  centerline=[p(20, 0), p(30, 0)])
    orphan = dict(fwd, id=99, successors=[], neighbors=[],
                  centerline=[p(500, 500), p(510, 500)])
    return {"lanes": [fwd, back, onward, orphan]}


def test_pick_driving_lane():
    m = fake_map()
    assert ln.pick_driving_lane(m)["id"] == 7
    assert ln.pick_driving_lane(m, "backward")["id"] == 9
    assert ln.pick_driving_lane({"lanes": []}) is None
    assert ln.pick_driving_lane(None) is None, "the arena world delivers no map"
    assert len(ln.driving_lanes(m)) == 4


def test_nearest_lane():
    m = fake_map()
    assert ln.nearest_lane(m["lanes"], p(12, 0.2))["id"] in (7, 8)
    assert ln.nearest_lane(m["lanes"], p(505, 500))["id"] == 99
    try:
        ln.nearest_lane([], p(0, 0))
    except ValueError:
        pass
    else:
        raise AssertionError("nearest_lane with no lanes must raise")


def test_reachable_lanes():
    lanes = fake_map()["lanes"]
    ids = sorted(lane["id"] for lane in ln.reachable_lanes(lanes, 7))
    assert ids == [7, 8, 9], ids
    ids = sorted(lane["id"] for lane in ln.reachable_lanes(lanes, 7, lane_changes=False))
    assert ids == [7, 8], "without lane changes the backward lane is unreachable"
    assert [lane["id"] for lane in ln.reachable_lanes(lanes, 99)] == [99]


def test_lane_points_starts_at_the_car():
    lane = ln.pick_driving_lane(fake_map())
    assert len(ln.lane_points(lane)) == 3
    ahead = ln.lane_points(lane, p(9, 1))
    assert len(ahead) == 2 and ahead[0]["x"] == 10.0, ahead
    assert ln.lane_points(lane, p(1000, 0))[0]["x"] == 20.0, "past the end: last point"


def test_lane_plan_shape():
    lane = ln.pick_driving_lane(fake_map())
    plan = ln.lane_plan(lane, p(-5, 0), 6.0)
    assert len(plan) == 3
    for wp in plan:
        assert set(wp) == {"position", "speed"}, wp
        assert set(wp["position"]) == {"x", "y", "z"}, wp
        assert isinstance(wp["speed"], float)
    assert close(plan[1]["position"]["y"], 1.0), "height must survive the plan"
    assert all(wp["speed"] == 6.0 for wp in plan)


def test_lane_plan_with_a_speed_profile():
    lane = ln.pick_driving_lane(fake_map())
    pts = ln.lane_points(lane, p(0, 0))
    plan = ln.lane_plan(lane, p(0, 0), sp.speed_profile(pts, 9.0))
    assert [wp["speed"] for wp in plan] == sp.speed_profile(pts, 9.0)


def test_waypoints_rejects_a_mismatched_profile():
    try:
        ln.waypoints([p(0, 0), p(1, 0)], [3.0])
    except ValueError:
        pass
    else:
        raise AssertionError("a short speed list must raise, not silently truncate")


# --- reflexes -------------------------------------------------------------


def test_brake_on_ttc_wire_shape():
    assert rx.brake_on_ttc(2.0) == {
        "sensor": "ground_truth",
        "measure": {"kind": "time_to_collision"},
        "operator": "less_than",
        "threshold": 2.0,
        "action": "brake",
        "priority": 10,
    }


def test_stop_on_ttc_wire_shape():
    assert rx.stop_on_ttc(1.2, sensor="radar", priority=20) == {
        "sensor": "radar",
        "measure": {"kind": "time_to_collision"},
        "operator": "less_than",
        "threshold": 1.2,
        "action": "stop_and_hold",
        "priority": 20,
    }


def test_stop_and_hold_above_wire_shape():
    assert rx.stop_and_hold_above(0.5) == {
        "sensor": "ground_truth",
        "measure": {"kind": "speed"},
        "operator": "greater_than",
        "threshold": 0.5,
        "action": "stop_and_hold",
        "priority": 10,
    }


def test_distance_to_measure_shape():
    assert rx.distance_to(p(1, 2, y=3)) == {
        "kind": "distance_to",
        "target": {"x": 1.0, "y": 3.0, "z": 2.0},
    }
    got = rx.rule("ground_truth", rx.distance_to(p(1, 2)), "less_than", 5.0, "brake")
    assert got["measure"]["kind"] == "distance_to"


def test_rule_rejects_bad_enums():
    for bad in (
        lambda: rx.rule("ground_truth", rx.ttc(), "lessThan", 1.0, "brake"),
        lambda: rx.rule("ground_truth", rx.ttc(), "less_than", 1.0, "hold"),
        lambda: rx.rule("ground_truth", "time_to_collision", "less_than", 1.0, "brake"),
        lambda: rx.rule("", rx.ttc(), "less_than", 1.0, "brake"),
    ):
        try:
            bad()
        except ValueError:
            continue
        raise AssertionError("a malformed rule must raise, not reach the server")


def test_rule_numbers_are_json_types():
    r = rx.brake_on_ttc(2, priority=True)  # ints and bools sneak in from config
    assert isinstance(r["threshold"], float) and r["threshold"] == 2.0
    assert isinstance(r["priority"], int) and not isinstance(r["priority"], bool)


def main():
    tests = sorted(
        (name, fn)
        for name, fn in globals().items()
        if name.startswith("test_") and callable(fn)
    )
    failed = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as e:
            failed.append((name, f"assertion failed: {e}"))
        except Exception as e:  # a crash is a failure too, not a stack trace
            failed.append((name, f"{type(e).__name__}: {e}"))
    for name, why in failed:
        print(f"FAIL {name}: {why}")
    print(f"{len(tests) - len(failed)}/{len(tests)} shotgun self-tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
