"""shotgun -- a co-driver's toolkit for swarm agents.

Pure functions for the parts of driving an agent has to work out for itself:
where a point is on a lane, which lane to take, how fast to take it, and what
the safety reflexes should say. No sockets, no async, no state -- the agent
keeps its own connection, join and loop, and calls these on the data it gets.

    from shotgun import lane_plan, pick_driving_lane, speed_profile

    lane = pick_driving_lane(joined["map"])
    plan = lane_plan(lane, joined["position"], 6.0)
    await ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))

Positions are the wire's Vec3 dicts, `{"x": .., "y": .., "z": ..}`, Y-up.
Distances are horizontal (X/Z) and in metres; speeds in m/s.

`sys.path[0]` is the running script's directory, so a client in
`clients/python/` imports this with no install step. Run the self-test with
`python3 -m shotgun.selftest`.
"""

from .geometry import arc_lengths, dist, dist2, nearest_index, project_point
from .lanes import (
    driving_lanes,
    lane_plan,
    lane_points,
    nearest_lane,
    pick_driving_lane,
    reachable_lanes,
    waypoints,
)
from .reflexes import (
    GROUND_TRUTH,
    brake_on_ttc,
    distance_to,
    rule,
    stop_and_hold_above,
    stop_on_ttc,
    ttc,
)
from .speed import A_BRAKE, A_LAT, curvature, retime, speed_profile

__all__ = [
    # geometry
    "arc_lengths",
    "dist",
    "dist2",
    "nearest_index",
    "project_point",
    # lanes
    "driving_lanes",
    "lane_plan",
    "lane_points",
    "nearest_lane",
    "pick_driving_lane",
    "reachable_lanes",
    "waypoints",
    # speed
    "A_BRAKE",
    "A_LAT",
    "curvature",
    "retime",
    "speed_profile",
    # reflexes
    "GROUND_TRUTH",
    "brake_on_ttc",
    "distance_to",
    "rule",
    "stop_and_hold_above",
    "stop_on_ttc",
    "ttc",
]
