"""Reading the map delivered at join, and laying a plan down a lane.

The road is a known prior: the server hands the whole lane graph in the
`joined` message, perfect and free. These functions turn that JSON into a
lane to drive and a list of waypoints to submit. Nothing here talks to the
server -- the caller still sends `submit_plan` itself.

The map's shape is `shotgun.wire.MapData`, mirroring
`crates/protocol/src/map.rs`.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from typing import Literal, Optional

from .geometry import dist2, nearest_index
from .wire import LaneData, MapData, SpeedSpec, Vec3, Waypoint


def driving_lanes(map_data: Optional[MapData]) -> list[LaneData]:
    """Every drivable lane in the delivered map.

    `[]` if there is no map -- the arena world delivers none.
    """
    if not map_data:
        return []
    return [lane for lane in map_data.get("lanes", []) if lane["kind"] == "driving"]


def pick_driving_lane(
    map_data: Optional[MapData],
    direction: Literal["forward", "backward"] = "forward",
) -> Optional[LaneData]:
    """The first drivable lane running `direction`, or None.

    Enough for the single-road maps; on a real town use `nearest_lane` to find
    the one the car actually spawned on.
    """
    for lane in driving_lanes(map_data):
        if lane["direction"] == direction:
            return lane
    return None


def nearest_lane(lanes: Sequence[LaneData], p: Vec3) -> LaneData:
    """The lane with the centerline *vertex* closest to point `p`.

    Vertex, not projected point: on a sparsely sampled centerline that can
    differ, so use `project_point` if you need the true perpendicular
    distance. Raises ValueError if `lanes` is empty.
    """
    if not lanes:
        raise ValueError("nearest_lane: no lanes")
    return min(lanes, key=lambda lane: min(dist2(c, p) for c in lane["centerline"]))


def reachable_lanes(
    lanes: Sequence[LaneData],
    start_id: int,
    lane_changes: bool = True,
) -> list[LaneData]:
    """Every lane reachable from `start_id` by driving.

    A breadth-first walk of the delivered graph over `successors`, plus
    `neighbors` when `lane_changes` is set. Includes the start lane, first,
    and the rest in breadth-first order -- so picking one is reproducible run
    to run. Use it to check a destination is actually drivable to before
    asking the server to route -- the router will refuse an unreachable one.
    """
    by_id = {lane["id"]: lane for lane in lanes}
    seen = {start_id}
    found: list[LaneData] = []
    queue = deque([start_id])
    while queue:
        lane = by_id.get(queue.popleft())
        if lane is None:
            continue
        found.append(lane)
        edges = lane["successors"] + (lane["neighbors"] if lane_changes else [])
        for nxt in edges:
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return found


def lane_points(lane: LaneData, start: Optional[Vec3] = None) -> list[Vec3]:
    """The lane's centerline points from the one nearest `start` to the end.

    Dropping the points behind the car keeps it driving forward instead of
    turning back to the lane's origin. All points if `start` is None.
    """
    center = lane["centerline"]
    if start is None or not center:
        return list(center)
    return list(center[nearest_index(center, start):])


def waypoints(points: Sequence[Vec3], speed: SpeedSpec) -> list[Waypoint]:
    """Plan waypoints from points, with a speed on each.

    `speed` is one number for the whole path, or a sequence of per-point
    speeds (from `shotgun.speed.speed_profile`) the same length as `points`.
    """
    if isinstance(speed, (int, float)):
        speeds = [float(speed)] * len(points)
    else:
        speeds = [float(s) for s in speed]
        if len(speeds) != len(points):
            raise ValueError(
                f"waypoints: {len(points)} points but {len(speeds)} speeds"
            )
    return [
        {
            "position": {"x": p["x"], "y": p.get("y", 0.0), "z": p["z"]},
            "speed": s,
        }
        for p, s in zip(points, speeds)
    ]


def lane_plan(
    lane: LaneData,
    start: Optional[Vec3],
    speed: SpeedSpec,
) -> list[Waypoint]:
    """A plan down `lane` from the point nearest `start` to the lane's end.

    `speed` is a number or a per-point sequence, as in `waypoints`.
    """
    return waypoints(lane_points(lane, start), speed)
