"""The JSON shapes the server sends and accepts, as `TypedDict`s.

These mirror `crates/protocol/src/map.rs` and `messages.rs` field for field.
They are annotations, not classes to instantiate: the values are plain dicts
straight off `json.loads`, and nothing here validates or converts. Their job is
to make a type checker catch `lane["succesors"]` before the sim does, and to
put the wire contract in one readable place.

Keep them in step with the Rust. A field renamed there and not here becomes a
type checker that is confidently wrong, which is worse than no annotation.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Literal, TypedDict, Union


class _GroundPoint(TypedDict):
    """The keys every point must have.

    Not used directly -- see `Vec3`.
    """

    x: float
    z: float


class Vec3(_GroundPoint, total=False):
    """A wire position, Y-up, in metres.

    `y` is declared optional because every function here reads it with
    `.get("y", 0.0)`: the server always sends all three, but a hand-built plain
    XZ dict works too. Everything shotgun *returns* has all three.
    """

    y: float


class Waypoint(TypedDict):
    """One entry in a plan: where to be, and how fast to be going there."""

    position: Vec3
    speed: float


class LaneData(TypedDict):
    """One lane of the map delivered at join.

    `kind` is deliberately `str`, not a `Literal`: `protocol::LaneKind` has only
    `driving` today but names shoulder and sidewalk as arriving with the
    importer, and `driving_lanes` exists to filter exactly those out.
    """

    id: int
    kind: str
    direction: Literal["forward", "backward"]
    width: float
    centerline: list[Vec3]
    successors: list[int]
    predecessors: list[int]
    neighbors: list[int]


class MapData(TypedDict):
    """The road prior in `joined["map"]`.

    Absent entirely in the arena world.
    """

    lanes: list[LaneData]


class TimeToCollisionMeasure(TypedDict):
    """Seconds until collision with the nearest closing obstacle."""

    kind: Literal["time_to_collision"]


class SpeedMeasure(TypedDict):
    """The entity's own speed, m/s."""

    kind: Literal["speed"]


class DistanceToMeasure(TypedDict):
    """Distance from the entity to a fixed world point, in metres."""

    kind: Literal["distance_to"]
    target: Vec3


#: What a reflex rule reads. Mirrors `protocol::SensorKind`, an internally
#: tagged enum, so the `kind` value discriminates the three shapes.
SensorKind = Union[TimeToCollisionMeasure, SpeedMeasure, DistanceToMeasure]

#: Comparison a rule applies to its reading. Mirrors `protocol::Operator`.
Operator = Literal["less_than", "greater_than"]

#: What a triggered rule does. Mirrors `protocol::ReflexAction`.
ReflexAction = Literal["brake", "stop_and_hold"]

#: A number for the whole path, or one per point.
SpeedSpec = Union[float, Sequence[float]]


class ReflexRule(TypedDict):
    """One registered reflex rule, as `register_reflexes` expects it."""

    sensor: str
    measure: SensorKind
    operator: Operator
    threshold: float
    action: ReflexAction
    priority: int
