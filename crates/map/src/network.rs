use glam::Vec3;

use crate::geometry::{Polyline, Projection};

/// An opaque lane identifier. **Not** a vector index into `RoadNetwork.lanes` --
/// an importer may assign arbitrary ids (e.g. from OpenDRIVE lane keys), so
/// look lanes up with [`RoadNetwork::lane`], never by position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LaneId(pub usize);

/// What a lane is for. Only driving lanes exist today; shoulders, sidewalks,
/// etc. slot in here as the importer grows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneKind {
    Driving,
}

/// Travel direction of a lane relative to its geometry's start→end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
}

/// One lane: a drivable strip described by its centerline and width. An agent
/// lays a path down `center`; the vehicle drives it.
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    pub id: LaneId,
    pub kind: LaneKind,
    pub direction: Direction,
    /// Lane centerline, Y-up, meters.
    pub center: Polyline,
    /// Constant lane width (per-vertex widths can come later).
    pub width: f32,
    /// Per-centerline-vertex superelevation angle (radians, signed), parallel to
    /// `center.points()`. Positive raises the **+offset** edge -- the left-hand
    /// normal of the centerline's *stored* tangent (its geometry direction), which
    /// for a `Backward` lane is opposite its travel direction. Consumers deriving
    /// a surface normal must roll about `center.tangents()`, not travel, or a
    /// backward lane's normal disagrees with its own (correct) baked heights.
    /// Empty means a flat lane (bank ≡ 0); any non-empty profile must have exactly
    /// `center.points().len()` entries. The centerline points already carry the
    /// banked *height* (reference-line pivot); this angle is the surface tilt the
    /// mesh cant and the FMU conform read.
    pub bank: Vec<f32>,
    /// Lanes reachable by driving off this lane's exit (travel-direction) end.
    /// May fan out (a junction) or be empty (a dead end / unlinked lane). Built
    /// by an importer from road/lane links and junctions; empty otherwise.
    pub successors: Vec<LaneId>,
    /// Lanes that drive into this lane -- the reverse of `successors`.
    pub predecessors: Vec<LaneId>,
    /// Adjacent same-section, same-direction lanes you can change into (lateral
    /// lane-change edges). Empty if there's no neighbor to change to.
    pub neighbors: Vec<LaneId>,
}

/// The "compiled map": everything a consumer needs, baked and format-agnostic.
/// A flat list of lanes for now; road grouping and a routing graph arrive with
/// the OpenDRIVE importer, when there's real structure to represent.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RoadNetwork {
    pub lanes: Vec<Lane>,
}

impl Lane {
    /// Superelevation angle (radians, signed) at arc length `s`, interpolated
    /// between vertices; zero everywhere on a flat lane. Positive raises the
    /// left edge -- see [`Lane::bank`].
    pub fn bank_at(&self, s: f32) -> f32 {
        if self.bank.is_empty() {
            return 0.0;
        }
        debug_assert_eq!(
            self.bank.len(),
            self.center.points().len(),
            "a non-empty bank profile must be parallel to the centerline"
        );
        let (i, t) = self.center.locate(s);
        self.bank[i] + (self.bank[i + 1] - self.bank[i]) * t
    }
}

impl RoadNetwork {
    /// The lane with this id, by identity (not position), so ids stay valid
    /// however an importer assigns them.
    ///
    /// Importers hand out ids sequentially, so the id is almost always its own
    /// index -- try that first and verify, falling back to a scan when it is
    /// not. The fallback keeps the by-identity contract; the fast path keeps
    /// the router off an O(lanes) probe per Dijkstra pop. That probe costs
    /// about a third of a route across Town07 (673 lanes) today, which is
    /// tolerable -- but `route` is answered on the sim thread, so the cost
    /// lands on every agent's tick, and it grows with the square of the map.
    pub fn lane(&self, id: LaneId) -> Option<&Lane> {
        match self.lanes.get(id.0) {
            Some(lane) if lane.id == id => Some(lane),
            _ => self.lanes.iter().find(|l| l.id == id),
        }
    }

    pub fn driving_lanes(&self) -> impl Iterator<Item = &Lane> {
        self.lanes.iter().filter(|l| l.kind == LaneKind::Driving)
    }

    /// The lowest point of any lane centerline (Y-up, metres) -- how far down
    /// the road legitimately reaches. `None` if the network has no lanes. Used
    /// to set an off-map fall floor relative to the terrain, so a map that dips
    /// well below zero (a valley, an underpass) isn't mistaken for freefall.
    pub fn min_elevation(&self) -> Option<f32> {
        self.lanes
            .iter()
            .flat_map(|l| l.center.points())
            .map(|p| p.y)
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// The lanes reachable by driving off `id`'s exit end (its `successors`).
    pub fn successors(&self, id: LaneId) -> impl Iterator<Item = &Lane> {
        self.lane(id)
            .into_iter()
            .flat_map(|l| l.successors.iter())
            .filter_map(|s| self.lane(*s))
    }

    /// The driving lane whose centerline is nearest `point`, with the
    /// projection onto it -- the lane an agent/vehicle is in, and its
    /// lane-keeping error.
    pub fn nearest_lane(&self, point: Vec3) -> Option<(LaneId, Projection)> {
        self.driving_lanes()
            .map(|lane| (lane.id, lane.center.project(point)))
            .min_by(|(_, a), (_, b)| {
                let da = (point - a.point).length_squared();
                let db = (point - b.point).length_squared();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lane(id: usize, points: &[[f32; 3]]) -> Lane {
        Lane {
            id: LaneId(id),
            kind: LaneKind::Driving,
            direction: Direction::Forward,
            center: Polyline::new(points.iter().map(|p| Vec3::from_array(*p)).collect()),
            width: 3.5,
            bank: Vec::new(),
            successors: Vec::new(),
            predecessors: Vec::new(),
            neighbors: Vec::new(),
        }
    }

    #[test]
    fn nearest_lane_picks_the_closer_centerline() {
        let net = RoadNetwork {
            lanes: vec![
                lane(0, &[[0.0, 0.0, 2.0], [10.0, 0.0, 2.0]]),
                lane(1, &[[0.0, 0.0, -2.0], [10.0, 0.0, -2.0]]),
            ],
        };
        let (id, proj) = net.nearest_lane(Vec3::new(5.0, 0.0, 1.5)).expect("a lane");
        assert_eq!(id, LaneId(0));
        assert!((proj.point - Vec3::new(5.0, 0.0, 2.0)).length() < 1e-4);
    }

    #[test]
    fn nearest_lane_is_none_when_empty() {
        assert!(RoadNetwork::default().nearest_lane(Vec3::ZERO).is_none());
    }

    #[test]
    fn min_elevation_is_the_lowest_centerline_point() {
        // A network that dips to y=-40 (a deep valley) reports -40, not 0.
        let net = RoadNetwork {
            lanes: vec![
                lane(0, &[[0.0, 5.0, 0.0], [10.0, 2.0, 0.0]]),
                lane(1, &[[0.0, -40.0, 0.0], [10.0, -12.0, 0.0]]),
            ],
        };
        assert_eq!(net.min_elevation(), Some(-40.0));
        assert_eq!(RoadNetwork::default().min_elevation(), None);
    }

    #[test]
    fn lane_lookup_is_by_id_not_position() {
        // Ids need not equal vec positions -- an importer may assign arbitrary
        // ones. Position 0 holds id 17, position 1 holds id 4.
        let net = RoadNetwork {
            lanes: vec![
                Lane {
                    id: LaneId(17),
                    ..lane(0, &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
                },
                Lane {
                    id: LaneId(4),
                    ..lane(1, &[[0.0, 0.0, 5.0], [1.0, 0.0, 5.0]])
                },
            ],
        };
        assert_eq!(net.lane(LaneId(17)).map(|l| l.id), Some(LaneId(17)));
        assert_eq!(net.lane(LaneId(4)).map(|l| l.id), Some(LaneId(4)));
        assert!(net.lane(LaneId(0)).is_none()); // position 0, but not id 0
                                                // nearest_lane's returned id round-trips through lane().
        let (id, _) = net.nearest_lane(Vec3::new(0.5, 0.0, 0.0)).unwrap();
        assert!(net.lane(id).is_some());
    }

    // --- bank_at sampling (independent test pass) ----------------------------

    // A flat lane (empty bank) reads 0 everywhere and never panics, including at
    // and past the ends and below zero.
    #[test]
    fn bank_at_of_a_flat_lane_is_zero_and_never_panics() {
        let l = lane(0, &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]]);
        assert!(l.bank.is_empty());
        for s in [-10.0, -0.0, 0.0, 1.0, 2.0, 4.0, 4.0001, 1000.0] {
            assert_eq!(l.bank_at(s), 0.0, "flat bank_at({s})");
        }
    }

    // A non-empty profile interpolates linearly between vertices and clamps past
    // both ends. Centerline at x = 0, 2, 4 (two 2 m segments); bank = 0, 0.1,
    // 0.2 -- so bank_at grows linearly with s and flattens outside [0, 4].
    #[test]
    fn bank_at_interpolates_between_vertices_and_clamps() {
        let l = Lane {
            bank: vec![0.0, 0.1, 0.2],
            ..lane(0, &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        };
        // Exactly on vertices.
        assert!((l.bank_at(0.0) - 0.0).abs() < 1e-6, "{}", l.bank_at(0.0));
        assert!((l.bank_at(2.0) - 0.1).abs() < 1e-6, "{}", l.bank_at(2.0));
        assert!((l.bank_at(4.0) - 0.2).abs() < 1e-6, "{}", l.bank_at(4.0));
        // Midway through each segment -> the midpoint value.
        assert!((l.bank_at(1.0) - 0.05).abs() < 1e-6, "{}", l.bank_at(1.0));
        assert!((l.bank_at(3.0) - 0.15).abs() < 1e-6, "{}", l.bank_at(3.0));
        // Past the far end clamps to the last vertex; below zero to the first.
        assert!(
            (l.bank_at(100.0) - 0.2).abs() < 1e-6,
            "{}",
            l.bank_at(100.0)
        );
        assert!(
            (l.bank_at(-100.0) - 0.0).abs() < 1e-6,
            "{}",
            l.bank_at(-100.0)
        );
        // Exactly at length and just past it must not panic and stay clamped.
        let len = l.center.length();
        assert!((l.bank_at(len) - 0.2).abs() < 1e-6);
        assert!((l.bank_at(len + 5.0) - 0.2).abs() < 1e-6);
    }

    // The stored sign is preserved (a negative bank stays negative through
    // interpolation and clamping).
    #[test]
    fn bank_at_preserves_a_negative_profile() {
        let l = Lane {
            bank: vec![-0.2, -0.1, 0.0],
            ..lane(0, &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        };
        assert!((l.bank_at(0.0) + 0.2).abs() < 1e-6, "{}", l.bank_at(0.0));
        assert!((l.bank_at(1.0) + 0.15).abs() < 1e-6, "{}", l.bank_at(1.0));
        assert!((l.bank_at(-5.0) + 0.2).abs() < 1e-6, "clamp low");
    }
}
