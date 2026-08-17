use glam::Vec3;

use crate::geometry::{Polyline, Projection};

/// An opaque lane identifier. **Not** a vector index into `RoadNetwork.lanes` —
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

impl RoadNetwork {
    /// The lane with this id, by identity (not position), so ids stay valid
    /// however an importer assigns them.
    pub fn lane(&self, id: LaneId) -> Option<&Lane> {
        self.lanes.iter().find(|l| l.id == id)
    }

    pub fn driving_lanes(&self) -> impl Iterator<Item = &Lane> {
        self.lanes.iter().filter(|l| l.kind == LaneKind::Driving)
    }

    /// The lanes reachable by driving off `id`'s exit end (its `successors`).
    pub fn successors(&self, id: LaneId) -> impl Iterator<Item = &Lane> {
        self.lane(id)
            .into_iter()
            .flat_map(|l| l.successors.iter())
            .filter_map(|s| self.lane(*s))
    }

    /// The driving lane whose centerline is nearest `point`, with the
    /// projection onto it — the lane an agent/vehicle is in, and its
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
    fn lane_lookup_is_by_id_not_position() {
        // Ids need not equal vec positions — an importer may assign arbitrary
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
}
