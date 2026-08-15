use glam::Vec3;

use crate::geometry::{Polyline, Projection};

/// Index of a lane within its `RoadNetwork`.
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
}

/// The "compiled map": everything a consumer needs, baked and format-agnostic.
/// A flat list of lanes for now; road grouping and a routing graph arrive with
/// the OpenDRIVE importer, when there's real structure to represent.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RoadNetwork {
    pub lanes: Vec<Lane>,
}

impl RoadNetwork {
    pub fn lane(&self, id: LaneId) -> Option<&Lane> {
        self.lanes.get(id.0)
    }

    pub fn driving_lanes(&self) -> impl Iterator<Item = &Lane> {
        self.lanes.iter().filter(|l| l.kind == LaneKind::Driving)
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
    fn lane_lookup_by_id() {
        let net = RoadNetwork {
            lanes: vec![lane(0, &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])],
        };
        assert!(net.lane(LaneId(0)).is_some());
        assert!(net.lane(LaneId(9)).is_none());
    }
}
