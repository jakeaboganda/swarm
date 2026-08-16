//! The static HD-map prior handed to an agent at join. Format-agnostic and
//! already baked to points -- the agent samples waypoints along a lane's
//! centerline to lay a path. Mirrors the sim-side `map::RoadNetwork`, but as a
//! wire type so `protocol` stays dependency-free.

use serde::{Deserialize, Serialize};

use crate::Vec3;

/// Travel direction of a lane relative to its centerline's start->end order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneDirection {
    Forward,
    Backward,
}

/// What a lane is for. Only driving lanes exist today; an agent drives only
/// these. Non-driving kinds (shoulder, sidewalk) arrive with the importer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneKind {
    Driving,
}

/// One lane as delivered to an agent: enough to lay a path down it. The
/// centerline is baked to points (Y-up, meters); the agent samples along it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LaneData {
    pub id: u64,
    pub kind: LaneKind,
    pub direction: LaneDirection,
    pub width: f32,
    pub centerline: Vec<Vec3>,
}

/// The road handed to an agent at join: its lanes. Static and perfect -- the
/// road is a known prior. Dynamic traffic is separate (perceived, impaired, via
/// the perception pathway). Present only in the automotive world.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapData {
    pub lanes: Vec<LaneData>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_data_round_trips() {
        let map = MapData {
            lanes: vec![
                LaneData {
                    id: 0,
                    kind: LaneKind::Driving,
                    direction: LaneDirection::Forward,
                    width: 3.5,
                    centerline: vec![Vec3::new(0.0, 0.0, 1.75), Vec3::new(10.0, 0.4, 1.75)],
                },
                LaneData {
                    id: 1,
                    kind: LaneKind::Driving,
                    direction: LaneDirection::Backward,
                    width: 3.5,
                    centerline: vec![Vec3::new(0.0, 0.0, -1.75), Vec3::new(10.0, 0.4, -1.75)],
                },
            ],
        };
        let json = serde_json::to_string(&map).expect("serialize");
        let back: MapData = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(map, back, "round-trip mismatch, json was: {json}");
    }
}
