use std::f32::consts::FRAC_PI_2;

use glam::Vec3;

use crate::geometry::{left_normal, Polyline};
use crate::network::{Direction, Lane, LaneId, LaneKind, RoadNetwork};

const LANE_WIDTH: f32 = 3.5;
const STRAIGHT: f32 = 40.0;
const RADIUS: f32 = 30.0;
const STEP: f32 = 2.0;
const GRADE: f32 = 0.04; // gentle 4% uphill, so the road is genuinely 3D

/// A hand-authored two-lane road: a straight along +X, then a 90° left curve,
/// climbing at a constant grade. Stands in for a loaded OpenDRIVE map so the
/// whole pipeline (collider, vehicle, rendering, agents) can be built and
/// tested before the importer exists.
pub fn demo_road() -> RoadNetwork {
    let reference = reference_line();
    // Two opposing lanes, offset ±half a lane width from the reference line.
    let forward = offset_line(&reference, -LANE_WIDTH / 2.0);
    let backward = offset_line(&reference, LANE_WIDTH / 2.0);
    RoadNetwork {
        lanes: vec![
            Lane {
                id: LaneId(0),
                kind: LaneKind::Driving,
                direction: Direction::Forward,
                center: forward,
                width: LANE_WIDTH,
            },
            Lane {
                id: LaneId(1),
                kind: LaneKind::Driving,
                direction: Direction::Backward,
                center: backward,
                width: LANE_WIDTH,
            },
        ],
    }
}

/// The road's reference centerline, sampled to points.
fn reference_line() -> Polyline {
    let mut points = Vec::new();

    // Straight along +X.
    let mut x = 0.0;
    while x <= STRAIGHT {
        points.push(Vec3::new(x, x * GRADE, 0.0));
        x += STEP;
    }

    // A 90° left arc off the end of the straight. Heading rotates +X → −Z; the
    // arc centre sits a radius to the left, at (STRAIGHT, ·, −RADIUS).
    let mut angle = STEP / RADIUS;
    while angle <= FRAC_PI_2 + 1e-4 {
        let s = STRAIGHT + RADIUS * angle;
        points.push(Vec3::new(
            STRAIGHT + RADIUS * angle.sin(),
            s * GRADE,
            RADIUS * (angle.cos() - 1.0),
        ));
        angle += STEP / RADIUS;
    }

    Polyline::new(points)
}

/// Offset a polyline laterally by `offset` (positive = left of travel).
fn offset_line(line: &Polyline, offset: f32) -> Polyline {
    let points = line.points();
    let n = points.len();
    let shifted = (0..n)
        .map(|i| {
            let heading = if i + 1 < n {
                points[i + 1] - points[i]
            } else {
                points[i] - points[i - 1]
            };
            points[i] + left_normal(heading) * offset
        })
        .collect();
    Polyline::new(shifted)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_road_has_two_driving_lanes() {
        assert_eq!(demo_road().driving_lanes().count(), 2);
    }

    #[test]
    fn forward_lane_starts_heading_plus_x_and_climbs() {
        let net = demo_road();
        let lane = net.lane(LaneId(0)).expect("forward lane");
        let start = lane.center.pose_at(0.0);
        assert!(start.heading.x > 0.9, "heading {:?}", start.heading);
        // The far end is higher than the start (the constant grade).
        let end = lane.center.pose_at(lane.center.length());
        assert!(end.position.y > start.position.y + 1.0);
    }

    #[test]
    fn lane_runs_past_the_straight_into_the_curve() {
        let lane_len = demo_road().lane(LaneId(0)).unwrap().center.length();
        assert!(lane_len > STRAIGHT, "length {lane_len}");
    }

    #[test]
    fn nearest_lane_resolves_each_side_of_the_road() {
        let net = demo_road();
        // Just off the start on the +Z side is the forward lane (offset −w/2 by
        // left_normal(+X)=−Z lands it at +z), and −Z side is the backward lane.
        let (near_plus_z, _) = net.nearest_lane(Vec3::new(1.0, 0.0, 1.6)).unwrap();
        let (near_minus_z, _) = net.nearest_lane(Vec3::new(1.0, 0.0, -1.6)).unwrap();
        assert_eq!(near_plus_z, LaneId(0));
        assert_eq!(near_minus_z, LaneId(1));
    }
}
