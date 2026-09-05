//! A banked oval racetrack: a closed stadium loop, flat on the straights and
//! super-elevated (canted) through the two curves, with a sampling API that
//! returns the road surface point, heading, bank angle and up-normal anywhere on
//! it -- everything the `FmuVehicle` road-conform needs to drape a car onto the
//! canted surface and feed the bank into its dynamics.
//!
//! `Lane` has no bank field, and threading one through every construction site +
//! the OpenDRIVE importer would be invasive. So the bank lives in this
//! self-contained [`BankedTrack`] bundle instead: a routing-ready
//! [`RoadNetwork`] (its centerline is flat -- the cant is in the bank profile
//! and the banked mesh, not the centerline) plus a per-vertex bank profile, a
//! banked surface mesh, and the sampling API. A production map would fold bank
//! into `Lane`; this keeps the demo track contained.

use std::f32::consts::PI;

use glam::{Quat, Vec3};

use crate::geometry::{left_normal, Polyline};
use crate::mesh::Mesh;
use crate::network::{Direction, Lane, LaneId, LaneKind, RoadNetwork};

const STRAIGHT: f32 = 70.0; // length of each straight (m)
const RADIUS: f32 = 26.0; // curve radius at the centerline (m)
const WIDTH: f32 = 10.0; // lane width (m) -- wide, so the cant reads
const STEP: f32 = 2.5; // centerline sample spacing (m)
/// Superelevation at the curve apex (rad, ~12 deg). Positive raises the LEFT
/// edge; the oval turns consistently right, so its left edge is always the outer
/// (raised) one.
const PEAK_BANK: f32 = 0.21;

/// The road surface at one station: everything needed to place + orient a body
/// on the (possibly canted) road, and to feed the bank into a vehicle model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoadSample {
    /// Centerline surface point (Y-up, m).
    pub point: Vec3,
    /// Unit tangent (direction of travel), in the XZ plane.
    pub heading: Vec3,
    /// Superelevation at this station (rad, signed; positive raises the left
    /// edge). ~0 on the straights, peaks through the curves.
    pub bank: f32,
    /// Surface up-normal, tilted from +Y by `bank` about the tangent.
    pub up: Vec3,
}

/// A closed banked oval: a routing-ready network plus its bank profile, banked
/// mesh and sampling. See the module docs for why the bank lives here, not on
/// `Lane`.
#[derive(Debug, Clone)]
pub struct BankedTrack {
    /// One closed driving lane whose successor wraps to itself, so an agent can
    /// lap. Its centerline is flat (elevation only); the cant is in `bank`.
    pub network: RoadNetwork,
    center: Polyline,
    /// Per-centerline-vertex bank angle (rad, signed), parallel to
    /// `center.points()`.
    bank: Vec<f32>,
    /// Cumulative arc length at each vertex (parallel to the points), for
    /// interpolating the bank by arc length.
    cumulative: Vec<f32>,
    width: f32,
}

/// Smoothstep from `e0` to `e1` (works when `e0 > e1`, i.e. decreasing).
fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Bank envelope across a curve, as a fraction `f` in `[0, 1]`: rises over the
/// first quarter, holds at 1, falls over the last quarter -- so the cant meets
/// the flat straights at 0 at both curve ends.
fn curve_ramp(f: f32) -> f32 {
    const E: f32 = 0.25;
    smoothstep(0.0, E, f) * smoothstep(1.0, 1.0 - E, f)
}

/// Build the closed, banked oval described in the module docs.
pub fn banked_oval() -> BankedTrack {
    let half_s = STRAIGHT * 0.5;
    let curve_pts = ((PI * RADIUS) / STEP).ceil() as usize;
    let mut raw: Vec<(Vec3, f32)> = Vec::new();

    // Phase 1 -- bottom straight (z = -R), heading +X, flat.
    let mut x = -half_s;
    while x <= half_s + 1e-3 {
        raw.push((Vec3::new(x, 0.0, -RADIUS), 0.0));
        x += STEP;
    }
    // Phase 2 -- right curve, center (half_s, 0, 0), theta -pi/2 -> +pi/2.
    for k in 1..=curve_pts {
        let f = k as f32 / curve_pts as f32;
        let theta = -PI / 2.0 + f * PI;
        let p = Vec3::new(half_s + RADIUS * theta.cos(), 0.0, RADIUS * theta.sin());
        raw.push((p, PEAK_BANK * curve_ramp(f)));
    }
    // Phase 3 -- top straight (z = +R), heading -X, flat.
    let mut x = half_s;
    while x >= -half_s - 1e-3 {
        raw.push((Vec3::new(x, 0.0, RADIUS), 0.0));
        x -= STEP;
    }
    // Phase 4 -- left curve, center (-half_s, 0, 0), theta +pi/2 -> +3pi/2.
    for k in 1..=curve_pts {
        let f = k as f32 / curve_pts as f32;
        let theta = PI / 2.0 + f * PI;
        let p = Vec3::new(-half_s + RADIUS * theta.cos(), 0.0, RADIUS * theta.sin());
        raw.push((p, PEAK_BANK * curve_ramp(f)));
    }
    // Close the loop back onto the first point.
    raw.push((Vec3::new(-half_s, 0.0, -RADIUS), 0.0));

    // Dedup consecutive near-equal points (phase joins land on the same point).
    let mut points: Vec<Vec3> = Vec::new();
    let mut bank: Vec<f32> = Vec::new();
    for (p, b) in raw {
        if points.last().is_none_or(|l| (*l - p).length() >= 1e-3) {
            points.push(p);
            bank.push(b);
        }
    }

    let mut cumulative = Vec::with_capacity(points.len());
    let mut acc = 0.0;
    cumulative.push(0.0);
    for pair in points.windows(2) {
        acc += (pair[1] - pair[0]).length();
        cumulative.push(acc);
    }

    let center = Polyline::new(points);
    let network = RoadNetwork {
        lanes: vec![Lane {
            id: LaneId(0),
            kind: LaneKind::Driving,
            direction: Direction::Forward,
            center: center.clone(),
            width: WIDTH,
            // A closed loop: driving off the exit end re-enters the same lane.
            successors: vec![LaneId(0)],
            predecessors: vec![LaneId(0)],
            neighbors: Vec::new(),
        }],
    };

    BankedTrack {
        network,
        center,
        bank,
        cumulative,
        width: WIDTH,
    }
}

impl BankedTrack {
    /// Total loop length (m).
    pub fn length(&self) -> f32 {
        self.center.length()
    }

    /// Bank angle (rad) at arc length `s`, interpolated between vertices.
    fn bank_at(&self, s: f32) -> f32 {
        let s = s.clamp(0.0, self.length());
        let i = self
            .cumulative
            .partition_point(|&c| c <= s)
            .saturating_sub(1)
            .min(self.bank.len() - 2);
        let seg = self.cumulative[i + 1] - self.cumulative[i];
        let t = if seg > 1e-6 {
            (s - self.cumulative[i]) / seg
        } else {
            0.0
        };
        self.bank[i] + (self.bank[i + 1] - self.bank[i]) * t
    }

    /// Sample the road at arc length `s` along the loop.
    pub fn sample_at(&self, s: f32) -> RoadSample {
        let pose = self.center.pose_at(s);
        let bank = self.bank_at(s);
        RoadSample {
            point: pose.position,
            heading: pose.heading,
            bank,
            up: banked_up(pose.heading, bank),
        }
    }

    /// Sample the road nearest a world point -- the road-conform's entry point:
    /// project the body's position onto the centerline, then read the surface
    /// there.
    pub fn sample_near(&self, point: Vec3) -> RoadSample {
        self.sample_at(self.center.project(point).s)
    }

    /// The banked surface mesh: a quad strip whose ribs are tilted about the
    /// tangent by the local bank, so the outer edge of each curve rides higher
    /// than the inner one. The `map` collider + viewer render this.
    pub fn banked_mesh(&self) -> Mesh {
        let mut mesh = Mesh::default();
        let points = self.center.points();
        let tangents = self.center.tangents();
        let half = self.width * 0.5;
        for i in 0..points.len() {
            let along = tangents[i];
            // Lateral, tilted about the tangent by the bank: +bank raises the
            // left rib.
            let lateral = Quat::from_axis_angle(along, self.bank[i]) * left_normal(along);
            let up = along.cross(lateral).normalize_or(Vec3::Y);
            mesh.vertices.push(points[i] + lateral * half);
            mesh.normals.push(up);
            mesh.vertices.push(points[i] - lateral * half);
            mesh.normals.push(up);
        }
        for i in 0..points.len() as u32 - 1 {
            let l0 = i * 2;
            let (r0, l1, r1) = (l0 + 1, l0 + 2, l0 + 3);
            mesh.indices.extend_from_slice(&[l0, r0, r1, l0, r1, l1]);
        }
        mesh
    }
}

/// The surface up-normal for a road with horizontal tangent `heading`, tilted
/// from +Y about that tangent by `bank` (positive raises the left edge).
fn banked_up(heading: Vec3, bank: f32) -> Vec3 {
    (Quat::from_axis_angle(heading, bank) * Vec3::Y).normalize_or(Vec3::Y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oval_is_one_closed_driving_lane() {
        let track = banked_oval();
        let lanes: Vec<_> = track.network.driving_lanes().collect();
        assert_eq!(lanes.len(), 1);
        // Successor wraps to itself -- an agent can lap.
        assert_eq!(lanes[0].successors, vec![LaneId(0)]);
        // Geometrically closed: first and last centerline points coincide.
        let pts = lanes[0].center.points();
        assert!(
            (pts[0] - pts[pts.len() - 1]).length() < 1e-3,
            "loop should close: {:?} vs {:?}",
            pts[0],
            pts[pts.len() - 1]
        );
    }

    #[test]
    fn straights_are_flat_and_curves_are_banked() {
        let track = banked_oval();
        // A point partway along the bottom straight: near-zero bank.
        let on_straight = track.sample_near(Vec3::new(0.0, 0.0, -RADIUS));
        assert!(
            on_straight.bank.abs() < 0.01,
            "straight bank {}",
            on_straight.bank
        );
        // The right-curve apex (max +X): near the peak bank.
        let half_s = STRAIGHT * 0.5;
        let apex = track.sample_near(Vec3::new(half_s + RADIUS, 0.0, 0.0));
        assert!(
            apex.bank > 0.9 * PEAK_BANK,
            "apex bank {} should approach {}",
            apex.bank,
            PEAK_BANK
        );
    }

    #[test]
    fn straight_up_is_vertical_curve_up_is_tilted() {
        let track = banked_oval();
        let straight = track.sample_near(Vec3::new(0.0, 0.0, -RADIUS));
        assert!(straight.up.abs_diff_eq(Vec3::Y, 1e-3));
        let half_s = STRAIGHT * 0.5;
        let apex = track.sample_near(Vec3::new(half_s + RADIUS, 0.0, 0.0));
        // Tilted: the up-normal leans off vertical but still points up.
        assert!(apex.up.y < 0.99 && apex.up.y > 0.9, "up.y {}", apex.up.y);
        assert!(
            (apex.up - Vec3::Y).length() > 0.05,
            "curve up should be visibly tilted"
        );
    }

    #[test]
    fn banked_mesh_raises_the_outer_edge_at_a_curve_apex() {
        let track = banked_oval();
        let mesh = track.banked_mesh();
        mesh.validate().expect("banked mesh is a valid trimesh");
        // Ribs are pushed as [left, right] pairs. Find the rib nearest the
        // right-curve apex and check the left (outer) vertex rides above the
        // right (inner) one; on a straight rib they are level.
        let half_s = STRAIGHT * 0.5;
        let apex_xz = Vec3::new(half_s + RADIUS, 0.0, 0.0);
        let mut best = 0usize;
        let mut best_d = f32::INFINITY;
        for i in (0..mesh.vertices.len()).step_by(2) {
            let mid = (mesh.vertices[i] + mesh.vertices[i + 1]) * 0.5;
            let d = (Vec3::new(mid.x, 0.0, mid.z) - apex_xz).length();
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        let left = mesh.vertices[best];
        let right = mesh.vertices[best + 1];
        assert!(
            left.y - right.y > 0.5,
            "outer (left) edge {} should ride above inner (right) {} at the apex",
            left.y,
            right.y
        );

        // A straight rib (near the bottom straight center) is level.
        let straight_xz = Vec3::new(0.0, 0.0, -RADIUS);
        let mut s_best = 0usize;
        let mut s_d = f32::INFINITY;
        for i in (0..mesh.vertices.len()).step_by(2) {
            let mid = (mesh.vertices[i] + mesh.vertices[i + 1]) * 0.5;
            let d = (Vec3::new(mid.x, 0.0, mid.z) - straight_xz).length();
            if d < s_d {
                s_d = d;
                s_best = i;
            }
        }
        assert!(
            (mesh.vertices[s_best].y - mesh.vertices[s_best + 1].y).abs() < 0.05,
            "straight rib should be level"
        );
    }

    #[test]
    fn sample_near_tracks_heading_around_the_loop() {
        let track = banked_oval();
        // On the bottom straight the car heads +X; on the top straight, -X.
        let bottom = track.sample_near(Vec3::new(0.0, 0.0, -RADIUS));
        assert!(
            bottom.heading.x > 0.9,
            "bottom heading {:?}",
            bottom.heading
        );
        let top = track.sample_near(Vec3::new(0.0, 0.0, RADIUS));
        assert!(top.heading.x < -0.9, "top heading {:?}", top.heading);
    }
}
