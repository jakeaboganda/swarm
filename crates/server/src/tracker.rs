//! Path tracking: how far along its plan a body is, and where to aim next.
//!
//! Pure geometry in the ground plane -- motion is ground-constrained, so a
//! lane that climbs must not read as a longer path than the car drives along
//! the ground, and the aim point never has a vertical component.
//!
//! Progress is **measured, never consumed**: the plan stays exactly as the
//! agent submitted it, so an agent that re-plans never has to reconcile its
//! own path against server-side bookkeeping. The cost is one piece of state
//! (how far along the path the body was last tick), and that state is only
//! ever meaningful for the plan it was measured on -- see [`Progress`].
//!
//! The path is deliberately not `map::Polyline`: that one is 3D road geometry
//! with no notion of a commanded speed, and it projects globally.

use std::collections::VecDeque;
use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use protocol::messages::Waypoint;

/// The final waypoint is reached once the body's projection is within this
/// distance of the end of the path.
pub const ARRIVAL_TOLERANCE: f32 = 0.5;

/// Inside this distance *along the path* of the end, commanded speed scales
/// down linearly toward zero ("arrive" behavior) so the body settles on the
/// destination instead of overshooting and orbiting it. Vertices partway along
/// the path are driven through at the speed the plan asked for -- the router
/// samples at 2 m, exactly this radius, so slowing at each of them would leave
/// a routed car permanently inside the ramp.
pub const ARRIVE_RADIUS: f32 = 2.0;

/// Lookahead per unit speed (seconds of travel ahead) -- the driver's preview
/// time, and the tracker's one real tuning constant.
///
/// It sets the whole tradeoff. A pursuit tracker holds a curve by sitting
/// slightly wide of it (the tires need slip angle to make the radius, and a
/// proportional law can only ask for the extra curvature by carrying an
/// error), and that error grows in proportion to the lookahead: measured on
/// `maps/testtrack.xodr`, worst cross-track ran 0.12 / 0.17 / 0.48 / 0.93 m at
/// 0.25 / 0.4 / 0.8 / 1.2 s of preview. Shorter is tighter -- until the loop
/// starts ringing. The first steering reversals appeared at 0.25 s and 23 m/s,
/// so this sits at twice that, with the aim point still at least two
/// wheelbases out at every speed the car reaches.
const LOOKAHEAD_TIME: f32 = 0.4;

/// Lookahead floor (m). Below the car's own wheelbase (2.6 m) the aim point is
/// inside the vehicle and the pursuit geometry asks for a circle tighter than
/// the steering lock can describe, so the floor sits just above it.
const MIN_LOOKAHEAD: f32 = 3.0;

/// Lookahead ceiling (m). Cross-track error grows at roughly 5 cm per metre of
/// lookahead on this plant (see [`LOOKAHEAD_TIME`]), so a 10 m ceiling holds it
/// near half a metre -- under a third of a 3.5 m lane's half-width -- however
/// fast an agent asks to go.
const MAX_LOOKAHEAD: f32 = 10.0;

/// How far behind the last known progress the projection still looks, so a
/// body that gets shoved backwards can correct.
const SEARCH_BACK: f32 = 1.0;

/// How far ahead of the last known progress the projection looks. One 64 Hz
/// tick covers 0.4 m at 25 m/s, so this is ~25 ticks of slack for a body that
/// gets thrown along its path -- while staying well under the separation of
/// the two legs of any corner the car can physically take (a radius-R hairpin
/// puts its legs 2R apart, and R is bounded below by grip), so the search
/// cannot skip a lap of a self-intersecting path.
const SEARCH_FORWARD: f32 = 10.0;

/// Where a body is along its plan. Only meaningful for the plan it was
/// measured on: a new plan is a new path, and an arc length on the old one
/// says nothing about it. `agent::Plan` stamps this with the plan version and
/// discards it when the version moves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Progress {
    /// Arc length, in the ground plane, from the plan's first waypoint.
    pub s: f32,
    /// Index of the first waypoint the body has not yet passed. The remaining
    /// plan, for anything that wants to show or count it.
    pub next_vertex: usize,
}

/// What the tracker asks of the body this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tracking {
    /// Drive at `velocity` (ground plane, magnitude = commanded speed),
    /// steering at an aim point `lookahead` metres from the body.
    Drive { velocity: Vec3, lookahead: f32 },
    /// The plan is complete -- the body reached the end of the path.
    Arrived,
    /// The plan yields no usable direction this tick (degenerate or
    /// non-finite geometry). Demand nothing, but the plan still stands.
    Hold,
}

/// One tick of tracking: the progress to carry forward, and what to do with it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tracked {
    pub progress: Progress,
    pub tracking: Tracking,
}

/// A plan's waypoints as a ground-plane path: vertices, the speed commanded at
/// each, and cumulative arc length.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanPath {
    points: Vec<Vec3>,
    speeds: Vec<f32>,
    cumulative: Vec<f32>,
}

impl PlanPath {
    /// The path a plan describes, or `None` if the plan is empty or its
    /// geometry is not finite.
    pub fn new(waypoints: &VecDeque<Waypoint>) -> Option<Self> {
        if waypoints.is_empty() {
            return None;
        }
        let points: Vec<Vec3> = waypoints
            .iter()
            .map(|w| Vec3::new(w.position.x, 0.0, w.position.z))
            .collect();
        let speeds: Vec<f32> = waypoints.iter().map(|w| w.speed.max(0.0)).collect();
        let mut cumulative = Vec::with_capacity(points.len());
        let mut acc = 0.0;
        cumulative.push(0.0);
        for pair in points.windows(2) {
            acc += (pair[1] - pair[0]).length();
            cumulative.push(acc);
        }
        // Geometry that is not finite is not a path. `inbound::sanitize_plan`
        // keeps those waypoints out of a plan; refusing them here as well is
        // what stops a `NaN` ever reaching the aim-point arithmetic, and from
        // there `ExternalForce`.
        if !acc.is_finite()
            || !points.iter().all(|p| p.is_finite())
            || !speeds.iter().all(|s| s.is_finite())
        {
            return None;
        }
        Some(Self {
            points,
            speeds,
            cumulative,
        })
    }

    /// Total ground-plane length. Zero for a one-waypoint plan.
    pub fn length(&self) -> f32 {
        self.cumulative.last().copied().unwrap_or(0.0)
    }

    /// A one-waypoint plan is a destination, not a path: there is no geometry
    /// to look along, so the aim point is the waypoint itself and arrival is
    /// simply being close enough to it.
    fn is_destination(&self) -> bool {
        self.points.len() < 2
    }

    fn segments(&self) -> usize {
        self.points.len().saturating_sub(1)
    }

    /// The segment containing arc length `s`, clamped to a real segment.
    fn segment(&self, s: f32) -> usize {
        self.cumulative
            .partition_point(|&c| c <= s)
            .saturating_sub(1)
            .min(self.segments().saturating_sub(1))
    }

    /// Nearest point on the path to `position`, as an arc length, searched
    /// only within [`SEARCH_BACK`]/[`SEARCH_FORWARD`] of `from`.
    ///
    /// The window is the whole point. A globally nearest point can be a lap
    /// away on any circuit, or -- for a body not yet on its path -- the path's
    /// own end, which would read as "already arrived" and drop a plan the
    /// agent had just submitted. Searching forward from where the body
    /// actually was can do neither.
    fn project(&self, position: Vec3, from: f32) -> Progress {
        let position = planar(position);
        if self.is_destination() {
            return Progress {
                s: 0.0,
                next_vertex: 0,
            };
        }
        let mut best = None;
        let mut best_dist = f32::INFINITY;
        for i in 0..self.segments() {
            if self.cumulative[i + 1] < from - SEARCH_BACK
                || self.cumulative[i] > from + SEARCH_FORWARD
            {
                continue;
            }
            let a = self.points[i];
            let ab = self.points[i + 1] - a;
            let len2 = ab.length_squared();
            let t = if len2 > 1e-9 {
                ((position - a).dot(ab) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let distance = (position - (a + ab * t)).length_squared();
            if distance < best_dist {
                best_dist = distance;
                best = Some(Progress {
                    s: self.cumulative[i] + ab.length() * t,
                    next_vertex: i + 1,
                });
            }
        }
        // Nothing searched (a `from` outside the path, or a non-finite
        // position): keep the progress we had rather than inventing one.
        best.unwrap_or_else(|| {
            let s = if from.is_finite() { from } else { 0.0 }.clamp(0.0, self.length());
            Progress {
                s,
                next_vertex: self.segment(s) + 1,
            }
        })
    }

    /// Point at arc length `s`. Past the end the path continues along its own
    /// final arc -- see [`PlanPath::beyond_end`].
    fn point_at(&self, s: f32) -> Vec3 {
        let length = self.length();
        if s > length {
            return self.beyond_end(s - length);
        }
        let i = self.segment(s.max(0.0));
        let span = self.cumulative[i + 1] - self.cumulative[i];
        let t = if span > 1e-6 {
            (s - self.cumulative[i]) / span
        } else {
            0.0
        };
        self.points[i].lerp(self.points[i + 1], t)
    }

    /// The path continued `distance` past its last waypoint, along its own
    /// final arc: same tangent, same curvature.
    ///
    /// The aim point has to stay a full lookahead ahead right to the last
    /// metre, or the pursuit geometry tightens into exactly the steering spike
    /// this tracker exists to remove. Continuing along the final *tangent*
    /// would do that much, but it straightens the wheel over the last few
    /// metres of every path -- and a plan that ends mid-corner is precisely
    /// where that costs: the car leaves the bend tangentially while it is
    /// still coasting to a stop. Continuing the arc keeps it in the corner.
    fn beyond_end(&self, distance: f32) -> Vec3 {
        let last = self.points[self.points.len() - 1];
        let (tangent, curvature) = self.final_arc();
        if tangent == Vec3::ZERO {
            return last;
        }
        // Beyond a quarter turn the extrapolation says more about the last two
        // segments than about anything real, so the arc stops turning there.
        let turn = (curvature * distance).clamp(-FRAC_PI_2, FRAC_PI_2);
        if turn.abs() < 1e-4 {
            return last + tangent * distance;
        }
        let radius = 1.0 / curvature;
        let centre = last + left_of(tangent) * radius;
        rotate_left(last - centre, turn) + centre
    }

    /// Unit tangent and signed curvature (rad/m, +ve turning left) *at the
    /// path's last waypoint*, from its last two real segments. A zero tangent
    /// means the path has no length at all.
    ///
    /// The segments are chords, so the final one's direction lags the true
    /// tangent at that vertex by half of its own turn. The correction is worth
    /// making: the extrapolation runs several metres, and the error it leaves
    /// grows with every one of them.
    fn final_arc(&self) -> (Vec3, f32) {
        let mut segments = self
            .points
            .windows(2)
            .rev()
            .map(|pair| pair[1] - pair[0])
            .filter(|delta| delta.length_squared() > 1e-9);
        let Some(last) = segments.next() else {
            return (Vec3::ZERO, 0.0);
        };
        let direction = last.normalize();
        let Some(previous) = segments.next() else {
            return (direction, 0.0);
        };
        let before = previous.normalize();
        let turn = (before.z * direction.x - before.x * direction.z).atan2(before.dot(direction));
        let curvature = turn / (0.5 * (previous.length() + last.length()));
        (
            rotate_left(direction, 0.5 * curvature * last.length()),
            curvature,
        )
    }

    /// Commanded speed at arc length `s`, interpolated between the bracketing
    /// waypoints. Interpolated rather than stepped at each vertex: a plan
    /// sampled every few metres would otherwise step the speed command at
    /// exactly the interval that excites the chassis.
    fn speed_at(&self, s: f32) -> f32 {
        if self.is_destination() {
            return self.speeds[0];
        }
        let s = s.clamp(0.0, self.length());
        let i = self.segment(s);
        let span = self.cumulative[i + 1] - self.cumulative[i];
        let t = if span > 1e-6 {
            (s - self.cumulative[i]) / span
        } else {
            0.0
        };
        self.speeds[i] + (self.speeds[i + 1] - self.speeds[i]) * t
    }

    /// Track `position` against this path.
    ///
    /// `body_speed` is the body's own ground speed, which sets the lookahead
    /// (a fixed lookahead is either twitchy at speed or lazy when slow).
    /// `from` is last tick's progress; `None` -- a plan not tracked yet --
    /// starts at the beginning of the path, because a path is something to
    /// drive from its start, and because the alternative (the nearest point
    /// anywhere on it) can pick a later lap or the far end.
    ///
    /// The returned velocity is always finite: a path that yields no usable
    /// direction reports [`Tracking::Hold`], so nothing non-finite can reach
    /// `ExternalForce`. `inbound::sanitize_plan` already keeps those out of a
    /// plan; this is the last guard before the force is applied.
    pub fn track(&self, position: Vec3, body_speed: f32, from: Option<f32>) -> Tracked {
        let progress = self.project(position, from.unwrap_or(0.0));
        let position = planar(position);
        let remaining = if self.is_destination() {
            (self.points[0] - position).length()
        } else {
            self.length() - progress.s
        };
        if !remaining.is_finite() {
            return Tracked {
                progress,
                tracking: Tracking::Hold,
            };
        }
        if remaining <= ARRIVAL_TOLERANCE {
            return Tracked {
                progress,
                tracking: Tracking::Arrived,
            };
        }

        // `max` returns the non-NaN side, so a NaN speed becomes a zero one.
        let lookahead_distance =
            (LOOKAHEAD_TIME * body_speed.max(0.0)).clamp(MIN_LOOKAHEAD, MAX_LOOKAHEAD);
        let aim = if self.is_destination() {
            self.points[0]
        } else {
            self.point_at(progress.s + lookahead_distance)
        };
        let delta = aim - position;
        let lookahead = delta.length();
        let speed = self.speed_at(progress.s).max(0.0) * (remaining / ARRIVE_RADIUS).min(1.0);
        let velocity = delta / lookahead * speed;
        let tracking = if lookahead > 1e-3 && lookahead.is_finite() && velocity.is_finite() {
            Tracking::Drive {
                velocity,
                lookahead,
            }
        } else {
            Tracking::Hold
        };
        Tracked { progress, tracking }
    }
}

/// Drop a vector onto the ground plane.
fn planar(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z)
}

/// `v` rotated a quarter turn to the left, keeping its length. For +X this is
/// -Z, matching the convention that +ve angles and curvatures turn left.
fn left_of(v: Vec3) -> Vec3 {
    Vec3::new(v.z, 0.0, -v.x)
}

/// `v` rotated `angle` radians to the left in the ground plane.
fn rotate_left(v: Vec3, angle: f32) -> Vec3 {
    v * angle.cos() + left_of(v) * angle.sin()
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::Vec3 as WireVec3;

    fn plan(points: &[(f32, f32)], speed: f32) -> PlanPath {
        let waypoints: VecDeque<Waypoint> = points
            .iter()
            .map(|(x, z)| Waypoint {
                position: WireVec3::new(*x, 0.0, *z),
                speed,
            })
            .collect();
        PlanPath::new(&waypoints).expect("a non-empty plan")
    }

    /// A straight path down +X from the origin, sampled every 4 m, the way the
    /// server's router and every lane-following agent lay one out. Starting at
    /// the origin makes a body's x coordinate its arc length.
    fn straight(metres: usize, speed: f32) -> PlanPath {
        let points: Vec<(f32, f32)> = (0..=metres / 4).map(|i| (i as f32 * 4.0, 0.0)).collect();
        plan(&points, speed)
    }

    fn drive(tracked: Tracked) -> (Vec3, f32) {
        match tracked.tracking {
            Tracking::Drive {
                velocity,
                lookahead,
            } => (velocity, lookahead),
            other => panic!("expected to be driving, got {other:?}"),
        }
    }

    #[test]
    fn the_aim_point_never_collapses_onto_a_vertex() {
        // The bug this tracker replaces: aiming at the next *vertex* and
        // dropping it at 0.5 m made the aim distance collapse 4.0 -> 0.5 m
        // within every leg, an 8x swing in the effective steering gain,
        // spiking just before each vertex. An aim point interpolated a fixed
        // distance *along* the path cannot do that -- there is no vertex for
        // it to land on.
        let path = straight(400, 15.0);
        let mut shortest = f32::INFINITY;
        let mut longest: f32 = 0.0;
        let mut station = 0.0;
        while station < path.length() - ARRIVAL_TOLERANCE {
            let body = Vec3::new(station, 0.9, 0.0);
            let (_, lookahead) = drive(path.track(body, 15.0, Some(station)));
            shortest = shortest.min(lookahead);
            longest = longest.max(lookahead);
            station += 0.1;
        }
        // The body is on a straight path, so the aim point is exactly one
        // lookahead ahead of it -- the same distance at every station, right
        // up to the last metre.
        let expected = LOOKAHEAD_TIME * 15.0;
        assert!(
            (shortest - expected).abs() < 1e-3 && (longest - expected).abs() < 1e-3,
            "aim distance wandered between {shortest} and {longest} m,              not a steady {expected} m"
        );
    }

    #[test]
    fn past_its_end_the_path_carries_on_along_its_own_arc() {
        // The aim point stays a full lookahead ahead even in the last metres,
        // and where it goes decides what the wheel does there. Continuing the
        // final *tangent* would straighten the car out of a corner while it is
        // still in one; continuing the arc keeps it on the curve the path was
        // describing.
        let radius = 30.0f32;
        let points: Vec<(f32, f32)> = (0..=20)
            .map(|i| {
                // A left-hand arc of `radius`, sampled every 1.5 m, starting at
                // the origin heading +X.
                let theta = i as f32 * 1.5 / radius;
                (radius * theta.sin(), -radius * (1.0 - theta.cos()))
            })
            .collect();
        let path = plan(&points, 8.0);
        let centre = Vec3::new(0.0, 0.0, -radius);
        for extra in [1.0f32, 5.0, 10.0] {
            let beyond = path.point_at(path.length() + extra);
            assert!(
                ((beyond - centre).length() - radius).abs() < 0.05,
                "{extra} m past the end landed {:.2} m from the arc's centre,                  not {radius}",
                (beyond - centre).length()
            );
        }

        // A straight path still continues straight.
        let straight_path = straight(100, 8.0);
        let beyond = straight_path.point_at(straight_path.length() + 5.0);
        assert!((beyond - Vec3::new(105.0, 0.0, 0.0)).length() < 1e-3);
    }

    #[test]
    fn the_lookahead_scales_with_speed_between_its_bounds() {
        let path = straight(400, 20.0);
        let at = |speed| drive(path.track(Vec3::new(40.0, 0.9, 0.0), speed, Some(40.0))).1;
        // Stopped, the floor holds -- an aim point inside the wheelbase is
        // geometry no steering lock can satisfy.
        assert!((at(0.0) - MIN_LOOKAHEAD).abs() < 1e-3);
        assert!((at(10.0) - LOOKAHEAD_TIME * 10.0).abs() < 1e-3);
        // And it stops growing at the ceiling.
        assert!((at(40.0) - MAX_LOOKAHEAD).abs() < 1e-3);
    }

    #[test]
    fn progress_is_searched_forward_so_a_path_that_doubles_back_cannot_jump() {
        // Two legs 1 m apart: out along z = 0, back along z = 1. A body at
        // z = 0.7 is nearer the *return* leg, so nearest-point-anywhere puts
        // it 30 m further up the plan, past the whole outbound leg. Searching
        // forward from where the body actually was keeps each pass on its own
        // leg -- the same position tracks as either, depending only on which
        // one it was on last tick, which is the only thing that can tell them
        // apart.
        let mut points: Vec<(f32, f32)> = (0..=30).map(|i| (i as f32, 0.0)).collect();
        points.extend((0..=30).rev().map(|i| (i as f32, 1.0)));
        let path = plan(&points, 8.0);
        let body = Vec3::new(15.0, 0.9, 0.7);

        let outbound = path.track(body, 8.0, Some(15.0));
        assert!(
            (outbound.progress.s - 15.0).abs() < 0.5,
            "the outbound leg jumped to s = {} from 15",
            outbound.progress.s
        );
        // Still driving *out*, not back.
        let (velocity, _) = drive(outbound);
        assert!(velocity.x > 0.0, "turned around: {velocity:?}");

        // The same body on the return leg tracks there instead. The outbound
        // leg is 30 m, plus 1 m across, so x = 15 on the way back is s = 46.
        let inbound = path.track(body, 8.0, Some(46.0));
        assert!(
            (inbound.progress.s - 46.0).abs() < 0.5,
            "the return leg reported s = {}",
            inbound.progress.s
        );
        let (velocity, _) = drive(inbound);
        assert!(velocity.x < 0.0, "still heading out on the return leg");
    }

    #[test]
    fn a_fresh_plan_is_picked_up_at_its_start_not_at_whichever_point_is_nearest() {
        // A path that loops back past the body: out 200 m, across 50, and all
        // the way back. The body sits on its first waypoint -- and 50 m from
        // its *last* one, which is much nearer than most of the path. Read as
        // a nearest-point problem this says "you are at the end, you have
        // arrived", and a plan the agent submitted a tick ago gets dropped
        // before the car moves. A path is driven from its start.
        let path = plan(&[(0.0, 0.0), (200.0, 0.0), (200.0, 50.0), (0.0, 50.0)], 8.0);
        let tracked = path.track(Vec3::new(0.0, 0.9, 0.0), 0.0, None);
        assert_eq!(tracked.progress.s, 0.0);
        let (velocity, _) = drive(tracked);
        assert!(
            velocity.x > 0.0 && velocity.z.abs() < 1e-3,
            "set off along {velocity:?} instead of down the first leg"
        );
    }

    #[test]
    fn a_body_at_the_first_vertex_has_not_arrived() {
        // Arrival is reaching the end of the path, not the first waypoint on
        // it. Popping vertices made those the same event for a one-vertex-long
        // plan; measured progress keeps them apart for every plan.
        let path = straight(200, 10.0);
        let at_first = path.track(Vec3::new(4.2, 0.9, 0.0), 10.0, Some(4.2));
        assert!(matches!(at_first.tracking, Tracking::Drive { .. }));

        let past_the_end = path.track(Vec3::new(205.0, 0.9, 0.0), 10.0, Some(198.0));
        assert_eq!(past_the_end.tracking, Tracking::Arrived);
    }

    #[test]
    fn a_mid_path_vertex_is_passed_at_the_speed_the_plan_asked_for() {
        // Slowing to settle onto a destination is right; doing it at every
        // vertex of a path is not -- the body is passing through those. The
        // server's own router samples at 2 m, exactly ARRIVE_RADIUS, so a
        // routed agent would otherwise spend every leg inside the ramp and
        // never reach the speed it asked for.
        let points: Vec<(f32, f32)> = (1..=60).map(|i| (i as f32 * 2.0, 0.0)).collect();
        let path = plan(&points, 8.0);
        for station in [2.0f32, 3.0, 40.1, 79.9] {
            let (velocity, _) = drive(path.track(Vec3::new(station, 0.9, 0.0), 8.0, Some(station)));
            assert!(
                (velocity.length() - 8.0).abs() < 1e-3,
                "passed a mid-path vertex at {} m/s instead of 8",
                velocity.length()
            );
        }
    }

    #[test]
    fn speed_ramps_down_over_the_last_metres_of_the_path() {
        // The ramp is distance *remaining along the path*, which is what makes
        // it a property of the destination rather than of whichever vertex
        // happens to be next.
        let path = straight(200, 10.0);
        let end = path.length();
        let (half, _) = drive(path.track(Vec3::new(end - 1.0, 0.9, 0.0), 10.0, Some(end - 1.0)));
        assert!(
            (half.length() - 5.0).abs() < 1e-3,
            "1 m from the end of a 2 m ramp the command was {} m/s, not 5",
            half.length()
        );
        let (full, _) = drive(path.track(Vec3::new(end - 20.0, 0.9, 0.0), 10.0, Some(end - 20.0)));
        assert!((full.length() - 10.0).abs() < 1e-3);
    }

    #[test]
    fn a_one_waypoint_plan_is_a_destination() {
        // No path to look along: aim at the point, ramp on the distance to it,
        // and arrive within tolerance of it -- exactly what a plain seek did.
        let path = plan(&[(10.0, 0.0)], 5.0);
        let (velocity, lookahead) = drive(path.track(Vec3::ZERO, 5.0, None));
        assert!((velocity.length() - 5.0).abs() < 1e-3);
        assert!((lookahead - 10.0).abs() < 1e-3);

        let (slowed, _) = drive(path.track(Vec3::new(9.0, 0.9, 0.0), 5.0, None));
        assert!((slowed.length() - 2.5).abs() < 1e-3);

        // Arrived when horizontally close, despite resting a metre above the
        // ground-plane waypoint.
        let arrived = path.track(Vec3::new(9.8, 1.0, 0.1), 5.0, None);
        assert_eq!(arrived.tracking, Tracking::Arrived);
    }

    #[test]
    fn the_remaining_plan_starts_at_the_vertex_the_body_has_not_passed() {
        let path = straight(200, 10.0);
        // Waypoints are at 0, 4, 8, 12 ... so 9 m along, the first one the
        // body has not passed is #3 (at 12 m).
        assert_eq!(path.project(Vec3::new(9.0, 0.9, 0.0), 9.0).next_vertex, 3);
        assert_eq!(path.project(Vec3::new(1.0, 0.9, 0.0), 1.0).next_vertex, 1);
    }

    #[test]
    fn the_tracker_never_demands_a_non_finite_velocity() {
        // Sanitization upstream (`inbound::sanitize_plan`) keeps these out of
        // a plan, but this is the last step before `DesiredVelocity` reaches
        // `ExternalForce` and Rapier, so it holds the line on its own.
        let nasty = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::MAX,
            f32::MIN,
            0.0,
            -0.0,
            1.0,
        ];
        for &a in &nasty {
            for &b in &nasty {
                for &speed in &nasty {
                    let waypoints: VecDeque<Waypoint> = [(a, a), (b, b)]
                        .iter()
                        .map(|(x, z)| Waypoint {
                            position: WireVec3::new(*x, 0.0, *z),
                            speed,
                        })
                        .collect();
                    let Some(path) = PlanPath::new(&waypoints) else {
                        continue; // no finite path, so nothing to demand
                    };
                    for from in [None, Some(a), Some(0.0)] {
                        let tracked = path.track(Vec3::new(a, 1.0, b), speed, from);
                        assert!(
                            tracked.progress.s.is_finite(),
                            "progress {} is not finite",
                            tracked.progress.s
                        );
                        if let Tracking::Drive {
                            velocity,
                            lookahead,
                        } = tracked.tracking
                        {
                            assert!(
                                velocity.is_finite() && lookahead.is_finite(),
                                "track({a}, {b}, {speed}) = {velocity:?} / {lookahead}"
                            );
                            assert!(velocity.length() >= 0.0, "negative speed leaked through");
                        }
                    }
                }
            }
        }
    }
}
