//! Path tracking end to end: the plan the agent submitted stays its own, and
//! the car holds its lane at every speed it is asked to hold it at.
//!
//! `testtrack.rs` drives the same track at one speed to check the car survives
//! it. These are about the *tracker*: how the server turns a plan into a
//! steering command, and what that costs in cross-track error as speed rises.

mod support;

use bevy_rapier3d::prelude::Velocity;
use glam::Vec3 as GlamVec3;
use movement::RaycastVehicle;
use protocol::messages::{ClientMessage, Waypoint};
use protocol::scenario::{Embodiment, ScenarioConfig};
use server::scenario_state::ScenarioState;
use support::{scenario, Sim, TestAgent};

const TRACK: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../maps/testtrack.xodr");
/// Waypoint spacing along the lane (m), as an agent or the server's router
/// would sample it.
const SPACING: f32 = 4.0;

fn track_scenario() -> ScenarioConfig {
    let mut config = scenario(&["car"]);
    config.roster[0].embodiment = Embodiment::RaycastVehicle;
    config.map = Some(TRACK.to_string());
    config
}

/// Flat ground, and wide enough that the plans below finish well inside the
/// walls -- these are about the plan, not about the arena.
fn arena_scenario() -> ScenarioConfig {
    let mut config = scenario(&["car"]);
    config.roster[0].embodiment = Embodiment::RaycastVehicle;
    config.arena.width = 400.0;
    config.arena.depth = 400.0;
    config
}

/// A running sim with the car settled onto its suspension.
fn settled(config: ScenarioConfig) -> (Sim, TestAgent) {
    let mut sim = Sim::new(config);
    let agent = sim.join("car");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    sim.step(64);
    (sim, agent)
}

/// The speed profile a planner produces: cap each point by what its curvature
/// can physically hold, then walk backwards so the car can actually slow down
/// in time to obey the cap.
///
/// This belongs to the agent, not the server -- the server drives the speeds it
/// is given. It is here because asking for 25 m/s through a 25 m radius corner
/// is asking for 2.5 g, which no tire in this sim has; without a profile the
/// sweep below would be measuring grip, not tracking.
fn speed_profile(points: &[GlamVec3], cruise: f32, lateral: f32, braking: f32) -> Vec<f32> {
    let count = points.len();
    let mut speeds = vec![cruise; count];
    for i in 1..count - 1 {
        let (before, after) = (points[i] - points[i - 1], points[i + 1] - points[i]);
        let (back, ahead) = (before.length(), after.length());
        if back < 1e-3 || ahead < 1e-3 {
            continue;
        }
        // Curvature from the turn between adjacent segments, then v = sqrt(a/k).
        let turn = before.normalize().angle_between(after.normalize());
        let curvature = turn / (0.5 * (back + ahead));
        if curvature > 1e-5 {
            speeds[i] = speeds[i].min((lateral / curvature).sqrt());
        }
    }
    for i in (0..count - 1).rev() {
        let step = (points[i + 1] - points[i]).length();
        let reachable = (speeds[i + 1] * speeds[i + 1] + 2.0 * braking * step).sqrt();
        speeds[i] = speeds[i].min(reachable);
    }
    speeds
}

#[test]
fn the_car_holds_its_lane_at_every_speed_it_is_asked_to() {
    // The regression this tracker exists for. Aiming at the next *vertex* and
    // dropping it at 0.5 m collapsed the aim distance 4.0 -> 0.5 m within
    // every leg -- an eightfold swing in effective steering gain, spiking just
    // before each vertex. Below about 12 m/s the car corrected before the
    // spike mattered; above it, the car swerved out of the lane: 15 m/s left
    // the road at 832 m, 18 at 640 m, 25 at 629 m.
    //
    // Half the tires' 1.0 g goes to the profile, which leaves the tracker
    // margin for grade, camber and its own error -- so what is measured here
    // is tracking, not grip.
    const LATERAL_LIMIT: f32 = 4.9;
    const BRAKING_LIMIT: f32 = 3.0;

    let net = server::load_map(Some(TRACK))
        .expect("the track loads")
        .expect("the track is a road");
    let half_track = RaycastVehicle::default().half_track;

    for cruise in [12.0f32, 15.0, 18.0, 25.0] {
        let (mut sim, agent) = settled(track_scenario());
        let (lane_id, start) = net
            .nearest_lane(sim.position_of("car"))
            .expect("a lane under the car");
        let lane = net.lane(lane_id).expect("the lane it just reported");
        let half_width = lane.width * 0.5;

        // Stop short of the end: the tarmac simply stops there, and driving off
        // the end of it is not what this is measuring.
        let finish = lane.center.length() - 20.0;
        let mut points = Vec::new();
        let mut s = start.s;
        while s < finish {
            s += SPACING;
            points.push(lane.center.point_at(s.min(finish)));
        }
        assert!(points.len() > 200, "the track plan is suspiciously short");

        let speeds = speed_profile(&points, cruise, LATERAL_LIMIT, BRAKING_LIMIT);
        let waypoints: Vec<Waypoint> = points
            .iter()
            .zip(&speeds)
            .map(|(p, speed)| Waypoint {
                position: protocol::Vec3::new(p.x, p.y, p.z),
                speed: *speed,
            })
            .collect();
        agent.send(ClientMessage::SubmitPlan { waypoints });
        sim.expect("the plan to land", |sim| {
            (sim.plan_version("car") == 1).then_some(())
        });

        let (mut worst, mut worst_at, mut reached, mut top) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for tick in 0..40_000 {
            sim.step_quiet(1);
            assert!(
                sim.entity_of("car").is_some(),
                "cruise {cruise}: the car left the world at tick {tick}, \
                 {reached:.0} m along the track"
            );
            let projection = lane.center.project(sim.position_of("car"));
            reached = reached.max(projection.s);
            top = top.max(sim.component::<Velocity>("car").linear.length());
            if projection.offset.abs() > worst {
                worst = projection.offset.abs();
                worst_at = projection.s;
            }
            assert!(
                worst + half_track <= half_width,
                "cruise {cruise}: a wheel left the lane {worst_at:.0} m along -- \
                 centre {worst:.2} m off the centreline of a {:.1} m lane",
                lane.width
            );
            if sim.plan_waypoints("car").is_empty() {
                break;
            }
        }
        assert!(
            reached > finish - SPACING * 2.0,
            "cruise {cruise}: only got {reached:.0} m along a {finish:.0} m plan"
        );
        println!(
            "cruise {cruise:5.1} -> top {top:5.2} m/s, drove {reached:.0} m, \
             worst cross-track {worst:.2} m at {worst_at:.0} m"
        );
    }
}

#[test]
fn the_plan_the_agent_submitted_is_not_consumed_as_it_is_driven() {
    // Tracking measures progress rather than eating the plan, so an agent's
    // path stays exactly as it sent it -- there is no server-side remainder
    // for a re-plan to have to reconcile with. What *is* observable is how
    // much of it is left, and that the whole thing is dropped once driven.
    let (mut sim, agent) = settled(arena_scenario());
    let start = sim.position_of("car");
    let count = 20;
    let waypoints: Vec<Waypoint> = (1..=count)
        .map(|i| Waypoint {
            position: protocol::Vec3::new(start.x + i as f32 * SPACING, 0.0, start.z),
            speed: 8.0,
        })
        .collect();
    agent.send(ClientMessage::SubmitPlan { waypoints });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });

    // Drive past a good stretch of it.
    sim.expect("the car to pass the fifth waypoint", |sim| {
        (sim.position_of("car").x - start.x > SPACING * 5.0).then_some(())
    });
    assert_eq!(
        sim.plan_waypoints("car").len(),
        count,
        "the server consumed the agent's plan as it drove"
    );
    let left = sim.plan_remaining("car");
    assert!(
        left < count && left > 0,
        "{left} of {count} waypoints still ahead after driving past five"
    );

    // And the plan still ends: once the path is driven it is dropped whole,
    // which is what "the plan ran out" looks like to an agent.
    sim.expect("the plan to run out", |sim| {
        sim.plan_waypoints("car").is_empty().then_some(())
    });
    let travelled = sim.position_of("car").x - start.x;
    assert!(
        travelled > SPACING * count as f32 - 2.0,
        "the plan was dropped {travelled:.1} m in, short of its {:.0} m end",
        SPACING * count as f32
    );
}

#[test]
fn a_new_plan_discards_the_progress_measured_against_the_old_one() {
    // The one place re-planning and tracking genuinely interact. Progress is
    // an arc length along *a* plan; carried onto a different one it is
    // meaningless, and a shorter replacement would read as already finished --
    // the car would drop the new plan on the tick it arrived and coast to a
    // stop. So the stamp on the progress is checked, and stale progress
    // discarded.
    let (mut sim, agent) = settled(arena_scenario());
    let start = sim.position_of("car");
    let far: Vec<Waypoint> = (1..=25)
        .map(|i| Waypoint {
            position: protocol::Vec3::new(start.x + i as f32 * SPACING, 0.0, start.z),
            speed: 8.0,
        })
        .collect();
    agent.send(ClientMessage::SubmitPlan { waypoints: far });
    sim.expect("the first plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });
    sim.expect("the car to get 30 m along it", |sim| {
        (sim.position_of("car").x - start.x > 30.0).then_some(())
    });

    // A replacement that is *shorter* than the progress already made: 16 m of
    // path, against 30 m of arc length measured on the old one.
    let here = sim.position_of("car");
    let replacement: Vec<Waypoint> = (1..=4)
        .map(|i| Waypoint {
            position: protocol::Vec3::new(here.x + i as f32 * SPACING, 0.0, here.z),
            speed: 8.0,
        })
        .collect();
    agent.send(ClientMessage::SubmitPlan {
        waypoints: replacement,
    });
    sim.expect("the replacement to land", |sim| {
        (sim.plan_version("car") == 2).then_some(())
    });

    sim.step(16);
    assert!(
        !sim.plan_waypoints("car").is_empty(),
        "the new plan was dropped on arrival: progress from the old one was \
         carried over onto it"
    );
    assert_eq!(sim.plan_waypoints("car").len(), 4);
    // And it is driven, not merely held.
    sim.expect("the car to finish the replacement", |sim| {
        sim.plan_waypoints("car").is_empty().then_some(())
    });
    assert!(
        sim.position_of("car").x - here.x > 12.0,
        "only covered {:.1} m of a 16 m replacement",
        sim.position_of("car").x - here.x
    );
}
