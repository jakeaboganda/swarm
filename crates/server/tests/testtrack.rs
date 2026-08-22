//! Driving `maps/testtrack.xodr` end to end.
//!
//! The track exists so wheel and tire behaviour is exercised deliberately --
//! launch, a hard stop, a crest and a dip, two corners of different radius,
//! and a decreasing-radius spiral. `clients/python/wheels_demo.py` drives it
//! to a fixed script for tuning by eye; this drives the same road to check the
//! car survives it at all, which is the part worth having a machine watch.
//!
//! Deliberately *not* a handling-quality test. It asserts the car stays on the
//! road and reaches the end, not that it does so gracefully -- tuning comes
//! later, and an assertion on how it feels would go yellow the first time a
//! tire constant moves.

mod support;

use bevy::prelude::*;
use map::RoadNetwork;
use movement::{RaycastVehicle, Wheels};
use protocol::messages::{ClientMessage, Waypoint};
use protocol::scenario::{Embodiment, ScenarioConfig};
use server::scenario_state::ScenarioState;
use support::{scenario, Sim, TestAgent};

/// Absolute, because `cargo test` runs with the crate root as its working
/// directory while the server runs from the workspace root.
const TRACK: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../maps/testtrack.xodr");
/// Waypoint spacing along the lane (m).
const SPACING: f32 = 4.0;

fn track_scenario() -> ScenarioConfig {
    let mut config = scenario(&["car"]);
    config.roster[0].embodiment = Embodiment::RaycastVehicle;
    config.map = Some(TRACK.to_string());
    config
}

fn car_on_the_track() -> (Sim, TestAgent, RoadNetwork) {
    let mut sim = Sim::new(track_scenario());
    let agent = sim.join("car");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    sim.step(64);
    let net = server::load_map(Some(TRACK))
        .expect("the track loads")
        .expect("the track is a road");
    (sim, agent, net)
}

#[test]
fn the_car_drives_the_whole_test_track_without_leaving_it() {
    let (mut sim, agent, net) = car_on_the_track();
    let here = sim.position_of("car");
    let (lane_id, projection) = net.nearest_lane(here).expect("a lane under the car");
    let lane = net.lane(lane_id).expect("the lane it just reported");
    let length = lane.center.length();
    let half_width = lane.width * 0.5;
    let half_track = RaycastVehicle::default().half_track;

    // Stop short of the end: the track simply stops there, and driving off the
    // end of the tarmac is not what this is measuring.
    let finish = length - 20.0;
    let mut waypoints = Vec::new();
    let mut s = projection.s;
    while s < finish {
        s += SPACING;
        let point = lane.center.point_at(s.min(finish));
        waypoints.push(Waypoint {
            position: protocol::Vec3::new(point.x, point.y, point.z),
            speed: 12.0,
        });
    }
    assert!(
        waypoints.len() > 200,
        "the track plan is suspiciously short"
    );
    agent.send(ClientMessage::SubmitPlan { waypoints });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });

    let (mut worst_offset, mut reached, mut max_roll) = (0.0f32, 0.0f32, 0.0f32);
    let mut airborne_ticks = 0usize;
    for tick in 0..20_000 {
        sim.step_quiet(1);
        assert!(
            sim.entity_of("car").is_some(),
            "the car left the world at tick {tick}, {reached:.0} m along the track"
        );
        let position = sim.position_of("car");
        let projection = lane.center.project(position);
        reached = reached.max(projection.s);
        worst_offset = worst_offset.max(projection.offset.abs());
        let transform: Transform = sim.component("car");
        max_roll = max_roll.max(transform.right().y.asin().to_degrees().abs());
        if !sim.component::<Wheels>("car").all_planted() {
            airborne_ticks += 1;
        }
        assert!(
            projection.offset.abs() + half_track <= half_width,
            "tick {tick}, {reached:.0} m in: a wheel left the lane -- centre \
             {:.2} m off the centreline of a {:.1} m lane",
            projection.offset.abs(),
            lane.width
        );
        if sim.plan_waypoints("car").is_empty() {
            break;
        }
    }

    assert!(
        reached > finish - SPACING * 2.0,
        "only got {reached:.0} m along a {length:.0} m track"
    );
    println!(
        "drove {reached:.0} m of {length:.0}: worst offset {worst_offset:.2} m, \
         max roll {max_roll:.1} deg, {airborne_ticks} ticks with a wheel off the ground"
    );
}
