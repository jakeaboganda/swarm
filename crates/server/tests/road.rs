//! Driving the built-in `demo` road: a 40 m straight into a 90-degree
//! left-hander of 30 m radius, climbing at a constant 4% grade.
//!
//! The arena tests (`wheels.rs`) run on flat ground with no curvature; these
//! are the ones that can catch a path tracker or a steering law that only
//! works in a straight line.

mod support;

use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use map::{Lane, RoadNetwork};
use movement::{RaycastVehicle, Wheels};
use protocol::messages::{ClientMessage, Waypoint};
use protocol::scenario::{Embodiment, ScenarioConfig};
use server::scenario_state::ScenarioState;
use support::{scenario, Sim, TestAgent};

/// Waypoint spacing along the lane (m).
const SPACING: f32 = 4.0;

fn road_scenario() -> ScenarioConfig {
    let mut config = scenario(&["car"]);
    config.roster[0].embodiment = Embodiment::RaycastVehicle;
    config.map = Some("demo".into());
    config
}

/// A running sim on the demo road, with the car settled onto its suspension,
/// plus the map the agent would have been handed at join.
fn car_on_the_demo_road() -> (Sim, TestAgent, RoadNetwork) {
    let mut sim = Sim::new(road_scenario());
    let agent = sim.join("car");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    sim.step(64);
    let net = server::load_map(Some("demo"))
        .expect("the demo map loads")
        .expect("the demo map is a road");
    (sim, agent, net)
}

/// The lane-following plan an agent lays from the map it is handed at join:
/// find the lane under the car, then sample its centreline ahead, stopping
/// `short_of` metres before the lane's end.
fn lane_plan(net: &RoadNetwork, from: Vec3, short_of: f32, speed: f32) -> (&Lane, Vec<Waypoint>) {
    let (lane_id, projection) = net.nearest_lane(from).expect("a lane under the car");
    let lane = net.lane(lane_id).expect("the lane it just reported");
    let finish = lane.center.length() - short_of;
    let mut waypoints = Vec::new();
    let mut s = projection.s;
    while s < finish {
        s += SPACING;
        let p = lane.center.point_at(s.min(finish));
        waypoints.push(Waypoint {
            position: protocol::Vec3::new(p.x, p.y, p.z),
            speed,
        });
    }
    (lane, waypoints)
}

/// Half the car's track: how far its outermost wheel sits from the chassis
/// centre, and so how much lane the chassis has to leave spare on each side.
fn half_track() -> f32 {
    RaycastVehicle::default().half_track
}

#[test]
fn the_car_keeps_its_lane_all_the_way_round_the_corner() {
    // 90 degrees of 30 m radius at 8 m/s asks for v^2/r = 2.1 m/s^2 of lateral
    // grip, a fifth of what the tires have -- so this is a tracking test, not a
    // grip test. Anything that runs wide here is the steering law or the path
    // tracker, not the tires.
    let (mut sim, agent, net) = car_on_the_demo_road();
    let (lane, waypoints) = lane_plan(&net, sim.position_of("car"), 0.0, 8.0);
    let half_width = lane.width * 0.5;
    let end = lane.center.length();
    assert!(!waypoints.is_empty(), "the plan is empty");
    agent.send(ClientMessage::SubmitPlan { waypoints });
    // Wait for it to land. The plan crosses a socket, so it is not in the ECS
    // on the next step -- and "the plan is empty" reads true before it has
    // ever been full, which would end the run below on tick zero.
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });

    let mut worst = 0.0f32;
    let mut reached = 0.0f32;
    for tick in 0..1400 {
        sim.step(1);
        assert!(
            sim.entity_of("car").is_some(),
            "the car left the world at tick {tick}"
        );
        let projection = lane.center.project(sim.position_of("car"));
        worst = worst.max(projection.offset.abs());
        reached = reached.max(projection.s);
        if sim.plan_waypoints("car").is_empty() {
            break;
        }
    }

    assert!(
        reached > end - SPACING,
        "only got {reached:.1} m along a {end:.1} m lane"
    );
    // A fifth of a lane half-width, i.e. 35 cm on a 3.5 m lane. Generous: the
    // tracker holds a few centimetres. The claim being pinned is that the car
    // follows the arc, not that it follows it to any particular precision.
    assert!(
        worst < half_width * 0.2,
        "ran wide through the corner: {worst:.2} m off a {:.1} m lane's centreline",
        lane.width
    );
    assert!(
        worst + half_track() <= half_width,
        "a wheel left the lane surface: centre {worst:.2} m off the centreline"
    );
}

#[test]
fn the_car_stays_on_the_road_when_its_plan_ends_mid_corner() {
    // The plan deliberately stops 18 m short of the lane's end, which leaves
    // the car braking from cruising speed with about 30 degrees of the arc
    // still to run. Braking is proportional -- an exponential decay with no
    // finite stopping distance -- so the stop takes the better part of ten
    // metres, and every one of them has to follow the bend. Straighten the
    // wheel when the plan empties and the car spends that distance leaving the
    // corner tangentially, off the outside edge, and falls off the world.
    let (mut sim, agent, net) = car_on_the_demo_road();
    let (lane, waypoints) = lane_plan(&net, sim.position_of("car"), 18.0, 8.0);
    let half_width = lane.width * 0.5;
    assert!(!waypoints.is_empty(), "the plan is empty");
    agent.send(ClientMessage::SubmitPlan { waypoints });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });

    let started_at = lane.center.project(sim.position_of("car")).s;
    let (mut worst, mut slowest) = (0.0f32, f32::INFINITY);
    let mut plan_ended = false;
    for tick in 0..1600 {
        sim.step(1);
        assert!(
            sim.entity_of("car").is_some(),
            "the car left the world at tick {tick}"
        );
        let offset = lane.center.project(sim.position_of("car")).offset.abs();
        worst = worst.max(offset);
        assert!(
            offset + half_track() <= half_width,
            "tick {tick}: a wheel left the lane surface -- centre {offset:.2} m off \
             the centreline of a {:.1} m lane",
            lane.width
        );
        plan_ended |= sim.plan_waypoints("car").is_empty();
        if plan_ended {
            let velocity: Velocity = sim.component("car");
            slowest = slowest.min(velocity.linear.length());
        }
    }

    assert!(plan_ended, "the plan never ran out");
    // Without this the test passes for a car that never moved: the plan would
    // read empty, the offset would stay at zero, and it would have stopped
    // because it never started.
    let travelled = lane.center.project(sim.position_of("car")).s - started_at;
    assert!(
        travelled > 40.0,
        "the car only covered {travelled:.1} m before stopping"
    );
    // It actually came to rest, rather than coasting on round the corner.
    assert!(
        slowest < 0.2,
        "never stopped: slowest was {slowest:.2} m/s after the plan ended"
    );
    let wheels: Wheels = sim.component("car");
    assert!(
        wheels.all_planted(),
        "a wheel is off the road ({worst:.2} m worst offset): {wheels:?}"
    );
}
