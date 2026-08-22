//! What the wheels do once a real car is driving on them.
//!
//! The arena's ground is a flat plane at y = 0, so these run on terrain with
//! no grade and no camber and every number below has an arithmetic answer.
//! Handling quality is not asserted anywhere here -- that is judged by driving
//! the car, not by a test that goes yellow the first time a tire constant
//! changes.

mod support;

use bevy_rapier3d::prelude::Velocity;
use movement::{RaycastVehicle, Wheels};
use protocol::messages::{ClientMessage, Operator, ReflexAction, SensorKind, Waypoint};
use protocol::scenario::{Embodiment, ScenarioConfig};
use protocol::Vec3;
use server::scenario_state::ScenarioState;
use server::world::CAR_MASS;
use support::{rule, scenario, Sim, TestAgent};

const GRAVITY: f32 = 9.81;

fn car_scenario() -> ScenarioConfig {
    let mut config = scenario(&["car"]);
    config.roster[0].embodiment = Embodiment::RaycastVehicle;
    config
}

/// A running sim with one car, settled onto its suspension.
fn settled_car() -> (Sim, TestAgent) {
    let mut sim = Sim::new(car_scenario());
    let agent = sim.join("car");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    // The suspension is damped to ~0.4 of critical at ~2 Hz, so three seconds
    // is several settling times.
    sim.step(192);
    (sim, agent)
}

fn waypoint(x: f32, z: f32, speed: f32) -> Waypoint {
    Waypoint {
        position: Vec3::new(x, 0.0, z),
        speed,
    }
}

#[test]
fn wheel_state_is_populated_once_the_car_is_running() {
    let (sim, _agent) = settled_car();
    let vehicle: RaycastVehicle = sim.component("car");
    let wheels: Wheels = sim.component("car");

    assert!(
        wheels.all_planted(),
        "a wheel is off flat ground: {wheels:?}"
    );
    for (index, wheel) in wheels.0.iter().enumerate() {
        assert!(wheel.load > 0.0, "wheel {index} carries no load");
        assert!(
            wheel.compression > 0.0 && wheel.compression < vehicle.suspension_rest,
            "wheel {index} compression {} is outside its {} m of travel",
            wheel.compression,
            vehicle.suspension_rest
        );
        assert!(wheel.load.is_finite() && wheel.omega.is_finite());
    }

    // The suspension is holding the car up: at rest the four springs together
    // carry its weight, and nothing else does.
    let weight = CAR_MASS * GRAVITY;
    let carried = wheels.total_load();
    assert!(
        (carried - weight).abs() / weight < 0.15,
        "the springs carry {carried} N of a {weight} N car"
    );

    // And it is resting, not still falling or bouncing.
    let velocity: Velocity = sim.component("car");
    assert!(
        velocity.linear.length() < 0.5,
        "the car never settled: still moving at {} m/s",
        velocity.linear.length()
    );
}

#[test]
fn a_rolling_wheel_turns_at_road_speed() {
    // Free rolling is slip ratio zero, which means omega * radius equals road
    // speed. If the spin integration were wrong in scale or sign, this is
    // where it shows: the wheels would be stationary, or spinning backwards,
    // under a car that is plainly moving.
    // Straight down +X, which is the direction the car spawns facing -- a
    // corner would have the wheels legitimately slipping, and this test is
    // about the cruising case.
    let (mut sim, agent) = settled_car();
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(120.0, 0.0, 8.0)],
    });

    sim.expect("the car to get up to speed", |sim| {
        (sim.component::<Velocity>("car").linear.length() > 7.0).then_some(())
    });
    let speed = sim.component::<Velocity>("car").linear.length();
    let vehicle: RaycastVehicle = sim.component("car");
    let wheels: Wheels = sim.component("car");
    // The rear pair does not steer, so its rolling direction is the body's.
    for index in [2, 3] {
        let wheel = wheels.0[index];
        let surface_speed = wheel.omega * vehicle.wheel_radius;
        assert!(
            (surface_speed - speed).abs() < 1.5,
            "wheel {index} turns at {surface_speed} m/s under a car doing {speed} m/s"
        );
        // Turning the right way, at the right order of magnitude: a spin
        // integration wrong in sign or scale gives a stationary or reversed
        // wheel under a plainly moving car.
        assert!(
            surface_speed > 0.5 * speed,
            "wheel {index} turns at {surface_speed} m/s under a car doing {speed} m/s"
        );
    }
}

#[test]
fn an_airborne_wheel_reports_no_contact_and_no_load() {
    let (mut sim, _agent) = settled_car();
    sim.lift("car", 20.0);
    sim.step(2);

    let wheels: Wheels = sim.component("car");
    for (index, wheel) in wheels.0.iter().enumerate() {
        assert!(!wheel.contact, "wheel {index} found ground 20 m up");
        assert_eq!(wheel.load, 0.0, "wheel {index} is loaded in mid-air");
        assert_eq!(wheel.compression, 0.0);
    }
    assert_eq!(wheels.total_load(), 0.0);
}

#[test]
fn a_reflex_brake_locks_the_wheels() {
    // The payoff of separating brake torque from drive: a hard stop stops the
    // wheels turning while the car is still moving, which is a locked slide.
    // A single signed torque would have driven them backwards instead.
    let (mut sim, agent) = settled_car();
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(160.0, 0.0, 12.0)],
    });
    sim.expect("the car to get up to speed", |sim| {
        (sim.component::<Velocity>("car").linear.length() > 8.0).then_some(())
    });

    agent.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "ground_truth",
            SensorKind::Speed,
            Operator::GreaterThan,
            2.0,
            ReflexAction::StopAndHold,
        )],
    });

    let (omega, slip, speed) = sim.expect("the wheels to lock", |sim| {
        let wheels: Wheels = sim.component("car");
        let speed = sim.component::<Velocity>("car").linear.length();
        let locked = wheels.0.iter().find(|w| w.omega == 0.0 && w.contact);
        locked.map(|w| (w.omega, w.slip_ratio, speed))
    });

    assert_eq!(omega, 0.0);
    assert!(
        speed > 1.0,
        "the car had already stopped ({speed} m/s); that is not a locked slide"
    );
    // A stopped wheel under a moving car is, by definition, fully locked.
    assert!(
        slip < -0.9,
        "a stopped wheel under a car doing {speed} m/s reported slip {slip}"
    );
}

#[test]
fn a_densely_sampled_plan_is_driven_at_the_speed_the_agent_asked_for() {
    // The plan's speed is the agent's to set -- `request_route` takes it and
    // stamps it on every waypoint precisely so the agent keeps that authority.
    // The router samples at 2 m, which is exactly `ARRIVE_RADIUS`, so applying
    // the arrive-slowdown at every waypoint left a routed car permanently
    // inside the ramp: it cruised at roughly two thirds of what it asked for
    // and never once reached it.
    //
    // Flat ground on purpose. The driver is proportional, so on a grade it has
    // a real steady-state offset (about 1.5 m/s on the demo road's 4%), and
    // that would confound what this measures. Here the only load is drag,
    // worth ~9%.
    const ASKED: f32 = 8.0;
    const SPACING: f32 = 2.0;
    let (mut sim, agent) = settled_car();
    let start = sim.position_of("car");

    // Straight down +X, the way the car already faces, sampled as densely as
    // the server's own router would.
    let waypoints = (1..=60)
        .map(|i| waypoint(start.x + i as f32 * SPACING, start.z, ASKED))
        .collect();
    agent.send(ClientMessage::SubmitPlan { waypoints });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car") == 1).then_some(())
    });

    let mut fastest = 0.0f32;
    for _ in 0..900 {
        sim.step(1);
        if sim.plan_waypoints("car").is_empty() {
            break;
        }
        fastest = fastest.max(sim.component::<Velocity>("car").linear.length());
    }
    assert!(
        fastest > ASKED * 0.85,
        "asked for {ASKED} m/s down a straight line and never got past {fastest:.2}"
    );
}
