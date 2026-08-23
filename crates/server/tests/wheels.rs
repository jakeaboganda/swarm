//! What the wheels do once a real car is driving on them.
//!
//! The arena's ground is a flat plane at y = 0, so these run on terrain with
//! no grade and no camber and every number below has an arithmetic answer.
//! Handling quality is not asserted anywhere here -- that is judged by driving
//! the car, not by a test that goes yellow the first time a tire constant
//! changes.

mod support;

use bevy::math::{Quat, Vec3 as Vec3f};
use bevy::transform::components::Transform as Placement;
use bevy_rapier3d::prelude::Velocity;
use movement::{wheel_offset, RaycastVehicle, Wheels};
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
        // Wrapped, so a long drive doesn't lose visible precision in f32.
        assert!(
            (0.0..std::f32::consts::TAU).contains(&wheel.angle),
            "wheel {index} spin angle {} is not wrapped",
            wheel.angle
        );
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

/// A viz transform as Bevy maths, so a node's world pose can be composed the
/// way a viewer's scene graph composes it.
fn placement(transform: &viz::Transform) -> Placement {
    Placement {
        translation: Vec3f::new(
            transform.position.x,
            transform.position.y,
            transform.position.z,
        ),
        rotation: Quat::from_xyzw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
        scale: Vec3f::ONE,
    }
}

#[test]
fn wheel_state_reaches_a_viewer() {
    // The wire half of the wheel work. A viewer is told the car's node tree
    // once and, every frame, where each node is -- because it can derive none
    // of it. The claim is not just that the numbers arrive: it is that
    // composing them the way a scene graph does puts each wheel where the
    // physics has it, on the ground, turned the right way. The viewer no
    // longer composes anything, so this is now assertable end to end.
    let (mut sim, _agent) = settled_car();
    let viewer = sim.watch_viz(true);

    // The scene-init a viewer gets on connect carries the tree.
    let root = sim.expect_viz(&viewer, "the car's node tree", |message| match message {
        viz::ServerToViewer::SceneInit(init) => init
            .entities
            .iter()
            .find(|e| e.id.0 == "car")
            .map(|e| e.root.clone()),
        _ => None,
    });
    let vehicle: RaycastVehicle = sim.component("car");
    let names: Vec<&str> = root.children.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, viz::WHEEL_NODES.to_vec());
    // Every wheel attaches at the same height, so one is enough to bound them.
    let attach_y = wheel_offset(0, &vehicle).y;
    for child in &root.children {
        match child.geometry {
            Some(viz::Geometry::Cylinder { radius, height }) => {
                assert_eq!(
                    radius, vehicle.wheel_radius,
                    "{} is drawn a size the physics does not use",
                    child.name
                );
                assert!(height > 0.0, "{} has no width", child.name);
            }
            ref other => panic!("{} is not a cylinder: {other:?}", child.name),
        }
        // A settled car's descriptor carries its *current* suspension, not the
        // full extension it spawned at -- a viewer joining mid-scenario sees
        // the wheels where they are.
        assert!(
            child.transform.position.y > attach_y - vehicle.suspension_rest,
            "{} is drawn at full extension under a settled car",
            child.name
        );
    }

    // Every frame carries the nodes that moved: the body and four wheels.
    let nodes = sim.expect_viz(
        &viewer,
        "a frame with the car's nodes",
        |message| match message {
            viz::ServerToViewer::Frame(frame) => frame
                .entities
                .iter()
                .find(|e| e.id.0 == "car")
                .map(|e| e.nodes.clone())
                .filter(|nodes| nodes.len() > 1),
            _ => None,
        },
    );
    assert_eq!(nodes.len(), 5, "a car is a body node and four wheels");
    let body = nodes
        .iter()
        .find(|n| n.path.is_root())
        .map(|n| placement(&n.transform))
        .expect("the body's own node");
    let wheels: Wheels = sim.component("car");

    for (index, name) in viz::WHEEL_NODES.iter().enumerate() {
        let path = viz::NodePath::root().child(name);
        let local = nodes
            .iter()
            .find(|n| n.path == path)
            .map(|n| placement(&n.transform))
            .unwrap_or_else(|| panic!("no update for {name}"));

        // 1. Where the sim has it. The wheel hangs below its attach point by
        //    whatever suspension is left extended, so the wire and the
        //    suspension state have to agree to the millimetre.
        let attach = wheel_offset(index, &vehicle);
        let extension = vehicle.suspension_rest - wheels.0[index].compression;
        assert!(
            (local.translation.y - (attach.y - extension)).abs() < 0.01,
            "{name} is drawn at {} but its suspension puts it at {}",
            local.translation.y,
            attach.y - extension
        );
        assert!(
            extension > 0.0 && extension < vehicle.suspension_rest,
            "{name} is outside its {} m of travel",
            vehicle.suspension_rest
        );

        // 2. Composed the way a viewer composes it, the wheel is on the
        //    ground: its centre sits one radius above the arena floor at
        //    y = 0. This is the assertion the old rig could not make -- the
        //    viewer did the composing, so a rig missing its rest length drew
        //    the wheels a suspension-length in the air and nothing caught it.
        let world = body.mul_transform(local);
        assert!(
            (world.translation.y - vehicle.wheel_radius).abs() < 0.02,
            "{name} is drawn with its centre {} m up, not one {} m radius above the ground",
            world.translation.y,
            vehicle.wheel_radius
        );
        // And directly over the point the drive system casts from.
        let cast_from = body.transform_point(attach);
        assert!(
            (world.translation.x - cast_from.x).abs() < 0.01
                && (world.translation.z - cast_from.z).abs() < 0.01,
            "{name} is drawn at {:?}, not under the point the physics casts from {cast_from:?}",
            world.translation
        );

        // 3. And it is a wheel, not a drum standing on its end: its axle runs
        //    across the car, level with the ground.
        let axle = world.rotation * Vec3f::Y;
        assert!(
            axle.dot(*body.right()).abs() > 0.99,
            "{name}'s axle points {axle:?}, not across the body"
        );
    }

    // And the diagnostics ride the debug layer, not the scene layer.
    let debug = sim.expect_viz(
        &viewer,
        "a debug frame with wheels",
        |message| match message {
            viz::ServerToViewer::DebugFrame(frame) => frame
                .entities
                .iter()
                .find(|e| e.id.0 == "car")
                .map(|e| e.wheels.clone())
                .filter(|w| !w.is_empty()),
            _ => None,
        },
    );
    assert_eq!(debug.len(), 4);
    assert!(
        debug.iter().all(|w| w.contact),
        "a wheel reported no ground contact while parked on flat ground"
    );
}

#[test]
fn an_agent_without_wheels_sends_none_to_the_viewer() {
    // A holonomic puck is one node. It must have no children rather than four
    // degenerate ones, and its frames must carry its root alone -- or a viewer
    // would draw wheels on a puck.
    let mut sim = Sim::new(scenario(&["puck"]));
    let _agent = sim.join("puck");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    let viewer = sim.watch_viz(false);

    let root = sim.expect_viz(&viewer, "the puck's node tree", |message| match message {
        viz::ServerToViewer::SceneInit(init) => init
            .entities
            .iter()
            .find(|e| e.id.0 == "puck")
            .map(|e| e.root.clone()),
        _ => None,
    });
    assert!(
        root.children.is_empty(),
        "a holonomic puck was given child nodes: {:?}",
        root.children
    );
    assert!(matches!(root.geometry, Some(viz::Geometry::Capsule { .. })));

    let nodes = sim.expect_viz(&viewer, "a frame for the puck", |message| match message {
        viz::ServerToViewer::Frame(frame) => frame
            .entities
            .iter()
            .find(|e| e.id.0 == "puck")
            .map(|e| e.nodes.clone()),
        _ => None,
    });
    assert_eq!(nodes.len(), 1, "a wheelless agent sent more than its body");
    assert!(nodes[0].path.is_root());
}
