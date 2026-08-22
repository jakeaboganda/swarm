//! Reproducibility: the same seed and the same inputs give the same run.
//!
//! Deliberately scoped to *this machine, this binary*. Rapier does not promise
//! cross-platform bit-identity and neither does the floating-point math above
//! it, so a test claiming more than same-binary reproducibility would flake
//! and then be ignored, which is worse than not having it.

mod support;

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use protocol::messages::{ClientMessage, Waypoint};
use protocol::scenario::{ScenarioConfig, SensorDef, SensorSource, SensorSpec};
use protocol::Vec3;
use server::scenario_state::ScenarioState;
use support::{scenario, Sim};

/// A device that perceives everything but reports it noisily, so the noise
/// stream itself is what the comparison is about.
fn noisy(name: &str) -> SensorDef {
    SensorDef {
        name: name.to_string(),
        source: SensorSource::Simulated,
        spec: Some(SensorSpec {
            position_noise: 1.0,
            velocity_noise: 1.0,
            ..Default::default()
        }),
    }
}

fn with_devices(mut config: ScenarioConfig, devices: &[SensorDef]) -> ScenarioConfig {
    for slot in &mut config.roster {
        slot.sensors = devices.to_vec();
    }
    config
}

/// Runs a two-agent scenario for `ticks` and returns every detection car-1
/// perceived through `device`, as a flat list of coordinates.
fn perceived_stream(config: ScenarioConfig, device: &str, ticks: usize) -> Vec<[f32; 3]> {
    let mut sim = Sim::new(config);
    let first = sim.join("car-1");
    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    // Keep it moving, so the perceived set is not one constant reading.
    first.send(ClientMessage::SubmitPlan {
        waypoints: vec![Waypoint {
            position: Vec3::new(0.0, 0.0, 30.0),
            speed: 5.0,
        }],
    });

    let mut stream = Vec::new();
    for _ in 0..ticks {
        sim.step(1);
        for detection in sim.delivered("car-1", device) {
            stream.push([
                detection.position.x,
                detection.position.y,
                detection.position.z,
            ]);
        }
    }
    assert!(!stream.is_empty(), "nothing was perceived through {device}");
    stream
}

#[test]
fn the_same_seed_reproduces_an_identical_perceived_world() {
    // The noise is seeded per (scenario_seed, agent, device, tick), so two runs
    // of the same scenario must impair perception identically -- otherwise a
    // scenario cannot be replayed and a perception bug cannot be reproduced.
    let config = || {
        let mut c = with_devices(scenario(&["car-1", "car-2"]), &[noisy("radar")]);
        c.seed = 20_260_821;
        c
    };
    let first = perceived_stream(config(), "radar", 60);
    let second = perceived_stream(config(), "radar", 60);
    assert_eq!(
        first, second,
        "the same seed produced a different noise stream"
    );

    // And a different seed genuinely changes it -- otherwise the equality above
    // would prove nothing but that the noise is switched off.
    let mut other = config();
    other.seed += 1;
    let different = perceived_stream(other, "radar", 60);
    assert_ne!(first, different, "the scenario seed had no effect");
}

#[test]
fn changing_only_the_device_name_changes_the_noise_stream() {
    // Two devices with identical specs on the same agent in the same run must
    // not share a noise stream: they are separate instruments, and correlated
    // errors would make a two-sensor agent look better than it is.
    let mut config = with_devices(
        scenario(&["car-1", "car-2"]),
        &[noisy("radar"), noisy("lidar")],
    );
    config.seed = 7;

    let mut sim = Sim::new(config);
    let first = sim.join("car-1");
    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    first.send(ClientMessage::SubmitPlan {
        waypoints: vec![Waypoint {
            position: Vec3::new(0.0, 0.0, 30.0),
            speed: 5.0,
        }],
    });

    let mut radar = Vec::new();
    let mut lidar = Vec::new();
    for _ in 0..60 {
        sim.step(1);
        for (device, into) in [("radar", &mut radar), ("lidar", &mut lidar)] {
            for detection in sim.delivered("car-1", device) {
                into.push([detection.position.x, detection.position.z]);
            }
        }
    }
    assert!(!radar.is_empty() && radar.len() == lidar.len());
    assert_ne!(
        radar, lidar,
        "two devices with the same spec drew the same noise"
    );
}

/// Hashes the position of every agent on every tick of a run.
fn position_trace(config: ScenarioConfig, ticks: usize) -> u64 {
    let mut sim = Sim::new(config);
    let agent = sim.join("car");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![
            Waypoint {
                position: Vec3::new(30.0, 0.0, 0.0),
                speed: 8.0,
            },
            Waypoint {
                position: Vec3::new(60.0, 0.0, -20.0),
                speed: 8.0,
            },
        ],
    });

    let mut hasher = DefaultHasher::new();
    let mut samples = 0;
    for _ in 0..ticks {
        sim.step(1);
        if sim.entity_of("car").is_none() {
            break; // drove off the road; the trace up to here still counts
        }
        let position = sim.position_of("car");
        position.x.to_bits().hash(&mut hasher);
        position.y.to_bits().hash(&mut hasher);
        position.z.to_bits().hash(&mut hasher);
        samples += 1;
    }
    assert!(samples > 100, "the run ended after only {samples} ticks");
    hasher.finish()
}

#[test]
fn an_afap_run_is_reproducible_across_two_runs_of_the_same_binary() {
    // The highest-value regression net a physics sim can have: one number that
    // moves the moment anything in the control loop, the integration order, or
    // the physics configuration changes. When it moves without an intended
    // change, something drifted.
    let road_car = || {
        let mut config = scenario(&["car"]);
        config.map = Some("demo".into());
        config.roster[0].embodiment = protocol::scenario::Embodiment::RaycastVehicle;
        config
    };
    let first = position_trace(road_car(), 640);
    let second = position_trace(road_car(), 640);
    assert_eq!(
        first, second,
        "two runs of the same scenario in the same binary diverged"
    );
}
