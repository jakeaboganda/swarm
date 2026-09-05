//! The per-tick control loop's invariants -- the `CLAUDE.md` "implementation
//! guardrails", which until now were held only by prose. Each of these is a
//! correctness requirement that was found and fixed on paper; a regression in
//! any of them is silent without a test.

mod support;

use std::collections::HashMap;

use bevy::prelude::Vec3 as BevyVec3;
use bevy_rapier3d::prelude::{ExternalForce, Velocity};
use movement::{seek_force, DesiredVelocity, Holonomic};
use protocol::messages::{
    ClientMessage, Operator, ReflexAction, SensorKind, ServerMessage, Waypoint,
};
use protocol::scenario::{
    AgentSlot, ArenaConfig, Embodiment, FmuConfig, FmuFrame, FmuGround, FmuInputs, FmuOutputs,
    Pace, ScenarioConfig, TimeConfig,
};
use protocol::Vec3;
use server::scenario_state::ScenarioState;
use support::{rule, scenario, short_range_sensor, with_sensor, Sim};

fn waypoint(x: f32, z: f32, speed: f32) -> Waypoint {
    Waypoint {
        position: Vec3::new(x, 0.0, z),
        speed,
    }
}

/// A running one-agent sim with a plan already driving it.
fn driving(plan: Vec<Waypoint>) -> (Sim, support::TestAgent) {
    let mut sim = Sim::new(scenario(&["car-1"]));
    let agent = sim.join("car-1");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    agent.send(ClientMessage::SubmitPlan { waypoints: plan });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });
    (sim, agent)
}

#[test]
fn external_force_is_overwritten_not_accumulated_across_ticks() {
    // `ExternalForce` persists across ticks in bevy_rapier3d, so a `+=` here
    // compounds forever: by tick three the body is being shoved with three
    // times the force the controller asked for, and it only gets worse.
    let (mut sim, _agent) = driving(vec![waypoint(60.0, 0.0, 12.0)]);
    let model: Holonomic = sim.component("car-1");

    for step in 1..=3 {
        // The controller reads the body's velocity *before* the physics step,
        // so that is what the force it wrote must correspond to.
        let velocity: Velocity = sim.component("car-1");
        sim.step(1);
        let desired: DesiredVelocity = sim.component("car-1");
        let force: ExternalForce = sim.component("car-1");

        let single_step = seek_force(
            desired.value,
            velocity.linear,
            model.gain,
            if desired.urgent {
                model.brake_max_force
            } else {
                model.max_force
            },
        );
        assert!(
            (force.force - single_step).length() < 1e-3,
            "step {step}: force {:?} is not this step's force {single_step:?} \
             (accumulating would give roughly {step}x)",
            force.force
        );
        // The clamp in `seek_force` is what makes the ceiling a ceiling;
        // accumulation escapes it because the sum is never re-clamped.
        assert!(force.force.length() <= model.brake_max_force + 1e-3);
    }
}

#[test]
fn a_plan_submitted_this_tick_steers_the_body_on_this_tick() {
    // The observable form of "drain -> arbitrate -> ApplyForce, in that order,
    // inside one fixed step". If `drain_transport` ran after arbitration, a
    // plan would always take effect a tick late.
    let mut sim = Sim::new(scenario(&["car-1"]));
    let agent = sim.join("car-1");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    let before: DesiredVelocity = sim.component("car-1");
    assert_eq!(
        before.value,
        BevyVec3::ZERO,
        "idle agent should not be steering"
    );

    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(50.0, 0.0, 10.0)],
    });
    // Returns on the very step that drained the plan.
    sim.expect("the plan to be drained", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });

    let desired: DesiredVelocity = sim.component("car-1");
    let force: ExternalForce = sim.component("car-1");
    assert!(
        desired.value.length() > 0.0,
        "the plan drained this step but nothing steered toward it"
    );
    assert!(
        force.force.length() > 0.0,
        "the plan steered this step but no force was applied for it"
    );
}

#[test]
fn a_simulated_reflex_reads_the_set_delivered_this_frame() {
    // car-2 sits in car-1's path. car-1's only device is short-sighted, so
    // ground truth would have it braking long before the device can see
    // anything. The rule must follow the device, not the truth.
    let config = with_sensor(
        scenario(&["car-1", "car-2"]),
        short_range_sensor("myopic", 2.0, 0),
    );
    let mut sim = Sim::new(config);
    let first = sim.join("car-1");
    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    first.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "myopic",
            SensorKind::TimeToCollision,
            Operator::LessThan,
            2.0,
            ReflexAction::Brake,
        )],
    });
    // Drive car-1 straight at car-2 (the roster spawns them 3m apart on X).
    first.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(30.0, 0.0, 6.0)],
    });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });

    // While the device delivers nothing, the rule must stay quiet however
    // close the truth says the other car is.
    let mut saw_empty_delivery = false;
    let fired_with = sim.expect("the myopic reflex to fire", |sim| {
        let delivered = sim.delivered("car-1", "myopic");
        let urgent = sim.component::<DesiredVelocity>("car-1").urgent;
        if delivered.is_empty() {
            saw_empty_delivery = true;
            assert!(!urgent, "a reflex fired on a device that delivered nothing");
            return None;
        }
        urgent.then_some(delivered.len())
    });

    assert!(
        saw_empty_delivery,
        "the device saw car-2 immediately; the test never exercised the gap"
    );
    assert!(fired_with > 0);
    // And the truth was there to be seen the whole time -- the device, not the
    // world, is what held the rule back.
    assert!(sim.entity_of("car-2").is_some());
}

#[test]
fn a_simulated_reflex_and_the_4002_stream_see_an_identical_set() {
    // `PerceivedWorlds` is the single source both read, so they can never
    // disagree -- a doc claim with real weight: an agent debugging its own
    // reflexes off the :4002 stream is entitled to assume it sees exactly what
    // the server-side rule saw, on the tick the rule saw it.
    let config = with_sensor(
        scenario(&["car-1", "car-2"]),
        short_range_sensor("radar", 50.0, 0),
    );
    let mut sim = Sim::new(config);
    let first = sim.join("car-1");
    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    let stream = sim.watch_perception("car-1");
    // Keep both cars moving, so the perceived set actually changes tick to
    // tick and a stale comparison would show up.
    first.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(0.0, 40.0, 6.0)],
    });

    // What the reflex layer held, tick by tick, as the run went along, keyed
    // by the stamp the matching frame carries.
    //
    // That stamp is one behind the `Tick` value visible after the step:
    // `route_perception` is ordered before `arbitrate`, and `arbitrate` is
    // what advances the counter. So the frame a reflex reads on the step that
    // ends at tick N went out stamped N-1. The contents are the same object --
    // which is what this test is about -- but the two numbers an agent would
    // correlate on differ by one.
    let mut held_at: HashMap<u64, Vec<sensors::Detection>> = HashMap::new();
    for _ in 0..80 {
        sim.step(1);
        held_at.insert(sim.tick() - 1, sim.delivered("car-1", "radar"));
    }

    let frames = stream.drain();
    assert!(
        !frames.is_empty(),
        "the agent received no perception at all"
    );
    let mut compared = 0;
    for frame in &frames {
        let Some(held) = held_at.get(&frame.tick) else {
            continue; // A frame from before the recording window.
        };
        assert_eq!(
            frame.detections.len(),
            held.len(),
            "tick {}: the stream and the reflex layer disagree on how much \
             car-1 perceives",
            frame.tick
        );
        for (wire, held) in frame.detections.iter().zip(held.iter()) {
            assert_eq!(wire.id, held.id);
            assert_eq!(
                (wire.position.x, wire.position.y, wire.position.z),
                (held.position.x, held.position.y, held.position.z),
                "tick {}: detection {} differs between the two views",
                frame.tick,
                wire.id
            );
            assert_eq!(wire.radius, held.radius);
        }
        compared += 1;
    }
    assert!(
        compared >= 10,
        "only {compared} frames lined up with a recorded tick; the test \
         proved almost nothing"
    );
}

#[test]
fn a_reflex_brake_uses_the_brake_force_ceiling_not_the_cruise_one() {
    // "Brake as fast as possible" must not be limited by the ceiling tuned
    // for smooth cornering. `movement` tests that the two ceilings exist and
    // differ; this pins that a *reflex* actually selects the higher one.
    let (mut sim, agent) = driving(vec![waypoint(200.0, 0.0, 20.0)]);
    let model: Holonomic = sim.component("car-1");
    assert!(model.brake_max_force > model.max_force);

    agent.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "ground_truth",
            SensorKind::Speed,
            Operator::GreaterThan,
            5.0,
            ReflexAction::Brake,
        )],
    });

    let force = sim.expect("the brake reflex to fire above the cruise ceiling", |sim| {
        let desired: DesiredVelocity = sim.component("car-1");
        let force: ExternalForce = sim.component("car-1");
        (desired.urgent && force.force.length() > model.max_force + 1e-3)
            .then_some(force.force.length())
    });
    assert!(
        force <= model.brake_max_force + 1e-3,
        "braking force {force} exceeded even the brake ceiling"
    );
}

#[test]
fn stop_and_hold_clears_the_plan_exactly_once() {
    let (mut sim, agent) = driving(vec![
        waypoint(200.0, 0.0, 20.0),
        waypoint(200.0, 50.0, 20.0),
        waypoint(0.0, 50.0, 20.0),
    ]);
    assert_eq!(sim.plan_waypoints("car-1").len(), 3);

    agent.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "ground_truth",
            SensorKind::Speed,
            Operator::GreaterThan,
            5.0,
            ReflexAction::StopAndHold,
        )],
    });

    sim.expect("stop_and_hold to clear the plan", |sim| {
        sim.plan_waypoints("car-1").is_empty().then_some(())
    });
    // Dropping the plan is not a new plan: the version an agent watches must
    // not move, or it would read as "the server accepted a submission".
    assert_eq!(sim.plan_version("car-1"), 1);

    // It stays cleared -- a re-fire has nothing left to clear and must not
    // resurrect or re-version anything.
    sim.step(30);
    assert!(sim.plan_waypoints("car-1").is_empty());
    assert_eq!(sim.plan_version("car-1"), 1);
}

#[test]
fn a_plan_submitted_after_stop_and_hold_takes_effect() {
    // The recovery path: a held agent is not a dead one.
    let (mut sim, agent) = driving(vec![waypoint(200.0, 0.0, 20.0)]);
    agent.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "ground_truth",
            SensorKind::Speed,
            Operator::GreaterThan,
            5.0,
            ReflexAction::StopAndHold,
        )],
    });
    sim.expect("stop_and_hold to clear the plan", |sim| {
        sim.plan_waypoints("car-1").is_empty().then_some(())
    });

    // Let it come to rest so the rule clears (past the hysteresis margin).
    sim.expect("the reflex to release", |sim| {
        (!sim.component::<DesiredVelocity>("car-1").urgent).then_some(())
    });

    // Deregister the rule, then drive again -- and actually move.
    agent.send(ClientMessage::RegisterReflexes { rules: vec![] });
    let start = sim.position_of("car-1");
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(start.x, start.z + 40.0, 8.0)],
    });
    sim.expect("the new plan to land", |sim| {
        (sim.plan_version("car-1") == 2).then_some(())
    });
    sim.expect("the agent to drive on the new plan", |sim| {
        ((sim.position_of("car-1") - start).length() > 2.0).then_some(())
    });
}

#[test]
fn reflex_fired_carries_the_tick_and_plan_version_of_the_firing_step() {
    // An agent uses these to tell whether an event refers to the plan it just
    // submitted or one already superseded; a stale tick makes that impossible.
    let (mut sim, agent) = driving(vec![waypoint(200.0, 0.0, 20.0)]);
    agent.send(ClientMessage::RegisterReflexes {
        rules: vec![rule(
            "ground_truth",
            SensorKind::Speed,
            Operator::GreaterThan,
            5.0,
            ReflexAction::Brake,
        )],
    });

    // Server-side, the step on which the reflex first took over.
    sim.expect("the reflex to fire", |sim| {
        sim.component::<DesiredVelocity>("car-1")
            .urgent
            .then_some(())
    });
    let firing_tick = sim.tick();
    let plan_version = sim.plan_version("car-1");

    // The first `reflex_fired` on the wire is that step's.
    let (tick, version, action) =
        sim.expect_message(&agent, "a reflex_fired event", |message| match message {
            ServerMessage::ReflexFired {
                tick,
                plan_version,
                action,
            } => Some((*tick, *plan_version, *action)),
            _ => None,
        });
    assert_eq!(tick, firing_tick);
    assert_eq!(version, plan_version);
    assert_eq!(action, ReflexAction::Brake);
}

/// A one-slot scenario whose single agent is an `FmuVehicle` bound to `fmu_path`.
/// The binding names are placeholders -- these tests only exercise the load/bind
/// failure path, which never gets far enough to touch them.
fn fmu_scenario(name: &str, fmu_path: &str) -> ScenarioConfig {
    ScenarioConfig {
        arena: ArenaConfig {
            width: 100.0,
            depth: 100.0,
        },
        roster: vec![AgentSlot {
            name: name.to_string(),
            embodiment: Embodiment::FmuVehicle,
            sensors: vec![],
            color: None,
            scale: None,
            fmu: Some(FmuConfig {
                path: fmu_path.to_string(),
                inputs: FmuInputs {
                    steer: "steer".into(),
                    throttle: "throttle".into(),
                    brake: "brake".into(),
                    bank: None,
                },
                ground: FmuGround {
                    height: "z_road".into(),
                    normal_z: None,
                    friction: None,
                },
                outputs: FmuOutputs {
                    x: "x".into(),
                    y: "y".into(),
                    z: "z".into(),
                    yaw: "yaw".into(),
                    roll: None,
                    pitch: None,
                },
                frame: FmuFrame::SimYUp,
            }),
        }],
        seed: 0,
        map: None,
        time: TimeConfig {
            duration: None,
            pace: Pace::Afap,
        },
    }
}

/// The committed Open-Car-Dynamics FMU fixture (a real vehicle-dynamics model
/// wrapped as FMI 3.0 CS). Absolute path from this crate's manifest dir.
const OCD_FMU: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../fixtures/opencardynamics-fmu/opencardynamics.fmu"
);

/// A two-slot scenario: a plain holonomic "anchor" (so the OCD car lands at
/// roster index 1, i.e. a NONZERO spawn x -- see `spawn_position` -- which is
/// what makes the tick-1-teleport-to-origin regression distinguishable from a
/// correct spawn-rebase; a single-slot roster spawns at x=0 either way) plus
/// the real OCD FMU car. The binding uses OCD's actual variable names;
/// `normal_z` is omitted (OCD, a planar model, has none); `frame` is
/// `ocd_z_up` (OCD's x-fwd/y-left/z-up convention -- see `dynamics_fmi::
/// FmuFrame`).
fn ocd_scenario(anchor_name: &str, car_name: &str) -> ScenarioConfig {
    ScenarioConfig {
        arena: ArenaConfig {
            width: 200.0,
            depth: 200.0,
        },
        roster: vec![
            AgentSlot {
                name: anchor_name.to_string(),
                embodiment: Embodiment::Holonomic,
                sensors: vec![],
                color: None,
                scale: None,
                fmu: None,
            },
            AgentSlot {
                name: car_name.to_string(),
                embodiment: Embodiment::FmuVehicle,
                sensors: vec![],
                color: None,
                scale: None,
                fmu: Some(FmuConfig {
                    path: OCD_FMU.to_string(),
                    inputs: FmuInputs {
                        steer: "steer".into(),
                        throttle: "throttle".into(),
                        brake: "brake".into(),
                        bank: None,
                    },
                    ground: FmuGround {
                        height: "ground_height".into(),
                        normal_z: None,
                        friction: Some("ground_friction".into()),
                    },
                    outputs: FmuOutputs {
                        x: "x".into(),
                        y: "y".into(),
                        z: "z".into(),
                        yaw: "yaw".into(),
                        roll: None,
                        pitch: None,
                    },
                    frame: FmuFrame::OcdZUp,
                }),
            },
        ],
        seed: 0,
        map: None,
        time: TimeConfig {
            duration: None,
            pace: Pace::Afap,
        },
    }
}

#[test]
fn an_fmu_vehicle_driven_by_open_car_dynamics_actually_moves() {
    // End-to-end, slice C: the real OCD FMU loads + binds at join (so `join`
    // gets `Joined`, not `Error`), starts at its LANE/ARENA SPAWN POSE (not
    // teleported to the FMU's own origin), and under a forward plan drives in
    // the plan's direction in SIM coordinates, staying near ground level.
    //
    // The car is roster slot 1 (a leading holonomic "anchor" fills slot 0), so
    // its spawn x is nonzero -- see `ocd_scenario`'s doc for why a single-slot
    // roster can't distinguish the tick-1-teleport-to-origin bug this test
    // guards against (a single car's arena spawn x/z is 0 too).
    let mut sim = Sim::new(ocd_scenario("anchor", "ocd-car"));
    let _anchor = sim.join("anchor");
    let agent = sim.join("ocd-car"); // asserts Joined -- the FMU loaded + bound
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    // The spawn pose `agent_spawn_transform` computed for this car (same
    // function, same arguments the real spawn path uses) -- our ground truth
    // for "did it start where it was placed", not the FMU's own origin.
    let spacing = 3.0;
    let index = 1usize;
    let total = 2usize;
    let offset = index as f32 - (total.saturating_sub(1) as f32) / 2.0;
    let base = BevyVec3::new(offset * spacing, server::world::AGENT_RADIUS * 2.0, 0.0);
    let expected_spawn =
        server::world::agent_spawn_transform(Embodiment::FmuVehicle, base, None, 1.0, index);
    let expected_forward = *expected_spawn.forward();

    // One tick to let the FMU stamp its first pose, before any plan is
    // submitted: this is the pure spawn-rebase check, uncontaminated by driven
    // motion.
    sim.step(1);
    let spawned = sim.position_of("ocd-car");
    eprintln!(
        "OCD spawn: expected={:?} actual={spawned:?}",
        expected_spawn.translation
    );
    assert!(
        (spawned - expected_spawn.translation).length() < 1.0,
        "car did not start at its spawn pose (expected {:?}, got {spawned:?}) -- \
         looks like the tick-1 teleport-to-FMU-origin regression",
        expected_spawn.translation
    );

    // Drive straight ahead, fast, so the longitudinal controller commands real
    // throttle from a standstill. The plan is in world/sim coordinates; the
    // car's spawn heading (whatever `agent_spawn_transform` gave it) points at
    // it directly, so "drives in the plan's direction" and "drives along its
    // own spawn-forward" are the same check here.
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(150.0, 0.0, 20.0)],
    });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("ocd-car") == 1).then_some(())
    });

    sim.step(60);
    let mid = sim.position_of("ocd-car");
    sim.step(60);
    let end = sim.position_of("ocd-car");

    let mid_d = (mid - spawned).length();
    let end_d = (end - spawned).length();
    // Progress specifically along the car's own spawn-forward direction (the
    // plan's direction) -- not just "moved somewhere", which a sideways or
    // backwards slide would also satisfy.
    let mid_forward = (mid - spawned).dot(expected_forward);
    let end_forward = (end - spawned).dot(expected_forward);
    eprintln!(
        "OCD drive: spawned={spawned:?} mid={mid:?} end={end:?} mid_d={mid_d} end_d={end_d} \
         mid_forward={mid_forward} end_forward={end_forward}"
    );

    assert!(
        end_d > 1.0,
        "OCD FMU car barely moved ({end_d} m) -- the model is not being driven"
    );
    assert!(
        end_d > mid_d,
        "displacement did not grow ({mid_d} -> {end_d}) -- car not accelerating forward"
    );
    // SIGN-DEPENDENT: relies on `frame::to_sim_local`'s provisional axis-remap
    // signs (OCD forward -> sim -Z) composing with the spawn heading to point
    // the same way the plan does. If the orchestrator's empirical drive shows
    // the car moving in the plan's OPPOSITE direction, flip the sign in
    // `to_sim_local` (not here) and this assertion should then pass unchanged;
    // if it instead needs a magnitude/threshold tweak, that's expected too.
    assert!(
        end_forward > 1.0,
        "car did not progress along its own forward direction ({end_forward} m) -- \
         it moved, but not toward the plan (sign-dependent on the OCD axis remap)"
    );
    assert!(
        end_forward > mid_forward,
        "forward progress did not grow ({mid_forward} -> {end_forward})"
    );
    // Sim-Y should stay near the spawn's ride height, not fly up/down: OCD's
    // SINGLE_TRACK model has no heave DOF (see the OCD-FMU README), so a
    // correct remap leaves world-Y essentially at the spawn height throughout.
    // A gross drift here would mean OCD's up or lateral axis is leaking into
    // sim-Y (the pre-slice-C symptom noted in the worklog).
    assert!(
        (mid.y - expected_spawn.translation.y).abs() < 2.0,
        "mid-drive sim-Y drifted from ride height: {} vs {}",
        mid.y,
        expected_spawn.translation.y
    );
    assert!(
        (end.y - expected_spawn.translation.y).abs() < 2.0,
        "end-drive sim-Y drifted from ride height: {} vs {}",
        end.y,
        expected_spawn.translation.y
    );
}

#[test]
fn a_bad_fmu_config_rejects_the_join_without_ending_the_scenario() {
    // An `FmuVehicle` slot whose `.fmu` cannot be loaded must fail the *join*
    // cleanly -- an `Error` to the agent, the slot left pending -- and must NOT
    // panic or end the scenario. A join failure is malformed input, not a
    // disconnect. (A non-vehicle FMU that fails to *bind* takes the same path;
    // both are unit-tested in `fmu_setup`. The happy path -- a real vehicle FMU
    // that loads + binds + drives -- is
    // `an_fmu_vehicle_driven_by_open_car_dynamics_actually_moves` above.)
    let mut sim = Sim::new(fmu_scenario("ghost", "does/not/exist.fmu"));
    let agent = sim.connect();
    agent.send(ClientMessage::Join {
        name: "ghost".into(),
    });

    sim.expect_message(&agent, "a clean FMU load error", |message| match message {
        ServerMessage::Error { message } => message.contains("ghost").then_some(()),
        _ => None,
    });

    // The slot is still unfilled: the scenario never started, and crucially
    // never ended over a config error.
    assert_eq!(sim.state(), ScenarioState::WaitingForRoster);
    // Stepping on does not panic and does not end the scenario.
    sim.step(5);
    assert_eq!(sim.state(), ScenarioState::WaitingForRoster);
}

/// The committed double-track OCD FMU (roll/pitch outputs + a bank input),
/// wrapped for the banked-track demo.
const OCD_DT_FMU: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../fixtures/opencardynamics-dt-fmu/opencardynamics_dt.fmu"
);

/// A banked-oval scenario with one FmuVehicle bound to the double-track OCD FMU,
/// binding its bank input + roll/pitch outputs so the car conforms to (and its
/// dynamics respond to) the canted road.
fn banked_ocd_scenario(name: &str) -> ScenarioConfig {
    ScenarioConfig {
        arena: ArenaConfig {
            width: 400.0,
            depth: 400.0,
        },
        roster: vec![AgentSlot {
            name: name.to_string(),
            embodiment: Embodiment::FmuVehicle,
            sensors: vec![],
            color: None,
            scale: None,
            fmu: Some(FmuConfig {
                path: OCD_DT_FMU.to_string(),
                inputs: FmuInputs {
                    steer: "steer".into(),
                    throttle: "throttle".into(),
                    brake: "brake".into(),
                    bank: Some("bank".into()),
                },
                ground: FmuGround {
                    height: "ground_height".into(),
                    normal_z: None,
                    friction: Some("ground_friction".into()),
                },
                outputs: FmuOutputs {
                    x: "x".into(),
                    y: "y".into(),
                    z: "z".into(),
                    yaw: "yaw".into(),
                    roll: Some("roll".into()),
                    pitch: Some("pitch".into()),
                },
                frame: FmuFrame::OcdZUp,
            }),
        }],
        seed: 0,
        map: Some("banked_oval".into()),
        time: TimeConfig {
            duration: None,
            pace: Pace::Afap,
        },
    }
}

#[test]
fn an_ocd_car_banks_on_the_canted_oval() {
    // End-to-end slice 3: the double-track OCD FMU drives the banked oval and the
    // road-conform drapes it onto the canted surface, so on a banked curve the
    // car's up-axis tilts off vertical (it leans into the bank). Sign-agnostic
    // (angle magnitude), so it holds regardless of the bank-force sign the
    // orchestrator settles.
    let mut sim = Sim::new(banked_ocd_scenario("ocd-car"));
    let agent = sim.join("ocd-car"); // FMU loads + binds (roll/pitch/bank too)
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    // Route a lap of the oval from the track centerline, so the car drives off
    // the start straight and into a banked curve.
    let track = map::banked_oval();
    let len = track.length();
    let n = 32u32;
    let waypoints: Vec<_> = (1..=n)
        .map(|i| {
            let p = track.sample_at(len * (i as f32) / (n as f32)).point;
            waypoint(p.x, p.z, 16.0)
        })
        .collect();
    agent.send(ClientMessage::SubmitPlan { waypoints });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("ocd-car") == 1).then_some(())
    });

    // On the start straight the car sits ~level; drive on and record the largest
    // lean (up-axis angle off vertical) as it reaches the banked curve.
    let start = sim.position_of("ocd-car");
    let level = {
        let up = sim.component::<bevy::prelude::Transform>("ocd-car").rotation * BevyVec3::Y;
        up.angle_between(BevyVec3::Y)
    };
    let mut max_tilt = 0f32;
    for _ in 0..600 {
        sim.step(1);
        let up = sim.component::<bevy::prelude::Transform>("ocd-car").rotation * BevyVec3::Y;
        max_tilt = max_tilt.max(up.angle_between(BevyVec3::Y));
    }
    let moved = (sim.position_of("ocd-car") - start).length();
    eprintln!("banked drive: level_tilt={level} moved={moved} max_tilt={max_tilt} rad");

    assert!(level < 0.03, "car was not level on the start straight ({level} rad)");
    assert!(moved > 20.0, "car did not drive the oval ({moved} m)");
    // The oval banks to ~0.21 rad; conform tilts the car to the surface, so on a
    // curve it leans well past level.
    assert!(
        max_tilt > 0.08,
        "car never banked (max tilt {max_tilt} rad) -- road-conform is not tilting it"
    );
}
