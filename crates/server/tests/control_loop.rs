//! The per-tick control loop's invariants — the `CLAUDE.md` "implementation
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
        // The clamp in `seek_force` is the whole point of a force ceiling;
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
