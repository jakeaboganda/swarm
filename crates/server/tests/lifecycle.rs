//! The scenario state machine, end to end over a real socket.
//!
//! This is the state machine with the largest blast radius in the project: a
//! bug here ends every agent's run at once, or fails to end one that should
//! be over. Each test drives the real `build_app` graph through the real
//! agent pathway.

mod support;

use std::time::Duration;

use protocol::messages::{ClientMessage, ServerMessage, Waypoint};
use protocol::Vec3;
use server::scenario_state::ScenarioState;
use server::transport_bridge::ReconnectGrace;
use support::{scenario, Sim};
use tokio_tungstenite::tungstenite::Message;

/// A grace window short enough to expire inside a test, still exercising the
/// real `expire_reconnects` path.
const SHORT_GRACE: Duration = Duration::from_millis(200);

fn waypoint(x: f32, speed: f32) -> Waypoint {
    Waypoint {
        position: Vec3::new(x, 0.0, 0.0),
        speed,
    }
}

fn error_message(message: &ServerMessage) -> Option<String> {
    match message {
        ServerMessage::Error { message } => Some(message.clone()),
        _ => None,
    }
}

#[test]
fn one_connection_cannot_fill_two_roster_slots() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let agent = sim.join("car-1");

    // The same socket now claims the second slot. Accepting it would orphan
    // the first entity -- still driving, still holding this live socket, but
    // no longer reachable by connection -- and start a two-agent scenario
    // with one client in it.
    agent.send(ClientMessage::Join {
        name: "car-2".into(),
    });
    let error = sim.expect_message(&agent, "the second join to be refused", error_message);
    assert!(
        error.contains("already controls an agent"),
        "unexpected refusal: {error}"
    );

    sim.step(10);
    assert_eq!(sim.state(), ScenarioState::WaitingForRoster);
    assert_eq!(sim.spawned_agents(), vec!["car-1".to_string()]);
}

#[test]
fn joining_an_already_filled_slot_errors_and_keeps_the_connection() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let _first = sim.join("car-1");

    let second = sim.connect();
    second.send(ClientMessage::Join {
        name: "car-1".into(),
    });
    let error = sim.expect_message(&second, "the taken slot to be refused", error_message);
    assert!(
        error.contains("not an unfilled roster slot"),
        "unexpected refusal: {error}"
    );

    // The connection stays open: it can still take the slot it is entitled to.
    second.send(ClientMessage::Join {
        name: "car-2".into(),
    });
    sim.expect_message(&second, "car-2 to be joined", |message| {
        matches!(message, ServerMessage::Joined { agent_id, .. } if agent_id.0 == "car-2")
            .then_some(())
    });
}

#[test]
fn the_scenario_starts_only_when_every_slot_is_filled() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let _first = sim.join("car-1");

    sim.step(10);
    assert_eq!(sim.state(), ScenarioState::WaitingForRoster);
    assert!(
        !sim.physics_active(),
        "physics runs before the roster fills"
    );

    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    assert!(sim.physics_active());
}

#[test]
fn a_disconnect_before_start_reopens_the_slot() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let first = sim.join("car-1");
    first.disconnect();

    // The orphaned entity is removed and the slot goes back on the pending
    // list, so a fresh client can take it.
    sim.expect("car-1's entity to be despawned", |sim| {
        sim.spawned_agents().is_empty().then_some(())
    });
    assert_eq!(sim.state(), ScenarioState::WaitingForRoster);

    let _replacement = sim.join("car-1");
    assert_eq!(sim.spawned_agents(), vec!["car-1".to_string()]);
}

#[test]
fn a_disconnect_while_running_opens_a_grace_window_rather_than_ending() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let first = sim.join("car-1");
    let second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    first.disconnect();

    // Well inside the (default, 8s) window: the run continues for everyone
    // else, and the dropped agent's entity keeps coasting on its last plan.
    sim.expect_silence(&second, 60);
    assert_eq!(sim.state(), ScenarioState::Running);
    assert!(sim.physics_active());
    assert!(sim.entity_of("car-1").is_some());
}

#[test]
fn a_reconnect_inside_the_window_resumes_the_same_entity_and_plan() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let first = sim.join("car-1");
    let _second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    first.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(20.0, 3.0)],
    });
    sim.expect("the plan to land", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });
    let entity = sim.entity_of("car-1").expect("car-1 has an entity");
    let plan = sim.plan_waypoints("car-1");

    first.disconnect();
    sim.step(5);

    let reconnected = sim.connect();
    reconnected.send(ClientMessage::Join {
        name: "car-1".into(),
    });
    sim.expect_message(&reconnected, "the reconnect to be accepted", |message| {
        matches!(message, ServerMessage::Joined { agent_id, .. } if agent_id.0 == "car-1")
            .then_some(())
    });

    // Same entity, same plan: a reconnect resumes a run, it does not restart
    // the agent.
    assert_eq!(sim.entity_of("car-1"), Some(entity));
    assert_eq!(sim.plan_version("car-1"), 1);
    assert_eq!(sim.plan_waypoints("car-1"), plan);

    // And the scenario survives the window it would otherwise have expired in.
    sim.step(30);
    assert_eq!(sim.state(), ScenarioState::Running);
}

#[test]
fn an_expired_grace_window_ends_the_scenario_and_freezes_physics() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    sim.app.insert_resource(ReconnectGrace(SHORT_GRACE));
    let first = sim.join("car-1");
    let second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    first.disconnect();

    let reason = sim.expect_message(&second, "the scenario to end", |message| match message {
        ServerMessage::ScenarioEnded { reason } => Some(reason.clone()),
        _ => None,
    });
    assert!(
        reason.contains("car-1") && reason.contains("reconnect"),
        "unexpected end reason: {reason}"
    );

    // Freeze-and-inspect: the world stops where it was rather than being torn
    // down, and the remaining agent's entity is still there to look at.
    assert_eq!(sim.state(), ScenarioState::Ended);
    sim.expect("physics to freeze", |sim| {
        (!sim.physics_active()).then_some(())
    });
    assert!(sim.entity_of("car-2").is_some());
}

#[test]
fn a_join_after_the_scenario_ended_gets_scenario_ended() {
    let mut config = scenario(&["car-1"]);
    config.time.duration = Some(0.25);
    let mut sim = Sim::new(config);
    let _first = sim.join("car-1");
    sim.expect("the run to reach its deadline", |sim| {
        (sim.state() == ScenarioState::Ended).then_some(())
    });

    // A latecomer is accepted onto the socket and told immediately, rather
    // than being left hanging or refused as an unknown slot.
    let latecomer = sim.connect();
    latecomer.send(ClientMessage::Join {
        name: "car-1".into(),
    });
    let reason = sim.expect_message(&latecomer, "scenario_ended", |message| match message {
        ServerMessage::ScenarioEnded { reason } => Some(reason.clone()),
        _ => None,
    });
    assert_eq!(reason, "duration reached");
}

#[test]
fn a_duration_deadline_ends_the_run_on_the_exact_tick() {
    // 1 sim-second at 64 Hz.
    let mut config = scenario(&["car-1"]);
    config.time.duration = Some(1.0);
    let mut sim = Sim::new(config);
    let _agent = sim.join("car-1");

    sim.expect("the run to end at its deadline", |sim| {
        (sim.state() == ScenarioState::Ended).then_some(())
    });
    assert_eq!(sim.tick(), 64);
}

#[test]
fn a_malformed_message_does_not_end_the_scenario() {
    let mut sim = Sim::new(scenario(&["car-1"]));
    let agent = sim.join("car-1");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    agent.send_raw(Message::text(
        "{\"type\": \"submit_plan\", \"waypoints\": \"nope\"}",
    ));
    sim.expect_message(&agent, "an error reply", error_message);

    // The transport-level contract, carried all the way up: a JSON typo is
    // not a disconnect, and a disconnect is what ends a scenario.
    sim.step(30);
    assert_eq!(sim.state(), ScenarioState::Running);

    // The connection still controls its entity.
    agent.send(ClientMessage::SubmitPlan {
        waypoints: vec![waypoint(5.0, 2.0)],
    });
    sim.expect("the following plan to land", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });
}

#[test]
fn an_off_road_despawn_notifies_its_agent_and_the_run_continues() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    let first = sim.join("car-1");
    let second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    sim.put_below_floor("car-1");

    let agent_id = sim.expect_message(&first, "an off_road notice", |message| match message {
        ServerMessage::OffRoad { agent_id } => Some(agent_id.0.clone()),
        _ => None,
    });
    assert_eq!(agent_id, "car-1");

    // The agent stays connected -- this must not trip the
    // disconnect-ends-the-scenario path -- and everyone else drives on.
    sim.expect("car-1's entity to be despawned", |sim| {
        (sim.entity_of("car-1").is_none()).then_some(())
    });
    sim.expect_silence(&second, 30);
    assert_eq!(sim.state(), ScenarioState::Running);
    assert!(sim.physics_active());
}

#[test]
fn a_disconnect_from_an_already_despawned_agent_does_not_end_the_scenario() {
    let mut sim = Sim::new(scenario(&["car-1", "car-2"]));
    sim.app.insert_resource(ReconnectGrace(SHORT_GRACE));
    let first = sim.join("car-1");
    let second = sim.join("car-2");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    sim.put_below_floor("car-1");
    sim.expect("car-1 to go off-road", |sim| {
        sim.entity_of("car-1").is_none().then_some(())
    });

    // Its agent now hangs up. There is no entity and no registry row left, so
    // there is nothing to open a grace window for -- and no reason to end the
    // run for the agents still driving.
    first.disconnect();
    sim.step(60); // Well past SHORT_GRACE.
    assert_eq!(sim.state(), ScenarioState::Running);
    sim.expect_silence(&second, 10);
}

#[test]
fn a_non_finite_waypoint_does_not_get_reported_as_off_road() {
    let mut sim = Sim::new(scenario(&["car-1"]));
    let agent = sim.join("car-1");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });
    let start = sim.position_of("car-1");

    // Valid JSON, hostile values -- and not even exotic ones. `NaN` cannot
    // cross JSON (it serializes to `null`, which the deserializer refuses),
    // but any number literal past the f32 range parses straight to infinity:
    // `3.5e38` is what an agent's own overflowed arithmetic looks like on the
    // wire. Untreated, it reaches `DesiredVelocity` -> `ExternalForce` ->
    // Rapier and NaNs the body's position; `despawn_off_road` then tests
    // `y >= floor`, which is false for NaN, so the agent is told its car went
    // off the road -- a confident report of the wrong cause.
    agent.send_raw(Message::text(
        r#"{"type":"submit_plan","waypoints":[
             {"position":{"x":3.5e38,"y":0.0,"z":0.0},"speed":5.0},
             {"position":{"x":0.0,"y":0.0,"z":-1e39},"speed":5.0},
             {"position":{"x":10.0,"y":0.0,"z":0.0},"speed":1e39}
           ]}"#,
    ));
    let error = sim.expect_message(&agent, "the plan to be refused", error_message);
    assert!(
        error.contains("waypoints are usable"),
        "unexpected refusal: {error}"
    );

    for _ in 0..60 {
        sim.step(1);
        if let Some(message) = agent.try_recv() {
            panic!("expected no further message, got {message:?}");
        }
        assert!(
            sim.position_of("car-1").is_finite(),
            "body position went non-finite"
        );
    }
    assert_eq!(sim.state(), ScenarioState::Running);
    // The refused plan never displaced the agent, and never bumped its version.
    assert!((sim.position_of("car-1") - start).length() < 1.0);
    assert_eq!(sim.plan_version("car-1"), 0);
}

#[test]
fn a_partly_non_finite_plan_keeps_its_usable_waypoints() {
    let mut sim = Sim::new(scenario(&["car-1"]));
    let agent = sim.join("car-1");
    sim.expect("the scenario to start", |sim| {
        (sim.state() == ScenarioState::Running).then_some(())
    });

    agent.send_raw(Message::text(
        r#"{"type":"submit_plan","waypoints":[
             {"position":{"x":1e39,"y":0.0,"z":0.0},"speed":5.0},
             {"position":{"x":8.0,"y":0.0,"z":0.0},"speed":3.0},
             {"position":{"x":0.0,"y":0.0,"z":9.0},"speed":-4.0}
           ]}"#,
    ));
    sim.expect("the cleaned plan to land", |sim| {
        (sim.plan_version("car-1") == 1).then_some(())
    });
    assert_eq!(
        sim.plan_waypoints("car-1"),
        vec![
            waypoint(8.0, 3.0),
            Waypoint {
                position: Vec3::new(0.0, 0.0, 9.0),
                // A negative speed clamps rather than driving the entity backwards
                // along its own path.
                speed: 0.0,
            }
        ]
    );
}
