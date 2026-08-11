use bevy::prelude::*;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration, Velocity};
use protocol::messages::{AgentId, ClientMessage, EntityState, ServerMessage, StateSnapshot};
use protocol::Vec3 as WireVec3;
use transport::{ConnectionEvent, TransportHandle};

use std::time::{Duration, Instant};

use crate::agent::{
    AgentName, AgentRegistry, AwaitingReconnect, Connection, PendingRoster, Plan, Reflexes,
};
use crate::events::ReflexFired;
use crate::scenario::Roster;
use crate::scenario_state::{EndReason, ScenarioState, Tick};
use crate::world::spawn_agent;

/// How long a mid-scenario agent has to reconnect (re-`Join` by name)
/// before the scenario ends. Absorbs a transient network blip on a slow,
/// flaky agent without nuking the run for everyone else.
const RECONNECT_GRACE: Duration = Duration::from_secs(8);

/// Cap on inbound messages processed per drain so a burst can't stall the
/// tick unboundedly. The transport channel is itself bounded, so this is a
/// second line of defense.
const MAX_INBOUND_PER_DRAIN: usize = 512;

#[derive(Resource)]
pub struct Transport(pub TransportHandle);

fn finite(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}

/// Non-finite floats serialize to JSON `null`, which agents can't parse
/// back into a `Vec3`; coerce them to zero at the wire boundary.
fn to_wire(v: Vec3) -> WireVec3 {
    WireVec3::new(finite(v.x), finite(v.y), finite(v.z))
}

/// Spawn position for the Nth roster slot: spread evenly along X, just
/// above the ground so capsules don't spawn embedded in it.
fn spawn_position(index: usize, total: usize) -> Vec3 {
    let spacing = 3.0;
    let offset = (index as f32) - (total.saturating_sub(1) as f32) / 2.0;
    Vec3::new(offset * spacing, crate::world::AGENT_RADIUS * 2.0, 0.0)
}

/// Drains transport's channels each tick and applies their effect to the
/// ECS world: spawning agents on `Join`, updating plans/reflexes, replying
/// to `GetState`, and handling disconnects. Runs regardless of
/// `ScenarioState` — `Join` and disconnects are meaningful in more than one
/// state.
#[allow(clippy::too_many_arguments)]
pub fn drain_transport(
    mut transport: ResMut<Transport>,
    mut commands: Commands,
    mut registry: ResMut<AgentRegistry>,
    mut pending: ResMut<PendingRoster>,
    mut awaiting: ResMut<AwaitingReconnect>,
    roster: Res<Roster>,
    state: Res<State<ScenarioState>>,
    mut next_state: ResMut<NextState<ScenarioState>>,
    end_reason: Res<EndReason>,
    tick: Res<Tick>,
    viz_res: Res<crate::viz_broadcast::Viz>,
    mut query: Query<(&Transform, &Velocity, &mut Plan, &mut Reflexes, &AgentName)>,
) {
    while let Ok(event) = transport.0.events.try_recv() {
        if let ConnectionEvent::Disconnected(connection) = event {
            let Some(entity) = registry.remove_connection(connection) else {
                continue;
            };
            let name = query
                .get(entity)
                .map(|(_, _, _, _, name)| name.0.clone())
                .unwrap_or_else(|_| "unknown agent".to_string());
            match *state.get() {
                // Mid-scenario: don't end immediately — give the agent a
                // grace window to reconnect. Its entity keeps coasting on
                // its last plan/reflexes; `expire_reconnects` ends the
                // scenario if the deadline passes. The entity is never
                // despawned in this path, so viewers keep showing it —
                // frozen with the rest of the world once the scenario ends,
                // which matches the freeze-and-inspect end state.
                ScenarioState::Running => {
                    awaiting.mark(name, Instant::now() + RECONNECT_GRACE);
                }
                // Pre-start: reopen the slot so the agent (or another) can
                // fill it, and remove the orphaned entity.
                ScenarioState::WaitingForRoster => {
                    viz_res.0.broadcast_reliable(&viz::ServerToViewer::Event(
                        viz::SceneEvent::EntityDespawned {
                            id: viz::EntityId(name.clone()),
                        },
                    ));
                    registry.remove_name(&name);
                    pending.0.push(name);
                    commands.entity(entity).despawn();
                }
                ScenarioState::Ended => {}
            }
        }
    }

    let mut drained = 0;
    while drained < MAX_INBOUND_PER_DRAIN {
        let Ok(inbound) = transport.0.inbound.try_recv() else {
            break;
        };
        drained += 1;
        let connection = inbound.connection;
        match inbound.message {
            ClientMessage::Join { name } => match *state.get() {
                ScenarioState::Ended => {
                    transport.0.send(
                        connection,
                        ServerMessage::ScenarioEnded {
                            reason: end_reason.0.clone().unwrap_or_default(),
                        },
                    );
                }
                ScenarioState::Running => {
                    // A join while running is only valid as a reconnect of
                    // an agent inside its grace window.
                    if awaiting.is_awaiting(&name) {
                        if let Some(entity) = registry.by_name(&name) {
                            awaiting.reconnected(&name);
                            registry.insert(connection, name.clone(), entity);
                            commands.entity(entity).insert(Connection(connection));
                            let position = query
                                .get(entity)
                                .map(|(transform, ..)| transform.translation)
                                .unwrap_or(Vec3::ZERO);
                            transport.0.send(
                                connection,
                                ServerMessage::Joined {
                                    agent_id: AgentId(name),
                                    position: to_wire(position),
                                },
                            );
                        }
                    } else {
                        transport.0.send(
                            connection,
                            ServerMessage::Error {
                                message: "scenario already running".into(),
                            },
                        );
                    }
                }
                ScenarioState::WaitingForRoster => {
                    if !pending.0.contains(&name) {
                        transport.0.send(
                            connection,
                            ServerMessage::Error {
                                message: format!("'{name}' is not an unfilled roster slot"),
                            },
                        );
                        continue;
                    }
                    let index = roster
                        .0
                        .roster
                        .iter()
                        .position(|slot| slot.name == name)
                        .expect("checked above");
                    let embodiment = roster.0.roster[index].embodiment;
                    let sensors = roster.0.roster[index].sensors;
                    let position = spawn_position(index, roster.0.roster.len());
                    let entity = spawn_agent(
                        &mut commands,
                        &name,
                        position,
                        connection,
                        embodiment,
                        sensors,
                    );
                    registry.insert(connection, name.clone(), entity);
                    pending.0.retain(|pending_name| pending_name != &name);

                    transport.0.send(
                        connection,
                        ServerMessage::Joined {
                            agent_id: AgentId(name),
                            position: to_wire(position),
                        },
                    );

                    if pending.0.is_empty() {
                        next_state.set(ScenarioState::Running);
                    }
                }
            },
            ClientMessage::SubmitPlan { waypoints } => {
                if let Some(entity) = registry.by_connection(connection) {
                    if let Ok((_, _, mut plan, _, _)) = query.get_mut(entity) {
                        plan.waypoints = waypoints.into_iter().collect();
                        plan.version += 1;
                    }
                }
            }
            ClientMessage::RegisterReflexes { rules } => {
                if let Some(entity) = registry.by_connection(connection) {
                    if let Ok((_, _, _, mut reflexes, _)) = query.get_mut(entity) {
                        reflexes.0 = rules.into_iter().map(sensors::ActiveRule::new).collect();
                    }
                }
            }
            ClientMessage::GetState => {
                let entities = query
                    .iter()
                    .map(|(transform, velocity, plan, _, name)| EntityState {
                        agent_id: AgentId(name.0.clone()),
                        position: to_wire(transform.translation),
                        velocity: to_wire(velocity.linear),
                        plan_version: plan.version,
                    })
                    .collect();
                transport.0.send(
                    connection,
                    ServerMessage::State(StateSnapshot {
                        tick: tick.0,
                        entities,
                    }),
                );
            }
        }
    }
}

/// Ends the scenario if any awaiting-reconnect agent's grace window has
/// elapsed without it coming back.
pub fn expire_reconnects(
    awaiting: Res<AwaitingReconnect>,
    mut next_state: ResMut<NextState<ScenarioState>>,
    mut end_reason: ResMut<EndReason>,
) {
    if let Some(name) = awaiting.expired(Instant::now()) {
        end_reason.0 = Some(format!("{name} did not reconnect within grace window"));
        next_state.set(ScenarioState::Ended);
    }
}

fn set_physics_active(
    query: &mut Query<&mut RapierConfiguration, With<DefaultRapierContext>>,
    active: bool,
) {
    match query.single_mut() {
        Ok(mut config) => config.physics_pipeline_active = active,
        // The default Rapier context is created in PreStartup, so a miss
        // here means a plugin-ordering change broke an assumption — loud is
        // better than silently leaving physics in its prior state.
        Err(err) => warn!("could not set physics_pipeline_active={active}: {err}"),
    }
}

pub fn activate_physics(mut query: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    set_physics_active(&mut query, true);
}

pub fn deactivate_physics(mut query: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    set_physics_active(&mut query, false);
}

pub fn notify_scenario_ended(transport: Res<Transport>, end_reason: Res<EndReason>) {
    let reason = end_reason.0.clone().unwrap_or_default();
    transport
        .0
        .broadcast(ServerMessage::ScenarioEnded { reason });
}

/// Forwards each `ReflexFired` message emitted by arbitration to the
/// owning agent as `ServerMessage::ReflexFired`, so an agent learns its
/// plan was overridden without polling.
pub fn forward_reflex_fired(
    transport: Res<Transport>,
    mut reflex_fired: MessageReader<ReflexFired>,
    query: Query<&Connection>,
) {
    for event in reflex_fired.read() {
        if let Ok(connection) = query.get(event.entity) {
            transport.0.send(
                connection.0,
                ServerMessage::ReflexFired {
                    tick: event.tick,
                    plan_version: event.plan_version,
                    action: event.action,
                },
            );
        }
    }
}
