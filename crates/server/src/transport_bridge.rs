use bevy::prelude::*;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration, Velocity};
use protocol::messages::{AgentId, ClientMessage, EntityState, ServerMessage, StateSnapshot};
use protocol::Vec3 as WireVec3;
use transport::{ConnectionEvent, TransportHandle};

use crate::agent::{AgentName, AgentRegistry, Connection, PendingRoster, Plan, Reflexes};
use crate::events::ReflexFired;
use crate::scenario::Roster;
use crate::scenario_state::{EndReason, ScenarioState, Tick};
use crate::world::spawn_agent;

#[derive(Resource)]
pub struct Transport(pub TransportHandle);

fn to_wire(v: Vec3) -> WireVec3 {
    WireVec3::new(v.x, v.y, v.z)
}

/// Spawn position for the Nth roster slot: spread evenly along X, just
/// above the ground so capsules don't spawn embedded in it.
fn spawn_position(index: usize, total: usize) -> Vec3 {
    let spacing = 3.0;
    let offset = (index as f32) - (total.saturating_sub(1) as f32) / 2.0;
    Vec3::new(offset * spacing, crate::world::AGENT_RADIUS * 2.0, 0.0)
}

/// Drains transport's channels every frame and applies their effect to the
/// ECS world: spawning agents on `Join`, updating plans/reflexes, replying
/// to `GetState`, and ending the scenario on disconnect. Runs regardless
/// of `ScenarioState` — `Join` only matters while waiting for the roster,
/// but a disconnect can end the scenario at any point after an agent has
/// joined.
#[allow(clippy::too_many_arguments)]
pub fn drain_transport(
    mut transport: ResMut<Transport>,
    mut commands: Commands,
    mut registry: ResMut<AgentRegistry>,
    mut pending: ResMut<PendingRoster>,
    roster: Res<Roster>,
    state: Res<State<ScenarioState>>,
    mut next_state: ResMut<NextState<ScenarioState>>,
    mut end_reason: ResMut<EndReason>,
    tick: Res<Tick>,
    mut query: Query<(&Transform, &Velocity, &mut Plan, &mut Reflexes, &AgentName)>,
) {
    while let Ok(event) = transport.0.events.try_recv() {
        if let ConnectionEvent::Disconnected(connection) = event {
            if let Some(entity) = registry.remove_connection(connection) {
                if *state.get() != ScenarioState::Ended {
                    let name = query
                        .get(entity)
                        .map(|(_, _, _, _, name)| name.0.clone())
                        .unwrap_or_else(|_| "unknown agent".to_string());
                    end_reason.0 = Some(format!("{name} disconnected"));
                    next_state.set(ScenarioState::Ended);
                }
            }
        }
    }

    while let Ok(inbound) = transport.0.inbound.try_recv() {
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
                    transport.0.send(
                        connection,
                        ServerMessage::Error {
                            message: "scenario already running".into(),
                        },
                    );
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
                    let position = spawn_position(index, roster.0.roster.len());
                    let entity = spawn_agent(&mut commands, &name, position, connection);
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

pub fn activate_physics(mut query: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    if let Ok(mut config) = query.single_mut() {
        config.physics_pipeline_active = true;
    }
}

pub fn deactivate_physics(mut query: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    if let Ok(mut config) = query.single_mut() {
        config.physics_pipeline_active = false;
    }
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
