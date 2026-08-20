use bevy::prelude::*;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration, Velocity};
use protocol::messages::{
    AgentId, ClientMessage, EntityState, ServerMessage, StateSnapshot, Waypoint,
};
use protocol::Vec3 as WireVec3;
use transport::{ConnectionEvent, TransportHandle};

use std::time::{Duration, Instant};

use crate::agent::{
    AgentName, AgentRegistry, AwaitingReconnect, Connection, PendingRoster, Plan, Reflexes,
};
use crate::events::ReflexFired;
use crate::pulse::PulseStates;
use crate::scenario::Roster;
use crate::scenario_state::{EndReason, ScenarioState, Tick};
use crate::time_budget::{reached, Deadline, TICK_HZ};
use crate::world::{agent_spawn_transform, spawn_agent, to_map_data, MapWorld};

/// How long a mid-scenario agent has to reconnect (re-`Join` by name)
/// before the scenario ends. Absorbs a transient network blip on a slow,
/// flaky agent without nuking the run for everyone else.
const RECONNECT_GRACE: Duration = Duration::from_secs(8);

/// Cap on inbound messages processed per drain so a burst can't stall the
/// tick unboundedly. The transport channel is itself bounded, so this is a
/// second line of defense.
const MAX_INBOUND_PER_DRAIN: usize = 512;

/// A vehicle whose Y falls below this has left the drivable surface (drove off
/// an edge, or spawned over a gap) and is falling out of the world -- nothing
/// legitimately drives this far under the road. `despawn_off_road` removes it.
const FLOOR_Y: f32 = -5.0;

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
    mut pulse_states: ResMut<PulseStates>,
    viz_res: Res<crate::viz_broadcast::Viz>,
    map_world: Res<MapWorld>,
    mut query: Query<(&Transform, &Velocity, &mut Plan, &mut Reflexes, &AgentName)>,
) {
    // The static road prior, delivered with every `Joined`. `None` in the arena
    // world. Built once per tick; cheap, and only cloned onto an actual join.
    let map_payload = map_world.0.as_ref().map(to_map_data);
    while let Ok(event) = transport.0.events.try_recv() {
        if let ConnectionEvent::Disconnected(connection) = event {
            // Drop any step-callback state for the gone connection; a reconnect
            // re-subscribes fresh.
            pulse_states.0.remove(&connection);
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
                                    map: map_payload.clone(),
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
                    let sensors = roster.0.roster[index].sensors.clone();
                    let color =
                        roster.0.roster[index]
                            .color
                            .map(|[r, g, b]| viz::Color { r, g, b });
                    let scale = roster.0.roster[index].scale.unwrap_or(1.0);
                    let base = spawn_position(index, roster.0.roster.len());
                    let transform =
                        agent_spawn_transform(embodiment, base, map_world.0.as_ref(), scale, index);
                    let spawned_at = transform.translation;
                    let entity = spawn_agent(
                        &mut commands,
                        &name,
                        transform,
                        connection,
                        embodiment,
                        sensors,
                        color,
                        scale,
                    );
                    registry.insert(connection, name.clone(), entity);
                    pending.0.retain(|pending_name| pending_name != &name);

                    transport.0.send(
                        connection,
                        ServerMessage::Joined {
                            agent_id: AgentId(name),
                            position: to_wire(spawned_at),
                            map: map_payload.clone(),
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
            ClientMessage::RequestRoute { from, to, speed } => {
                // Route over the loaded map, stamping the agent's cruise speed
                // onto every waypoint (the plan owns speed). Empty if no map or
                // no path -- the agent decides what to do with the reply.
                let waypoints = map_world
                    .0
                    .as_ref()
                    .and_then(|net| {
                        net.route(
                            Vec3::new(from.x, from.y, from.z),
                            Vec3::new(to.x, to.y, to.z),
                        )
                    })
                    .unwrap_or_default()
                    .into_iter()
                    .map(|p| Waypoint {
                        position: to_wire(p),
                        speed,
                    })
                    .collect();
                transport
                    .0
                    .send(connection, ServerMessage::Route { waypoints });
            }
            ClientMessage::Subscribe => {
                // Opt into step pulses. Anchored to the current tick so the
                // first pulse's dt starts from now, not tick 0.
                pulse_states
                    .0
                    .entry(connection)
                    .or_default()
                    .subscribe(tick.0);
            }
            ClientMessage::Ack { tick: acked } => {
                if let Some(state) = pulse_states.0.get_mut(&connection) {
                    state.ack(acked);
                }
            }
        }
    }
}

/// Ends the scenario once the run reaches its `time.duration` deadline (if it
/// has one). Runs only while `Running`; the deadline is in ticks, so afap
/// reaches it sooner in wall-clock but at the same sim-time.
pub fn enforce_duration(
    tick: Res<Tick>,
    deadline: Res<Deadline>,
    mut next_state: ResMut<NextState<ScenarioState>>,
    mut end_reason: ResMut<EndReason>,
) {
    if reached(tick.0, deadline.0) {
        end_reason.0 = Some("duration reached".into());
        next_state.set(ScenarioState::Ended);
    }
}

/// Pushes a step pulse to each subscribed agent that isn't awaiting an ack. The
/// sim never waits: a slow agent simply gets fewer pulses, each with a larger
/// `dt`. `plan_version` reflects the plan currently driving the agent's entity.
pub fn send_pulses(
    transport: Res<Transport>,
    mut pulse_states: ResMut<PulseStates>,
    tick: Res<Tick>,
    registry: Res<AgentRegistry>,
    query: Query<&Plan>,
) {
    for (connection, entity) in registry.connections() {
        let state = pulse_states.0.entry(connection).or_default();
        if let Some(dt) = state.poll(tick.0, TICK_HZ) {
            let plan_version = query.get(entity).map(|p| p.version).unwrap_or(0);
            transport.0.send(
                connection,
                ServerMessage::Tick {
                    tick: tick.0,
                    dt,
                    plan_version,
                },
            );
        }
    }
}

/// Removes any vehicle that has fallen below the world floor -- it lost its
/// road (drove off an edge, or a bad spawn) and would otherwise fall forever.
/// Despawns the entity, drops it from the registry and pulse state, tells
/// viewers it's gone, and notifies its agent with `OffRoad`. The agent stays
/// connected (so this never trips the disconnect-ends-the-scenario path); the
/// run continues for everyone else.
pub fn despawn_off_road(
    mut commands: Commands,
    mut registry: ResMut<AgentRegistry>,
    mut pulse_states: ResMut<PulseStates>,
    transport: Res<Transport>,
    viz_res: Res<crate::viz_broadcast::Viz>,
    query: Query<(Entity, &Transform, &AgentName, &Connection)>,
) {
    for (entity, transform, name, connection) in &query {
        if transform.translation.y >= FLOOR_Y {
            continue;
        }
        transport.0.send(
            connection.0,
            ServerMessage::OffRoad {
                agent_id: AgentId(name.0.clone()),
            },
        );
        viz_res.0.broadcast_reliable(&viz::ServerToViewer::Event(
            viz::SceneEvent::EntityDespawned {
                id: viz::EntityId(name.0.clone()),
            },
        ));
        registry.remove_connection(connection.0);
        registry.remove_name(&name.0);
        pulse_states.0.remove(&connection.0);
        commands.entity(entity).despawn();
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
