use bevy::prelude::*;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration, Velocity};
use movement::FmuStore;
use protocol::messages::{
    AgentId, ClientMessage, EntityState, ServerMessage, StateSnapshot, Waypoint,
};
use protocol::scenario::Embodiment;
use protocol::Vec3 as WireVec3;
use transport::{ConnectionEvent, TransportHandle};

use std::time::{Duration, Instant};

use crate::agent::{
    AgentName, AgentRegistry, AwaitingReconnect, Connection, PendingRoster, Plan, Reflexes,
};
use crate::events::ReflexFired;
use crate::inbound::{sanitize_plan, sanitize_rules};
use crate::pulse::PulseStates;
use crate::scenario::Roster;
use crate::scenario_state::{EndReason, ScenarioState, Tick};
use crate::time_budget::{reached, Deadline, TICK_HZ};
use crate::world::{agent_spawn_transform, spawn_agent, to_map_data, MapWorld};

/// How long a mid-scenario agent has to reconnect (re-`Join` by name)
/// before the scenario ends. Absorbs a transient network blip on a slow,
/// flaky agent without nuking the run for everyone else.
pub const RECONNECT_GRACE: Duration = Duration::from_secs(8);

/// The reconnect grace window in force for this run. A resource rather than
/// the bare constant so a test can shorten it and still exercise the real
/// expiry path.
#[derive(Resource, Clone, Copy)]
pub struct ReconnectGrace(pub Duration);

impl Default for ReconnectGrace {
    fn default() -> Self {
        Self(RECONNECT_GRACE)
    }
}

/// Cap on inbound messages processed per drain so a burst can't stall the
/// tick unboundedly. The transport channel is itself bounded, so this is a
/// second line of defense.
const MAX_INBOUND_PER_DRAIN: usize = 512;

/// How far below the lowest road surface a vehicle must fall to count as "off
/// the map". I guess ten meters is enough.
const FALL_MARGIN: f32 = 10.0;

/// The Y below which a vehicle has left the map and should be despawned.
/// Computed once from the loaded map's elevation (`floor_for`).
#[derive(Resource, Clone, Copy)]
pub struct Floor(pub f32);

/// The off-map fall floor for a world: `FALL_MARGIN` below the lowest lane in
/// the road network, or below the arena ground (y=0) when there's no map.
pub fn floor_for(map: Option<&map::RoadNetwork>) -> Floor {
    let lowest = map.and_then(|net| net.min_elevation()).unwrap_or(0.0);
    Floor(lowest - FALL_MARGIN)
}

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
/// `ScenarioState` -- `Join` and disconnects are meaningful in more than one
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
    grace: Res<ReconnectGrace>,
    // The FMU handle store is `NonSend` (a loaded FMU is `!Send`), so populating
    // it at join makes this a main-thread system -- which it already is.
    mut fmu_store: NonSendMut<FmuStore>,
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
                // Mid-scenario: don't end immediately -- give the agent a
                // grace window to reconnect. Its entity keeps coasting on
                // its last plan/reflexes; `expire_reconnects` ends the
                // scenario if the deadline passes. The entity is never
                // despawned in this path, so viewers keep showing it --
                // frozen with the rest of the world once the scenario ends,
                // which matches the freeze-and-inspect end state.
                ScenarioState::Running => {
                    awaiting.mark(name, Instant::now() + grace.0);
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
                // One connection controls one entity. Without this, a single
                // client could `Join` every slot in turn: each `insert`
                // overwrites `by_connection`, so the earlier entities are
                // orphaned -- still driving, still holding a live socket, but
                // unreachable for `reflex_fired` and for disconnect handling.
                // A scripted client could start a multi-agent scenario alone.
                _ if registry.by_connection(connection).is_some() => {
                    transport.0.send(
                        connection,
                        ServerMessage::Error {
                            message: "this connection already controls an agent".into(),
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
                    // An FmuVehicle slot loads + resolves its FMU now, at spawn.
                    // A missing/incompatible `.fmu` or an unresolvable binding is
                    // a clean error sent back to the agent; the slot stays pending
                    // (fixable + rejoinable) rather than half-spawning an entity
                    // or panicking. `validate_fmu` already guarantees the config
                    // is present for this embodiment.
                    let (fmu_handle, fmu_binding) = if embodiment == Embodiment::FmuVehicle {
                        match &roster.0.roster[index].fmu {
                            Some(cfg) => match crate::fmu_setup::load_fmu_vehicle(cfg, &name) {
                                Ok((fmu, binding, frame)) => (Some(fmu), Some((binding, frame))),
                                Err(err) => {
                                    transport.0.send(
                                        connection,
                                        ServerMessage::Error {
                                            message: format!(
                                                "FMU setup failed for '{name}': {err}"
                                            ),
                                        },
                                    );
                                    continue;
                                }
                            },
                            None => {
                                transport.0.send(
                                    connection,
                                    ServerMessage::Error {
                                        message: format!(
                                            "'{name}' is an fmu_vehicle with no fmu config"
                                        ),
                                    },
                                );
                                continue;
                            }
                        }
                    } else {
                        (None, None)
                    };
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
                        fmu_binding,
                    );
                    // The FMU handle is keyed by the spawned entity; dropping it
                    // (on despawn) frees the instance (see `free_despawned_fmus`).
                    if let Some(fmu) = fmu_handle {
                        fmu_store.insert(entity, Box::new(fmu));
                    }
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
            // Agent-supplied floats reach Rapier through the plan, so they are
            // sanitized here rather than trusted: a `NaN` waypoint would
            // poison `DesiredVelocity` -> `ExternalForce` and surface as a
            // confidently-mislabelled `OffRoad` (`y >= floor` is false for
            // `NaN`). A refused payload gets an `error` and leaves the
            // entity's current plan -- and its version -- untouched.
            ClientMessage::SubmitPlan { waypoints } => {
                let Some(entity) = registry.by_connection(connection) else {
                    continue;
                };
                match sanitize_plan(waypoints) {
                    Ok(waypoints) => {
                        if let Ok((_, _, mut plan, _, _)) = query.get_mut(entity) {
                            plan.waypoints = waypoints.into_iter().collect();
                            plan.version += 1;
                        }
                    }
                    Err(err) => transport.0.send(
                        connection,
                        ServerMessage::Error {
                            message: err.to_string(),
                        },
                    ),
                }
            }
            ClientMessage::RegisterReflexes { rules } => {
                let Some(entity) = registry.by_connection(connection) else {
                    continue;
                };
                match sanitize_rules(rules) {
                    Ok(rules) => {
                        if let Ok((_, _, _, mut reflexes, _)) = query.get_mut(entity) {
                            reflexes.0 = rules.into_iter().map(sensors::ActiveRule::new).collect();
                        }
                    }
                    Err(err) => transport.0.send(
                        connection,
                        ServerMessage::Error {
                            message: err.to_string(),
                        },
                    ),
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
    floor: Res<Floor>,
    transport: Res<Transport>,
    viz_res: Res<crate::viz_broadcast::Viz>,
    query: Query<(Entity, &Transform, &AgentName, &Connection)>,
) {
    for (entity, transform, name, connection) in &query {
        if transform.translation.y >= floor.0 {
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
        // here means a plugin-ordering change broke an assumption -- loud is
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
