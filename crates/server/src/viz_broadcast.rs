use bevy::prelude::*;
use movement::{DesiredVelocity, RaycastVehicle, Wheels};

use crate::agent::Plan;
use crate::scenario::ArenaBounds;
use crate::scenario_state::{ScenarioState, Tick};
use crate::viz_nodes::{node_updates, to_viz_transform};

/// Emit a viz frame every N physics ticks — ~32 Hz at the fixed tick.
/// Gating on the tick (rather than a wall-clock timer) makes frames uniform
/// in sim-time, so the viewer can interpolate on the frame `tick` cleanly.
const TICKS_PER_FRAME: u64 = 2;

/// The viz broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Viz(pub viz::VizHandle);

/// The static viz facts about an entity — everything in an
/// `EntityDescriptor` except where its nodes are right now. Attached to walls,
/// ground, and agents so the broadcast systems can describe the scene without
/// knowing how any of it was built.
#[derive(Component, Clone)]
pub struct VizEntity {
    pub id: viz::EntityId,
    pub name: String,
    pub kind: viz::EntityKind,
    pub color: viz::Color,
    /// The entity's node tree: names, geometry, and how the nodes are nested.
    /// The transforms in it are a starting pose only — every descriptor is
    /// sent with the live ones applied (see `descriptor`).
    pub root: viz::EntityNode,
    /// The agent's sensing region for the debug envelope overlay; `None` for
    /// static geometry.
    pub sensors: Option<viz::SensorView>,
}

pub fn viz_embodiment(embodiment: protocol::scenario::Embodiment) -> viz::Embodiment {
    match embodiment {
        protocol::scenario::Embodiment::Holonomic => viz::Embodiment::Holonomic,
        protocol::scenario::Embodiment::CarLike => viz::Embodiment::CarLike,
        protocol::scenario::Embodiment::FullVehicle => viz::Embodiment::FullVehicle,
        protocol::scenario::Embodiment::RaycastVehicle => viz::Embodiment::RaycastVehicle,
        protocol::scenario::Embodiment::FmuVehicle => viz::Embodiment::FmuVehicle,
    }
}

fn viz_state(state: ScenarioState) -> viz::ScenarioState {
    match state {
        ScenarioState::WaitingForRoster => viz::ScenarioState::WaitingForRoster,
        ScenarioState::Running => viz::ScenarioState::Running,
        ScenarioState::Ended => viz::ScenarioState::Ended,
    }
}

/// What describing an entity takes: its static facts, its body pose, and the
/// vehicle state the sim poses its child nodes from.
type DescribeQuery<'a> = (
    &'a VizEntity,
    &'a Transform,
    Option<&'a RaycastVehicle>,
    Option<&'a Wheels>,
);

/// The entity as it is right now: its tree, with the root placed at the body's
/// world pose and every other node placed where the sim has it. A viewer
/// joining mid-scenario therefore sees the wheels where they are, not where
/// they were at spawn.
fn descriptor(
    entity: &VizEntity,
    transform: &Transform,
    vehicle: Option<&RaycastVehicle>,
    wheels: Option<&Wheels>,
) -> viz::EntityDescriptor {
    let mut root = entity.root.clone();
    root.transform = to_viz_transform(transform);
    for update in node_updates(vehicle, wheels) {
        // Both come from the same builder, so an unresolved path is a
        // construction bug, not bad input.
        let placed = root.set_transform(&update.path, update.transform);
        debug_assert!(placed, "no node at {:?} on {:?}", update.path, entity.id);
    }
    viz::EntityDescriptor {
        id: entity.id.clone(),
        name: entity.name.clone(),
        kind: entity.kind,
        color: entity.color,
        root,
        sensors: entity.sensors,
    }
}

/// Sends a fresh scene-init to each newly connected viewer so it catches up
/// before following the live stream.
pub fn drain_viz_events(
    mut viz: ResMut<Viz>,
    bounds: Res<ArenaBounds>,
    state: Res<State<ScenarioState>>,
    tick: Res<Tick>,
    fixed_time: Res<Time<Fixed>>,
    query: Query<DescribeQuery>,
) {
    let tick_rate = 1.0 / fixed_time.timestep().as_secs_f32();
    while let Ok(event) = viz.0.events.try_recv() {
        let viz::VizEvent::ViewerConnected { id, .. } = event else {
            continue;
        };
        let entities = query
            .iter()
            .map(|(e, t, vehicle, wheels)| descriptor(e, t, vehicle, wheels))
            .collect();
        let scene = viz::ServerToViewer::SceneInit(viz::SceneInit {
            protocol_version: viz::PROTOCOL_VERSION,
            tick: tick.0,
            tick_rate,
            state: viz_state(*state.get()),
            arena: viz::ArenaBounds {
                width: bounds.half_width * 2.0,
                depth: bounds.half_depth * 2.0,
            },
            entities,
        });
        viz.0.send_scene_init(id, &scene);
    }
}

/// Broadcasts an `EntitySpawned` for each agent that appeared this frame.
/// Static geometry is only ever sent in scene-init, so it's skipped here.
pub fn broadcast_spawns(viz: Res<Viz>, query: Query<DescribeQuery, Added<VizEntity>>) {
    for (entity, transform, vehicle, wheels) in &query {
        if entity.kind.is_dynamic() {
            let event =
                viz::SceneEvent::EntitySpawned(descriptor(entity, transform, vehicle, wheels));
            viz.0.broadcast_reliable(&viz::ServerToViewer::Event(event));
        }
    }
}

/// Broadcasts the current scenario state on a transition.
pub fn broadcast_state(viz: Res<Viz>, state: Res<State<ScenarioState>>) {
    let event = viz::SceneEvent::ScenarioState {
        state: viz_state(*state.get()),
    };
    viz.0.broadcast_reliable(&viz::ServerToViewer::Event(event));
}

/// What a frame needs from each entity. Named because it is six items wide:
/// the identity and node poses that go in the scene layer, the plan and reflex
/// state that go in the debug layer, and the vehicle state, which straddles
/// both (it places the wheel nodes and it carries their diagnostics).
type FrameQuery<'a> = (
    &'a VizEntity,
    &'a Transform,
    Option<&'a Plan>,
    Option<&'a DesiredVelocity>,
    Option<&'a RaycastVehicle>,
    Option<&'a Wheels>,
);

/// Every `TICKS_PER_FRAME` physics ticks, broadcasts a scene frame (the nodes
/// that moved) plus a debug frame (plans + reflex flags), stamped with the
/// current tick so the viewer can interpolate on sim-time.
pub fn broadcast_frames(
    mut last_emit: Local<u64>,
    viz: Res<Viz>,
    tick: Res<Tick>,
    overlay: Res<crate::perception_router::PerceptionOverlay>,
    query: Query<FrameQuery>,
) {
    if tick.0 < *last_emit + TICKS_PER_FRAME {
        return;
    }
    *last_emit = tick.0;

    let mut frame = Vec::new();
    let mut debug = Vec::new();
    for (entity, transform, plan, desired, vehicle, wheels) in &query {
        if !entity.kind.is_dynamic() {
            continue;
        }
        // Only the nodes that moved: the body always, plus a car's four
        // wheels. Each is posed here, in sim-space, because a viewer cannot
        // derive any of it -- a locked wheel's spin is not the body's speed,
        // and suspension travel is invisible from outside the vehicle.
        let mut nodes = vec![viz::NodeUpdate {
            path: viz::NodePath::root(),
            transform: to_viz_transform(transform),
        }];
        nodes.extend(node_updates(vehicle, wheels));
        frame.push(viz::EntityFrame {
            id: entity.id.clone(),
            nodes,
        });
        // What is *left* of the plan. The tracker does not consume waypoints
        // (the agent's path stays as it submitted it), so the ones already
        // driven past are skipped here rather than removed there.
        let plan_points = plan
            .map(|p| {
                p.remaining()
                    .map(|w| viz::Vec3::new(w.position.x, w.position.y, w.position.z))
                    .collect()
            })
            .unwrap_or_default();
        debug.push(viz::EntityDebug {
            id: entity.id.clone(),
            plan: plan_points,
            reflex_active: desired.map(|d| d.urgent).unwrap_or(false),
            // The agent's latest perceived set (keyed by name == viz id).
            detections: overlay.blips_for(&entity.id.0),
            // Diagnostics rather than geometry, so they ride the debug layer.
            wheels: wheels
                .map(|w| {
                    w.0.iter()
                        .map(|wheel| viz::WheelDebug {
                            slip_ratio: wheel.slip_ratio,
                            slip_angle: wheel.slip_angle,
                            contact: wheel.contact,
                        })
                        .collect()
                })
                .unwrap_or_default(),
        });
    }

    viz.0
        .broadcast_frame(&viz::ServerToViewer::Frame(viz::Frame {
            tick: tick.0,
            entities: frame,
        }));
    viz.0
        .broadcast_debug(&viz::ServerToViewer::DebugFrame(viz::DebugFrame {
            tick: tick.0,
            entities: debug,
        }));
}
