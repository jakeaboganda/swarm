use bevy::prelude::*;
use movement::DesiredVelocity;

use crate::agent::Plan;
use crate::scenario::ArenaBounds;
use crate::scenario_state::{ScenarioState, Tick};

/// Emit a viz frame every N physics ticks — ~32 Hz at the fixed tick.
/// Gating on the tick (rather than a wall-clock timer) makes frames uniform
/// in sim-time, so the viewer can interpolate on the frame `tick` cleanly.
const TICKS_PER_FRAME: u64 = 2;

/// The viz broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Viz(pub viz::VizHandle);

/// The static viz facts about an entity — everything in an
/// `EntityDescriptor` except its live transform. Attached to walls, ground,
/// and agents so the broadcast systems can describe the scene without
/// knowing how any of it was built.
#[derive(Component, Clone)]
pub struct VizEntity {
    pub id: viz::EntityId,
    pub name: String,
    pub kind: viz::EntityKind,
    pub shape: viz::Shape,
    pub color: viz::Color,
    /// The agent's sensing region for the debug envelope overlay; `None` for
    /// static geometry.
    pub sensors: Option<viz::SensorView>,
    /// Wheel geometry for a viewer to draw, for entities that have wheels.
    /// Fixed for the life of the entity, so it rides the descriptor rather
    /// than every frame.
    pub wheels: Option<viz::WheelRig>,
}

pub fn viz_embodiment(embodiment: protocol::scenario::Embodiment) -> viz::Embodiment {
    match embodiment {
        protocol::scenario::Embodiment::Holonomic => viz::Embodiment::Holonomic,
        protocol::scenario::Embodiment::CarLike => viz::Embodiment::CarLike,
        protocol::scenario::Embodiment::FullVehicle => viz::Embodiment::FullVehicle,
        protocol::scenario::Embodiment::RaycastVehicle => viz::Embodiment::RaycastVehicle,
    }
}

fn viz_state(state: ScenarioState) -> viz::ScenarioState {
    match state {
        ScenarioState::WaitingForRoster => viz::ScenarioState::WaitingForRoster,
        ScenarioState::Running => viz::ScenarioState::Running,
        ScenarioState::Ended => viz::ScenarioState::Ended,
    }
}

fn to_vec3(v: Vec3) -> viz::Vec3 {
    viz::Vec3::new(v.x, v.y, v.z)
}

fn to_transform(transform: &Transform) -> viz::Transform {
    viz::Transform {
        position: to_vec3(transform.translation),
        rotation: viz::Quat::new(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    }
}

fn descriptor(entity: &VizEntity, transform: &Transform) -> viz::EntityDescriptor {
    viz::EntityDescriptor {
        id: entity.id.clone(),
        name: entity.name.clone(),
        kind: entity.kind,
        shape: entity.shape.clone(),
        color: entity.color,
        transform: to_transform(transform),
        sensors: entity.sensors,
        wheels: entity.wheels,
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
    query: Query<(&VizEntity, &Transform)>,
) {
    let tick_rate = 1.0 / fixed_time.timestep().as_secs_f32();
    while let Ok(event) = viz.0.events.try_recv() {
        let viz::VizEvent::ViewerConnected { id, .. } = event else {
            continue;
        };
        let entities = query.iter().map(|(e, t)| descriptor(e, t)).collect();
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
pub fn broadcast_spawns(viz: Res<Viz>, query: Query<(&VizEntity, &Transform), Added<VizEntity>>) {
    for (entity, transform) in &query {
        if entity.kind.is_dynamic() {
            let event = viz::SceneEvent::EntitySpawned(descriptor(entity, transform));
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

/// Every `TICKS_PER_FRAME` physics ticks, broadcasts a scene frame
/// (positions/orientations) plus a debug frame (plans + reflex flags),
/// stamped with the current tick so the viewer can interpolate on sim-time.
/// What a frame needs from each entity. Named because it is five items wide:
/// the identity and pose that go in the scene layer, the plan and reflex state
/// that go in the debug layer, and the wheels, which straddle both.
type FrameQuery<'a> = (
    &'a VizEntity,
    &'a Transform,
    Option<&'a Plan>,
    Option<&'a DesiredVelocity>,
    Option<&'a movement::Wheels>,
);

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
    for (entity, transform, plan, desired, wheels) in &query {
        if !entity.kind.is_dynamic() {
            continue;
        }
        // Geometry a viewer cannot derive: a locked wheel's spin is not the
        // body's speed, and suspension travel is invisible from outside.
        let wheel_poses = wheels
            .map(|w| {
                w.0.iter()
                    .map(|wheel| viz::WheelPose {
                        steer: wheel.steer,
                        spin: wheel.angle,
                        travel: wheel.compression,
                        load: wheel.load,
                    })
                    .collect()
            })
            .unwrap_or_default();
        frame.push(viz::EntityFrame {
            id: entity.id.clone(),
            transform: to_transform(transform),
            wheels: wheel_poses,
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
