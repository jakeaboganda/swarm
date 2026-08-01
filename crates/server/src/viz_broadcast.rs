use std::time::Duration;

use bevy::prelude::*;
use movement::DesiredVelocity;

use crate::agent::Plan;
use crate::scenario::ArenaBounds;
use crate::scenario_state::{ScenarioState, Tick};

/// Frames per second on the viz stream, decoupled from the physics tick and
/// the render frame rate.
const VIZ_FRAME_HZ: f32 = 30.0;

/// The viz broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Viz(pub viz::VizHandle);

/// Gates the frame stream to `VIZ_FRAME_HZ`.
#[derive(Resource)]
pub struct VizFrameTimer(pub Timer);

impl Default for VizFrameTimer {
    fn default() -> Self {
        Self(Timer::new(
            Duration::from_secs_f32(1.0 / VIZ_FRAME_HZ),
            TimerMode::Repeating,
        ))
    }
}

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
}

pub fn viz_embodiment(embodiment: protocol::scenario::Embodiment) -> viz::Embodiment {
    match embodiment {
        protocol::scenario::Embodiment::Holonomic => viz::Embodiment::Holonomic,
        protocol::scenario::Embodiment::CarLike => viz::Embodiment::CarLike,
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
        shape: entity.shape,
        color: entity.color,
        transform: to_transform(transform),
    }
}

/// Sends a fresh scene-init to each newly connected viewer so it catches up
/// before following the live stream.
pub fn drain_viz_events(
    mut viz: ResMut<Viz>,
    bounds: Res<ArenaBounds>,
    state: Res<State<ScenarioState>>,
    tick: Res<Tick>,
    query: Query<(&VizEntity, &Transform)>,
) {
    while let Ok(event) = viz.0.events.try_recv() {
        let viz::VizEvent::ViewerConnected { id, .. } = event else {
            continue;
        };
        let entities = query.iter().map(|(e, t)| descriptor(e, t)).collect();
        let scene = viz::ServerToViewer::SceneInit(viz::SceneInit {
            protocol_version: viz::PROTOCOL_VERSION,
            tick: tick.0,
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

/// Samples dynamic entities at `VIZ_FRAME_HZ` and broadcasts a scene frame
/// (positions/orientations) plus a debug frame (plans + reflex flags).
pub fn broadcast_frames(
    time: Res<Time>,
    mut timer: ResMut<VizFrameTimer>,
    viz: Res<Viz>,
    tick: Res<Tick>,
    query: Query<(
        &VizEntity,
        &Transform,
        Option<&Plan>,
        Option<&DesiredVelocity>,
    )>,
) {
    if !timer.0.tick(time.delta()).just_finished() {
        return;
    }

    let mut frame = Vec::new();
    let mut debug = Vec::new();
    for (entity, transform, plan, desired) in &query {
        if !entity.kind.is_dynamic() {
            continue;
        }
        frame.push(viz::EntityFrame {
            id: entity.id.clone(),
            transform: to_transform(transform),
        });
        let plan_points = plan
            .map(|p| {
                p.waypoints
                    .iter()
                    .map(|w| viz::Vec3::new(w.position.x, w.position.y, w.position.z))
                    .collect()
            })
            .unwrap_or_default();
        debug.push(viz::EntityDebug {
            id: entity.id.clone(),
            plan: plan_points,
            reflex_active: desired.map(|d| d.urgent).unwrap_or(false),
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
