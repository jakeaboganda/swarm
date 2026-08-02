use std::collections::HashMap;

use bevy::prelude::*;
use viz::{EntityDescriptor, EntityId, SceneEvent, ServerToViewer, Shape};

use crate::client::VizStream;
use crate::overlay::{DebugData, Trail};

/// Maps a viz `EntityId` to the Bevy entity rendering it.
#[derive(Resource, Default)]
pub struct EntityMap(HashMap<EntityId, Entity>);

/// Latest scenario/arena info from the stream.
#[derive(Resource, Default)]
pub struct ViewerState {
    pub scenario: Option<viz::ScenarioState>,
    pub arena: Option<viz::ArenaBounds>,
}

/// Smoothly moves an entity from where it was toward the newest frame over
/// the measured time between frames, so 30 Hz frame updates render as
/// continuous motion instead of discrete jumps.
#[derive(Component)]
pub struct Interp {
    previous: Transform,
    target: Transform,
    elapsed: f32,
    interval: f32,
}

impl Interp {
    fn new(transform: Transform) -> Self {
        Self {
            previous: transform,
            target: transform,
            elapsed: 0.0,
            interval: 1.0 / 30.0,
        }
    }

    /// Aim at a new frame, starting from `current` (where the entity is
    /// right now) so there's no jump if the previous interpolation hadn't
    /// finished. `elapsed` since the last frame is the real inter-frame
    /// interval, so playback tracks the actual (jittery) frame rate.
    fn retarget(&mut self, current: Transform, target: Transform) {
        self.previous = current;
        self.target = target;
        self.interval = self.elapsed.clamp(1.0 / 240.0, 1.0 / 10.0);
        self.elapsed = 0.0;
    }
}

fn to_transform(transform: &viz::Transform) -> Transform {
    Transform {
        translation: Vec3::new(
            transform.position.x,
            transform.position.y,
            transform.position.z,
        ),
        rotation: Quat::from_xyzw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
        scale: Vec3::ONE,
    }
}

fn mesh_for(shape: &Shape, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    match shape {
        Shape::Capsule {
            radius,
            half_length,
        } => meshes.add(Capsule3d::new(*radius, half_length * 2.0)),
        Shape::Cuboid { half_extents } => meshes.add(Cuboid::new(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        )),
    }
}

fn material_for(
    descriptor: &EntityDescriptor,
    materials: &mut Assets<StandardMaterial>,
) -> Handle<StandardMaterial> {
    let color = descriptor.color;
    // Give dynamic entities a little emissive lift so they read as the
    // "actors" against the static geometry.
    let emissive = if descriptor.kind.is_dynamic() {
        LinearRgba::rgb(color.r * 0.25, color.g * 0.25, color.b * 0.25)
    } else {
        LinearRgba::BLACK
    };
    materials.add(StandardMaterial {
        base_color: Color::srgb(color.r, color.g, color.b),
        emissive,
        ..default()
    })
}

fn spawn_entity(
    commands: &mut Commands,
    map: &mut EntityMap,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    descriptor: &EntityDescriptor,
) {
    let transform = to_transform(&descriptor.transform);
    let mut entity = commands.spawn((
        Mesh3d(mesh_for(&descriptor.shape, meshes)),
        MeshMaterial3d(material_for(descriptor, materials)),
        transform,
    ));
    if descriptor.kind.is_dynamic() {
        entity.insert((DebugData::default(), Trail::default(), Interp::new(transform)));
    }
    map.0.insert(descriptor.id.clone(), entity.id());
}

/// Drains the viz stream and applies it to the rendered world: (re)builds
/// the scene on `SceneInit`, adds/removes entities on lifecycle events, then
/// applies the latest frame / debug frame. Defensive against the stream:
/// frames for unknown ids are ignored, and spawns are idempotent.
#[allow(clippy::too_many_arguments)]
pub fn apply_stream(
    mut stream: ResMut<VizStream>,
    mut commands: Commands,
    mut map: ResMut<EntityMap>,
    mut state: ResMut<ViewerState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut frame_targets: Query<(&Transform, &mut Interp)>,
    mut debug: Query<&mut DebugData>,
) {
    // Reliable, ordered messages first (scene-init resets before any frame).
    while let Ok(message) = stream.reliable.try_recv() {
        match message {
            ServerToViewer::SceneInit(init) => {
                // A scene-init is a full reset (also how a reconnect
                // re-syncs after a sim restart).
                for (_, entity) in map.0.drain() {
                    commands.entity(entity).despawn();
                }
                info!(
                    "scene-init: {} entities, state {:?}",
                    init.entities.len(),
                    init.state
                );
                state.arena = Some(init.arena);
                state.scenario = Some(init.state);
                for descriptor in &init.entities {
                    spawn_entity(
                        &mut commands,
                        &mut map,
                        &mut meshes,
                        &mut materials,
                        descriptor,
                    );
                }
            }
            ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor)) => {
                if !map.0.contains_key(&descriptor.id) {
                    spawn_entity(
                        &mut commands,
                        &mut map,
                        &mut meshes,
                        &mut materials,
                        &descriptor,
                    );
                }
            }
            ServerToViewer::Event(SceneEvent::EntityDespawned { id }) => {
                if let Some(entity) = map.0.remove(&id) {
                    commands.entity(entity).despawn();
                }
            }
            ServerToViewer::Event(SceneEvent::ScenarioState { state: scenario }) => {
                state.scenario = Some(scenario);
            }
            // Frames/debug frames are keep-latest (below), never reliable.
            ServerToViewer::Frame(_) | ServerToViewer::DebugFrame(_) => {}
        }
    }

    // Then the newest frame (keep-latest: intermediate frames are coalesced
    // away, so the queue can never grow). Each frame retargets the
    // interpolation rather than snapping the transform.
    if stream.frame.has_changed().unwrap_or(false) {
        if let Some(frame) = stream.frame.borrow_and_update().clone() {
            for entity_frame in &frame.entities {
                if let Some(&entity) = map.0.get(&entity_frame.id) {
                    if let Ok((transform, mut interp)) = frame_targets.get_mut(entity) {
                        interp.retarget(*transform, to_transform(&entity_frame.transform));
                    }
                }
            }
        }
    }

    if stream.debug.has_changed().unwrap_or(false) {
        if let Some(debug_frame) = stream.debug.borrow_and_update().clone() {
            for entity_debug in &debug_frame.entities {
                if let Some(&entity) = map.0.get(&entity_debug.id) {
                    if let Ok(mut data) = debug.get_mut(entity) {
                        data.plan = entity_debug
                            .plan
                            .iter()
                            .map(|p| Vec3::new(p.x, p.y, p.z))
                            .collect();
                        data.reflex_active = entity_debug.reflex_active;
                    }
                }
            }
        }
    }
}

/// Advances each entity's interpolation toward the latest frame, producing
/// smooth motion between the 30 Hz frames.
pub fn interpolate(time: Res<Time>, mut query: Query<(&mut Transform, &mut Interp)>) {
    let dt = time.delta_secs();
    for (mut transform, mut interp) in &mut query {
        interp.elapsed += dt;
        let alpha = (interp.elapsed / interp.interval).clamp(0.0, 1.0);
        transform.translation = interp
            .previous
            .translation
            .lerp(interp.target.translation, alpha);
        transform.rotation = interp.previous.rotation.slerp(interp.target.rotation, alpha);
    }
}

/// Frames the top-down camera to the arena once its bounds arrive (and if
/// they change between scenarios).
pub fn frame_camera(state: Res<ViewerState>, mut camera: Query<&mut Transform, With<Camera3d>>) {
    if !state.is_changed() {
        return;
    }
    let Some(arena) = state.arena else {
        return;
    };
    let height = arena.width.max(arena.depth) * 0.9;
    if let Ok(mut transform) = camera.single_mut() {
        *transform = Transform::from_xyz(0.0, height, height).looking_at(Vec3::ZERO, Vec3::Y);
    }
}

/// Spawns the camera and light. The camera frames a 50-unit arena from
/// above; scene geometry arrives over the stream.
pub fn setup_camera(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let height = 45.0;
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, height, height).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
