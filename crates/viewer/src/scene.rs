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
    let mut entity = commands.spawn((
        Mesh3d(mesh_for(&descriptor.shape, meshes)),
        MeshMaterial3d(material_for(descriptor, materials)),
        to_transform(&descriptor.transform),
    ));
    if descriptor.kind.is_dynamic() {
        entity.insert((DebugData::default(), Trail::default()));
    }
    map.0.insert(descriptor.id.clone(), entity.id());
}

/// Drains the viz stream and applies it to the rendered world: (re)builds
/// the scene on `SceneInit`, adds/removes entities on lifecycle events, and
/// updates transforms / debug data from frames. Defensive against the
/// stream: frames for unknown ids are ignored, and spawns are idempotent.
#[allow(clippy::too_many_arguments)]
pub fn apply_stream(
    mut stream: ResMut<VizStream>,
    mut commands: Commands,
    mut map: ResMut<EntityMap>,
    mut state: ResMut<ViewerState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut transforms: Query<&mut Transform>,
    mut debug: Query<&mut DebugData>,
) {
    while let Ok(message) = stream.0.try_recv() {
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
            ServerToViewer::Frame(frame) => {
                for entity_frame in &frame.entities {
                    if let Some(&entity) = map.0.get(&entity_frame.id) {
                        if let Ok(mut transform) = transforms.get_mut(entity) {
                            *transform = to_transform(&entity_frame.transform);
                        }
                    }
                }
            }
            ServerToViewer::DebugFrame(debug_frame) => {
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
