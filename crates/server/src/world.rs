use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use movement::{DesiredVelocity, Holonomic};
use protocol::scenario::ArenaConfig;
use transport::ConnectionId;

use crate::agent::{AgentName, Connection, Plan, Reflexes};
use crate::viz::Trail;

const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
const GROUND_HALF_THICKNESS: f32 = 0.1;
pub const AGENT_RADIUS: f32 = 0.5;
const AGENT_HALF_HEIGHT: f32 = 0.5;

/// Shared render handles created once at startup and reused for every agent
/// spawned later (agents are created on `Join`, where asset access isn't
/// available).
#[derive(Resource)]
pub struct AgentRenderAssets {
    pub mesh: Handle<Mesh>,
    pub material: Handle<StandardMaterial>,
}

/// Spawns the ground, four bounding walls, a light, and a camera framing
/// the arena — each with a render mesh, not just a physics collider (a
/// collider alone draws nothing). Also builds and inserts the shared agent
/// render handles.
pub fn spawn_arena(
    commands: &mut Commands,
    arena: &ArenaConfig,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let half_width = arena.width / 2.0;
    let half_depth = arena.depth / 2.0;

    let ground_material = materials.add(Color::srgb(0.26, 0.28, 0.32));
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(half_width, GROUND_HALF_THICKNESS, half_depth),
        Mesh3d(meshes.add(Cuboid::new(
            arena.width,
            GROUND_HALF_THICKNESS * 2.0,
            arena.depth,
        ))),
        MeshMaterial3d(ground_material),
        Transform::from_xyz(0.0, -GROUND_HALF_THICKNESS, 0.0),
        Friction::new(0.5),
        Restitution::new(0.1),
    ));

    let wall_half_thickness = WALL_THICKNESS / 2.0;
    let wall_material = materials.add(Color::srgb(0.45, 0.47, 0.52));
    // (center_x, center_z, half_x, half_z) for the four walls.
    let walls = [
        (
            0.0,
            -half_depth - wall_half_thickness,
            half_width + WALL_THICKNESS,
            wall_half_thickness,
        ),
        (
            0.0,
            half_depth + wall_half_thickness,
            half_width + WALL_THICKNESS,
            wall_half_thickness,
        ),
        (
            -half_width - wall_half_thickness,
            0.0,
            wall_half_thickness,
            half_depth + WALL_THICKNESS,
        ),
        (
            half_width + wall_half_thickness,
            0.0,
            wall_half_thickness,
            half_depth + WALL_THICKNESS,
        ),
    ];
    for (x, z, half_x, half_z) in walls {
        commands.spawn((
            RigidBody::Fixed,
            Collider::cuboid(half_x, WALL_HEIGHT / 2.0, half_z),
            Mesh3d(meshes.add(Cuboid::new(half_x * 2.0, WALL_HEIGHT, half_z * 2.0))),
            MeshMaterial3d(wall_material.clone()),
            Transform::from_xyz(x, WALL_HEIGHT / 2.0, z),
            Friction::new(0.5),
            Restitution::new(0.1),
        ));
    }

    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let camera_height = arena.width.max(arena.depth) * 0.9;
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, camera_height, camera_height).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(AgentRenderAssets {
        mesh: meshes.add(Capsule3d::new(AGENT_RADIUS, AGENT_HALF_HEIGHT * 2.0)),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.45, 0.1),
            emissive: LinearRgba::rgb(0.25, 0.08, 0.0),
            ..default()
        }),
    });
}

/// Spawns one agent's entity: a dynamic, ground-constrained capsule body
/// with the `Holonomic` movement model and a render mesh. Rotation is fully
/// locked — v1 doesn't need a facing direction for physics (see
/// `movement`'s cosmetic `face_velocity_direction` system for the
/// visual-only yaw).
pub fn spawn_agent(
    commands: &mut Commands,
    name: &str,
    position: Vec3,
    connection: ConnectionId,
    render: &AgentRenderAssets,
) -> Entity {
    commands
        .spawn((
            AgentName(name.to_string()),
            Connection(connection),
            Plan::default(),
            Reflexes::default(),
            DesiredVelocity::default(),
            Holonomic::default(),
            Mesh3d(render.mesh.clone()),
            MeshMaterial3d(render.material.clone()),
            Transform::from_translation(position),
            Trail::default(),
            // Physics components, nested so the whole spawn stays within
            // Bevy's per-tuple bundle element limit.
            (
                RigidBody::Dynamic,
                Collider::capsule_y(AGENT_HALF_HEIGHT, AGENT_RADIUS),
                Velocity::zero(),
                ExternalForce::default(),
                LockedAxes::ROTATION_LOCKED,
                Damping {
                    linear_damping: 0.75,
                    angular_damping: 0.0,
                },
                // Frictionless against the ground: the agent is a planar
                // mover, and ground Coulomb friction would impose a stiction
                // floor that a proportional controller can't overcome,
                // leaving a dead-band where low commanded speeds produce no
                // motion at all. `Min` combine means the agent contributes
                // zero regardless of the other collider's friction.
                // Deceleration comes from linear damping and the controller,
                // not the surface.
                Friction {
                    coefficient: 0.0,
                    combine_rule: CoefficientCombineRule::Min,
                },
                Restitution::new(0.1),
            ),
        ))
        .id()
}
