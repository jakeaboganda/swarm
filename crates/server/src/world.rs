use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use movement::{DesiredVelocity, Holonomic};
use protocol::scenario::ArenaConfig;
use transport::ConnectionId;

use crate::agent::{AgentName, Connection, Plan, Reflexes};

const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
const GROUND_HALF_THICKNESS: f32 = 0.1;
pub const AGENT_RADIUS: f32 = 0.5;
const AGENT_HALF_HEIGHT: f32 = 0.5;

/// Spawns the ground plane, four bounding walls, a light, and a camera
/// framing the arena — the "walls first" world description, loaded from
/// the scenario JSON's `arena` section.
pub fn spawn_arena(commands: &mut Commands, arena: &ArenaConfig) {
    let half_width = arena.width / 2.0;
    let half_depth = arena.depth / 2.0;

    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(half_width, GROUND_HALF_THICKNESS, half_depth),
        Transform::from_xyz(0.0, -GROUND_HALF_THICKNESS, 0.0),
        Friction::new(0.5),
        Restitution::new(0.1),
    ));

    let wall_half_thickness = WALL_THICKNESS / 2.0;

    let spawn_wall = |commands: &mut Commands, x: f32, z: f32, half_x: f32, half_z: f32| {
        commands.spawn((
            RigidBody::Fixed,
            Collider::cuboid(half_x, WALL_HEIGHT / 2.0, half_z),
            Transform::from_xyz(x, WALL_HEIGHT / 2.0, z),
            Friction::new(0.5),
            Restitution::new(0.1),
        ));
    };

    spawn_wall(
        commands,
        0.0,
        -half_depth - wall_half_thickness,
        half_width + WALL_THICKNESS,
        wall_half_thickness,
    );
    spawn_wall(
        commands,
        0.0,
        half_depth + wall_half_thickness,
        half_width + WALL_THICKNESS,
        wall_half_thickness,
    );
    spawn_wall(
        commands,
        -half_width - wall_half_thickness,
        0.0,
        wall_half_thickness,
        half_depth + WALL_THICKNESS,
    );
    spawn_wall(
        commands,
        half_width + wall_half_thickness,
        0.0,
        wall_half_thickness,
        half_depth + WALL_THICKNESS,
    );

    commands.spawn((
        DirectionalLight {
            illuminance: 6_000.0,
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
}

/// Spawns one agent's entity: a dynamic, ground-constrained capsule body
/// with the `Holonomic` movement model. Rotation is fully locked — v1
/// doesn't need a facing direction for physics (see `movement`'s cosmetic
/// `face_velocity_direction` system for the visual-only yaw).
pub fn spawn_agent(
    commands: &mut Commands,
    name: &str,
    position: Vec3,
    connection: ConnectionId,
) -> Entity {
    commands
        .spawn((
            AgentName(name.to_string()),
            Connection(connection),
            Plan::default(),
            Reflexes::default(),
            DesiredVelocity::default(),
            Holonomic::default(),
            RigidBody::Dynamic,
            Collider::capsule_y(AGENT_HALF_HEIGHT, AGENT_RADIUS),
            Transform::from_translation(position),
            Velocity::zero(),
            ExternalForce::default(),
            LockedAxes::ROTATION_LOCKED,
            Damping {
                linear_damping: 0.75,
                angular_damping: 0.0,
            },
            // Frictionless against the ground: the agent is a planar mover,
            // and ground Coulomb friction would impose a stiction floor that
            // a proportional controller can't overcome, leaving a dead-band
            // where low commanded speeds produce no motion at all. `Min`
            // combine means the agent contributes zero regardless of the
            // other collider's friction. Deceleration comes from linear
            // damping and the controller, not the surface.
            Friction {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
            Restitution::new(0.1),
        ))
        .id()
}
