use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use movement::{CarLike, DesiredVelocity, Holonomic};
use protocol::scenario::{ArenaConfig, Embodiment};
use transport::ConnectionId;

use crate::agent::{AgentName, Connection, Plan, Reflexes};
use crate::viz_broadcast::{viz_embodiment, VizEntity};

const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
const GROUND_HALF_THICKNESS: f32 = 0.1;
pub const AGENT_RADIUS: f32 = 0.5;
const AGENT_HALF_HEIGHT: f32 = 0.5;

// Colors are viz metadata, not rendering: the headless sim never draws
// anything, it just tells viewers what color each entity should be.
const GROUND_COLOR: viz::Color = viz::Color {
    r: 0.26,
    g: 0.28,
    b: 0.32,
};
const WALL_COLOR: viz::Color = viz::Color {
    r: 0.45,
    g: 0.47,
    b: 0.52,
};
const AGENT_COLOR: viz::Color = viz::Color {
    r: 0.95,
    g: 0.45,
    b: 0.1,
};

/// Spawns the ground and four bounding walls as static physics bodies, each
/// tagged with a `VizEntity` so viewers can render it. No meshes, camera, or
/// light — the sim is headless; rendering lives in the viewer.
pub fn spawn_arena(commands: &mut Commands, arena: &ArenaConfig) {
    let half_width = arena.width / 2.0;
    let half_depth = arena.depth / 2.0;

    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(half_width, GROUND_HALF_THICKNESS, half_depth),
        Transform::from_xyz(0.0, -GROUND_HALF_THICKNESS, 0.0),
        Friction::new(0.5),
        Restitution::new(0.1),
        VizEntity {
            id: viz::EntityId("ground".into()),
            name: "ground".into(),
            kind: viz::EntityKind::Static,
            shape: viz::Shape::Cuboid {
                half_extents: viz::Vec3::new(half_width, GROUND_HALF_THICKNESS, half_depth),
            },
            color: GROUND_COLOR,
        },
    ));

    let wall_half_thickness = WALL_THICKNESS / 2.0;
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
    for (index, (x, z, half_x, half_z)) in walls.into_iter().enumerate() {
        commands.spawn((
            RigidBody::Fixed,
            Collider::cuboid(half_x, WALL_HEIGHT / 2.0, half_z),
            Transform::from_xyz(x, WALL_HEIGHT / 2.0, z),
            Friction::new(0.5),
            Restitution::new(0.1),
            VizEntity {
                id: viz::EntityId(format!("wall-{index}")),
                name: format!("wall-{index}"),
                kind: viz::EntityKind::Static,
                shape: viz::Shape::Cuboid {
                    half_extents: viz::Vec3::new(half_x, WALL_HEIGHT / 2.0, half_z),
                },
                color: WALL_COLOR,
            },
        ));
    }
}

/// Spawns one agent's entity: a dynamic, ground-constrained capsule body
/// with the movement model matching its declared `embodiment`, tagged with a
/// `VizEntity` for viewers. Rotation is fully locked — models steer
/// kinematically; the visual yaw viewers render comes from `movement`'s
/// `face_velocity_direction`, which sets the transmitted Transform rotation.
pub fn spawn_agent(
    commands: &mut Commands,
    name: &str,
    position: Vec3,
    connection: ConnectionId,
    embodiment: Embodiment,
) -> Entity {
    let mut entity = commands.spawn((
        AgentName(name.to_string()),
        Connection(connection),
        Plan::default(),
        Reflexes::default(),
        DesiredVelocity::default(),
        Transform::from_translation(position),
        VizEntity {
            id: viz::EntityId(name.to_string()),
            name: name.to_string(),
            kind: viz::EntityKind::Agent {
                embodiment: viz_embodiment(embodiment),
            },
            shape: viz::Shape::Capsule {
                radius: AGENT_RADIUS,
                half_length: AGENT_HALF_HEIGHT,
            },
            color: AGENT_COLOR,
        },
        // Physics components, nested so the whole spawn stays within Bevy's
        // per-tuple bundle element limit.
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
            // Frictionless against the ground: the agent is a planar mover,
            // and ground Coulomb friction would impose a stiction floor a
            // proportional controller can't overcome, leaving a dead-band
            // where low commanded speeds produce no motion. `Min` combine
            // means the agent contributes zero regardless of the other
            // collider's friction. Deceleration comes from linear damping
            // and the controller, not the surface.
            Friction {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
            Restitution::new(0.1),
        ),
    ));
    // The movement model is a distinct component type per embodiment, so it
    // is inserted here rather than in the shared bundle above.
    match embodiment {
        Embodiment::Holonomic => entity.insert(Holonomic::default()),
        Embodiment::CarLike => entity.insert(CarLike::default()),
    };
    entity.id()
}
