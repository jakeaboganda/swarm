use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use movement::{CarLike, DesiredVelocity, FullVehicle, Holonomic, PhysicalYaw, RaycastVehicle};
use protocol::scenario::{ArenaConfig, Embodiment, SensorDef, SensorSource};
use transport::ConnectionId;

use crate::agent::{AgentName, Connection, Plan, Reflexes};
use crate::perception_router::Perceiver;
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
const ROAD_COLOR: viz::Color = viz::Color {
    r: 0.20,
    g: 0.20,
    b: 0.22,
};

/// The loaded road network, or `None` for the flat arena world. Selects which
/// world the Startup system builds (see `setup_world`).
#[derive(Resource, Default)]
pub struct MapWorld(pub Option<map::RoadNetwork>);

fn to_viz_vec(v: &Vec3) -> viz::Vec3 {
    viz::Vec3::new(v.x, v.y, v.z)
}

/// Spawns the road as one static trimesh collider, tagged with a `VizEntity`
/// carrying its surface mesh so viewers render it. The baked `RoadNetwork` is
/// the single source; the collider and the viewer see the same triangles.
pub fn spawn_road(commands: &mut Commands, road: &map::RoadNetwork) {
    let mesh = road.surface_mesh();
    let triangles: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect();
    // The mesh is our own generated geometry; a trimesh failure here is a
    // construction bug, not a runtime condition.
    let collider =
        Collider::trimesh(mesh.vertices.clone(), triangles).expect("road surface is a valid mesh");

    commands.spawn((
        RigidBody::Fixed,
        collider,
        Transform::IDENTITY,
        Friction::new(0.9),
        Restitution::new(0.0),
        VizEntity {
            id: viz::EntityId("road".into()),
            name: "road".into(),
            kind: viz::EntityKind::Static,
            shape: viz::Shape::Mesh {
                positions: mesh.vertices.iter().map(to_viz_vec).collect(),
                normals: mesh.normals.iter().map(to_viz_vec).collect(),
                indices: mesh.indices,
            },
            color: ROAD_COLOR,
            sensors: None,
        },
    ));
}

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
            sensors: None,
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
                sensors: None,
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
    sensors: Vec<SensorDef>,
) -> Entity {
    // The debug envelope shows the first simulated device (agents usually have
    // one); drawing several overlapping envelopes is a later refinement.
    let sensor_view = sensors
        .iter()
        .find(|d| d.source == SensorSource::Simulated)
        .and_then(|d| d.spec)
        .map(|s| viz::SensorView {
            range: s.range,
            fov_half_angle: s.fov_half_angle,
        });
    // A raycast vehicle is a box that rides on suspension and faces +X to
    // start; the planar embodiments are capsules at the spawn point.
    let is_car = matches!(embodiment, Embodiment::RaycastVehicle);
    // Place a car ~10 m onto the road (so all four wheels clear the start
    // edge) and at roughly its suspension ride height above the graded surface,
    // so it settles gently instead of launching off over-compressed springs.
    // Proper map-aware spawning onto a lane is P3.
    let position = if is_car {
        Vec3::new(position.x + 10.0, 1.6, position.z)
    } else {
        position
    };
    let (viz_shape, collider) = if is_car {
        (
            viz::Shape::Cuboid {
                half_extents: viz::Vec3::new(0.8, 0.4, 1.4),
            },
            Collider::cuboid(0.8, 0.4, 1.4),
        )
    } else {
        (
            viz::Shape::Capsule {
                radius: AGENT_RADIUS,
                half_length: AGENT_HALF_HEIGHT,
            },
            Collider::capsule_y(AGENT_HALF_HEIGHT, AGENT_RADIUS),
        )
    };
    let transform = if is_car {
        Transform::from_translation(position).looking_to(Vec3::X, Vec3::Y)
    } else {
        Transform::from_translation(position)
    };
    let mut entity = commands.spawn((
        AgentName(name.to_string()),
        Connection(connection),
        Plan::default(),
        Reflexes::default(),
        DesiredVelocity::default(),
        Perceiver(sensors),
        transform,
        VizEntity {
            id: viz::EntityId(name.to_string()),
            name: name.to_string(),
            kind: viz::EntityKind::Agent {
                embodiment: viz_embodiment(embodiment),
            },
            shape: viz_shape,
            color: AGENT_COLOR,
            sensors: sensor_view,
        },
        // Physics components, nested so the whole spawn stays within Bevy's
        // per-tuple bundle element limit.
        (
            RigidBody::Dynamic,
            collider,
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
        Embodiment::FullVehicle => entity.insert((
            FullVehicle::default(),
            // Free yaw only (still no tipping/rolling) so heading is real
            // physics; PhysicalYaw tells the viewer-facing systems not to
            // overwrite it. These override the shared bundle's
            // `ROTATION_LOCKED` and zero angular damping.
            LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z,
            PhysicalYaw,
            // The capsule's own yaw (long-axis) inertia is tiny, so tire
            // torques would spin it instantly. Give it a sane yaw inertia and
            // some yaw damping. Tuned by feel — see DECISIONS.
            ColliderMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                mass: 1.5,
                principal_inertia_local_frame: Quat::IDENTITY,
                principal_inertia: Vec3::new(1.0, 2.0, 1.0),
            }),
            Damping {
                linear_damping: 0.75,
                angular_damping: 3.0,
            },
        )),
        Embodiment::RaycastVehicle => entity.insert((
            RaycastVehicle::default(),
            // Full 3D rotation: real roll and pitch on terrain and banking,
            // yaw from tire forces. Overrides the shared `ROTATION_LOCKED`.
            LockedAxes::empty(),
            PhysicalYaw,
            // A denser chassis (~14 kg) with the box's own inertia, so wheel
            // torques don't spin the light body and make the tire grip react
            // violently to its own rotation.
            ColliderMassProperties::Density(4.0),
            // A car coasts (little linear drag); some angular damping keeps it
            // from spinning up under tire torques. Tuned by feel.
            Damping {
                linear_damping: 0.2,
                angular_damping: 0.5,
            },
            // The chassis floats on suspension and rarely touches the ground;
            // give it grip for the times it bottoms out. Wheels grip via forces
            // regardless. Overrides the shared frictionless setting.
            Friction {
                coefficient: 0.8,
                combine_rule: CoefficientCombineRule::Average,
            },
        )),
    };
    entity.id()
}
