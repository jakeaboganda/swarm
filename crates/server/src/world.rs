use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use movement::{CarLike, DesiredVelocity, FullVehicle, Holonomic, PhysicalYaw, RaycastVehicle};
use protocol::map::{LaneData, LaneDirection, LaneKind as WireLaneKind, MapData};
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
/// A car's perception/collision radius -- a sphere roughly bounding the cuboid
/// chassis, used when it perceives or is perceived (TTC, avoidance). Not tied to
/// `AGENT_RADIUS`, which is the planar-capsule size.
const CAR_RADIUS: f32 = 1.0;

/// An entity's body radius for perception and time-to-collision: how big it is
/// as a sphere, both as a perceiver (`self_radius`) and as a perceived obstacle.
/// Set per entity at spawn so a scaled obstacle or a car is sized correctly,
/// rather than every entity assuming `AGENT_RADIUS`.
#[derive(Component, Clone, Copy)]
pub struct Radius(pub f32);

/// A car's chassis center rides this far above the lane surface at spawn, so it
/// settles gently onto its suspension instead of launching off over-compressed
/// springs (matched to the raycast vehicle's rest geometry).
const CAR_RIDE_HEIGHT: f32 = 1.2;
/// A car spawns this far along its lane (arc length), so all four wheels clear
/// the road's start edge rather than hanging off into space.
const CAR_START_S: f32 = 10.0;
/// When more cars share the map than there are forward lanes, cars that wrap
/// onto the same lane are staggered this far apart (arc length) so they spawn
/// single-file rather than stacked on top of each other.
const CAR_SPACING: f32 = 14.0;
/// An obstacle spawns this far along the lane -- downroad of the car, on the
/// straight, so the car has clear room to perceive it and brake.
const OBSTACLE_START_S: f32 = 32.0;

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

fn to_wire_vec(v: &Vec3) -> protocol::Vec3 {
    protocol::Vec3::new(v.x, v.y, v.z)
}

/// Convert the sim-side road network into the wire form delivered to agents at
/// join. Lanes with their baked centerlines -- everything an agent needs to lay
/// a lane-following path, nothing about the (perceived) dynamic world.
pub fn to_map_data(net: &map::RoadNetwork) -> MapData {
    MapData {
        lanes: net
            .lanes
            .iter()
            .map(|lane| LaneData {
                id: lane.id.0 as u64,
                kind: match lane.kind {
                    map::LaneKind::Driving => WireLaneKind::Driving,
                },
                direction: match lane.direction {
                    map::Direction::Forward => LaneDirection::Forward,
                    map::Direction::Backward => LaneDirection::Backward,
                },
                width: lane.width,
                centerline: lane.center.points().iter().map(to_wire_vec).collect(),
                successors: lane.successors.iter().map(|l| l.0 as u64).collect(),
                predecessors: lane.predecessors.iter().map(|l| l.0 as u64).collect(),
                neighbors: lane.neighbors.iter().map(|l| l.0 as u64).collect(),
            })
            .collect(),
    }
}

/// The first forward driving lane, if the world has a road.
fn forward_lane(map: Option<&map::RoadNetwork>) -> Option<&map::Lane> {
    map.and_then(|net| {
        net.driving_lanes()
            .find(|l| l.direction == map::Direction::Forward)
    })
}

/// A car needs a lane at least this long to spawn safely at `CAR_START_S`.
/// Shorter lanes (junction connectors are often only a few metres) clamp the
/// spawn to the lane's end -- which can hang over a seam between road pieces in
/// the trimesh, so the raycast vehicle finds no surface and falls through.
/// Requiring headroom past the spawn station keeps cars on solid road.
const MIN_SPAWN_LANE_LEN: f32 = CAR_START_S + 6.0;

/// Forward driving lanes long enough to spawn a car on, in network order, for
/// spreading a fleet across the map. Falls back to *all* forward lanes if none
/// clear the length bar (a tiny map), so a car still spawns somewhere.
fn forward_lanes(map: Option<&map::RoadNetwork>) -> Vec<&map::Lane> {
    let all: Vec<&map::Lane> = map
        .map(|net| {
            net.driving_lanes()
                .filter(|l| l.direction == map::Direction::Forward)
                .collect()
        })
        .unwrap_or_default();
    let safe: Vec<&map::Lane> = all
        .iter()
        .copied()
        .filter(|l| l.center.length() >= MIN_SPAWN_LANE_LEN)
        .collect();
    if safe.is_empty() {
        all
    } else {
        safe
    }
}

/// Which forward lane (by position in `forward_lanes`) a car with spawn `index`
/// takes, and how far along it to start. Cars fan out round-robin across the
/// `n_lanes` forward lanes; any that wrap onto an already-used lane are
/// staggered by `CAR_SPACING` so a fleet larger than the lane count still
/// spawns single-file rather than stacked. `n_lanes` must be non-zero.
fn car_placement(n_lanes: usize, index: usize) -> (usize, f32) {
    let lane_idx = index % n_lanes;
    let ring = index / n_lanes;
    (lane_idx, CAR_START_S + ring as f32 * CAR_SPACING)
}

/// Where an agent's entity spawns and which way it faces. In the automotive
/// world both are placed *in a forward lane*: cars fan out across the forward
/// lanes by spawn `index` (`car_placement`), each facing along its lane at ride
/// height, so a fleet doesn't stack on one spot; a non-car -- an obstacle -- is
/// dropped further downroad at `OBSTACLE_START_S`, resting on the surface. A
/// scaled body is lifted by its resting half-height so it settles rather than
/// spawning embedded. Without a map (the arena world) everything spawns at its
/// roster `base`.
pub fn agent_spawn_transform(
    embodiment: Embodiment,
    base: Vec3,
    map: Option<&map::RoadNetwork>,
    scale: f32,
    index: usize,
) -> Transform {
    let rest_half = (AGENT_RADIUS + AGENT_HALF_HEIGHT) * scale;
    if !matches!(embodiment, Embodiment::RaycastVehicle) {
        // A non-car in a road world is an obstacle: drop it onto the lane
        // downroad of the car (heavy scaled bodies can't reliably drive there
        // themselves). Arena world: its roster base, lifted clear of the ground.
        if let Some(lane) = forward_lane(map) {
            let pose = lane.center.pose_at(OBSTACLE_START_S);
            return Transform::from_translation(pose.position + Vec3::Y * (rest_half + 0.2));
        }
        return Transform::from_translation(Vec3::new(base.x, rest_half + 0.2, base.z));
    }
    let lanes = forward_lanes(map);
    if !lanes.is_empty() {
        let (lane_idx, start_s) = car_placement(lanes.len(), index);
        let pose = lanes[lane_idx].center.pose_at(start_s);
        let heading = if pose.heading.length_squared() > 1e-6 {
            pose.heading
        } else {
            Vec3::X
        };
        return Transform::from_translation(pose.position + Vec3::Y * CAR_RIDE_HEIGHT)
            .looking_to(heading, Vec3::Y);
    }
    // No lane to place onto: keep the car above ground and facing +X.
    Transform::from_translation(Vec3::new(base.x, CAR_RIDE_HEIGHT, base.z))
        .looking_to(Vec3::X, Vec3::Y)
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
    // Checked at load by `app::load_map`, which is where an imported (and
    // therefore untrusted) map is rejected with its filename attached. By here
    // a failure would be a construction bug, not a bad input file.
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
// Spawning an agent genuinely needs all of these (world handle, identity,
// pose, connection, and the per-slot embodiment/sensors/color/scale); bundling
// them into a struct would just move the argument list, not shorten it.
#[allow(clippy::too_many_arguments)]
pub fn spawn_agent(
    commands: &mut Commands,
    name: &str,
    transform: Transform,
    connection: ConnectionId,
    embodiment: Embodiment,
    sensors: Vec<SensorDef>,
    color: Option<viz::Color>,
    scale: f32,
) -> Entity {
    let color = color.unwrap_or(AGENT_COLOR);
    // The debug envelope shows the first simulated device (agents usually have
    // one); drawing several overlapping envelopes is a later refinement.
    let sensor_view = sensors
        .iter()
        .find(|d| d.source == SensorSource::Simulated)
        .and_then(|d| d.spec)
        .map(|s| viz::SensorView {
            range: s.range,
            fov_half_angle: s.fov_half_angle,
            vertical_fov_half_angle: s.vertical_fov_half_angle,
        });
    // A raycast vehicle is a box that rides on suspension; the planar
    // embodiments are capsules. The spawn pose is computed by the caller (see
    // `agent_spawn_transform`, which places a car in its lane).
    let is_car = matches!(embodiment, Embodiment::RaycastVehicle);
    // Perception/TTC size: a car is a ~1 m sphere; a planar body is its scaled
    // capsule radius. Threaded so an obstacle is perceived at its true size.
    let body_radius = if is_car {
        CAR_RADIUS
    } else {
        AGENT_RADIUS * scale
    };
    let (viz_shape, collider) = if is_car {
        (
            viz::Shape::Cuboid {
                half_extents: viz::Vec3::new(0.8, 0.4, 1.4),
            },
            Collider::cuboid(0.8, 0.4, 1.4),
        )
    } else {
        // A scaled capsule is the obvious-obstacle case; the raycast chassis
        // above ignores scale (its size is dynamics-tuned).
        let radius = AGENT_RADIUS * scale;
        let half_length = AGENT_HALF_HEIGHT * scale;
        (
            viz::Shape::Capsule {
                radius,
                half_length,
            },
            Collider::capsule_y(half_length, radius),
        )
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
            color,
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
    entity.insert(Radius(body_radius));
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fleet_smaller_than_lanes_gets_a_distinct_lane_each() {
        // With more forward lanes than cars, every car takes its own lane at
        // the same start distance -- nobody wraps or stacks.
        let n_lanes = 8;
        let placements: Vec<_> = (0..5).map(|i| car_placement(n_lanes, i)).collect();
        let lanes: Vec<usize> = placements.iter().map(|(l, _)| *l).collect();
        assert_eq!(lanes, vec![0, 1, 2, 3, 4]);
        for (_, start_s) in placements {
            assert_eq!(start_s, CAR_START_S);
        }
    }

    #[test]
    fn fleet_larger_than_lanes_wraps_and_staggers() {
        // Only two lanes for four cars: they round-robin across the lanes, and
        // the wrapped cars start a `CAR_SPACING` further down so they spawn
        // single-file behind the first pair rather than on top of them.
        let n_lanes = 2;
        assert_eq!(car_placement(n_lanes, 0), (0, CAR_START_S));
        assert_eq!(car_placement(n_lanes, 1), (1, CAR_START_S));
        assert_eq!(car_placement(n_lanes, 2), (0, CAR_START_S + CAR_SPACING));
        assert_eq!(car_placement(n_lanes, 3), (1, CAR_START_S + CAR_SPACING));
    }
}
