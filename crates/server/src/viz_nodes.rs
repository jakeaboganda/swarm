//! An entity's viz node tree: how the sim lays it out, and where each node is
//! right now.
//!
//! The sim owns every node transform; a viewer only draws a node where it is
//! told. So the suspension arithmetic lives here, beside the rig it reads —
//! not in a viewer, which is where a stale copy of `suspension_rest` on the
//! wire once drew the wheels 0.2 m above the ground.

use bevy::prelude::*;
use movement::{wheel_offset, RaycastVehicle, WheelState, Wheels};
use viz::{EntityNode, Geometry, NodePath, NodeUpdate};

/// Tyre section width (m). Cosmetic only — nothing in the physics has a notion
/// of how wide a wheel is, so it lives with the rest of what a viewer needs to
/// draw rather than in the vehicle model.
const WHEEL_WIDTH: f32 = 0.22;

/// A Bevy/glTF cylinder stands on its end, so a wheel is turned a quarter turn
/// to lay its axle across the body. Spin is then a rotation about that axle.
fn upright_to_axle() -> Quat {
    Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)
}

/// Where a wheel sits in the body's own frame, given what the suspension and
/// the tire are doing.
///
/// Position: the attach point, dropped by however much suspension is still
/// extended — so compression *raises* the wheel relative to the body, because
/// the body is what has come down. Rotation: steer about the body's up axis,
/// spin about the axle, then the quarter turn that lays the axle across.
pub fn wheel_pose(index: usize, vehicle: &RaycastVehicle, wheel: &WheelState) -> Transform {
    let extension = vehicle.suspension_rest - wheel.compression;
    Transform {
        translation: wheel_offset(index, vehicle) - Vec3::Y * extension,
        rotation: Quat::from_rotation_y(wheel.steer)
            * Quat::from_rotation_x(wheel.angle)
            * upright_to_axle(),
        scale: Vec3::ONE,
    }
}

pub fn to_viz_vec3(v: Vec3) -> viz::Vec3 {
    viz::Vec3::new(v.x, v.y, v.z)
}

pub fn to_viz_transform(transform: &Transform) -> viz::Transform {
    viz::Transform {
        position: to_viz_vec3(transform.translation),
        rotation: viz::Quat::new(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    }
}

/// The four wheel children of a car's root node, in `viz::WHEEL_NODES` order —
/// the same order the drive system and the debug diagnostics use.
pub fn wheel_nodes(vehicle: &RaycastVehicle, wheels: &Wheels) -> Vec<EntityNode> {
    viz::WHEEL_NODES
        .iter()
        .enumerate()
        .map(|(index, name)| {
            EntityNode::new(
                *name,
                to_viz_transform(&wheel_pose(index, vehicle, &wheels.0[index])),
                Geometry::Cylinder {
                    radius: vehicle.wheel_radius,
                    height: WHEEL_WIDTH,
                },
            )
        })
        .collect()
}

/// Where every *non-root* node of an entity is now. Empty for anything without
/// wheels, which is everything but a car: its frame is then just its root.
pub fn node_updates(vehicle: Option<&RaycastVehicle>, wheels: Option<&Wheels>) -> Vec<NodeUpdate> {
    let (Some(vehicle), Some(wheels)) = (vehicle, wheels) else {
        return Vec::new();
    };
    viz::WHEEL_NODES
        .iter()
        .enumerate()
        .map(|(index, name)| NodeUpdate {
            path: NodePath::root().child(name),
            transform: to_viz_transform(&wheel_pose(index, vehicle, &wheels.0[index])),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wheel(compression: f32, steer: f32, angle: f32) -> WheelState {
        WheelState {
            compression,
            steer,
            angle,
            ..WheelState::default()
        }
    }

    #[test]
    fn a_fully_extended_wheel_hangs_the_whole_suspension_below_its_attach() {
        // The bug this refactor makes unrepresentable: the wheel centre is the
        // attach point *minus the suspension length*, and whoever knows the
        // rest length has to be whoever does the subtraction. Sending the
        // attach point alone and expecting the far end to subtract drew the
        // wheels a rest-length (0.2 m) up in the air.
        let vehicle = RaycastVehicle::default();
        for index in 0..4 {
            let attach = wheel_offset(index, &vehicle);
            let pose = wheel_pose(index, &vehicle, &wheel(0.0, 0.0, 0.0));
            assert_eq!(pose.translation.y, attach.y - vehicle.suspension_rest);
            assert_eq!(
                (pose.translation.x, pose.translation.z),
                (attach.x, attach.z)
            );
        }
    }

    #[test]
    fn compression_raises_the_wheel_relative_to_the_body() {
        // The body comes down onto its springs, so body-local the wheel goes
        // up. Getting this backwards puts the wheels through the floor at
        // rest, which looks like a physics bug and is not one.
        let vehicle = RaycastVehicle::default();
        let extended = wheel_pose(0, &vehicle, &wheel(0.0, 0.0, 0.0)).translation;
        let compressed = wheel_pose(0, &vehicle, &wheel(0.06, 0.0, 0.0)).translation;
        assert!(compressed.y > extended.y);
        assert!((compressed.y - extended.y - 0.06).abs() < 1e-6);
        assert_eq!((compressed.x, compressed.z), (extended.x, extended.z));
    }

    #[test]
    fn a_wheels_axle_points_across_the_body() {
        // The cylinder is modelled standing on its end; unturned it would roll
        // sideways. Its axis must end up along the body's lateral axis.
        let axle =
            wheel_pose(0, &RaycastVehicle::default(), &wheel(0.0, 0.0, 0.0)).rotation * Vec3::Y;
        assert!(
            axle.dot(Vec3::X).abs() > 0.99,
            "axle points {axle:?}, not across the body"
        );
    }

    #[test]
    fn steering_turns_the_wheel_about_the_body_up_axis() {
        // A steered wheel's axle swings in the ground plane; it must not tilt.
        let axle =
            wheel_pose(0, &RaycastVehicle::default(), &wheel(0.0, 0.4, 0.0)).rotation * Vec3::Y;
        assert!(axle.y.abs() < 1e-5, "steering tilted the axle: {axle:?}");
        assert!(axle.dot(Vec3::X).abs() < 0.99, "steering did nothing");
    }

    #[test]
    fn spin_turns_the_wheel_about_its_own_axle() {
        // Spinning must leave the axle where it is -- only the tread moves.
        let vehicle = RaycastVehicle::default();
        let spun = wheel_pose(0, &vehicle, &wheel(0.0, 0.0, 2.0)).rotation;
        assert!(
            (spun * Vec3::Y).dot(Vec3::X).abs() > 0.99,
            "spin moved the axle to {:?}",
            spun * Vec3::Y
        );
        let quarter =
            wheel_pose(0, &vehicle, &wheel(0.0, 0.0, std::f32::consts::FRAC_PI_2)).rotation;
        let tread = quarter * Vec3::Z;
        assert!(
            tread.dot(Vec3::Z).abs() < 0.01,
            "a quarter turn of spin did not move the tread: {tread:?}"
        );
    }

    #[test]
    fn a_car_gets_four_wheel_nodes_and_everything_else_gets_none() {
        let vehicle = RaycastVehicle::default();
        let wheels = Wheels::default();
        let nodes = wheel_nodes(&vehicle, &wheels);
        let names: Vec<&str> = nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(names, viz::WHEEL_NODES.to_vec());
        assert!(nodes.iter().all(|n| matches!(
            n.geometry,
            Some(Geometry::Cylinder { radius, height })
                if radius == vehicle.wheel_radius && height == WHEEL_WIDTH
        )));

        assert_eq!(node_updates(Some(&vehicle), Some(&wheels)).len(), 4);
        assert!(node_updates(None, None).is_empty(), "a puck has no wheels");
        assert!(node_updates(Some(&vehicle), None).is_empty());
    }
}
