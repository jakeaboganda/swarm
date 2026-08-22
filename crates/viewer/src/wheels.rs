//! Rendering a vehicle's wheels.
//!
//! The rig (radius, width, suspension length, where the four wheels attach)
//! arrives once with the entity; each wheel's pose arrives every frame. Wheels
//! are spawned as *children* of the body, so the body's own interpolated
//! transform carries them and the playback clock needs to know nothing about
//! them.
//!
//! The poses are applied as they arrive rather than interpolated. Steer and
//! suspension travel move by tiny amounts between frames, and spin aliases at
//! speed however it is handled -- so interpolating them would add machinery to
//! the most delicate code in the viewer to fix something nobody can see.

use bevy::prelude::*;

/// Colour of a wheel that is rolling normally.
const ROLLING: Color = Color::srgb(0.10, 0.10, 0.12);
/// A locked wheel: stopped while the car is still moving.
const LOCKED: Color = Color::srgb(0.85, 0.15, 0.15);
/// A spinning wheel: turning faster than the road is passing.
const SPINNING: Color = Color::srgb(0.20, 0.45, 0.95);
/// Off the ground, so doing nothing at all.
const AIRBORNE: Color = Color::srgb(0.35, 0.35, 0.40);

/// Slip at or below this is a locked wheel; at or above the positive one it is
/// wheelspin. Well inside the +/-1 extremes, so a tint means "clearly
/// slipping" rather than "rounding error".
const LOCKED_SLIP: f32 = -0.5;
const SPINNING_SLIP: f32 = 0.5;

/// The latest wheel state for one vehicle, mirrored from the viz stream. Lives
/// on the body; its wheel children read it.
#[derive(Component, Default)]
pub struct WheelState {
    pub poses: Vec<viz::WheelPose>,
    pub diagnostics: Vec<viz::WheelDebug>,
}

/// One rendered wheel: which rig slot it is, and where it sits when the
/// suspension is fully extended.
#[derive(Component)]
pub struct Wheel {
    pub index: usize,
    /// Body-local centre at full extension: the attach point, dropped by the
    /// suspension's rest length. Compression raises the wheel from here.
    pub extended: Vec3,
}

/// A Bevy cylinder stands on its end, so a wheel is turned a quarter turn to
/// put its axis along the body's lateral axis. Spin is then a rotation about
/// that same axis.
fn upright_to_axle() -> Quat {
    Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)
}

/// Builds the four wheels of `rig` as children of `body`.
pub fn spawn_wheels(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    body: Entity,
    rig: &viz::WheelRig,
) {
    let mesh = meshes.add(Cylinder::new(rig.radius, rig.width));
    for (index, offset) in rig.offsets.iter().enumerate() {
        // Its own material per wheel, because the tint is per-wheel state.
        let material = materials.add(StandardMaterial {
            base_color: ROLLING,
            perceptual_roughness: 0.9,
            ..default()
        });
        let extended = Vec3::new(offset.x, offset.y - rig.rest, offset.z);
        let wheel = commands
            .spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material),
                Transform::from_translation(extended).with_rotation(upright_to_axle()),
                Wheel { index, extended },
            ))
            .id();
        commands.entity(body).add_child(wheel);
    }
}

/// Where a wheel's centre sits, body-local, for a given suspension travel.
///
/// `travel` is compression from full extension, so it *raises* the wheel
/// relative to the body -- the body is what has come down.
pub fn wheel_translation(extended: Vec3, travel: f32) -> Vec3 {
    Vec3::new(extended.x, extended.y + travel, extended.z)
}

/// The rotation that steers a wheel and spins it about its axle.
pub fn wheel_rotation(steer: f32, spin: f32) -> Quat {
    Quat::from_rotation_y(steer) * Quat::from_rotation_x(spin) * upright_to_axle()
}

/// Poses each wheel from its vehicle's latest frame: raised by the suspension's
/// compression, turned by the steering, spun by its own angle.
pub fn pose_wheels(
    vehicles: Query<(&WheelState, &Children)>,
    mut wheels: Query<(&Wheel, &mut Transform)>,
) {
    for (state, children) in &vehicles {
        for &child in children {
            let Ok((wheel, mut transform)) = wheels.get_mut(child) else {
                continue;
            };
            let Some(pose) = state.poses.get(wheel.index) else {
                continue;
            };
            transform.translation = wheel_translation(wheel.extended, pose.travel);
            transform.rotation = wheel_rotation(pose.steer, pose.spin);
        }
    }
}

/// Tints each wheel by what it is doing, so lockup and wheelspin are visible
/// rather than inferred. Without it the only window into the tire model is a
/// per-tick dump on stderr.
pub fn tint_wheels(
    vehicles: Query<(&WheelState, &Children)>,
    wheels: Query<(&Wheel, &MeshMaterial3d<StandardMaterial>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (state, children) in &vehicles {
        for &child in children {
            let Ok((wheel, handle)) = wheels.get(child) else {
                continue;
            };
            let Some(diagnostic) = state.diagnostics.get(wheel.index) else {
                continue;
            };
            let Some(mut material) = materials.get_mut(&handle.0) else {
                continue;
            };
            material.base_color = if !diagnostic.contact {
                AIRBORNE
            } else if diagnostic.slip_ratio <= LOCKED_SLIP {
                LOCKED
            } else if diagnostic.slip_ratio >= SPINNING_SLIP {
                SPINNING
            } else {
                ROLLING
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compression_raises_the_wheel_relative_to_the_body() {
        // The body comes down onto its springs, so body-local the wheel goes
        // up. Getting this backwards puts the wheels through the floor at
        // rest, which looks like a physics bug and is not one.
        let extended = Vec3::new(0.8, -0.55, -1.3);
        assert_eq!(wheel_translation(extended, 0.0), extended);
        let compressed = wheel_translation(extended, 0.06);
        assert!(compressed.y > extended.y);
        assert_eq!((compressed.x, compressed.z), (extended.x, extended.z));
    }

    #[test]
    fn a_wheels_axle_points_across_the_body() {
        // The cylinder is modelled standing on its end; unturned it would roll
        // sideways. Its axis must end up along the body's lateral axis.
        let axle = wheel_rotation(0.0, 0.0) * Vec3::Y;
        assert!(
            axle.dot(Vec3::X).abs() > 0.99,
            "axle points {axle:?}, not across the body"
        );
    }

    #[test]
    fn steering_turns_the_wheel_about_the_body_up_axis() {
        // A steered wheel's axle swings in the ground plane; it must not tilt.
        let axle = wheel_rotation(0.4, 0.0) * Vec3::Y;
        assert!(axle.y.abs() < 1e-5, "steering tilted the axle: {axle:?}");
        assert!(axle.dot(Vec3::X).abs() < 0.99, "steering did nothing");
    }

    #[test]
    fn spin_turns_the_wheel_about_its_own_axle() {
        // Spinning must leave the axle where it is -- only the tread moves.
        let axle = wheel_rotation(0.0, 2.0) * Vec3::Y;
        assert!(
            axle.dot(Vec3::X).abs() > 0.99,
            "spin moved the axle to {axle:?}"
        );
        let tread = wheel_rotation(0.0, std::f32::consts::FRAC_PI_2) * Vec3::Z;
        assert!(
            (tread.dot(Vec3::Z)).abs() < 0.01,
            "a quarter turn of spin did not move the tread: {tread:?}"
        );
    }
}
