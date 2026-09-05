use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, Velocity};

use crate::model::{BodyState, DesiredVelocity, MovementModel, PhysicalYaw};

/// Drives physics for every entity with movement model `M`. Monomorphized
/// per concrete component type (registered once per embodiment in
/// `MovementPlugin`) rather than dispatched via `Box<dyn MovementModel>` --
/// keeps entities in contiguous per-archetype storage.
pub fn apply_movement_force<M: MovementModel>(
    time: Res<Time>,
    mut query: Query<(
        &mut M,
        &DesiredVelocity,
        &Velocity,
        &Transform,
        &mut ExternalForce,
    )>,
) {
    let dt = time.delta_secs();
    for (mut model, desired, velocity, transform, mut force) in &mut query {
        let forward = transform.forward();
        let heading = Vec3::new(forward.x, 0.0, forward.z).normalize_or(Vec3::X);
        let body = BodyState {
            velocity: velocity.linear,
            yaw_rate: velocity.angular.y,
            heading,
        };
        let actuation = model.drive(*desired, body, dt);
        // Overwrite, not accumulate: ExternalForce persists across ticks in
        // bevy_rapier3d, so leaving these as `+=` would compound forever.
        force.force = actuation.force;
        force.torque = Vec3::Y * actuation.yaw_torque;
    }
}

/// Cosmetic only: rotates the rendered mesh to face the direction of
/// travel. Physics rotation stays locked for the fake-yaw models (see
/// `server`'s spawn setup) -- entities don't need a facing direction to move,
/// but visually gliding sideways reads as broken for a self-driving-car
/// mental model. Skips `PhysicalYaw` bodies, whose orientation is real
/// physics and must not be overwritten.
pub fn face_velocity_direction(
    mut query: Query<(&Velocity, &mut Transform), Without<PhysicalYaw>>,
) {
    const MIN_SPEED: f32 = 0.05;
    for (velocity, mut transform) in &mut query {
        let horizontal = Vec3::new(velocity.linear.x, 0.0, velocity.linear.z);
        if horizontal.length_squared() > MIN_SPEED * MIN_SPEED {
            *transform = transform.looking_to(horizontal.normalize(), Vec3::Y);
        }
    }
}
