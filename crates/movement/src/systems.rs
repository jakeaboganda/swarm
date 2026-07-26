use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, Velocity};

use crate::model::{DesiredVelocity, MovementModel};

/// Drives physics for every entity with movement model `M`. Monomorphized
/// per concrete component type (registered once per embodiment in
/// `MovementPlugin`) rather than dispatched via `Box<dyn MovementModel>` —
/// keeps entities in contiguous per-archetype storage.
pub fn apply_movement_force<M: MovementModel>(
    mut query: Query<(&M, &DesiredVelocity, &Velocity, &mut ExternalForce)>,
) {
    for (model, desired, velocity, mut force) in &mut query {
        // Overwrite, not accumulate: ExternalForce persists across ticks in
        // bevy_rapier3d, so leaving this as `+=` would compound forever.
        force.force = model.compute_force(*desired, velocity.linear);
    }
}

/// Cosmetic only: rotates the rendered mesh to face the direction of
/// travel. Physics rotation stays locked (see `server`'s spawn setup) —
/// entities don't need a facing direction to move, but visually gliding
/// sideways reads as broken for a self-driving-car mental model.
pub fn face_velocity_direction(mut query: Query<(&Velocity, &mut Transform)>) {
    const MIN_SPEED: f32 = 0.05;
    for (velocity, mut transform) in &mut query {
        let horizontal = Vec3::new(velocity.linear.x, 0.0, velocity.linear.z);
        if horizontal.length_squared() > MIN_SPEED * MIN_SPEED {
            *transform = transform.looking_to(horizontal.normalize(), Vec3::Y);
        }
    }
}
