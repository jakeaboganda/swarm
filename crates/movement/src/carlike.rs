use bevy::prelude::*;

use crate::model::{DesiredVelocity, MovementModel};

/// Non-holonomic (car-like) movement: the body accelerates only along the
/// direction it faces, turns that heading at a bounded rate (so it makes
/// sweeping turns, not instant ones), and grips laterally so it doesn't
/// slide sideways. Contrast `Holonomic`, which can push in any direction
/// immediately.
#[derive(Component, Clone, Copy, Debug)]
pub struct CarLike {
    /// Current facing, a unit vector in the xz-plane. Evolves over time.
    pub heading: Vec3,
    /// Proportional gain on forward speed error.
    pub gain: f32,
    /// Force ceiling for ordinary driving.
    pub max_force: f32,
    /// Higher force ceiling when a reflex is braking (see `DesiredVelocity`).
    pub brake_max_force: f32,
    /// Maximum heading change, in radians per second.
    pub max_turn_rate: f32,
    /// How strongly lateral (sideways) velocity is cancelled.
    pub grip: f32,
}

impl Default for CarLike {
    fn default() -> Self {
        Self {
            heading: Vec3::X,
            gain: 8.0,
            max_force: 40.0,
            brake_max_force: 120.0,
            max_turn_rate: 2.5,
            grip: 6.0,
        }
    }
}

impl MovementModel for CarLike {
    fn drive(&mut self, desired: DesiredVelocity, current_velocity: Vec3, dt: f32) -> Vec3 {
        let desired_speed = desired.value.length();

        // Steer the heading toward the desired direction, bounded by the
        // turn rate. When braking (desired velocity zero) there's no
        // direction to steer toward, so hold heading and just decelerate.
        if desired_speed > 1e-3 {
            self.heading = turn_toward(self.heading, desired.value, self.max_turn_rate * dt);
        }
        let heading = self.heading;

        // Forward thrust: drive the along-heading speed toward the target.
        let forward_speed = current_velocity.dot(heading);
        let max_force = if desired.urgent {
            self.brake_max_force
        } else {
            self.max_force
        };
        let thrust = ((desired_speed - forward_speed) * self.gain).clamp(-max_force, max_force);
        let forward_force = heading * thrust;

        // Grip: cancel the sideways component of velocity.
        let lateral_velocity = current_velocity - heading * forward_speed;
        let grip_force = -lateral_velocity * self.grip;

        forward_force + grip_force
    }
}

/// Rotate `heading` toward `target` by at most `max_step` radians, within
/// the xz-plane, returning a unit vector. A near-zero `target` leaves the
/// heading unchanged; a `max_step` large enough to reach the target snaps
/// straight to it.
fn turn_toward(heading: Vec3, target: Vec3, max_step: f32) -> Vec3 {
    let target = Vec3::new(target.x, 0.0, target.z);
    if target.length_squared() < 1e-6 {
        return heading;
    }
    let target = target.normalize();
    let heading = Vec3::new(heading.x, 0.0, heading.z).normalize_or(target);

    let angle = heading.dot(target).clamp(-1.0, 1.0).acos();
    if angle <= max_step {
        return target;
    }
    // Sign of the xz cross product tells us which way to turn.
    let cross_y = heading.x * target.z - heading.z * target.x;
    let step = if cross_y >= 0.0 { max_step } else { -max_step };
    rotate_xz(heading, step)
}

/// Rotate a vector by `angle` radians within the xz-plane.
fn rotate_xz(v: Vec3, angle: f32) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    Vec3::new(v.x * cos - v.z * sin, 0.0, v.x * sin + v.z * cos)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn angle_between(a: Vec3, b: Vec3) -> f32 {
        a.normalize().dot(b.normalize()).clamp(-1.0, 1.0).acos()
    }

    #[test]
    fn turn_snaps_when_step_reaches_target() {
        let result = turn_toward(Vec3::X, Vec3::Z, 2.0); // 2 rad > 90°
        assert!(angle_between(result, Vec3::Z) < 1e-4);
    }

    #[test]
    fn turn_is_bounded_and_moves_closer() {
        let before = angle_between(Vec3::X, Vec3::Z); // 90°
        let result = turn_toward(Vec3::X, Vec3::Z, 0.2); // only 0.2 rad
        let after = angle_between(result, Vec3::Z);
        assert!(after < before, "should be closer to target");
        assert!(
            (before - after - 0.2).abs() < 1e-3,
            "should turn exactly max_step"
        );
        assert!((result.length() - 1.0).abs() < 1e-4, "stays a unit vector");
    }

    #[test]
    fn zero_target_leaves_heading_unchanged() {
        assert_eq!(turn_toward(Vec3::X, Vec3::ZERO, 0.5), Vec3::X);
    }

    #[test]
    fn thrust_is_along_heading_not_sideways() {
        let mut car = CarLike {
            heading: Vec3::X,
            max_turn_rate: 0.0, // freeze heading so we test pure thrust
            ..Default::default()
        };
        // Desired points along +x at speed 5; car is stationary.
        let force = car.drive(
            DesiredVelocity {
                value: Vec3::new(5.0, 0.0, 0.0),
                urgent: false,
            },
            Vec3::ZERO,
            1.0 / 60.0,
        );
        assert!(force.x > 0.0);
        assert!(force.z.abs() < 1e-4, "no sideways force when heading is +x");
    }

    #[test]
    fn grip_cancels_sideways_velocity() {
        let mut car = CarLike {
            heading: Vec3::X,
            max_turn_rate: 0.0,
            gain: 0.0, // isolate the grip term from forward thrust
            ..Default::default()
        };
        // Moving purely sideways (+z) while facing +x.
        let force = car.drive(
            DesiredVelocity {
                value: Vec3::ZERO,
                urgent: false,
            },
            Vec3::new(0.0, 0.0, 3.0),
            1.0 / 60.0,
        );
        assert!(force.z < 0.0, "grip should push against the sideways slide");
    }
}
