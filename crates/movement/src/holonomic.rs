use bevy::prelude::*;

use crate::model::{Actuation, BodyState, DesiredVelocity, MovementModel};

// Note: `seek_force` (below) holds the shared proportional-controller math,
// unit-tested directly. `Holonomic::drive` is a thin wrapper over it.

/// Free horizontal movement: no minimum turn radius, no forward-only
/// constraint. Steers via force toward the desired velocity -- the only
/// embodiment implemented in v1.
#[derive(Component, Clone, Copy, Debug)]
pub struct Holonomic {
    pub gain: f32,
    pub max_force: f32,
    /// Force ceiling used when `DesiredVelocity::urgent` is set (a reflex
    /// is braking), instead of `max_force`. Must be higher than
    /// `max_force`, or "brake as fast as possible" is a lie.
    pub brake_max_force: f32,
}

impl Default for Holonomic {
    fn default() -> Self {
        Self {
            gain: 8.0,
            max_force: 40.0,
            brake_max_force: 120.0,
        }
    }
}

impl MovementModel for Holonomic {
    fn drive(&mut self, desired: DesiredVelocity, body: BodyState, _dt: f32) -> Actuation {
        let max_force = if desired.urgent {
            self.brake_max_force
        } else {
            self.max_force
        };
        Actuation {
            force: seek_force(desired.value, body.velocity, self.gain, max_force),
            yaw_torque: 0.0,
        }
    }
}

/// Proportional controller: force proportional to the velocity error,
/// clamped to `max_force`. Shared by both plan-following and reflex
/// actions (they only differ in what `desired_velocity` they supply).
pub fn seek_force(
    desired_velocity: Vec3,
    current_velocity: Vec3,
    gain: f32,
    max_force: f32,
) -> Vec3 {
    let error = desired_velocity - current_velocity;
    (error * gain).clamp_length_max(max_force)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn below_max_force_scales_linearly_with_gain() {
        let force = seek_force(Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO, 2.0, 1000.0);
        assert_eq!(force, Vec3::new(20.0, 0.0, 0.0));
    }

    #[test]
    fn above_max_force_is_clamped_but_keeps_direction() {
        let force = seek_force(Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO, 100.0, 40.0);
        assert!((force.length() - 40.0).abs() < 1e-4);
        assert!(force.x > 0.0);
    }

    #[test]
    fn at_target_velocity_force_is_zero() {
        let velocity = Vec3::new(3.0, 0.0, 4.0);
        let force = seek_force(velocity, velocity, 5.0, 40.0);
        assert_eq!(force, Vec3::ZERO);
    }
}
