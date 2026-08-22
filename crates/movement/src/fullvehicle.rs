use bevy::prelude::*;

use crate::model::{Actuation, BodyState, DesiredVelocity, MovementModel};

/// Below this forward speed the linear tire model is ill-conditioned (tiny
/// `vx` → huge slip angles), so cornering grip is faded out: a barely-moving
/// car can't corner, it has to build speed first. Realistic, and keeps the
/// math bounded.
const MIN_VX: f32 = 0.5;

/// Actuator commands — the "pedals + wheel". The plant's only inputs (`u`).
#[derive(Clone, Copy, Debug)]
pub struct Controls {
    /// Front road-wheel steer angle (radians). +ve steers toward the body's
    /// left (a positive yaw about +Y).
    pub steer: f32,
    /// Longitudinal drive/brake force along the heading (N). +ve accelerates.
    pub drive_force: f32,
}

/// Single-track ("bicycle") vehicle: real physical yaw plus lateral forces
/// from a linear tire model, so understeer/oversteer and sliding emerge
/// instead of being scripted. Contrast `CarLike`, whose yaw is cosmetic and
/// which can't slip.
///
/// The model is two pure layers: a **driver** (`driver`) mapping the
/// universal `DesiredVelocity` to `Controls`, and a **plant**
/// (`bicycle_step`) mapping those controls + read-back `BodyState` to the
/// forces Rapier then integrates.
#[derive(Component, Clone, Copy, Debug)]
pub struct FullVehicle {
    /// CG → front axle distance.
    pub l_f: f32,
    /// CG → rear axle distance.
    pub l_r: f32,
    /// Steering lock (max |steer|), radians.
    pub max_steer: f32,
    /// Proportional gain from heading error to target steer angle.
    pub steer_gain: f32,
    /// Max steering-angle change, radians per second.
    pub steer_rate: f32,
    /// Proportional gain on forward speed error → drive force.
    pub gain: f32,
    /// Drive/brake force ceiling for ordinary driving.
    pub max_force: f32,
    /// Higher ceiling when a reflex is braking (see `DesiredVelocity`).
    pub brake_max_force: f32,
    /// Front cornering stiffness (lateral force per radian of slip).
    pub c_f: f32,
    /// Rear cornering stiffness. Slightly stiffer than front → mild,
    /// stable understeer.
    pub c_r: f32,
    /// Clamp on |lateral force| per axle — the crude stand-in for a tire
    /// grip limit (no friction circle yet).
    pub max_lateral: f32,
    /// Current steering angle — the evolving actuator state.
    pub steer: f32,
}

impl Default for FullVehicle {
    fn default() -> Self {
        Self {
            l_f: 1.4,
            l_r: 1.4,
            max_steer: 0.6,
            steer_gain: 1.0,
            steer_rate: 3.0,
            gain: 8.0,
            max_force: 40.0,
            brake_max_force: 120.0,
            c_f: 60.0,
            c_r: 70.0,
            max_lateral: 40.0,
            steer: 0.0,
        }
    }
}

impl MovementModel for FullVehicle {
    fn drive(&mut self, desired: DesiredVelocity, body: BodyState, dt: f32) -> Actuation {
        let controls = driver(self, desired, body, dt);
        self.steer = controls.steer; // steering angle persists across ticks
        bicycle_step(self, body, controls)
    }
}

/// Left-hand perpendicular of a heading in the xz-plane (the +Y-rotation-by-
/// 90° direction). For heading +X this is −Z.
fn left_of(heading: Vec3) -> Vec3 {
    Vec3::new(heading.z, 0.0, -heading.x)
}

/// Yaw angle about +Y that rotates unit heading `h` onto unit direction `d`
/// (both in the xz-plane). +ve = the body's left.
fn signed_angle_xz(h: Vec3, d: Vec3) -> f32 {
    let sin = h.z * d.x - h.x * d.z;
    let cos = h.x * d.x + h.z * d.z;
    sin.atan2(cos)
}

/// Move `current` toward `target` by at most `max_step`.
fn step_toward(current: f32, target: f32, max_step: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= max_step {
        target
    } else {
        current + max_step * delta.signum()
    }
}

/// Driver (`DesiredVelocity` → `Controls`): the pedals + steering wheel.
/// A proportional speed controller for the longitudinal force and a
/// rate-limited proportional steering law for the wheel.
fn driver(v: &FullVehicle, desired: DesiredVelocity, body: BodyState, dt: f32) -> Controls {
    let target = Vec3::new(desired.value.x, 0.0, desired.value.z);
    let target_speed = target.length();

    // Throttle/brake: force toward the target forward speed.
    let forward_speed = body.velocity.dot(body.heading);
    let ceiling = if desired.urgent {
        v.brake_max_force
    } else {
        v.max_force
    };
    let drive_force = ((target_speed - forward_speed) * v.gain).clamp(-ceiling, ceiling);

    // Steering wheel: aim at the heading error, clamped to lock and slewed at
    // the steer rate. With no direction demanded -- the plan ran out, or a
    // reflex called for a stop -- the wheel is *held*, not centred. Centring is
    // a steering input nobody asked for, and braking is proportional, so the
    // car still has metres of stopping distance to cover: mid-corner it would
    // spend every one of them leaving the bend tangentially.
    let target_steer = if target_speed > 1e-3 {
        let error = signed_angle_xz(body.heading, target / target_speed);
        (v.steer_gain * error).clamp(-v.max_steer, v.max_steer)
    } else {
        v.steer
    };
    let steer = step_toward(v.steer, target_steer, v.steer_rate * dt);

    Controls { steer, drive_force }
}

/// Plant (`Controls` + `BodyState` → forces): single-track dynamics with a
/// linear tire model. Returns the world-frame force and yaw torque; Rapier
/// integrates them.
fn bicycle_step(v: &FullVehicle, body: BodyState, controls: Controls) -> Actuation {
    let heading = body.heading;
    let left = left_of(heading);

    let vx = body.velocity.dot(heading); // forward speed
    let vy = body.velocity.dot(left); // lateral speed (toward left)
    let r = body.yaw_rate;

    // Fade cornering out at low speed (see MIN_VX) and bound the atan2.
    let vx_eff = vx.max(MIN_VX);
    let grip = (vx / MIN_VX).clamp(0.0, 1.0);

    // Slip angles: wheel pointing direction vs. its velocity direction.
    let alpha_f = controls.steer - (vy + r * v.l_f).atan2(vx_eff);
    let alpha_r = -(vy - r * v.l_r).atan2(vx_eff);

    // Linear tire lateral forces (toward left), grip- and clamp-limited.
    let f_yf = (v.c_f * alpha_f).clamp(-v.max_lateral, v.max_lateral) * grip;
    let f_yr = (v.c_r * alpha_r).clamp(-v.max_lateral, v.max_lateral) * grip;

    let cos_steer = controls.steer.cos();
    let lateral = f_yf * cos_steer + f_yr;
    let force = heading * controls.drive_force + left * lateral;
    // Moment about CG: front force acts +l_f ahead, rear −l_r behind.
    let yaw_torque = v.l_f * f_yf * cos_steer - v.l_r * f_yr;

    Actuation { force, yaw_torque }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(velocity: Vec3, yaw_rate: f32) -> BodyState {
        BodyState {
            velocity,
            yaw_rate,
            heading: Vec3::X,
        }
    }

    // --- plant (bicycle_step) -------------------------------------------

    #[test]
    fn straight_line_has_no_lateral_force_or_yaw() {
        let v = FullVehicle::default();
        let act = bicycle_step(
            &v,
            body(Vec3::new(5.0, 0.0, 0.0), 0.0),
            Controls {
                steer: 0.0,
                drive_force: 10.0,
            },
        );
        assert!((act.force.x - 10.0).abs() < 1e-4, "thrust along heading");
        assert!(act.force.z.abs() < 1e-4, "no lateral force");
        assert!(act.yaw_torque.abs() < 1e-4, "no yaw");
    }

    #[test]
    fn positive_steer_turns_left_and_pushes_left() {
        let v = FullVehicle::default();
        // Heading +X, moving forward, steering +ve (toward left = −Z).
        let act = bicycle_step(
            &v,
            body(Vec3::new(5.0, 0.0, 0.0), 0.0),
            Controls {
                steer: 0.2,
                drive_force: 0.0,
            },
        );
        assert!(act.yaw_torque > 0.0, "positive steer → positive (left) yaw");
        assert!(act.force.z < 0.0, "lateral force points left (−Z) for +X");
    }

    #[test]
    fn sideways_slide_is_restored() {
        let v = FullVehicle::default();
        // Forward at 5 (so there's grip) while sliding toward left (−Z).
        let act = bicycle_step(
            &v,
            body(Vec3::new(5.0, 0.0, -3.0), 0.0),
            Controls {
                steer: 0.0,
                drive_force: 0.0,
            },
        );
        assert!(
            act.force.z > 0.0,
            "tire force opposes the leftward slide (pushes +Z)"
        );
    }

    #[test]
    fn no_cornering_below_min_speed() {
        let v = FullVehicle::default();
        // Stationary: grip faded to zero, so steering produces no side force.
        let act = bicycle_step(
            &v,
            body(Vec3::ZERO, 0.0),
            Controls {
                steer: 0.3,
                drive_force: 5.0,
            },
        );
        assert!(act.force.z.abs() < 1e-4, "no lateral force at rest");
        assert!(act.yaw_torque.abs() < 1e-4, "no yaw at rest");
        assert!(
            (act.force.x - 5.0).abs() < 1e-4,
            "drive force still applies"
        );
    }

    // --- driver ----------------------------------------------------------

    #[test]
    fn urgent_selects_the_brake_ceiling() {
        let v = FullVehicle::default();
        // Moving fast forward, commanded to stop → large negative force that
        // clamps differently depending on `urgent`.
        let fast = body(Vec3::new(30.0, 0.0, 0.0), 0.0);
        let normal = driver(
            &v,
            DesiredVelocity {
                value: Vec3::ZERO,
                urgent: false,
            },
            fast,
            1.0 / 60.0,
        );
        let urgent = driver(
            &v,
            DesiredVelocity {
                value: Vec3::ZERO,
                urgent: true,
            },
            fast,
            1.0 / 60.0,
        );
        assert!((normal.drive_force + v.max_force).abs() < 1e-4);
        assert!((urgent.drive_force + v.brake_max_force).abs() < 1e-4);
    }

    #[test]
    fn steers_toward_the_desired_side() {
        let v = FullVehicle::default();
        // Heading +X, desired points left (−Z) → positive steer.
        let controls = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(0.0, 0.0, -5.0),
                urgent: false,
            },
            body(Vec3::new(3.0, 0.0, 0.0), 0.0),
            1.0 / 60.0,
        );
        assert!(controls.steer > 0.0, "should steer toward −Z (left)");
    }

    #[test]
    fn steering_is_rate_limited() {
        let v = FullVehicle {
            steer: 0.0,
            ..Default::default()
        };
        let dt = 1.0 / 60.0;
        // Hard left desired, but one tick can't exceed steer_rate·dt.
        let controls = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(0.0, 0.0, -5.0),
                urgent: false,
            },
            body(Vec3::new(3.0, 0.0, 0.0), 0.0),
            dt,
        );
        assert!(controls.steer.abs() <= v.steer_rate * dt + 1e-6);
        assert!(controls.steer > 0.0);
    }

    #[test]
    fn a_stop_command_holds_the_wheel_rather_than_centring_it() {
        // A zero desired velocity says "no direction demanded" -- the plan ran
        // out, or a reflex called for a stop. It does not say "steer straight".
        // Centring is an active steering input nobody asked for, and mid-corner
        // it is the wrong one: braking is proportional, so the car has metres
        // of stopping distance left, and it spends all of them leaving the bend
        // tangentially.
        let v = FullVehicle {
            steer: 0.2,
            ..Default::default()
        };
        for urgent in [false, true] {
            let controls = driver(
                &v,
                DesiredVelocity {
                    value: Vec3::ZERO,
                    urgent,
                },
                body(Vec3::new(6.0, 0.0, 0.0), 0.0),
                1.0 / 60.0,
            );
            assert_eq!(
                controls.steer, v.steer,
                "urgent={urgent}: the wheel moved with nothing commanding it to"
            );
        }
    }

    #[test]
    fn drive_updates_persistent_steer_state() {
        let mut v = FullVehicle::default();
        let desired = DesiredVelocity {
            value: Vec3::new(0.0, 0.0, -5.0),
            urgent: false,
        };
        let b = body(Vec3::new(3.0, 0.0, 0.0), 0.0);
        v.drive(desired, b, 1.0 / 60.0);
        let after_one = v.steer;
        v.drive(desired, b, 1.0 / 60.0);
        assert!(
            after_one > 0.0 && v.steer > after_one,
            "steer slews over ticks"
        );
    }
}
