use std::fmt::Write as _;

use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, QueryFilter, ReadRapierContext, Velocity};

use crate::model::DesiredVelocity;

/// A hand-rolled raycast vehicle: four wheels ray-cast to the ground, with
/// spring-damper suspension, engine/brake, and lateral tire grip applied to the
/// chassis as forces. Roll and pitch are real physics (the chassis is a free
/// dynamic body that leans on banking and pitches on grade). This is the 3D,
/// terrain-following cousin of `FullVehicle`'s bicycle plant.
#[derive(Component, Clone, Copy, Debug)]
pub struct RaycastVehicle {
    /// Front/rear wheel offset from the chassis center, along forward.
    pub half_wheelbase: f32,
    /// Left/right wheel offset from center, along right.
    pub half_track: f32,
    /// Wheel attach height relative to center (negative = below).
    pub wheel_y: f32,
    pub wheel_radius: f32,
    /// Suspension length at full extension.
    pub suspension_rest: f32,
    /// Spring force per meter of compression.
    pub suspension_stiffness: f32,
    /// Damping force per (m/s) of compression speed.
    pub suspension_damping: f32,
    /// Clamp on a single wheel's suspension force.
    pub max_suspension_force: f32,
    /// Drive/brake force ceiling for ordinary driving.
    pub max_engine_force: f32,
    /// Higher ceiling when a reflex is braking (urgent).
    pub max_brake_force: f32,
    /// Proportional gain from forward-speed error to drive force.
    pub gain: f32,
    /// Steering lock (max |steer|), radians.
    pub max_steer: f32,
    /// Proportional gain from heading error to target steer angle.
    pub steer_gain: f32,
    /// Max steering-angle change, radians per second.
    pub steer_rate: f32,
    /// Lateral grip: force per (m/s) of sideways slip at a wheel.
    pub grip: f32,
    /// Clamp on a single wheel's lateral grip force.
    pub max_lateral: f32,
    /// Current steering angle, the evolving actuator state.
    pub steer: f32,
}

impl Default for RaycastVehicle {
    fn default() -> Self {
        Self {
            half_wheelbase: 1.3,
            half_track: 0.8,
            wheel_y: -0.35,
            wheel_radius: 0.35,
            suspension_rest: 0.5,
            // Sized to hold a ~14 kg chassis (Density 4.0 at spawn) at a small
            // ride-height compression, near-critically damped.
            suspension_stiffness: 500.0,
            suspension_damping: 45.0,
            max_suspension_force: 2500.0,
            max_engine_force: 120.0,
            max_brake_force: 320.0,
            gain: 20.0,
            max_steer: 0.55,
            steer_gain: 1.2,
            steer_rate: 3.0,
            // Moderate grip: strong enough to corner, not so strong it reacts
            // violently to the chassis's own spin.
            grip: 22.0,
            max_lateral: 45.0,
            steer: 0.0,
        }
    }
}

/// Actuator commands from the driver: a steering angle and a signed
/// longitudinal drive force (positive accelerates, negative brakes).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleControls {
    pub steer: f32,
    pub drive_force: f32,
}

/// Wheel layout: (forward sign, right sign, steered). All wheels drive; the
/// front pair steers.
const WHEELS: [(f32, f32, bool); 4] = [
    (1.0, 1.0, true),    // front-left
    (1.0, -1.0, true),   // front-right
    (-1.0, 1.0, false),  // rear-left
    (-1.0, -1.0, false), // rear-right
];

/// Driver: `DesiredVelocity` -> `VehicleControls`, i.e. the pedals + wheel. A
/// proportional speed controller for the drive force and a rate-limited
/// proportional steering law, the same shape as `FullVehicle`'s driver.
pub(crate) fn driver(
    vehicle: &RaycastVehicle,
    desired: DesiredVelocity,
    heading: Vec3,
    forward_speed: f32,
    dt: f32,
) -> VehicleControls {
    let target = Vec3::new(desired.value.x, 0.0, desired.value.z);
    let target_speed = target.length();

    let ceiling = if desired.urgent {
        vehicle.max_brake_force
    } else {
        vehicle.max_engine_force
    };
    let drive_force = ((target_speed - forward_speed) * vehicle.gain).clamp(-ceiling, ceiling);

    let target_steer = if target_speed > 1e-3 {
        let error = signed_angle_xz(heading, target / target_speed);
        (vehicle.steer_gain * error).clamp(-vehicle.max_steer, vehicle.max_steer)
    } else {
        0.0
    };
    let steer = step_toward(vehicle.steer, target_steer, vehicle.steer_rate * dt);

    VehicleControls { steer, drive_force }
}

/// Spring-damper suspension force along the suspension axis (never negative,
/// never past the clamp). Positive `compression_speed` is compressing.
pub(crate) fn suspension_force(
    compression: f32,
    compression_speed: f32,
    stiffness: f32,
    damping: f32,
    max: f32,
) -> f32 {
    if compression <= 0.0 {
        return 0.0;
    }
    (stiffness * compression + damping * compression_speed).clamp(0.0, max)
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

fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z).normalize_or(Vec3::X)
}

/// Drives every `RaycastVehicle`: run the driver, then per wheel cast a ray to
/// the ground and apply suspension + drive + lateral-grip forces to the
/// chassis (overwriting `ExternalForce`). Runs before the physics step. Reads
/// the physics context (immutably) to raycast against the world's colliders.
pub fn drive_raycast_vehicles(
    time: Res<Time>,
    rapier: ReadRapierContext,
    mut query: Query<(
        Entity,
        &mut RaycastVehicle,
        &DesiredVelocity,
        &Velocity,
        &Transform,
        &mut ExternalForce,
    )>,
) {
    let dt = time.delta_secs();
    let Ok(context) = rapier.single() else {
        return;
    };

    for (entity, mut vehicle, desired, velocity, transform, mut force) in &mut query {
        let center = transform.translation;
        let up = *transform.up();
        let right = *transform.right();
        let forward = *transform.forward();
        // Horizontal heading for the driver and drive direction; the full
        // (possibly tilted) body axes place the wheels.
        let heading = horizontal(forward);

        let forward_speed = velocity.linear.dot(heading);
        let controls = driver(&vehicle, *desired, heading, forward_speed, dt);
        vehicle.steer = controls.steer;
        let per_wheel_drive = controls.drive_force * 0.25;
        let steer_rot = Quat::from_axis_angle(up, vehicle.steer);
        let filter = QueryFilter::default().exclude_rigid_body(entity);
        let max_reach = vehicle.suspension_rest + vehicle.wheel_radius;

        let mut total_force = Vec3::ZERO;
        let mut total_torque = Vec3::ZERO;

        let debug = std::env::var("VEHICLE_DEBUG").is_ok();
        const NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];
        let mut wheels_dbg = String::new();

        for (i, (fb, lr, steered)) in WHEELS.into_iter().enumerate() {
            let attach = center
                + forward * (vehicle.half_wheelbase * fb)
                + right * (vehicle.half_track * lr)
                + up * vehicle.wheel_y;
            let Some((_, toi)) = context.cast_ray(attach, -up, max_reach, true, filter) else {
                if debug {
                    let _ = write!(wheels_dbg, " {}:air", NAMES[i]);
                }
                continue; // wheel off the ground: no force
            };

            let arm = attach - center;
            let point_velocity = velocity.linear + velocity.angular.cross(arm);

            // Suspension: spring-damper along the chassis up axis.
            let compression = (vehicle.suspension_rest - (toi - vehicle.wheel_radius)).max(0.0);
            let compression_speed = -point_velocity.dot(up);
            let spring = suspension_force(
                compression,
                compression_speed,
                vehicle.suspension_stiffness,
                vehicle.suspension_damping,
                vehicle.max_suspension_force,
            );
            apply(&mut total_force, &mut total_torque, up * spring, arm);

            // Longitudinal drive/brake along the heading, applied through the
            // center of mass (arm 0) so hard acceleration on the light chassis
            // doesn't pitch it into a wheelie. Weight transfer is a later
            // refinement.
            apply(
                &mut total_force,
                &mut total_torque,
                heading * per_wheel_drive,
                Vec3::ZERO,
            );

            // Lateral tire grip: cancel sideways slip at the wheel's axle.
            let lateral = if steered { steer_rot * right } else { right };
            let slip = point_velocity.dot(lateral);
            let grip = (-vehicle.grip * slip).clamp(-vehicle.max_lateral, vehicle.max_lateral);
            apply(&mut total_force, &mut total_torque, lateral * grip, arm);

            if debug {
                let _ = write!(
                    wheels_dbg,
                    " {}:c{:.2}/f{:.0}",
                    NAMES[i], compression, spring
                );
            }
        }

        // Overwrite, not accumulate (ExternalForce persists across ticks).
        force.force = total_force;
        force.torque = total_torque;

        if debug {
            // Chassis attitude: pitch = nose above horizontal (+ is nose-up),
            // roll = right side above horizontal.
            let pitch = transform.forward().y.asin().to_degrees();
            let roll = right.y.asin().to_degrees();
            eprintln!(
                "y={:.2} pitch={:+.1} roll={:+.1} fspeed={:.2} drive={:.0} steer={:+.2} \
                 F=({:.0},{:.0},{:.0}) T=({:.0},{:.0},{:.0}){}",
                center.y,
                pitch,
                roll,
                forward_speed,
                controls.drive_force,
                vehicle.steer,
                total_force.x,
                total_force.y,
                total_force.z,
                total_torque.x,
                total_torque.y,
                total_torque.z,
                wheels_dbg,
            );
        }
    }
}

/// Accumulate a force applied at chassis-relative point `arm` into the running
/// world-frame force and torque.
fn apply(total_force: &mut Vec3, total_torque: &mut Vec3, f: Vec3, arm: Vec3) {
    *total_force += f;
    *total_torque += arm.cross(f);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vehicle() -> RaycastVehicle {
        RaycastVehicle::default()
    }

    #[test]
    fn suspension_pushes_up_on_compression_and_clamps() {
        let v = vehicle();
        assert_eq!(
            suspension_force(
                0.0,
                0.0,
                v.suspension_stiffness,
                v.suspension_damping,
                400.0
            ),
            0.0
        );
        assert!(
            suspension_force(
                0.1,
                0.0,
                v.suspension_stiffness,
                v.suspension_damping,
                400.0
            ) > 0.0
        );
        // Big compression clamps to max.
        assert_eq!(
            suspension_force(
                100.0,
                0.0,
                v.suspension_stiffness,
                v.suspension_damping,
                400.0
            ),
            400.0
        );
        // Never negative even on fast rebound.
        assert_eq!(
            suspension_force(
                0.0,
                -100.0,
                v.suspension_stiffness,
                v.suspension_damping,
                400.0
            ),
            0.0
        );
    }

    #[test]
    fn driver_accelerates_toward_target_speed() {
        let v = vehicle();
        // Want 5 m/s forward, currently stopped -> positive drive force.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(5.0, 0.0, 0.0),
                urgent: false,
            },
            Vec3::X,
            0.0,
            1.0 / 60.0,
        );
        assert!(c.drive_force > 0.0, "drive {}", c.drive_force);
    }

    #[test]
    fn driver_brakes_when_overspeed() {
        let v = vehicle();
        // Moving 10 m/s but target is stop -> negative (braking) drive force.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::ZERO,
                urgent: true,
            },
            Vec3::X,
            10.0,
            1.0 / 60.0,
        );
        assert!(c.drive_force < 0.0, "drive {}", c.drive_force);
    }

    #[test]
    fn driver_steers_toward_a_sideways_target() {
        let v = vehicle();
        // Heading +X, want to go toward +X rotated left (-Z) -> nonzero steer,
        // rate-limited within one tick.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(3.0, 0.0, -3.0),
                urgent: false,
            },
            Vec3::X,
            3.0,
            1.0 / 60.0,
        );
        assert!(c.steer.abs() > 0.0);
        assert!(c.steer.abs() <= v.steer_rate / 60.0 + 1e-6);
    }
}
