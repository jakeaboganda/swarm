use std::fmt::Write as _;

use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, QueryFilter, ReadRapierContext, Velocity};

use crate::model::DesiredVelocity;
use crate::tire::{step_wheel, TireParams, WheelInput, WheelSpec};
use crate::wheel::Wheels;

/// A hand-rolled raycast vehicle: four wheels ray-cast to the ground, with
/// spring-damper suspension, per-wheel tire forces from slip, and engine/brake
/// torque applied to the wheels rather than to the body. Roll and pitch are
/// real physics (the chassis is a free dynamic body that leans on banking and
/// pitches on grade). This is the 3D, terrain-following cousin of
/// `FullVehicle`'s bicycle plant.
///
/// Every force scales with the load its wheel is carrying, so weight transfer
/// -- squat, dive, and the outside wheel gripping harder mid-corner -- emerges
/// from the suspension rather than being modelled anywhere.
#[derive(Component, Clone, Copy, Debug)]
pub struct RaycastVehicle {
    /// Front/rear wheel offset from the chassis center, along forward.
    pub half_wheelbase: f32,
    /// Left/right wheel offset from center, along right.
    pub half_track: f32,
    /// Wheel attach height relative to center (negative = below).
    pub wheel_y: f32,
    pub wheel_radius: f32,
    /// Rotational inertia of one wheel about its axle (kg.m^2).
    pub wheel_inertia: f32,
    /// Centre of mass in chassis-local coordinates. Below the geometric centre:
    /// it is the dominant term in both the rollover threshold and how much the
    /// body pitches under drive, so it is set explicitly rather than inherited
    /// from the collider's shape.
    pub center_of_mass: Vec3,
    /// Suspension length at full extension.
    pub suspension_rest: f32,
    /// Spring force per meter of compression.
    pub suspension_stiffness: f32,
    /// Damping force per (m/s) of compression speed.
    pub suspension_damping: f32,
    /// Clamp on a single wheel's suspension force.
    pub max_suspension_force: f32,
    /// Total drive torque ceiling for ordinary driving (N.m), split across the
    /// wheels.
    pub max_engine_torque: f32,
    /// Higher total ceiling when a reflex is braking (urgent).
    pub max_brake_torque: f32,
    /// Proportional gain from forward-speed error to torque.
    pub gain: f32,
    /// Steering lock (max |steer|), radians.
    pub max_steer: f32,
    /// Max steering-angle change, radians per second.
    pub steer_rate: f32,
    pub tire: TireParams,
    /// Current steering angle, the evolving actuator state.
    pub steer: f32,
}

impl Default for RaycastVehicle {
    fn default() -> Self {
        Self {
            half_wheelbase: 1.3,
            half_track: 0.8,
            wheel_y: -0.35,
            wheel_radius: 0.32,
            // A road wheel and tire, ~1.2 kg.m^2.
            wheel_inertia: 1.2,
            // ~0.5 m above the ground once settled, against a 1.6 m track:
            // a rollover threshold near 1.5 g, comfortably above what the
            // tires can generate, so the car slides at the limit rather than
            // tipping over.
            center_of_mass: Vec3::new(0.0, -0.31, 0.0),
            suspension_rest: 0.20,
            // Sized for a ~1300 kg chassis: about a third of the travel
            // compressed at rest, damped to roughly 40% of critical.
            suspension_stiffness: 55_000.0,
            suspension_damping: 3_500.0,
            max_suspension_force: 15_000.0,
            // ~0.35 g of drive and ~1 g of braking at the contact patch.
            max_engine_torque: 1_500.0,
            max_brake_torque: 5_000.0,
            gain: 200.0,
            max_steer: 0.55,
            steer_rate: 3.0,
            tire: TireParams::default(),
            steer: 0.0,
        }
    }
}

impl RaycastVehicle {
    /// The fixed per-wheel properties the tire model needs.
    pub fn wheel_spec(&self) -> WheelSpec {
        WheelSpec {
            radius: self.wheel_radius,
            inertia: self.wheel_inertia,
            tire: self.tire,
        }
    }

    /// Height of the chassis centre above the ground with the suspension at
    /// full extension and the wheels just touching -- where a car is spawned,
    /// so it settles down into its static compression rather than starting
    /// buried or dropping from a height.
    pub fn rest_ride_height(&self) -> f32 {
        -self.wheel_y + self.suspension_rest + self.wheel_radius
    }
}

/// Actuator commands from the driver: a steering angle plus separate throttle
/// and brake.
///
/// They are separate because brake torque opposes *wheel rotation* and clamps
/// to zero rather than through it. A single signed torque going negative would
/// drive the wheels backwards instead of locking them, and lockup is the whole
/// point of simulating wheels.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct VehicleControls {
    pub steer: f32,
    /// Total drive torque, never negative.
    pub engine_torque: f32,
    /// Total brake torque, never negative.
    pub brake_torque: f32,
}

/// Wheel layout: (forward sign, right sign, steered). All wheels drive; the
/// front pair steers.
const WHEELS: [(f32, f32, bool); 4] = [
    (1.0, 1.0, true),    // front-left
    (1.0, -1.0, true),   // front-right
    (-1.0, 1.0, false),  // rear-left
    (-1.0, -1.0, false), // rear-right
];

/// Chassis-local attach point of wheel `index`, in the body's own frame
/// (Bevy convention: +X right, +Y up, -Z forward). This is the rig geometry a
/// viewer needs to place a wheel, and the same offset the drive system rotates
/// into world space to cast from.
pub fn wheel_offset(index: usize, vehicle: &RaycastVehicle) -> Vec3 {
    let (fb, lr, _) = WHEELS[index % WHEELS.len()];
    Vec3::new(
        vehicle.half_track * lr,
        vehicle.wheel_y,
        -vehicle.half_wheelbase * fb,
    )
}

/// Driver: `DesiredVelocity` -> `VehicleControls`, i.e. the pedals + wheel. A
/// proportional speed controller split across throttle and brake, and a
/// rate-limited **pure pursuit** steering law.
pub(crate) fn driver(
    vehicle: &RaycastVehicle,
    desired: DesiredVelocity,
    heading: Vec3,
    forward_speed: f32,
    dt: f32,
) -> VehicleControls {
    let target = Vec3::new(desired.value.x, 0.0, desired.value.z);
    let target_speed = target.length();

    // Going too slow is throttle; too fast is brake. They are never both on.
    let error = target_speed - forward_speed;
    let (engine_torque, brake_torque) = if error >= 0.0 {
        ((error * vehicle.gain).min(vehicle.max_engine_torque), 0.0)
    } else if desired.urgent {
        // A reflex stop is an emergency stop: full pressure at once, not a
        // proportional amount. Braking in proportion to the speed error eases
        // off exactly when the situation calls for everything the tires have
        // -- and below roughly a quarter of this ceiling the brake cannot
        // overcome the tire's own torque, so the wheels never lock at all.
        (0.0, vehicle.max_brake_torque)
    } else {
        // Ordinary slowing stays proportional, and capped by the cruise
        // ceiling rather than the emergency one.
        (0.0, (-error * vehicle.gain).min(vehicle.max_engine_torque))
    };

    // Steering: pure pursuit, slewed at the steer rate. With no aim point --
    // the plan ran out, or a reflex called for a stop -- the wheel is *held*,
    // not centred. Centring is a steering input nobody asked for, and braking
    // is proportional, so the car still has metres of stopping distance to
    // cover: mid-corner it would spend every one of them leaving the bend
    // tangentially.
    let target_steer = if target_speed > 1e-3 && desired.lookahead > 1e-3 {
        pursue(vehicle, heading, target / target_speed * desired.lookahead)
    } else {
        vehicle.steer
    };
    let steer = step_toward(vehicle.steer, target_steer, vehicle.steer_rate * dt);

    VehicleControls {
        steer,
        engine_torque,
        brake_torque,
    }
}

/// Pure pursuit: the steer angle whose arc reaches the aim point.
///
/// `aim` is the aim point relative to the chassis centre, in the ground plane.
/// The geometry is exact rather than a tuned gain: a chord of length `L`
/// subtending `alpha` from the rear axle lies on a circle of curvature
/// `2 sin(alpha) / L`, and the bicycle model puts the front wheel at
/// `tan(steer) = wheelbase * curvature`. On a constant-radius bend with the
/// car on the path, that returns `atan(wheelbase / radius)` -- the exact steer
/// the corner needs -- for *every* lookahead.
///
/// Pursuit is defined from the rear axle, so the aim point is re-referenced
/// there first; measuring from the chassis centre instead would bias every
/// corner by roughly half a wheelbase's worth of curvature.
pub(crate) fn pursue(vehicle: &RaycastVehicle, heading: Vec3, aim: Vec3) -> f32 {
    let from_rear_axle = Vec3::new(aim.x, 0.0, aim.z) + heading * vehicle.half_wheelbase;
    let distance = from_rear_axle.length();
    if distance <= 1e-3 || !distance.is_finite() {
        return vehicle.steer;
    }
    let alpha = signed_angle_xz(heading, from_rear_axle / distance);
    // Behind the rear axle the pursuit circle degenerates -- sin(alpha) shrinks
    // back toward zero and the geometry unwinds the wheel toward straight,
    // which is the opposite of what an aim point over your shoulder calls for.
    // The tightest circle available is the honest answer.
    if alpha.abs() > std::f32::consts::FRAC_PI_2 {
        return vehicle.max_steer * alpha.signum();
    }
    let wheelbase = 2.0 * vehicle.half_wheelbase;
    let curvature = 2.0 * alpha.sin() / distance;
    (wheelbase * curvature)
        .atan()
        .clamp(-vehicle.max_steer, vehicle.max_steer)
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
/// the ground, step its spin against the tire, and apply suspension + tire
/// forces to the chassis (overwriting `ExternalForce`). Runs before the physics
/// step. Reads the physics context (immutably) to raycast against the world's
/// colliders.
pub fn drive_raycast_vehicles(
    time: Res<Time>,
    rapier: ReadRapierContext,
    mut query: Query<(
        Entity,
        &mut RaycastVehicle,
        &mut Wheels,
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

    for (entity, mut vehicle, mut wheels, desired, velocity, transform, mut force) in &mut query {
        let center = transform.translation;
        let rotation = transform.rotation;
        let up = *transform.up();
        let right = *transform.right();
        let forward = *transform.forward();
        // Horizontal heading for the driver; the full (possibly tilted) body
        // axes place the wheels and orient their forces.
        let heading = horizontal(forward);
        // Rapier applies `ExternalForce::torque` about the centre of mass, so
        // every arm below is measured from there, not from the collider origin.
        let com = center + rotation * vehicle.center_of_mass;

        let forward_speed = velocity.linear.dot(heading);
        let controls = driver(&vehicle, *desired, heading, forward_speed, dt);
        vehicle.steer = controls.steer;
        let steer_rot = Quat::from_axis_angle(up, vehicle.steer);
        let filter = QueryFilter::default().exclude_rigid_body(entity);
        let max_reach = vehicle.suspension_rest + vehicle.wheel_radius;
        let spec = vehicle.wheel_spec();
        let per_wheel = 0.25;

        let mut total_force = Vec3::ZERO;
        let mut total_torque = Vec3::ZERO;

        for (index, (_, _, steered)) in WHEELS.into_iter().enumerate() {
            let attach = center + rotation * wheel_offset(index, &vehicle);
            let steer = if steered { vehicle.steer } else { 0.0 };
            // The wheel's own axes: rolling direction and axle direction.
            let roll = if steered {
                steer_rot * forward
            } else {
                forward
            };
            let lateral = if steered { steer_rot * right } else { right };

            let arm_attach = attach - com;
            let point_velocity = velocity.linear + velocity.angular.cross(arm_attach);

            // Suspension: spring-damper along the chassis up axis. The tire
            // forces below act at the contact patch, which is what turns drive
            // and braking into squat and dive.
            let (contact, compression, load, arm) =
                match context.cast_ray(attach, -up, max_reach, true, filter) {
                    Some((_, toi)) => {
                        let compression =
                            (vehicle.suspension_rest - (toi - vehicle.wheel_radius)).max(0.0);
                        let load = suspension_force(
                            compression,
                            -point_velocity.dot(up),
                            vehicle.suspension_stiffness,
                            vehicle.suspension_damping,
                            vehicle.max_suspension_force,
                        );
                        (true, compression, load, (attach - up * toi) - com)
                    }
                    // Airborne: no load, so no grip. The wheel still responds
                    // to throttle and brake, which is why it spins up in
                    // mid-air and is stopped by a brake.
                    None => (false, 0.0, 0.0, arm_attach),
                };

            let wheel = &mut wheels.0[index];
            let stepped = step_wheel(
                &WheelInput {
                    omega: wheel.omega,
                    relaxed_slip_ratio: wheel.relaxed_slip_ratio,
                    relaxed_slip_angle: wheel.relaxed_slip_angle,
                    load,
                    forward_speed: point_velocity.dot(roll),
                    lateral_speed: point_velocity.dot(lateral),
                    engine_torque: controls.engine_torque * per_wheel,
                    brake_torque: controls.brake_torque * per_wheel,
                },
                &spec,
                dt,
            );

            if contact {
                apply(&mut total_force, &mut total_torque, up * load, arm);
                apply(
                    &mut total_force,
                    &mut total_torque,
                    roll * stepped.force.longitudinal,
                    arm,
                );
                apply(
                    &mut total_force,
                    &mut total_torque,
                    lateral * stepped.force.lateral,
                    arm,
                );
            }

            wheel.omega = stepped.omega;
            wheel.angle = (wheel.angle + stepped.omega * dt).rem_euclid(std::f32::consts::TAU);
            wheel.steer = steer;
            wheel.contact = contact;
            wheel.compression = compression;
            wheel.load = load;
            wheel.slip_ratio = stepped.slip_ratio;
            wheel.slip_angle = stepped.slip_angle;
            wheel.relaxed_slip_ratio = stepped.relaxed_slip_ratio;
            wheel.relaxed_slip_angle = stepped.relaxed_slip_angle;
        }

        // Overwrite, not accumulate (ExternalForce persists across ticks).
        force.force = total_force;
        force.torque = total_torque;

        if std::env::var("VEHICLE_DEBUG").is_ok() {
            debug_dump(&vehicle, &wheels, transform, &controls, forward_speed);
        }
    }
}

/// Per-tick chassis + wheel state on stderr, behind `VEHICLE_DEBUG`.
fn debug_dump(
    vehicle: &RaycastVehicle,
    wheels: &Wheels,
    transform: &Transform,
    controls: &VehicleControls,
    forward_speed: f32,
) {
    const NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];
    let mut per_wheel = String::new();
    for (name, wheel) in NAMES.iter().zip(wheels.0.iter()) {
        if wheel.contact {
            let _ = write!(
                per_wheel,
                " {name}:c{:.2}/N{:.0}/k{:+.2}",
                wheel.compression, wheel.load, wheel.slip_ratio
            );
        } else {
            let _ = write!(per_wheel, " {name}:air");
        }
    }
    // Chassis attitude: pitch = nose above horizontal (+ is nose-up),
    // roll = right side above horizontal.
    let pitch = transform.forward().y.asin().to_degrees();
    let roll = transform.right().y.asin().to_degrees();
    eprintln!(
        "y={:.2} pitch={:+.1} roll={:+.1} fspeed={:.2} engine={:.0} brake={:.0} \
         steer={:+.2} load={:.0}{}",
        transform.translation.y,
        pitch,
        roll,
        forward_speed,
        controls.engine_torque,
        controls.brake_torque,
        vehicle.steer,
        wheels.total_load(),
        per_wheel,
    );
}

/// Accumulate a force applied at centre-of-mass-relative point `arm` into the
/// running world-frame force and torque.
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
    fn the_suspension_holds_the_car_up_at_a_sane_ride_height() {
        // The spring has to carry a quarter of the car within its travel. Too
        // soft and it sits on the bump stops; too stiff and it never moves and
        // the wheels skip instead of following the road.
        let v = vehicle();
        let quarter_weight = 1300.0 * 9.81 / 4.0;
        let compression = quarter_weight / v.suspension_stiffness;
        assert!(
            compression > 0.02 && compression < v.suspension_rest * 0.5,
            "static compression {compression} m against {} m of travel",
            v.suspension_rest
        );
        assert!(v.max_suspension_force > quarter_weight * 2.0, "no headroom");
    }

    #[test]
    fn the_rollover_threshold_is_above_what_the_tires_can_generate() {
        // If it tips before it slides, hard cornering ends with the car on its
        // roof instead of understeering. This is the geometry check that keeps
        // that from happening.
        let v = vehicle();
        let com_height = v.rest_ride_height() + v.center_of_mass.y;
        let threshold = v.half_track / com_height;
        assert!(
            threshold > v.tire.mu * 1.3,
            "rollover threshold {threshold} g is too close to the tires' {} g",
            v.tire.mu
        );
    }

    #[test]
    fn wheels_are_placed_at_the_rig_offsets() {
        let v = vehicle();
        // Bevy's forward is -Z, so the front pair sits at negative local Z.
        assert_eq!(
            wheel_offset(0, &v),
            Vec3::new(v.half_track, v.wheel_y, -v.half_wheelbase),
            "front-left"
        );
        assert_eq!(
            wheel_offset(1, &v),
            Vec3::new(-v.half_track, v.wheel_y, -v.half_wheelbase),
            "front-right"
        );
        assert_eq!(
            wheel_offset(2, &v),
            Vec3::new(v.half_track, v.wheel_y, v.half_wheelbase),
            "rear-left"
        );
        assert_eq!(
            wheel_offset(3, &v),
            Vec3::new(-v.half_track, v.wheel_y, v.half_wheelbase),
            "rear-right"
        );

        // Rotated into world space it must reproduce, exactly, the attach
        // point built from the body axes -- including on a body that is
        // yawed, pitched and rolled.
        let transform = Transform::from_translation(Vec3::new(3.0, 1.0, -2.0)).looking_to(
            Vec3::new(1.0, 0.3, 1.0).normalize(),
            Vec3::new(0.1, 1.0, 0.0),
        );
        for (index, (fb, lr, _)) in WHEELS.into_iter().enumerate() {
            let expected = transform.translation
                + *transform.forward() * (v.half_wheelbase * fb)
                + *transform.right() * (v.half_track * lr)
                + *transform.up() * v.wheel_y;
            let actual = transform.translation + transform.rotation * wheel_offset(index, &v);
            assert!(
                (actual - expected).length() < 1e-4,
                "wheel {index}: {actual:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn driver_accelerates_toward_target_speed() {
        let v = vehicle();
        // Want 5 m/s forward, currently stopped -> throttle, no brake.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(5.0, 0.0, 0.0),
                urgent: false,
                lookahead: 0.0,
            },
            Vec3::X,
            0.0,
            1.0 / 60.0,
        );
        assert!(c.engine_torque > 0.0, "engine {}", c.engine_torque);
        assert_eq!(c.brake_torque, 0.0);
    }

    #[test]
    fn driver_brakes_when_overspeed() {
        let v = vehicle();
        // Moving 10 m/s but target is stop -> brake, and no throttle at all.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::ZERO,
                urgent: true,
                lookahead: 0.0,
            },
            Vec3::X,
            10.0,
            1.0 / 60.0,
        );
        assert!(c.brake_torque > 0.0, "brake {}", c.brake_torque);
        assert_eq!(c.engine_torque, 0.0, "throttle and brake both applied");
    }

    #[test]
    fn urgent_selects_the_brake_ceiling_not_the_cruise_one() {
        // The CLAUDE.md guardrail, at the driver: a reflex stop must not be
        // limited by the ceiling tuned for ordinary deceleration.
        let v = vehicle();
        let overspeed = |urgent, speed| {
            driver(
                &v,
                DesiredVelocity {
                    value: Vec3::ZERO,
                    urgent,
                    lookahead: 0.0,
                },
                Vec3::X,
                speed,
                1.0 / 60.0,
            )
            .brake_torque
        };
        assert!(v.max_brake_torque > v.max_engine_torque);
        // Far past the target, both saturate at their own ceiling.
        assert_eq!(overspeed(false, 60.0), v.max_engine_torque);
        assert_eq!(overspeed(true, 60.0), v.max_brake_torque);

        // The distinction that matters is at a *modest* overspeed, where
        // proportional braking is still below even the cruise ceiling. An
        // urgent stop asks for everything regardless; anything less and the
        // brake cannot beat the tire's own torque and the wheels never lock.
        let modest = 5.0;
        assert!(
            modest * v.gain < v.max_engine_torque,
            "pick an overspeed where proportional braking has not saturated"
        );
        assert!(
            overspeed(false, modest) < v.max_engine_torque,
            "ordinary braking should still be proportional here"
        );
        assert_eq!(overspeed(true, modest), v.max_brake_torque);
    }

    #[test]
    fn an_urgent_stop_can_out_torque_the_tire_it_is_braking_against() {
        // A brake that cannot exceed the tire's own torque never locks a
        // wheel, it just slows it a little. This pins the ceiling above that
        // threshold for a car carrying its own weight on grippy tarmac.
        let v = vehicle();
        let load_per_wheel = 1300.0 * 9.81 / 4.0;
        let tire_torque = v.tire.mu * load_per_wheel * v.wheel_radius;
        let brake_per_wheel = v.max_brake_torque * 0.25;
        assert!(
            brake_per_wheel > tire_torque,
            "brake {brake_per_wheel} N.m cannot overcome the tire's \
             {tire_torque} N.m, so the wheels will never lock"
        );
    }

    #[test]
    fn driver_steers_toward_a_sideways_aim_point() {
        let v = vehicle();
        // Heading +X, aim point 6 m away toward +X rotated left (-Z) ->
        // nonzero steer to the left, rate-limited within one tick.
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(3.0, 0.0, -3.0),
                urgent: false,
                lookahead: 6.0,
            },
            Vec3::X,
            3.0,
            1.0 / 60.0,
        );
        assert!(c.steer > 0.0, "steered {} away from the aim point", c.steer);
        assert!(c.steer.abs() <= v.steer_rate / 60.0 + 1e-6);
    }

    #[test]
    fn a_direction_with_no_aim_point_leaves_the_wheel_alone() {
        // `lookahead` is the geometry the steering law is made of. Without it
        // there is nothing to steer *to* -- only a direction, which says
        // nothing about how hard to turn -- so the wheel holds rather than
        // guessing.
        let mut v = vehicle();
        v.steer = 0.2;
        let c = driver(
            &v,
            DesiredVelocity {
                value: Vec3::new(3.0, 0.0, -3.0),
                urgent: false,
                lookahead: 0.0,
            },
            Vec3::X,
            3.0,
            1.0 / 60.0,
        );
        assert_eq!(c.steer, v.steer);
    }

    #[test]
    fn the_steer_is_the_arc_that_reaches_the_aim_point() {
        // The point of the geometric law: on a constant-radius bend with the
        // car on the path, it returns exactly the steer the corner needs --
        // atan(wheelbase / radius) -- and returns it for *every* lookahead.
        // A proportional law cannot: its answer is whatever the gain times the
        // heading error happens to be, which moves as the aim point moves.
        let v = vehicle();
        let radius = 60.0f32;
        let wheelbase = 2.0 * v.half_wheelbase;
        let needed = (wheelbase / radius).atan();
        for lookahead in [3.0f32, 6.0, 12.0] {
            // Heading +X with the rear axle at the origin, on a left-hand bend
            // centred at (0, 0, -radius). The aim point is the point of that
            // circle a chord `lookahead` away.
            let theta = 2.0 * (lookahead / (2.0 * radius)).asin();
            let from_rear = Vec3::new(radius * theta.sin(), 0.0, radius * (theta.cos() - 1.0));
            let steer = pursue(&v, Vec3::X, from_rear - Vec3::X * v.half_wheelbase);
            assert!(
                (steer - needed).abs() < 1e-3,
                "lookahead {lookahead} m on a {radius} m bend asked for {steer} rad,                  not the {needed} rad the corner needs"
            );
        }
    }

    #[test]
    fn an_aim_point_behind_the_car_asks_for_full_lock() {
        // sin(alpha) shrinks back toward zero past 90 degrees, so the raw
        // geometry would unwind the wheel toward straight for an aim point
        // over the driver's shoulder. The tightest circle available is the
        // honest answer.
        let v = vehicle();
        let behind_left = Vec3::new(-8.0, 0.0, -1.0);
        assert_eq!(pursue(&v, Vec3::X, behind_left), v.max_steer);
        let behind_right = Vec3::new(-8.0, 0.0, 1.0);
        assert_eq!(pursue(&v, Vec3::X, behind_right), -v.max_steer);
    }

    #[test]
    fn a_stop_command_holds_the_wheel_rather_than_centring_it() {
        // A zero desired velocity says "no direction demanded" -- the plan ran
        // out, or a reflex called for a stop. It does not say "steer straight".
        // Centring is an active steering input nobody asked for, and mid-corner
        // it is the wrong one: braking is proportional, so the car has metres
        // of stopping distance left, and it spends all of them leaving the bend
        // tangentially.
        let mut v = vehicle();
        v.steer = 0.2;
        for urgent in [false, true] {
            let c = driver(
                &v,
                DesiredVelocity {
                    value: Vec3::ZERO,
                    urgent,
                    lookahead: 0.0,
                },
                Vec3::X,
                6.0,
                1.0 / 60.0,
            );
            assert_eq!(
                c.steer, v.steer,
                "urgent={urgent}: the wheel moved with nothing commanding it to"
            );
        }
    }
}
