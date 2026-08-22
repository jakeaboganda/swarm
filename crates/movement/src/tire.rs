//! Tire and wheel-spin physics: the pure core of the wheeled vehicle.
//!
//! A wheel carries one piece of state -- its spin rate -- and everything else
//! here is a function of that plus the contact conditions. Kept free of ECS and
//! Rapier so it can be driven directly by tests; `raycast_vehicle` supplies the
//! per-wheel raycast and applies the resulting forces.

/// Below this ground speed (m/s) the slip denominator stops shrinking.
///
/// Slip ratio is `(wr - v) / |v|`, which is singular at a standstill -- and a
/// standing start is the first second of every scenario. Flooring the
/// denominator bounds slip instead of letting it run away.
pub const SLIP_SPEED_FLOOR: f32 = 2.0;

/// Tire coefficients, expressed *per unit vertical load* so that every force
/// scales with the load the suspension reports. That is what makes weight
/// transfer emerge rather than being modelled: a loaded outside wheel simply
/// generates more grip than an unloaded inside one.
#[derive(Clone, Copy, Debug)]
pub struct TireParams {
    /// Longitudinal stiffness per unit load (dimensionless).
    pub c_long: f32,
    /// Lateral stiffness per unit load, per radian of slip angle.
    pub c_lat: f32,
    /// Friction coefficient: the circle's radius is `mu * load`.
    pub mu: f32,
}

impl Default for TireParams {
    fn default() -> Self {
        Self {
            c_long: 15.0,
            c_lat: 12.0,
            mu: 1.0,
        }
    }
}

/// What one tire is pushing on the chassis with, in the wheel's own frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TireForce {
    /// Along the wheel's rolling direction; positive drives the car forward.
    pub longitudinal: f32,
    /// Across it; opposes lateral slip.
    pub lateral: f32,
    /// Whether the friction circle clipped the combined force.
    pub saturated: bool,
}

/// Fixed properties of one wheel.
#[derive(Clone, Copy, Debug)]
pub struct WheelSpec {
    pub radius: f32,
    /// Rotational inertia about the axle (kg.m^2).
    pub inertia: f32,
    pub tire: TireParams,
}

impl Default for WheelSpec {
    fn default() -> Self {
        Self {
            radius: 0.32,
            inertia: 1.2,
            tire: TireParams::default(),
        }
    }
}

/// This tick's contact conditions and driver commands for one wheel.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelInput {
    pub omega: f32,
    /// Vertical load from the suspension (N). Zero means airborne.
    pub load: f32,
    /// Contact-patch speed along the wheel's rolling direction (m/s).
    pub forward_speed: f32,
    /// Contact-patch speed across it (m/s).
    pub lateral_speed: f32,
    /// Always >= 0; drives the wheel in its rolling direction.
    pub engine_torque: f32,
    /// Always >= 0; opposes rotation and clamps to zero, never through it.
    pub brake_torque: f32,
}

/// One wheel after a step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelOutput {
    pub omega: f32,
    pub force: TireForce,
    pub slip_ratio: f32,
    pub slip_angle: f32,
}

/// Whether the wheel is actually on the ground and carrying weight. A
/// non-finite load counts as airborne rather than propagating into the forces.
fn has_contact(load: f32) -> bool {
    load.is_finite() && load > 0.0
}

/// Longitudinal slip: how much faster the contact patch is turning than the
/// road is passing. `0` is free rolling, `-1` is fully locked, positive is
/// wheelspin.
pub fn slip_ratio(omega: f32, radius: f32, forward_speed: f32) -> f32 {
    (omega * radius - forward_speed) / forward_speed.abs().max(SLIP_SPEED_FLOOR)
}

/// Lateral slip angle (radians): the angle between where the wheel points and
/// where it is actually going.
pub fn slip_angle(lateral_speed: f32, forward_speed: f32) -> f32 {
    (lateral_speed / forward_speed.abs().max(SLIP_SPEED_FLOOR)).atan()
}

/// Tire force from slip and load, clipped to the friction circle.
///
/// The circle is what couples the two axes: a wheel already using all its grip
/// to brake has none left to steer with, which is the whole reason
/// braking-while-cornering behaves.
pub fn tire_force(slip_ratio: f32, slip_angle: f32, load: f32, tire: &TireParams) -> TireForce {
    if !has_contact(load) {
        return TireForce::default(); // airborne: no contact, no force
    }
    let mut longitudinal = tire.c_long * slip_ratio * load;
    let mut lateral = -tire.c_lat * slip_angle * load;
    let magnitude = longitudinal.hypot(lateral);
    let limit = tire.mu * load;
    let saturated = magnitude > limit;
    if saturated && magnitude > 0.0 {
        let scale = limit / magnitude;
        longitudinal *= scale;
        lateral *= scale;
    }
    TireForce {
        longitudinal,
        lateral,
        saturated,
    }
}

/// The spin rate at which the tire's longitudinal force exactly balances the
/// applied torque, so the wheel would hold steady. `None` when the torque
/// exceeds what the tire can transmit -- that is genuine wheelspin, and there
/// is no equilibrium to settle at.
pub fn equilibrium_omega(
    engine_torque: f32,
    forward_speed: f32,
    load: f32,
    spec: &WheelSpec,
) -> Option<f32> {
    if !has_contact(load) || spec.radius <= 0.0 || spec.tire.c_long <= 0.0 {
        return None;
    }
    // The longitudinal force that would balance the applied torque.
    let needed = engine_torque / spec.radius;
    if needed.abs() > spec.tire.mu * load {
        return None; // more torque than the tire can transmit: wheelspin
    }
    let slip = needed / (spec.tire.c_long * load);
    let denominator = forward_speed.abs().max(SLIP_SPEED_FLOOR);
    Some((forward_speed + slip * denominator) / spec.radius)
}

/// Advances one wheel by `dt`: tire force, then spin, then braking.
///
/// The spin update is semi-implicit -- it solves for the next omega against the
/// tire's local slope rather than stepping to it -- because the wheel/tire
/// system is stiff at a 64 Hz tick. Stepped explicitly it does not settle: the
/// friction circle bounds the force, so instead of running away it flips
/// between extremes every tick. It is then clamped so it cannot cross its own
/// equilibrium, which is what stops a saturated wheel oscillating about road
/// speed.
pub fn step_wheel(input: &WheelInput, spec: &WheelSpec, dt: f32) -> WheelOutput {
    let slip_ratio_now = slip_ratio(input.omega, spec.radius, input.forward_speed);
    let slip_angle_now = slip_angle(input.lateral_speed, input.forward_speed);
    let force = tire_force(slip_ratio_now, slip_angle_now, input.load, &spec.tire);

    // How hard the tire's longitudinal force pushes back against a change in
    // spin. Saturated, the force no longer responds to omega at all, so the
    // slope is zero -- and a constant force is not stiff, so stepping it
    // explicitly is safe.
    let slope = if force.saturated || !has_contact(input.load) {
        0.0
    } else {
        let denominator = input.forward_speed.abs().max(SLIP_SPEED_FLOOR);
        spec.tire.c_long * input.load * spec.radius / denominator
    };
    let net_torque = input.engine_torque - spec.radius * force.longitudinal;
    let mut omega = input.omega + dt * net_torque / (spec.inertia + dt * spec.radius * slope);

    // Never step across the equilibrium. In the saturated region the step is
    // explicit, and without this it overshoots road speed and then oscillates
    // about it indefinitely instead of settling.
    if let Some(settled) =
        equilibrium_omega(input.engine_torque, input.forward_speed, input.load, spec)
    {
        if (input.omega - settled).signum() != (omega - settled).signum() {
            omega = settled;
        }
    }

    // Braking removes spin; it never reverses it. This clamp is what makes a
    // wheel lock and slide rather than spin backwards under a hard stop.
    if input.brake_torque > 0.0 && spec.inertia > 0.0 {
        let bleed = input.brake_torque / spec.inertia * dt;
        omega = if omega.abs() <= bleed {
            0.0
        } else {
            omega - bleed * omega.signum()
        };
    }

    WheelOutput {
        omega,
        force,
        slip_ratio: slip_ratio_now,
        slip_angle: slip_angle_now,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 64.0;

    fn spec() -> WheelSpec {
        WheelSpec::default()
    }

    /// A wheel carrying a quarter of a 1300 kg car.
    fn nominal_load() -> f32 {
        1300.0 * 9.81 / 4.0
    }

    #[test]
    fn slip_ratio_is_bounded_at_a_standstill() {
        // The singular case: v -> 0 is where the naive formula runs away, and
        // it is the first second of every scenario.
        assert_eq!(slip_ratio(0.0, 0.32, 0.0), 0.0);
        assert!(slip_ratio(50.0, 0.32, 0.0).is_finite());
        assert!(slip_ratio(50.0, 0.32, 1e-9).is_finite());
        assert!(slip_ratio(-50.0, 0.32, 0.0).is_finite());
        // Bounded by the floor: spinning at 50 rad/s against a stationary car
        // is (50 * 0.32) / SLIP_SPEED_FLOOR.
        let expected = 50.0 * 0.32 / SLIP_SPEED_FLOOR;
        assert!((slip_ratio(50.0, 0.32, 0.0) - expected).abs() < 1e-4);
    }

    #[test]
    fn a_free_rolling_wheel_settles_at_road_speed() {
        // Dropped onto a road already moving under it, with no torque, the
        // wheel must spin up to match and stay there -- not oscillate about it.
        let mut omega = 0.0;
        let input = |omega| WheelInput {
            omega,
            load: nominal_load(),
            forward_speed: 10.0,
            ..Default::default()
        };
        for _ in 0..64 {
            omega = step_wheel(&input(omega), &spec(), DT).omega;
        }
        let rolling = 10.0 / spec().radius;
        assert!(
            (omega - rolling).abs() < 0.5,
            "settled at {omega}, road speed is {rolling}"
        );
        // And it stays: another second changes nothing.
        for _ in 0..64 {
            omega = step_wheel(&input(omega), &spec(), DT).omega;
        }
        assert!((omega - rolling).abs() < 0.5, "drifted to {omega}");
    }

    #[test]
    fn a_locked_wheel_slides_at_slip_minus_one() {
        assert!((slip_ratio(0.0, 0.32, 10.0) + 1.0).abs() < 1e-6);
        // And through the whole step: enough brake to stop the wheel while the
        // car is still moving is, by definition, a locked slide.
        let out = step_wheel(
            &WheelInput {
                omega: 0.0,
                load: nominal_load(),
                forward_speed: 10.0,
                brake_torque: 100_000.0,
                ..Default::default()
            },
            &spec(),
            DT,
        );
        assert_eq!(out.omega, 0.0);
        assert!((out.slip_ratio + 1.0).abs() < 1e-6);
        // Sliding forward means the tire drags backward on the chassis.
        assert!(out.force.longitudinal < 0.0);
    }

    #[test]
    fn brake_torque_clamps_omega_to_zero_not_past_it() {
        // The distinction that makes lockup possible: braking removes spin, it
        // never reverses it. A single huge brake application stops the wheel
        // dead and leaves it there.
        let braking = |omega, brake_torque| WheelInput {
            omega,
            load: nominal_load(),
            forward_speed: 10.0,
            brake_torque,
            ..Default::default()
        };
        let out = step_wheel(&braking(30.0, 1.0e9), &spec(), DT);
        assert_eq!(out.omega, 0.0, "brake drove omega past zero");

        // A modest brake bleeds speed off gradually instead.
        let out = step_wheel(&braking(30.0, 200.0), &spec(), DT);
        assert!(out.omega > 0.0 && out.omega < 30.0, "omega {}", out.omega);
    }

    #[test]
    fn combined_force_never_exceeds_the_friction_circle() {
        let tire = TireParams::default();
        for load in [0.0, 1.0, nominal_load(), 10_000.0] {
            for slip in [-1.0, -0.3, 0.0, 0.3, 5.0] {
                for angle in [-1.0, -0.2, 0.0, 0.2, 1.0] {
                    let f = tire_force(slip, angle, load, &tire);
                    let magnitude = f.longitudinal.hypot(f.lateral);
                    assert!(
                        magnitude <= tire.mu * load + 1e-3,
                        "load {load} slip {slip} angle {angle}: |F| {magnitude} \
                         exceeds mu*load {}",
                        tire.mu * load
                    );
                    assert!(f.longitudinal.is_finite() && f.lateral.is_finite());
                }
            }
        }
    }

    #[test]
    fn an_airborne_wheel_generates_no_force() {
        let f = tire_force(0.5, 0.3, 0.0, &TireParams::default());
        assert_eq!(f.longitudinal, 0.0);
        assert_eq!(f.lateral, 0.0);
    }

    #[test]
    fn lateral_force_scales_with_load() {
        // Weight transfer emerges from this: the loaded outside wheel in a
        // corner grips harder than the unloaded inside one, with no code
        // anywhere that models "weight transfer".
        let tire = TireParams::default();
        let small = tire_force(0.0, 0.02, 1000.0, &tire);
        let large = tire_force(0.0, 0.02, 2000.0, &tire);
        assert!(!small.saturated && !large.saturated, "test is in the clip");
        assert!(
            (large.lateral / small.lateral - 2.0).abs() < 1e-3,
            "doubling load gave {} vs {}",
            large.lateral,
            small.lateral
        );
        // Lateral force opposes lateral slip.
        assert!(tire_force(0.0, 0.02, 1000.0, &tire).lateral < 0.0);
        assert!(tire_force(0.0, -0.02, 1000.0, &tire).lateral > 0.0);
    }

    /// Deliberately stiff: a light wheel on a very grippy, heavily loaded tire
    /// is the worst case for the spin ODE.
    fn stiff() -> WheelSpec {
        WheelSpec {
            radius: 0.32,
            inertia: 0.05,
            tire: TireParams {
                c_long: 40.0,
                c_lat: 30.0,
                mu: 2.0,
            },
        }
    }

    #[test]
    fn wheel_spin_stays_bounded_under_stiff_parameters() {
        // The guard. At 64 Hz the wheel/tire system's time constant is well
        // under the tick, so this diverges immediately under an explicit step
        // (see the test below). It must not here -- at any speed, including
        // the standstill where the stiffness is worst.
        for forward_speed in [0.0, 0.5, 5.0, 30.0] {
            let mut omega = 0.0;
            for _ in 0..5000 {
                omega = step_wheel(
                    &WheelInput {
                        omega,
                        load: 5000.0,
                        forward_speed,
                        engine_torque: 800.0,
                        ..Default::default()
                    },
                    &stiff(),
                    DT,
                )
                .omega;
                assert!(
                    omega.is_finite() && omega.abs() < 1.0e4,
                    "omega ran away to {omega} at {forward_speed} m/s"
                );
            }
        }
    }

    #[test]
    fn an_explicit_step_oscillates_where_the_implicit_one_settles() {
        // Pins *why* the update is semi-implicit, so the next person to
        // simplify it finds out here rather than by watching a car launch.
        // The friction circle bounds the force, so an explicit step does not
        // run to infinity -- it flips between extremes every tick, which is
        // just as broken and harder to spot.
        let spec = stiff();
        let (load, forward_speed) = (5000.0, 10.0);

        let mut omega = 0.0f32;
        let mut explicit_swing = 0.0f32;
        for step in 0..200 {
            let slip = slip_ratio(omega, spec.radius, forward_speed);
            let force = tire_force(slip, 0.0, load, &spec.tire);
            let next = omega + DT * (0.0 - spec.radius * force.longitudinal) / spec.inertia;
            if step > 100 {
                explicit_swing = explicit_swing.max((next - omega).abs());
            }
            omega = next;
        }

        let mut omega = 0.0f32;
        let mut implicit_swing = 0.0f32;
        for step in 0..200 {
            let next = step_wheel(
                &WheelInput {
                    omega,
                    load,
                    forward_speed,
                    ..Default::default()
                },
                &spec,
                DT,
            )
            .omega;
            if step > 100 {
                implicit_swing = implicit_swing.max((next - omega).abs());
            }
            omega = next;
        }

        assert!(
            implicit_swing < 0.1,
            "the implicit step never settled: still swinging {implicit_swing}"
        );
        assert!(
            (omega - forward_speed / spec.radius).abs() < 0.5,
            "the implicit step settled at {omega}, not road speed"
        );
        assert!(
            explicit_swing > 100.0,
            "explicit stepping swung only {explicit_swing}; the stiff fixture \
             is no longer stiff and the guard above proves nothing"
        );
    }
}
