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

/// Tire relaxation length (m): the distance the tire must roll for its force
/// to build to ~63% of the steady-state value. Real tires do not develop grip
/// instantly -- the carcass has to deflect first -- and modelling that is what
/// keeps this stable.
pub const RELAXATION_LENGTH: f32 = 0.5;

/// Floor on the relaxation time constant (s), which is what the chassis's
/// stability turns on.
///
/// A linear tire at a standstill is a ferociously stiff damper: around
/// 19 kN per m/s of lateral drift per wheel, which across the track works out
/// to ~129 kN.m per rad/s of yaw against a ~1100 kg.m^2 body -- a time
/// constant near 9 ms against a 15.6 ms tick. Rapier integrates the chassis
/// explicitly, so at that ratio the yaw response is past the stability edge
/// and settles into a limit cycle instead of to rest. Lagging the force by at
/// least this long keeps the loop's response slower than the tick, whatever
/// the speed.
pub const MIN_RELAXATION_TIME: f32 = 0.08;

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
    /// Last tick's relaxed slips -- the tire's carcass deflection, carried
    /// forward. Force is computed from these, not from the instantaneous slip.
    pub relaxed_slip_ratio: f32,
    pub relaxed_slip_angle: f32,
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
    /// Instantaneous slip this tick -- what the wheel is actually doing, and
    /// what a diagnostic overlay should show.
    pub slip_ratio: f32,
    pub slip_angle: f32,
    /// The relaxed slips the force was computed from, to be fed back in next
    /// tick.
    pub relaxed_slip_ratio: f32,
    pub relaxed_slip_angle: f32,
}

/// Whether the wheel is actually on the ground and carrying weight. A
/// non-finite load counts as airborne rather than propagating into the forces.
fn has_contact(load: f32) -> bool {
    load.is_finite() && load > 0.0
}

/// How fast the contact patch is travelling, floored.
///
/// Slip ratio, slip angle and the tire's carcass relaxation are all rates per
/// unit of rolling, and all three are singular as the wheel comes to rest --
/// which is the first second of every scenario. One floored patch speed for
/// all three keeps them consistent: at a standstill the tire behaves as if the
/// patch were creeping along at [`SLIP_SPEED_FLOOR`], rather than as if it had
/// stopped responding altogether.
fn patch_speed(forward_speed: f32) -> f32 {
    forward_speed.abs().max(SLIP_SPEED_FLOOR)
}

/// Longitudinal slip: how much faster the contact patch is turning than the
/// road is passing. `0` is free rolling, `-1` is fully locked, positive is
/// wheelspin.
pub fn slip_ratio(omega: f32, radius: f32, forward_speed: f32) -> f32 {
    (omega * radius - forward_speed) / patch_speed(forward_speed)
}

/// Lateral slip angle (radians): the angle between where the wheel points and
/// where it is actually going.
pub fn slip_angle(lateral_speed: f32, forward_speed: f32) -> f32 {
    (lateral_speed / patch_speed(forward_speed)).atan()
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

/// Time constant of the tire's carcass deflection (s): how long its force
/// takes to build to ~63% of the steady-state value.
///
/// Bounded at both ends. `RELAXATION_LENGTH / speed` runs away as the wheel
/// slows -- unbounded, a tire at rest takes minutes to develop any grip and
/// the drive wheels spin up against nothing -- so the patch speed is floored
/// the same way slip's is. [`MIN_RELAXATION_TIME`] then holds the other end,
/// keeping the chassis loop slower than the tick at speed.
fn relaxation_time(forward_speed: f32) -> f32 {
    (RELAXATION_LENGTH / patch_speed(forward_speed)).max(MIN_RELAXATION_TIME)
}

/// Fraction of the way from the tire's current deflection to the
/// instantaneous slip that it travels in `dt`.
///
/// Solved implicitly (`dt / (tau + dt)`), so it stays inside `0..1` for any
/// `dt` and the lag can never overshoot however stiff the tire.
fn relax_blend(forward_speed: f32, dt: f32) -> f32 {
    let tau = relaxation_time(forward_speed);
    dt / (tau + dt)
}

/// First-order lag toward `target` over the tire's relaxation time.
pub fn relax(previous: f32, target: f32, forward_speed: f32, dt: f32) -> f32 {
    previous + relax_blend(forward_speed, dt) * (target - previous)
}

/// Halvings in the spin solve below. The bracket is `2 * dt * radius * mu *
/// load / inertia` wide -- tens of rad/s -- so this many takes it well past
/// `f32`'s resolution.
const SPIN_SOLVE_STEPS: usize = 32;

/// Advances one wheel by `dt`: its spin, and the tire force that goes with it.
///
/// The wheel/tire pair is far stiffer than a 64 Hz tick -- a wheel's spin
/// settles against its tire in a couple of milliseconds -- so the step has to
/// be implicit or it does not converge, it rings. See [`solve_spin`].
pub fn step_wheel(input: &WheelInput, spec: &WheelSpec, dt: f32) -> WheelOutput {
    let slip_angle_now = slip_angle(input.lateral_speed, input.forward_speed);
    // The wheel's spin does not enter the slip angle, so the lateral side of
    // the tire relaxes on its own.
    let relaxed_slip_angle = relax(
        input.relaxed_slip_angle,
        slip_angle_now,
        input.forward_speed,
        dt,
    );

    let blend = relax_blend(input.forward_speed, dt);
    // Where the tire ends the step, as a function of where the spin ends it.
    // Writing both states against the same unknown is what makes the solve
    // below implicit in the pair rather than in the spin alone -- and the tire
    // force actually applied then matches the one the spin was solved against.
    let relaxed_at = |omega: f32| {
        let target = slip_ratio(omega, spec.radius, input.forward_speed);
        input.relaxed_slip_ratio + blend * (target - input.relaxed_slip_ratio)
    };
    let force_at = |omega: f32| {
        tire_force(
            relaxed_at(omega),
            relaxed_slip_angle,
            input.load,
            &spec.tire,
        )
    };

    let omega = solve_spin(input, spec, dt, &force_at);

    WheelOutput {
        omega,
        force: force_at(omega),
        slip_ratio: slip_ratio(omega, spec.radius, input.forward_speed),
        slip_angle: slip_angle_now,
        relaxed_slip_ratio: relaxed_at(omega),
        relaxed_slip_angle,
    }
}

/// Solves this tick's spin from the torque balance at the *end* of the step,
/// against the real tire curve rather than a straight line through it.
///
/// Every torque on the wheel belongs in the same balance -- engine, tire and
/// brake alike. A torque applied outside it is applied to a wheel whose tire
/// never gets to answer, and at this stiffness that is worth tens of rad/s a
/// tick: a linearisation is not enough either, because past the friction
/// circle the tire's force stops responding to spin at all and a tangent step
/// there is explicit in all but name.
///
/// `omega - input.omega - dt * torque(omega) / inertia` rises monotonically in
/// `omega` (more spin means more longitudinal force, so less net torque), and
/// the friction circle bounds that force to `mu * load`, so the root is
/// bracketed by the step the wheel would take at either extreme of that bound.
/// Bisection converges inside it in a fixed number of halvings.
fn solve_spin(
    input: &WheelInput,
    spec: &WheelSpec,
    dt: f32,
    force_at: &impl Fn(f32) -> TireForce,
) -> f32 {
    if spec.inertia <= 0.0 || dt <= 0.0 {
        return input.omega;
    }
    let limit = if has_contact(input.load) {
        spec.tire.mu * input.load
    } else {
        0.0 // airborne: the tire is not part of the balance
    };
    let brake = input.brake_torque.max(0.0);
    // Which way the brake acts: against the wheel's spin, or -- when the wheel
    // has already stopped -- against whatever is trying to turn it. That
    // second case is what keeps a locked wheel locked instead of letting the
    // tire spin it straight back up.
    let direction = if brake == 0.0 {
        0.0
    } else if input.omega != 0.0 {
        input.omega.signum()
    } else {
        let turning = input.engine_torque - spec.radius * force_at(0.0).longitudinal;
        if turning.abs() <= brake {
            return 0.0;
        }
        turning.signum()
    };

    let residual = |omega: f32| {
        let torque =
            input.engine_torque - spec.radius * force_at(omega).longitudinal - brake * direction;
        omega - input.omega - dt * torque / spec.inertia
    };
    let free = input.omega + dt * (input.engine_torque - brake * direction) / spec.inertia;
    let reach = dt * spec.radius * limit / spec.inertia;
    let (mut low, mut high) = (free - reach, free + reach);
    for _ in 0..SPIN_SOLVE_STEPS {
        let middle = 0.5 * (low + high);
        if residual(middle) < 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }
    let omega = 0.5 * (low + high);

    // Braking removes spin; it never reverses it. A wheel the brake would push
    // through zero stops there, and the stationary case above decides next
    // tick whether it stays stopped. This is what makes a wheel lock and slide
    // rather than spin backwards under a hard stop.
    if direction != 0.0 && omega * direction < 0.0 {
        0.0
    } else {
        omega
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
        let mut state = WheelInput {
            load: nominal_load(),
            forward_speed: 10.0,
            ..Default::default()
        };
        let mut omega = 0.0;
        for _ in 0..64 {
            state.omega = omega;
            let out = step_wheel(&state, &spec(), DT);
            omega = out.omega;
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
        }
        let rolling = 10.0 / spec().radius;
        assert!(
            (omega - rolling).abs() < 0.5,
            "settled at {omega}, road speed is {rolling}"
        );
        // And it stays: another second changes nothing.
        for _ in 0..64 {
            state.omega = omega;
            let out = step_wheel(&state, &spec(), DT);
            omega = out.omega;
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
        }
        assert!((omega - rolling).abs() < 0.5, "drifted to {omega}");
    }

    #[test]
    fn relaxation_is_bounded_at_a_standstill() {
        // The tire's lag is a distance rolled, so `sigma / speed` runs away as
        // the car slows: at rest it would take minutes to build any grip at
        // all and the drive wheels would spin up against nothing. Slip already
        // floors the patch speed for exactly this reason; the relaxation has
        // to use the same floor, or the standing start it exists for is the
        // one case it fails to cover.
        let longest = RELAXATION_LENGTH / SLIP_SPEED_FLOOR;
        for speed in [0.0, 1e-9, 0.01, 1.0, 2.0, 10.0, 50.0] {
            let tau = relaxation_time(speed);
            assert!(
                (MIN_RELAXATION_TIME..=longest).contains(&tau),
                "tau {tau} s at {speed} m/s is outside {MIN_RELAXATION_TIME}..{longest}"
            );
        }
        // And a blend factor is a fraction, at every speed and every step.
        for dt in [1.0 / 64.0, 1.0, 10.0] {
            for speed in [0.0, 5.0, 50.0] {
                let blend = relax_blend(speed, dt);
                assert!((0.0..=1.0).contains(&blend), "blend {blend}");
            }
        }
    }

    #[test]
    fn a_driven_quarter_car_settles_at_its_equilibrium_slip() {
        // Launch, then cruise, with the wheel driving the mass it carries --
        // the feedback path a fixed `forward_speed` cannot show. Under a
        // constant torque the tire has exactly one steady answer: the slip
        // that carries that torque. Reaching it and staying there is the job.
        let spec = spec();
        let mass = 1300.0 / 4.0;
        let load = mass * 9.81;
        let torque = 375.0; // a quarter of the engine's ceiling
        let mut state = WheelInput {
            load,
            engine_torque: torque,
            ..Default::default()
        };
        let mut speed = 0.0f32;
        let mut worst_swing = 0.0f32;
        let mut worst_slip = 0.0f32;
        for step in 0..512 {
            let out = step_wheel(&state, &spec, DT);
            if step > 256 {
                worst_swing = worst_swing.max((out.omega - state.omega).abs());
            }
            worst_slip = worst_slip.max(out.slip_ratio.abs());
            speed += DT * out.force.longitudinal / mass;
            state.omega = out.omega;
            state.forward_speed = speed;
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
        }
        // Nothing here can break traction: 375 N.m against a contact patch
        // that can carry mu * load * radius = 1020 N.m is a third of the grip
        // available, so the wheel must never run away from the road. It did --
        // to a surface speed of 15 m/s under a car doing 0.05 m/s -- because
        // the tire developed no force at all from rest.
        assert!(
            worst_slip < 0.5,
            "the wheel spun up to slip {worst_slip} against a tire it cannot break"
        );
        assert!(
            speed > 20.0,
            "the car only reached {speed} m/s under full torque"
        );
        // The slip that carries `torque`: F = torque / radius, and the linear
        // tire needs slip = F / (c_long * load) to produce it.
        let settled = torque / spec.radius / (spec.tire.c_long * load);
        let slip = slip_ratio(state.omega, spec.radius, speed);
        assert!(
            (slip - settled).abs() < 0.01,
            "cruising at slip {slip}, not the {settled} its torque calls for"
        );
        // Cruising, the wheel tracks the car: it gains only the spin the
        // car's own acceleration asks for, not tens of rad/s a tick.
        assert!(
            worst_swing < 1.0,
            "the wheel is swinging {worst_swing} rad/s a tick at cruise"
        );
    }

    #[test]
    fn a_braked_wheel_settles_where_the_tire_balances_the_brake() {
        // Brake torque has to be balanced against the tire within the step,
        // not subtracted from the wheel afterwards. Subtracted afterwards, a
        // brake well inside what the tire can hold still drags the wheel a
        // metre per second below road speed every tick, saturates it, and
        // stops the car at 1 g on a light touch of the pedal.
        let spec = spec();
        let speed = 10.0;
        let brake = 375.0; // the cruise ceiling, split four ways
        let mut state = WheelInput {
            omega: speed / spec.radius,
            load: nominal_load(),
            forward_speed: speed,
            brake_torque: brake,
            ..Default::default()
        };
        let mut out = step_wheel(&state, &spec, DT);
        for _ in 0..255 {
            state.omega = out.omega;
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
            out = step_wheel(&state, &spec, DT);
        }
        // Balance: the tire's torque about the axle cancels the brake's.
        let expected = -brake / spec.radius;
        assert!(!out.force.saturated, "a light brake saturated the tire");
        assert!(
            (out.force.longitudinal - expected).abs() < 0.1 * expected.abs(),
            "a {brake} N.m brake produced {} N of drag, not {expected}",
            out.force.longitudinal
        );
    }

    #[test]
    fn a_locked_wheel_slides_at_slip_minus_one() {
        assert!((slip_ratio(0.0, 0.32, 10.0) + 1.0).abs() < 1e-6);
        // And through the whole step: enough brake to stop the wheel while the
        // car is still moving is, by definition, a locked slide. The drag
        // builds over the tire's relaxation time rather than appearing at
        // once, so give it a few ticks to develop.
        let mut state = WheelInput {
            load: nominal_load(),
            forward_speed: 10.0,
            brake_torque: 100_000.0,
            ..Default::default()
        };
        let mut out = step_wheel(&state, &spec(), DT);
        for _ in 0..32 {
            state.omega = out.omega;
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
            out = step_wheel(&state, &spec(), DT);
        }
        assert_eq!(out.omega, 0.0);
        assert!((out.slip_ratio + 1.0).abs() < 1e-6);
        // Sliding forward means the tire drags backward on the chassis, at
        // the full friction circle.
        assert!(out.force.longitudinal < 0.0);
        assert!(
            out.force.longitudinal.abs() > 0.8 * spec().tire.mu * nominal_load(),
            "a fully locked wheel should be near the friction limit, got {}",
            out.force.longitudinal
        );
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

        // A modest brake bleeds spin off gradually instead. Measured on an
        // airborne wheel, where the brake is the only torque there is: with a
        // tire under it the wheel's spin is set by the *balance* of brake and
        // tire, and a wheel turning below road speed is being driven back up
        // to it faster than a light brake can drag it down -- so "a braked
        // wheel always slows" is not a claim about a wheel in contact. The
        // claim here is only that the brake is a torque and not an instant
        // stop, and that is what this measures.
        let out = step_wheel(
            &WheelInput {
                omega: 30.0,
                brake_torque: 200.0,
                forward_speed: 10.0,
                ..Default::default()
            },
            &spec(),
            DT,
        );
        let bled = 200.0 / spec().inertia * DT;
        assert!(
            (out.omega - (30.0 - bled)).abs() < 1e-3,
            "omega {} after bleeding {bled} rad/s off 30",
            out.omega
        );
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
            let mut state = WheelInput {
                load: 5000.0,
                forward_speed,
                engine_torque: 800.0,
                ..Default::default()
            };
            for _ in 0..5000 {
                let out = step_wheel(&state, &stiff(), DT);
                state.omega = out.omega;
                state.relaxed_slip_ratio = out.relaxed_slip_ratio;
                state.relaxed_slip_angle = out.relaxed_slip_angle;
                assert!(
                    state.omega.is_finite() && state.omega.abs() < 1.0e4,
                    "omega ran away to {} at {forward_speed} m/s",
                    state.omega
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

        let mut state = WheelInput {
            load,
            forward_speed,
            ..Default::default()
        };
        let mut omega = 0.0f32;
        let mut implicit_swing = 0.0f32;
        for step in 0..200 {
            state.omega = omega;
            let out = step_wheel(&state, &spec, DT);
            state.relaxed_slip_ratio = out.relaxed_slip_ratio;
            state.relaxed_slip_angle = out.relaxed_slip_angle;
            if step > 100 {
                implicit_swing = implicit_swing.max((out.omega - omega).abs());
            }
            omega = out.omega;
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
