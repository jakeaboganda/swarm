//! FMU-managed vehicle: dynamics computed by an external FMI 3.0 co-simulation
//! FMU rather than one of the force-based embodiments. The FMU integrates its
//! own pose; we impose it on a `KinematicPositionBased` Rapier body each tick,
//! so it shows up in perception/collision but is never pushed back (see the
//! FMU/FMI design in DECISIONS.md).
//!
//! Split, like the rest of movement, into a pure decision and a Bevy shell:
//! [`fmu_control_step`] is the testable core (plan -> pedals -> FMU inputs ->
//! doStep -> pose), and [`drive_fmu_vehicles`] is the system that samples the
//! ground and writes the pose.
//!
//! The FMU handle is `!Send + !Sync` (a foreign native binary), so it cannot be
//! an ECS component; it lives in the [`FmuStore`] `NonSend` resource, keyed by
//! entity, which confines FMU stepping to the main thread. The per-entity
//! plain-data ([`FmuVehicle`]) -- the driver and the resolved binding -- is an
//! ordinary component. `server` (its scenario layer) builds both at spawn.

use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::prelude::{QueryFilter, ReadRapierContext, Velocity};
use dynamics_fmi::{
    read_pose, Controls, Driver, DriverInput, FmuError, FmuInstance, Pose, ResolvedBinding,
    StepOutcome,
};

use crate::model::DesiredVelocity;

/// Where the ground ray starts, in metres above the chassis origin -- high
/// enough to sit above the surface before casting down.
const GROUND_RAY_START_ABOVE: f32 = 2.0;
/// How far down the ground ray reaches (m). Generous, to find the road under a
/// car on a grade.
const GROUND_RAY_MAX: f32 = 20.0;
/// Ground friction fed to the FMU. v1 has no per-surface friction data, so this
/// is a constant; a real friction field arrives with the per-wheel ground (v2).
const DEFAULT_FRICTION: f32 = 1.0;
/// Ground height used when the down-ray hits nothing (off the mesh / arena
/// floor): assume flat ground at world y = 0.
const FLAT_GROUND_HEIGHT: f32 = 0.0;

/// Per-entity FMU-vehicle state that is plain data (so it is a normal, `Send +
/// Sync` component): the driver (carrying its PI integrator) and the resolved
/// role -> value-reference binding, plus the FMU's running communication-point
/// time. The FMU handle itself lives in [`FmuStore`], not here.
///
/// The FMU integrates yaw as a real DOF, so a spawned FmuVehicle body MUST also
/// carry [`crate::model::PhysicalYaw`]; otherwise the cosmetic
/// `face_velocity_direction` system overwrites the FMU's heading with a
/// velocity-facing one every frame. (`server` inserts it at spawn, like it does
/// for `FullVehicle`/`RaycastVehicle`.)
#[derive(Component, Debug, Clone)]
pub struct FmuVehicle {
    /// Plan -> pedals controller; `&mut` each tick (holds integrator state).
    pub driver: Driver,
    /// Role -> FMI value references, resolved once at spawn.
    pub binding: ResolvedBinding,
    /// The FMU's current communication point (s), accumulated from `dt`. We
    /// drive this off our own clock; `StepOutcome::last_successful_time` is only
    /// advisory (an FMU with a coarser internal step lags it).
    pub elapsed: f64,
    /// Latched once the FMU reports `terminate_simulation`: after that the body
    /// is frozen and the FMU is never stepped again (stepping a terminated FMI
    /// instance is an unsupported state transition), until `server` despawns it.
    pub terminated: bool,
}

impl FmuVehicle {
    pub fn new(driver: Driver, binding: ResolvedBinding) -> Self {
        Self {
            driver,
            binding,
            elapsed: 0.0,
            terminated: false,
        }
    }
}

/// The per-entity FMU instances, held as a main-thread-only `NonSend` resource
/// because a loaded FMU is `!Send + !Sync`. `server` inserts one per spawned
/// FMU vehicle and removes it on despawn.
#[derive(Default)]
pub struct FmuStore(pub HashMap<Entity, Box<dyn FmuInstance>>);

impl FmuStore {
    pub fn insert(&mut self, entity: Entity, fmu: Box<dyn FmuInstance>) {
        self.0.insert(entity, fmu);
    }

    pub fn remove(&mut self, entity: Entity) -> Option<Box<dyn FmuInstance>> {
        self.0.remove(&entity)
    }

    pub fn get_mut(&mut self, entity: Entity) -> Option<&mut Box<dyn FmuInstance>> {
        self.0.get_mut(&entity)
    }
}

/// The ground under the chassis this tick, in engine (Y-up) terms. `height` is
/// the world y of the contact point; `normal_z` is the *up* component of the
/// surface normal (`normal.y` here) -- named for the FMU's own Z-up convention,
/// where the up-axis is Z, so `normal_z` is "how level the ground is" (1.0 flat,
/// less on a slope). `friction` is a coefficient.
#[derive(Debug, Clone, Copy)]
pub struct GroundSample {
    pub height: f32,
    pub normal_z: f32,
    pub friction: f32,
}

/// What one [`fmu_control_step`] produced: the pose to impose and the raw
/// doStep outcome (so the caller can react to terminate/early-return).
#[derive(Debug, Clone, Copy)]
pub struct FmuStep {
    pub pose: Pose,
    pub outcome: StepOutcome,
    pub controls: Controls,
}

/// The pure control step for one FMU vehicle, testable against a fake
/// [`FmuInstance`]. Turns the plan into pedals, pushes pedals + ground into the
/// FMU, advances it by `dt` from `time`, and reads the pose back. Only the
/// *bound* ground roles are written (an FMU that exposes no `friction` input has
/// `None` there). The `StepOutcome` is returned, never swallowed: a terminating
/// or early-returning FMU is the caller's to handle.
pub fn fmu_control_step(
    driver: &mut Driver,
    binding: &ResolvedBinding,
    fmu: &mut dyn FmuInstance,
    input: DriverInput,
    ground: GroundSample,
    time: f64,
    dt: f32,
) -> Result<FmuStep, FmuError> {
    let controls = driver.control(input, dt);

    // Driver actuators (always bound).
    fmu.set_input(binding.inputs.steer, controls.steer as f64)?;
    fmu.set_input(binding.inputs.throttle, controls.throttle as f64)?;
    fmu.set_input(binding.inputs.brake, controls.brake as f64)?;

    // Ground query (only the roles this FMU actually exposes).
    if let Some(vr) = binding.ground.height {
        fmu.set_input(vr, ground.height as f64)?;
    }
    if let Some(vr) = binding.ground.normal_z {
        fmu.set_input(vr, ground.normal_z as f64)?;
    }
    if let Some(vr) = binding.ground.friction {
        fmu.set_input(vr, ground.friction as f64)?;
    }

    let outcome = fmu.do_step(time, dt as f64)?;
    let pose = read_pose(fmu, &binding.outputs)?;
    Ok(FmuStep {
        pose,
        outcome,
        controls,
    })
}

/// Horizontal unit vector in the xz-plane, defaulting to +X when degenerate.
fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z).normalize_or(Vec3::X)
}

/// Steps every FMU vehicle and imposes its pose on the (kinematic) body.
///
/// Uses `NonSendMut<FmuStore>`, which makes this a main-thread system -- exactly
/// the confinement the `!Send` FMU handle needs. An entity with an `FmuVehicle`
/// component but no handle in the store (not yet populated) is simply skipped.
pub fn drive_fmu_vehicles(
    time: Res<Time>,
    rapier: ReadRapierContext,
    mut store: NonSendMut<FmuStore>,
    mut query: Query<(
        Entity,
        &DesiredVelocity,
        &Velocity,
        &mut Transform,
        &mut FmuVehicle,
    )>,
) {
    let dt = time.delta_secs();
    let Ok(context) = rapier.single() else {
        return;
    };

    for (entity, desired, velocity, mut transform, mut vehicle) in &mut query {
        // A terminated FMU is frozen: never step it again (stepping past a
        // terminate request is an unsupported FMI transition), and warn only
        // the once, not every tick.
        if vehicle.terminated {
            continue;
        }
        let Some(fmu) = store.get_mut(entity) else {
            continue;
        };

        let heading = horizontal(*transform.forward());
        let speed = velocity.linear.dot(heading);
        let input = DriverInput {
            desired_velocity: desired.value,
            lookahead: desired.lookahead,
            heading,
            speed,
            urgent: desired.urgent,
        };

        // Single-point ground under the chassis: cast straight down (world -Y)
        // from just above the body. v1 is one point; per-wheel Fz is v2.
        let origin = transform.translation + Vec3::Y * GROUND_RAY_START_ABOVE;
        let filter = QueryFilter::default().exclude_rigid_body(entity);
        let ground =
            match context.cast_ray_and_get_normal(origin, -Vec3::Y, GROUND_RAY_MAX, true, filter) {
                Some((_, hit)) => GroundSample {
                    height: hit.point.y,
                    normal_z: hit.normal.y,
                    friction: DEFAULT_FRICTION,
                },
                None => GroundSample {
                    height: FLAT_GROUND_HEIGHT,
                    normal_z: 1.0,
                    friction: DEFAULT_FRICTION,
                },
            };

        // Split-borrow the plain component so the driver (mut) and binding
        // (shared) can be passed together; copy the time out first.
        let veh = &mut *vehicle;
        let time_now = veh.elapsed;
        match fmu_control_step(
            &mut veh.driver,
            &veh.binding,
            fmu.as_mut(),
            input,
            ground,
            time_now,
            dt,
        ) {
            Ok(step) => {
                if step.outcome.terminate_simulation {
                    // The FMU asked to stop. Latch it so we neither step nor
                    // warn again; freeze the body in place. The scenario/
                    // lifecycle layer (server) owns despawn.
                    warn!("FMU for {entity:?} requested termination; freezing in place");
                    veh.terminated = true;
                    continue;
                }
                // early_return can't fire on our path -- `Fmu::do_step` sets
                // `early_return_allowed = false`, so a conformant FMU never
                // returns before `dt`. If a future config allows it, the pose is
                // at `last_successful_time` and advancing `elapsed` by the full
                // `dt` would drift; revisit here then.
                transform.translation = step.pose.position;
                transform.rotation = Quat::from_rotation_y(step.pose.yaw);
                veh.elapsed += dt as f64;
            }
            Err(err) => {
                warn!("FMU step failed for {entity:?}: {err}");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dynamics_fmi::{ResolvedGround, ResolvedInputs, ResolvedOutputs, ValueReference};
    use std::collections::HashMap;

    /// In-memory `FmuInstance`: a value store keyed by value reference, a
    /// configurable step outcome, and a flag that `do_step` ran. Pose outputs
    /// are seeded directly (this fake does no dynamics).
    struct FakeFmu {
        values: HashMap<ValueReference, f64>,
        outcome: StepOutcome,
        stepped: bool,
    }

    impl FakeFmu {
        fn new() -> Self {
            Self {
                values: HashMap::new(),
                outcome: StepOutcome {
                    event_handling_needed: false,
                    terminate_simulation: false,
                    early_return: false,
                    last_successful_time: 0.0,
                },
                stepped: false,
            }
        }
    }

    impl FmuInstance for FakeFmu {
        fn set_input(&mut self, vr: ValueReference, value: f64) -> Result<(), FmuError> {
            self.values.insert(vr, value);
            Ok(())
        }
        fn do_step(&mut self, current_time: f64, step_size: f64) -> Result<StepOutcome, FmuError> {
            self.stepped = true;
            let mut outcome = self.outcome;
            if outcome.last_successful_time == 0.0 {
                outcome.last_successful_time = current_time + step_size;
            }
            Ok(outcome)
        }
        fn get_output(&mut self, vr: ValueReference) -> Result<f64, FmuError> {
            self.values
                .get(&vr)
                .copied()
                .ok_or(FmuError::GetOutput { vr })
        }
    }

    // Value references used by the test binding. Inputs and outputs must not
    // collide (a real binding rejects that; here we just keep them distinct).
    const STEER: ValueReference = 1;
    const THROTTLE: ValueReference = 2;
    const BRAKE: ValueReference = 3;
    const HEIGHT: ValueReference = 4;
    const NORMAL_Z: ValueReference = 5;
    const OUT_X: ValueReference = 10;
    const OUT_Y: ValueReference = 11;
    const OUT_Z: ValueReference = 12;
    const OUT_YAW: ValueReference = 13;

    /// A binding with ground height + normal_z bound but friction absent, to
    /// prove optional ground roles are skipped.
    fn test_binding() -> ResolvedBinding {
        ResolvedBinding {
            inputs: ResolvedInputs {
                steer: STEER,
                throttle: THROTTLE,
                brake: BRAKE,
            },
            ground: ResolvedGround {
                height: Some(HEIGHT),
                normal_z: Some(NORMAL_Z),
                friction: None,
            },
            outputs: ResolvedOutputs {
                x: OUT_X,
                y: OUT_Y,
                z: OUT_Z,
                yaw: OUT_YAW,
            },
        }
    }

    fn ground() -> GroundSample {
        GroundSample {
            height: 0.5,
            normal_z: 0.98,
            friction: DEFAULT_FRICTION,
        }
    }

    fn cruise_input() -> DriverInput {
        DriverInput {
            // Aim straight ahead (+X), moderate speed, already heading +X so no
            // steering is called for.
            desired_velocity: Vec3::new(6.0, 0.0, 0.0),
            lookahead: 5.0,
            heading: Vec3::X,
            speed: 4.0,
            urgent: false,
        }
    }

    #[test]
    fn inputs_land_on_the_bound_value_references() {
        let mut driver = Driver::default();
        let binding = test_binding();
        let mut fmu = FakeFmu::new();
        // Seed the pose outputs the fake will report back.
        fmu.values.insert(OUT_X, 1.0);
        fmu.values.insert(OUT_Y, 0.0);
        fmu.values.insert(OUT_Z, 2.0);
        fmu.values.insert(OUT_YAW, 0.25);

        let step = fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            cruise_input(),
            ground(),
            0.0,
            1.0 / 64.0,
        )
        .expect("step");

        // Driver actuators reached their references.
        assert_eq!(fmu.values[&STEER], step.controls.steer as f64);
        assert_eq!(fmu.values[&THROTTLE], step.controls.throttle as f64);
        assert_eq!(fmu.values[&BRAKE], step.controls.brake as f64);
        // Bound ground roles reached theirs.
        assert_eq!(fmu.values[&HEIGHT], 0.5);
        // normal_z originates as f32 (a raycast normal component) and is set as
        // f64, so compare against the f32-widened value, not the f64 literal.
        assert_eq!(fmu.values[&NORMAL_Z], 0.98_f32 as f64);
        assert!(fmu.stepped, "do_step must run");
    }

    #[test]
    fn unbound_ground_role_is_not_set() {
        let mut driver = Driver::default();
        let binding = test_binding(); // friction: None
        let mut fmu = FakeFmu::new();
        for vr in [OUT_X, OUT_Y, OUT_Z, OUT_YAW] {
            fmu.values.insert(vr, 0.0);
        }
        fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            cruise_input(),
            ground(),
            0.0,
            1.0 / 64.0,
        )
        .expect("step");
        // No friction reference was bound, so nothing was written for it. The
        // only keys present are the three actuators, the two ground roles, and
        // the four seeded outputs -- never a stray friction write.
        assert_eq!(fmu.values.len(), 3 + 2 + 4);
    }

    #[test]
    fn pose_is_read_from_the_bound_outputs() {
        let mut driver = Driver::default();
        let binding = test_binding();
        let mut fmu = FakeFmu::new();
        fmu.values.insert(OUT_X, 3.0);
        fmu.values.insert(OUT_Y, 0.0);
        fmu.values.insert(OUT_Z, -4.0);
        fmu.values.insert(OUT_YAW, 1.5);

        let step = fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            cruise_input(),
            ground(),
            0.0,
            1.0 / 64.0,
        )
        .expect("step");
        assert_eq!(step.pose.position, Vec3::new(3.0, 0.0, -4.0));
        assert_eq!(step.pose.yaw, 1.5);
    }

    #[test]
    fn urgent_commands_full_brake_through_to_the_fmu() {
        let mut driver = Driver::default();
        let binding = test_binding();
        let mut fmu = FakeFmu::new();
        for vr in [OUT_X, OUT_Y, OUT_Z, OUT_YAW] {
            fmu.values.insert(vr, 0.0);
        }
        let mut input = cruise_input();
        input.urgent = true;

        let step = fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            input,
            ground(),
            0.0,
            1.0 / 64.0,
        )
        .expect("step");
        assert_eq!(step.controls.brake, 1.0);
        assert_eq!(step.controls.throttle, 0.0);
        assert_eq!(step.controls.steer, 0.0);
        assert_eq!(fmu.values[&BRAKE], 1.0);
        assert_eq!(fmu.values[&THROTTLE], 0.0);
    }

    #[test]
    fn terminate_outcome_is_surfaced_not_swallowed() {
        let mut driver = Driver::default();
        let binding = test_binding();
        let mut fmu = FakeFmu::new();
        for vr in [OUT_X, OUT_Y, OUT_Z, OUT_YAW] {
            fmu.values.insert(vr, 0.0);
        }
        fmu.outcome.terminate_simulation = true;

        let step = fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            cruise_input(),
            ground(),
            0.0,
            1.0 / 64.0,
        )
        .expect("step");
        assert!(
            step.outcome.terminate_simulation,
            "a terminating FMU must be reported to the caller"
        );
    }

    #[test]
    fn a_set_input_failure_propagates() {
        // A binding pointing at a reference the FMU rejects surfaces as an Err,
        // never a silent miss. Our fake accepts every set, so simulate rejection
        // with a fake that fails on a known reference.
        struct Rejecting;
        impl FmuInstance for Rejecting {
            fn set_input(&mut self, vr: ValueReference, _v: f64) -> Result<(), FmuError> {
                Err(FmuError::SetInput { vr })
            }
            fn do_step(&mut self, _t: f64, _s: f64) -> Result<StepOutcome, FmuError> {
                unreachable!("set fails first")
            }
            fn get_output(&mut self, vr: ValueReference) -> Result<f64, FmuError> {
                Err(FmuError::GetOutput { vr })
            }
        }
        let mut driver = Driver::default();
        let binding = test_binding();
        let mut fmu = Rejecting;
        let result = fmu_control_step(
            &mut driver,
            &binding,
            &mut fmu,
            cruise_input(),
            ground(),
            0.0,
            1.0 / 64.0,
        );
        assert!(result.is_err());
    }
}
