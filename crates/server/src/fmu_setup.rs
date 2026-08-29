//! Turning a scenario's `FmuConfig` into a live, bound FMU at spawn.
//!
//! Two steps, both fallible and both recoverable scenario-config problems (never
//! a panic): [`to_binding_spec`] maps the protocol schema onto
//! `dynamics_fmi`'s `BindingSpec` (a pure, dependency-free rename that has to be
//! kept in sync by test, since `protocol` and `dynamics-fmi` don't depend on
//! each other), and [`load_fmu_vehicle`] loads the `.fmu`, resolves the binding
//! against the FMU's own interface, and hands back the instance plus its
//! resolved role -> value-reference map.

use dynamics_fmi::{
    BindError, BindingSpec, Fmu, GroundBinding, InputBinding, LoadError, OutputBinding,
    ResolvedBinding,
};
use protocol::scenario::FmuConfig;
use thiserror::Error;

/// Why an `FmuVehicle` slot could not be brought up. Both variants are clean,
/// actionable scenario-config errors surfaced to the operator, not panics.
#[derive(Debug, Error)]
pub enum FmuSetupError {
    /// The `.fmu` could not be loaded/instantiated (missing file, not FMI 3.0,
    /// no Co-Simulation interface, dlopen failure, init rejected).
    #[error("loading FMU `{path}`: {source}")]
    Load { path: String, source: LoadError },
    /// The `.fmu` loaded, but the scenario's variable binding does not fit its
    /// interface (an unknown/mis-typed/mis-directed variable, or a duplicate).
    #[error("binding FMU `{path}`: {source}")]
    Bind { path: String, source: BindError },
}

/// Map the protocol `FmuConfig` onto `dynamics_fmi::BindingSpec`. Field names
/// match one-for-one; the only shape difference is ground `height`/`normal_z`,
/// which are required `String` in the schema and become `Some(..)` here (the
/// `GroundBinding` treats all three ground roles as optional). `friction`
/// passes straight through. This is the single place the two crates' names are
/// tied together, so `binding_spec_maps_every_role` guards it.
pub fn to_binding_spec(cfg: &FmuConfig) -> BindingSpec {
    BindingSpec {
        inputs: InputBinding {
            steer: cfg.inputs.steer.clone(),
            throttle: cfg.inputs.throttle.clone(),
            brake: cfg.inputs.brake.clone(),
        },
        ground: GroundBinding {
            height: Some(cfg.ground.height.clone()),
            normal_z: Some(cfg.ground.normal_z.clone()),
            friction: cfg.ground.friction.clone(),
        },
        outputs: OutputBinding {
            x: cfg.outputs.x.clone(),
            y: cfg.outputs.y.clone(),
            z: cfg.outputs.z.clone(),
            yaw: cfg.outputs.yaw.clone(),
        },
    }
}

/// Load the `.fmu`, then resolve the converted binding against its interface.
/// `instance_name` (the agent/slot id) identifies the instance in the FMU's own
/// logs. Returns the live instance and its resolved binding, or a clean
/// [`FmuSetupError`]; the caller stores the instance in the `NonSend`
/// `FmuStore` and puts the binding on the entity's `FmuVehicle` component.
pub fn load_fmu_vehicle(
    cfg: &FmuConfig,
    instance_name: &str,
) -> Result<(Fmu, ResolvedBinding), FmuSetupError> {
    let fmu = Fmu::load(&cfg.path, 0.0, instance_name).map_err(|source| FmuSetupError::Load {
        path: cfg.path.clone(),
        source,
    })?;
    let spec = to_binding_spec(cfg);
    let resolved = spec
        .resolve(fmu.model_description())
        .map_err(|source| FmuSetupError::Bind {
            path: cfg.path.clone(),
            source,
        })?;
    Ok((fmu, resolved))
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::scenario::{FmuGround, FmuInputs, FmuOutputs};

    /// The committed FMI 3.0 Reference FMU (a pure Van der Pol oscillator). It
    /// has no `Input`-causality variables and no pose outputs, so it can never
    /// be bound as a vehicle -- used here only for the error-path tests.
    const VANDERPOL: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../dynamics-fmi/tests/data/VanDerPol.fmu"
    );

    fn config(path: &str, friction: Option<&str>) -> FmuConfig {
        FmuConfig {
            path: path.into(),
            inputs: FmuInputs {
                steer: "delta".into(),
                throttle: "ax".into(),
                brake: "brk".into(),
            },
            ground: FmuGround {
                height: "z_road".into(),
                normal_z: "n_z".into(),
                friction: friction.map(Into::into),
            },
            outputs: FmuOutputs {
                x: "X".into(),
                y: "Y".into(),
                z: "Z".into(),
                yaw: "psi".into(),
            },
        }
    }

    #[test]
    fn binding_spec_maps_every_role() {
        let spec = to_binding_spec(&config("whatever.fmu", Some("mu")));
        assert_eq!(spec.inputs.steer, "delta");
        assert_eq!(spec.inputs.throttle, "ax");
        assert_eq!(spec.inputs.brake, "brk");
        // Required schema fields become Some(..).
        assert_eq!(spec.ground.height.as_deref(), Some("z_road"));
        assert_eq!(spec.ground.normal_z.as_deref(), Some("n_z"));
        // Optional friction passes straight through.
        assert_eq!(spec.ground.friction.as_deref(), Some("mu"));
        assert_eq!(spec.outputs.x, "X");
        assert_eq!(spec.outputs.y, "Y");
        assert_eq!(spec.outputs.z, "Z");
        assert_eq!(spec.outputs.yaw, "psi");
    }

    #[test]
    fn omitted_friction_maps_to_none() {
        let spec = to_binding_spec(&config("whatever.fmu", None));
        assert_eq!(spec.ground.friction, None);
        // The required ground roles are still bound.
        assert!(spec.ground.height.is_some());
        assert!(spec.ground.normal_z.is_some());
    }

    #[test]
    fn a_missing_fmu_is_a_clean_error_not_a_panic() {
        // Not `expect_err`: the Ok type holds a live `Fmu`, which is not `Debug`
        // (a foreign FMI handle), so match the error out by hand instead.
        let Err(err) = load_fmu_vehicle(&config("does/not/exist.fmu", None), "agent-1") else {
            panic!("a missing .fmu must fail, not load");
        };
        assert!(matches!(err, FmuSetupError::Load { .. }), "got {err:?}");
    }

    #[test]
    fn a_non_vehicle_fmu_fails_to_bind_cleanly() {
        // VanDerPol has no Input-causality variables and no pose outputs, so the
        // vehicle binding cannot resolve against it -- a clean Bind error, never
        // a panic. (Its `.fmu` DOES load, so this exercises the resolve path,
        // not the load path.)
        let Err(err) = load_fmu_vehicle(&config(VANDERPOL, None), "agent-1") else {
            panic!("VanDerPol cannot satisfy a vehicle binding");
        };
        assert!(matches!(err, FmuSetupError::Bind { .. }), "got {err:?}");
    }
}
