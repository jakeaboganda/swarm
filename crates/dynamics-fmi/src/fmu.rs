//! The real [`FmuInstance`], backed by the `fmi` crate: load a `.fmu`,
//! instantiate it for FMI 3.0 Co-Simulation, drive it per tick. Loading and
//! initialization live behind [`LoadError`] (a once-at-load, recoverable
//! scenario-config concern); the per-tick calls use [`FmuError`].

use std::path::Path;

use thiserror::Error;

use fmi::fmi3::schema::{
    AbstractVariableTrait, Causality as SchemaCausality, Fmi3ModelDescription,
    Variable as SchemaVariable,
};
use fmi::fmi3::{
    import::Fmi3Import, instance::InstanceCS, CoSimulation, Common, Fmi3Model, GetSet,
};
use fmi::traits::FmiImport;

use crate::instance::{FmuError, FmuInstance, StepOutcome, ValueReference};
use crate::model::{BaseType, Causality, ModelDescription, Variable};

/// Errors from loading/instantiating/initializing an FMU -- distinct from the
/// per-tick [`FmuError`]. These happen once, at load, and are recoverable
/// scenario-config problems (missing file, wrong FMI kind, no Co-Simulation
/// interface, dlopen failure), never a programmer-error panic.
#[derive(Debug, Error)]
pub enum LoadError {
    /// Importing or instantiating failed: bad archive, missing
    /// `modelDescription.xml`, not FMI 3.0, no Co-Simulation interface,
    /// unsupported platform, or a shared-library load failure.
    #[error(transparent)]
    Import(#[from] fmi::Error),
    /// The FMU rejected initialization mode.
    #[error("FMU initialization failed: {0:?}")]
    Init(fmi::fmi3::Fmi3Error),
}

/// A loaded, instantiated, initialized FMI 3.0 Co-Simulation FMU, ready to
/// `do_step`. Implements [`FmuInstance`] for the per-tick seam.
pub struct Fmu {
    // Drop runs in declaration order: `instance` frees itself
    // (`fmi3FreeInstance`) before `_import` tears down the extracted FMU
    // directory it was loaded from.
    instance: InstanceCS,
    model_description: ModelDescription,
    // Held only to keep the extracted FMU directory alive for `instance`.
    _import: Fmi3Import,
}

impl Fmu {
    /// Load a `.fmu`, instantiate it for FMI 3.0 Co-Simulation, and run it
    /// through initialization mode so it is ready to step. Communication starts
    /// at `start_time` (usually 0.0) -- pass the matching `current_time` to the
    /// first [`FmuInstance::do_step`]. `instance_name` identifies this instance
    /// in the FMU's own log/error messages; pass the agent/slot id so a message
    /// from one of several FMU vehicles in a scenario is attributable.
    pub fn load(
        path: impl AsRef<Path>,
        start_time: f64,
        instance_name: &str,
    ) -> Result<Self, LoadError> {
        let import: Fmi3Import = fmi::import::from_path(path)?;
        let model_description = build_model_description(import.model_description());

        // event_mode_used = false: after exit_initialization_mode the instance
        // is in Step Mode directly, ready for do_step. early_return_allowed =
        // false: full communication steps (our v1 assumption).
        let mut instance = import.instantiate_cs(
            instance_name,
            false, // visible
            false, // logging_on
            false, // event_mode_used
            false, // early_return_allowed
            &[],   // required_intermediate_variables
        )?;

        instance
            .enter_initialization_mode(None, start_time, None)
            .map_err(LoadError::Init)?;
        instance
            .exit_initialization_mode()
            .map_err(LoadError::Init)?;

        Ok(Self {
            instance,
            model_description,
            _import: import,
        })
    }

    /// The FMU's parsed interface, for resolving a [`crate::BindingSpec`].
    pub fn model_description(&self) -> &ModelDescription {
        &self.model_description
    }
}

impl FmuInstance for Fmu {
    fn set_input(&mut self, vr: ValueReference, value: f64) -> Result<(), FmuError> {
        self.instance
            .set_float64(&[vr], &[value])
            .map(|_| ())
            .map_err(|_| FmuError::SetInput { vr })
    }

    fn do_step(&mut self, current_time: f64, step_size: f64) -> Result<StepOutcome, FmuError> {
        let mut event_handling_needed = false;
        let mut terminate_simulation = false;
        let mut early_return = false;
        let mut last_successful_time = current_time;
        self.instance
            .do_step(
                current_time,
                step_size,
                true, // no_set_fmu_state_prior_to_current_point
                &mut event_handling_needed,
                &mut terminate_simulation,
                &mut early_return,
                &mut last_successful_time,
            )
            .map_err(|_| FmuError::DoStep {
                time: current_time,
                step: step_size,
            })?;
        Ok(StepOutcome {
            event_handling_needed,
            terminate_simulation,
            early_return,
            last_successful_time,
        })
    }

    fn get_output(&mut self, vr: ValueReference) -> Result<f64, FmuError> {
        let mut values = [0.0_f64];
        self.instance
            .get_float64(&[vr], &mut values)
            .map_err(|_| FmuError::GetOutput { vr })?;
        Ok(values[0])
    }
}

/// Build our [`ModelDescription`] from the fmi crate's parsed schema. The base
/// type is the schema `Variable` enum's variant; name/value-reference/causality
/// come off the common variable trait.
fn build_model_description(md: &Fmi3ModelDescription) -> ModelDescription {
    let variables = md
        .model_variables
        .variables
        .iter()
        .map(|v| {
            let (base_type, var): (BaseType, &dyn AbstractVariableTrait) = match v {
                SchemaVariable::Float32(x) => (BaseType::Float32, x),
                SchemaVariable::Float64(x) => (BaseType::Float64, x),
                SchemaVariable::Int8(x) => (BaseType::Int8, x),
                SchemaVariable::UInt8(x) => (BaseType::UInt8, x),
                SchemaVariable::Int16(x) => (BaseType::Int16, x),
                SchemaVariable::UInt16(x) => (BaseType::UInt16, x),
                SchemaVariable::Int32(x) => (BaseType::Int32, x),
                SchemaVariable::UInt32(x) => (BaseType::UInt32, x),
                SchemaVariable::Int64(x) => (BaseType::Int64, x),
                SchemaVariable::UInt64(x) => (BaseType::UInt64, x),
                SchemaVariable::Boolean(x) => (BaseType::Boolean, x),
                SchemaVariable::String(x) => (BaseType::String, x),
                SchemaVariable::Binary(x) => (BaseType::Binary, x),
                SchemaVariable::Clock(x) => (BaseType::Clock, x),
            };
            Variable {
                name: var.name().to_string(),
                value_reference: var.value_reference(),
                causality: map_causality(var.causality()),
                base_type,
            }
        })
        .collect();
    ModelDescription::new(variables)
}

fn map_causality(c: SchemaCausality) -> Causality {
    match c {
        SchemaCausality::Parameter => Causality::Parameter,
        SchemaCausality::CalculatedParameter => Causality::CalculatedParameter,
        SchemaCausality::Input => Causality::Input,
        SchemaCausality::Output => Causality::Output,
        SchemaCausality::Local => Causality::Local,
        SchemaCausality::Independent => Causality::Independent,
        SchemaCausality::Dependent => Causality::Dependent,
        SchemaCausality::StructuralParameter => Causality::StructuralParameter,
    }
}
