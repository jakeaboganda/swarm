//! Pure core for FMU-backed vehicle dynamics.
//!
//! The [`FmuInstance`] trait is the seam a real FMI 3.0 co-simulation instance
//! satisfies (slice 2, the `fmi`-crate-backed impl). Everything else here is
//! the Bevy/Rapier-free logic around it: [`BindingSpec`] resolution (roles ->
//! FMI value references), the plan-to-pedals [`Controller`], and [`read_pose`]. No
//! `fmi`/engine deps, so it unit-tests in isolation against an in-memory fake.

mod binding;
mod controller;
mod fmu;
mod frame;
mod instance;
mod model;
mod pose;

pub use binding::{
    BindError, BindingSpec, GroundBinding, InputBinding, OutputBinding, ResolvedBinding,
    ResolvedGround, ResolvedInputs, ResolvedOutputs,
};
pub use controller::{Controller, ControllerConfig, ControllerInput, Controls};
pub use fmu::{Fmu, LoadError};
pub use frame::{to_sim_local, FmuFrame};
pub use instance::{FmuError, FmuInstance, StepOutcome, ValueReference};
pub use model::{BaseType, Causality, ModelDescription, Variable};
pub use pose::{read_pose, Pose};
