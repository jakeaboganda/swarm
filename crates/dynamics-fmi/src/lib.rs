//! Pure core for FMU-backed vehicle dynamics.
//!
//! The [`FmuInstance`] trait is the seam a real FMI 3.0 co-simulation instance
//! satisfies (slice 2, the `fmi`-crate-backed impl). Everything else here is
//! the Bevy/Rapier-free logic around it: [`BindingSpec`] resolution (roles ->
//! FMI value references), the plan-to-pedals [`Driver`], and [`read_pose`]. No
//! `fmi`/engine deps, so it unit-tests in isolation against an in-memory fake.

mod binding;
mod driver;
mod instance;
mod model;
mod pose;

pub use binding::{
    BindError, BindingSpec, GroundBinding, InputBinding, OutputBinding, ResolvedBinding,
    ResolvedGround, ResolvedInputs, ResolvedOutputs,
};
pub use driver::{Controls, Driver, DriverConfig, DriverInput};
pub use instance::{FmuError, FmuInstance, ValueReference};
pub use model::{Causality, ModelDescription, Variable};
pub use pose::{read_pose, Pose};
