use thiserror::Error;

/// An FMI value reference -- the numeric handle an FMU addresses a variable by.
/// `u32` matches FMI's `fmi3ValueReference`.
pub type ValueReference = u32;

/// The per-tick surface a co-simulation FMU instance exposes once loaded and
/// initialized. Slice 2's `fmi`-crate-backed type implements this; the pure
/// logic here is written against the trait so it drives an in-memory fake in
/// tests. Value references must come from a resolved
/// [`crate::BindingSpec`]/[`crate::ResolvedBinding`]; the FMU rejects unknown
/// ones.
pub trait FmuInstance {
    /// Set an input variable (FMI Float64) by value reference.
    fn set_input(&mut self, vr: ValueReference, value: f64) -> Result<(), FmuError>;
    /// Advance the FMU by `step_size` seconds from `current_time`. The
    /// [`StepOutcome`] carries `fmi3DoStep`'s out-params -- a terminating or
    /// early-returning FMU must not be treated as a clean full step.
    fn do_step(&mut self, current_time: f64, step_size: f64) -> Result<StepOutcome, FmuError>;
    /// Read an output variable (FMI Float64) by value reference. `&mut` because
    /// the FFI layer a real impl wraps takes the instance mutably.
    fn get_output(&mut self, vr: ValueReference) -> Result<f64, FmuError>;
}

/// The out-params of one `fmi3DoStep`, kept faithful to the ABI so the coupling
/// layer (slices 2/4) can react instead of assuming every step advanced a full
/// `step_size`. On a normal step every flag is `false` and
/// `last_successful_time == current_time + step_size`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepOutcome {
    /// `eventHandlingNeeded`: the FMU has an event to process.
    pub event_handling_needed: bool,
    /// `terminateSimulation`: the FMU requests the run stop.
    pub terminate_simulation: bool,
    /// `earlyReturn`: the FMU returned before completing the requested step.
    pub early_return: bool,
    /// `lastSuccessfulTime`: the time the FMU actually reached.
    pub last_successful_time: f64,
}

/// A failure from the FMU's per-tick calls. Loading and instantiation errors
/// are a separate concern owned by slice 2's loader.
#[derive(Debug, Error)]
pub enum FmuError {
    /// The FMU rejected a set on this value reference (bad reference/state).
    #[error("FMU rejected setting value-reference {vr}")]
    SetInput { vr: ValueReference },
    /// `doStep` returned a non-OK status.
    #[error("FMU doStep failed at t={time} step={step}")]
    DoStep { time: f64, step: f64 },
    /// The FMU rejected a get on this value reference.
    #[error("FMU rejected reading value-reference {vr}")]
    GetOutput { vr: ValueReference },
}
