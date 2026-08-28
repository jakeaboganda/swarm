use thiserror::Error;

use crate::instance::ValueReference;
use crate::model::{Causality, ModelDescription};

/// The driver-actuator inputs every FMU vehicle exposes, named by the FMU's own
/// variable names. Fed from the [`crate::Driver`]; all three are required.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputBinding {
    pub steer: String,
    pub throttle: String,
    pub brake: String,
}

/// Optional ground inputs -- the one-way road query the FMU pushes against. v1
/// is single-point under the chassis; a field the FMU does not expose is left
/// unbound (`None`) and simply not fed.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct GroundBinding {
    pub height: Option<String>,
    pub normal_z: Option<String>,
    pub friction: Option<String>,
}

/// The pose outputs stamped onto the kinematic body each tick.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutputBinding {
    pub x: String,
    pub y: String,
    pub z: String,
    pub yaw: String,
}

/// A scenario's role -> variable-name map for an FMU vehicle. Slice 3's
/// `protocol::FmuConfig` mirrors this shape and maps into it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BindingSpec {
    pub inputs: InputBinding,
    pub ground: GroundBinding,
    pub outputs: OutputBinding,
}

/// Resolved driver inputs: names replaced by the value references the FMU
/// addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolvedInputs {
    pub steer: ValueReference,
    pub throttle: ValueReference,
    pub brake: ValueReference,
}

/// Resolved ground inputs; unbound roles stay `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ResolvedGround {
    pub height: Option<ValueReference>,
    pub normal_z: Option<ValueReference>,
    pub friction: Option<ValueReference>,
}

/// Resolved pose outputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolvedOutputs {
    pub x: ValueReference,
    pub y: ValueReference,
    pub z: ValueReference,
    pub yaw: ValueReference,
}

/// A [`BindingSpec`] with every role resolved to its value reference. Built
/// once at load by [`BindingSpec::resolve`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolvedBinding {
    pub inputs: ResolvedInputs,
    pub ground: ResolvedGround,
    pub outputs: ResolvedOutputs,
}

/// Why a binding could not be resolved against a model description.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum BindError {
    /// A role names a variable the FMU does not declare.
    #[error("role `{role}` names variable `{name}`, which the FMU does not declare")]
    UnknownVariable { role: String, name: String },
    /// A named variable exists but faces the wrong way (e.g. an output role
    /// pointed at an input variable).
    #[error("role `{role}` variable `{name}` has causality {actual:?}, expected {expected:?}")]
    WrongCausality {
        role: String,
        name: String,
        expected: Causality,
        actual: Causality,
    },
}

impl BindingSpec {
    /// Resolve every role to its value reference, checking the named variable
    /// exists and its causality matches the role's direction: inputs and ground
    /// must be `Input`, outputs must be `Output`.
    pub fn resolve(&self, md: &ModelDescription) -> Result<ResolvedBinding, BindError> {
        Ok(ResolvedBinding {
            inputs: ResolvedInputs {
                steer: resolve(md, "steer", &self.inputs.steer, Causality::Input)?,
                throttle: resolve(md, "throttle", &self.inputs.throttle, Causality::Input)?,
                brake: resolve(md, "brake", &self.inputs.brake, Causality::Input)?,
            },
            ground: ResolvedGround {
                height: resolve_opt(md, "ground.height", &self.ground.height, Causality::Input)?,
                normal_z: resolve_opt(
                    md,
                    "ground.normal_z",
                    &self.ground.normal_z,
                    Causality::Input,
                )?,
                friction: resolve_opt(
                    md,
                    "ground.friction",
                    &self.ground.friction,
                    Causality::Input,
                )?,
            },
            outputs: ResolvedOutputs {
                x: resolve(md, "x", &self.outputs.x, Causality::Output)?,
                y: resolve(md, "y", &self.outputs.y, Causality::Output)?,
                z: resolve(md, "z", &self.outputs.z, Causality::Output)?,
                yaw: resolve(md, "yaw", &self.outputs.yaw, Causality::Output)?,
            },
        })
    }
}

fn resolve(
    md: &ModelDescription,
    role: &str,
    name: &str,
    expected: Causality,
) -> Result<ValueReference, BindError> {
    let var = md
        .variable(name)
        .ok_or_else(|| BindError::UnknownVariable {
            role: role.to_string(),
            name: name.to_string(),
        })?;
    if var.causality != expected {
        return Err(BindError::WrongCausality {
            role: role.to_string(),
            name: name.to_string(),
            expected,
            actual: var.causality,
        });
    }
    Ok(var.value_reference)
}

fn resolve_opt(
    md: &ModelDescription,
    role: &str,
    name: &Option<String>,
    expected: Causality,
) -> Result<Option<ValueReference>, BindError> {
    match name {
        Some(n) => Ok(Some(resolve(md, role, n, expected)?)),
        None => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Variable;

    fn var(name: &str, vr: ValueReference, causality: Causality) -> Variable {
        Variable {
            name: name.into(),
            value_reference: vr,
            causality,
        }
    }

    fn spec() -> BindingSpec {
        BindingSpec {
            inputs: InputBinding {
                steer: "delta".into(),
                throttle: "ax".into(),
                brake: "brk".into(),
            },
            ground: GroundBinding {
                height: Some("z_road".into()),
                ..Default::default()
            },
            outputs: OutputBinding {
                x: "X".into(),
                y: "Y".into(),
                z: "Z".into(),
                yaw: "psi".into(),
            },
        }
    }

    fn full_md() -> ModelDescription {
        ModelDescription::new(vec![
            var("delta", 1, Causality::Input),
            var("ax", 2, Causality::Input),
            var("brk", 3, Causality::Input),
            var("z_road", 4, Causality::Input),
            var("X", 10, Causality::Output),
            var("Y", 11, Causality::Output),
            var("Z", 12, Causality::Output),
            var("psi", 13, Causality::Output),
        ])
    }

    #[test]
    fn resolves_names_to_value_references() {
        let r = spec().resolve(&full_md()).expect("resolve");
        assert_eq!(r.inputs.steer, 1);
        assert_eq!(r.inputs.throttle, 2);
        assert_eq!(r.inputs.brake, 3);
        assert_eq!(r.ground.height, Some(4));
        assert_eq!(r.ground.normal_z, None);
        assert_eq!(
            r.outputs,
            ResolvedOutputs {
                x: 10,
                y: 11,
                z: 12,
                yaw: 13
            }
        );
    }

    #[test]
    fn unknown_variable_is_rejected() {
        let md = ModelDescription::new(
            full_md()
                .variables()
                .iter()
                .filter(|v| v.name != "delta")
                .cloned()
                .collect(),
        );
        let err = spec().resolve(&md).unwrap_err();
        assert_eq!(
            err,
            BindError::UnknownVariable {
                role: "steer".into(),
                name: "delta".into(),
            }
        );
    }

    #[test]
    fn an_output_role_pointed_at_an_input_is_rejected() {
        let md = ModelDescription::new(vec![
            var("delta", 1, Causality::Input),
            var("ax", 2, Causality::Input),
            var("brk", 3, Causality::Input),
            var("z_road", 4, Causality::Input),
            var("X", 10, Causality::Input), // wrong: output role, input variable
            var("Y", 11, Causality::Output),
            var("Z", 12, Causality::Output),
            var("psi", 13, Causality::Output),
        ]);
        let err = spec().resolve(&md).unwrap_err();
        assert_eq!(
            err,
            BindError::WrongCausality {
                role: "x".into(),
                name: "X".into(),
                expected: Causality::Output,
                actual: Causality::Input,
            }
        );
    }

    #[test]
    fn a_ground_role_must_be_input_causality() {
        let md = ModelDescription::new(vec![
            var("delta", 1, Causality::Input),
            var("ax", 2, Causality::Input),
            var("brk", 3, Causality::Input),
            var("z_road", 4, Causality::Output), // wrong: ground is an input
            var("X", 10, Causality::Output),
            var("Y", 11, Causality::Output),
            var("Z", 12, Causality::Output),
            var("psi", 13, Causality::Output),
        ]);
        let err = spec().resolve(&md).unwrap_err();
        assert_eq!(
            err,
            BindError::WrongCausality {
                role: "ground.height".into(),
                name: "z_road".into(),
                expected: Causality::Input,
                actual: Causality::Output,
            }
        );
    }
}
