use crate::instance::ValueReference;

/// FMI variable causality -- who drives a variable. Binding validation only
/// cares about `Input`/`Output`; the rest are represented so a parsed model
/// description round-trips faithfully. Mirrors FMI 3.0's set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Causality {
    Parameter,
    CalculatedParameter,
    Input,
    Output,
    Local,
    Independent,
    Dependent,
    StructuralParameter,
}

/// FMI variable base type. We drive an FMU vehicle through Float64 inputs and
/// read Float64 outputs, so a binding must resolve to `Float64` variables --
/// this is what the resolver checks. The rest are represented so a parsed model
/// description is faithful and a mis-typed binding names a concrete type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaseType {
    Float32,
    Float64,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Boolean,
    String,
    Binary,
    Clock,
}

/// One scalar variable from an FMU's model description -- enough to resolve a
/// binding. (Start/unit are not needed by the pure logic.)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Variable {
    pub name: String,
    pub value_reference: ValueReference,
    pub causality: Causality,
    pub base_type: BaseType,
}

/// The parsed interface of an FMU. Slice 2 builds this from
/// `modelDescription.xml`; here it is constructed directly for tests.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ModelDescription {
    variables: Vec<Variable>,
}

impl ModelDescription {
    pub fn new(variables: Vec<Variable>) -> Self {
        Self { variables }
    }

    /// The variable with this name, if any. Linear scan -- model descriptions
    /// are small and this runs once at load, not per tick.
    pub fn variable(&self, name: &str) -> Option<&Variable> {
        self.variables.iter().find(|v| v.name == name)
    }

    pub fn variables(&self) -> &[Variable] {
        &self.variables
    }
}
