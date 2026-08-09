use serde::{Deserialize, Serialize};

/// Movement model an agent's entity is embodied with. `FullVehicle` (full
/// wheeled physics) is a future addition — this enum grows when it lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Embodiment {
    /// Free horizontal movement in any direction (puck/drone-like).
    Holonomic,
    /// Non-holonomic: forward-only thrust, bounded turn rate, lateral grip.
    CarLike,
    /// Single-track ("bicycle") dynamics: physical yaw plus tire-slip lateral
    /// forces, so understeer/oversteer emerge. Higher fidelity than `CarLike`.
    FullVehicle,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AgentSlot {
    pub name: String,
    pub embodiment: Embodiment,
}

/// A flat rectangular arena bounded by four walls, centered on the origin.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ArenaConfig {
    pub width: f32,
    pub depth: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioConfig {
    pub arena: ArenaConfig,
    pub roster: Vec<AgentSlot>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenario_config_round_trips() {
        let config = ScenarioConfig {
            arena: ArenaConfig {
                width: 50.0,
                depth: 50.0,
            },
            roster: vec![
                AgentSlot {
                    name: "car-1".into(),
                    embodiment: Embodiment::Holonomic,
                },
                AgentSlot {
                    name: "car-2".into(),
                    embodiment: Embodiment::CarLike,
                },
                AgentSlot {
                    name: "car-3".into(),
                    embodiment: Embodiment::FullVehicle,
                },
            ],
        };
        let json = serde_json::to_string_pretty(&config).expect("serialize");
        let back: ScenarioConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, back);
    }
}
