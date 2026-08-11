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

/// How well an agent perceives the world through its simulated sensors.
/// Every field degrades ground truth; the `Default` is deliberately near-
/// perfect (huge range, full field of view, no noise or latency) so that
/// omitting it leaves an agent perceiving as it did before simulated sensors
/// existed.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SensorSpec {
    /// Max detection radius (world units). Nothing past this is perceived.
    pub range: f32,
    /// Half-angle of the forward detection cone (radians), relative to the
    /// agent's heading. `>= PI` means no field-of-view limit (full 360°).
    pub fov_half_angle: f32,
    /// Gaussian sigma applied to each detected position component.
    pub position_noise: f32,
    /// Gaussian sigma applied to each detected velocity component.
    pub velocity_noise: f32,
    /// Perception is delivered this many ticks late.
    pub latency_ticks: u32,
}

impl Default for SensorSpec {
    fn default() -> Self {
        Self {
            // Finite (not INFINITY: serde_json renders that as null) but far
            // larger than any arena.
            range: 1.0e6,
            fov_half_angle: std::f32::consts::PI,
            position_noise: 0.0,
            velocity_noise: 0.0,
            latency_ticks: 0,
        }
    }
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

    #[test]
    fn sensor_spec_round_trips() {
        let spec = SensorSpec {
            range: 20.0,
            fov_half_angle: 1.2,
            position_noise: 0.3,
            velocity_noise: 0.1,
            latency_ticks: 4,
        };
        let json = serde_json::to_string(&spec).expect("serialize");
        let back: SensorSpec = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(spec, back);
    }

    #[test]
    fn default_sensor_spec_survives_json() {
        // The near-perfect default must round-trip (guards against a range of
        // INFINITY, which serde_json would turn into null).
        let json = serde_json::to_string(&SensorSpec::default()).expect("serialize");
        let back: SensorSpec = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(SensorSpec::default(), back);
    }
}
