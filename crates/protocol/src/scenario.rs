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
    /// Full 3D raycast vehicle: four ray-cast wheels with spring-damper
    /// suspension and tire grip, roll and pitch are real physics. Drives on the
    /// road's terrain (grade, banking). For the automotive world.
    RaycastVehicle,
}

/// The reserved device name every agent can read ground truth from without
/// declaring a sensor (the zero-friction safety-reflex source). A scenario
/// `SensorDef` may not reuse this name.
pub const GROUND_TRUTH_SENSOR: &str = "ground_truth";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AgentSlot {
    pub name: String,
    pub embodiment: Embodiment,
    /// The perceiving devices the world equips this agent with, referenced by
    /// name from its reflex rules. Omitted = none; the reserved
    /// `GROUND_TRUTH_SENSOR` device is always available regardless.
    #[serde(default)]
    pub sensors: Vec<SensorDef>,
    /// Optional viewer color as linear RGB in `0.0..=1.0`. Omitted uses the
    /// default agent color; set it to make a slot visually distinct (e.g. an
    /// obstacle vs. the driven car).
    #[serde(default)]
    pub color: Option<[f32; 3]>,
    /// Optional size multiplier for the body (viewer shape + collider). Omitted
    /// = 1.0. Lets a slot be a big, obvious obstacle rather than a default-size
    /// puck. Does not apply to the raycast-vehicle chassis.
    #[serde(default)]
    pub scale: Option<f32>,
}

/// A named perceiving device on an agent. A device is a perception *source*;
/// reflex predicates (`SensorKind`) are read from it. Its fidelity is
/// world-set (an agent can't declare its own perfect sensors and opt out of
/// impairment).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorDef {
    /// Unique per agent; how reflex rules reference this device.
    pub name: String,
    pub source: SensorSource,
    /// Impairment, required when `source` is `Simulated`, forbidden otherwise
    /// (validated at load).
    #[serde(default)]
    pub spec: Option<SensorSpec>,
}

/// Whether a device perceives ground truth (a perfect, instant fail-safe) or
/// an impaired, delayed simulation of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorSource {
    GroundTruth,
    Simulated,
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
    /// Perception is delivered this many physics ticks late. The server
    /// quantizes the delay down to its perception-frame interval, so the
    /// effective latency is the largest multiple of that interval not
    /// exceeding this value.
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
    /// Base seed for reproducible simulated-perception noise. Omitted in JSON
    /// defaults to 0.
    #[serde(default)]
    pub seed: u64,
    /// The road map to load. `None` (omitted) is the flat arena world;
    /// `Some(name)` selects the automotive world. Only the built-in "demo" road
    /// exists today; loading a real OpenDRIVE file by path arrives at P5.
    #[serde(default)]
    pub map: Option<String>,
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
                    sensors: vec![],
                    color: Some([0.9, 0.1, 0.1]),
                    scale: Some(2.5),
                },
                AgentSlot {
                    name: "car-2".into(),
                    embodiment: Embodiment::CarLike,
                    sensors: vec![
                        SensorDef {
                            name: "radar".into(),
                            source: SensorSource::Simulated,
                            spec: Some(SensorSpec {
                                range: 20.0,
                                fov_half_angle: 1.2,
                                position_noise: 0.3,
                                velocity_noise: 0.1,
                                latency_ticks: 4,
                            }),
                        },
                        SensorDef {
                            name: "bumper".into(),
                            source: SensorSource::GroundTruth,
                            spec: None,
                        },
                    ],
                    color: None,
                    scale: None,
                },
                AgentSlot {
                    name: "car-3".into(),
                    embodiment: Embodiment::FullVehicle,
                    sensors: vec![],
                    color: None,
                    scale: None,
                },
            ],
            seed: 42,
            map: Some("demo".into()),
        };
        let json = serde_json::to_string_pretty(&config).expect("serialize");
        let back: ScenarioConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, back);
    }

    #[test]
    fn omitted_sensors_and_seed_default() {
        // A scenario with no `sensors` and no `seed` parses to an empty sensor
        // list and seed 0.
        let json = r#"{
            "arena": { "width": 50.0, "depth": 50.0 },
            "roster": [{ "name": "car-1", "embodiment": "holonomic" }]
        }"#;
        let config: ScenarioConfig = serde_json::from_str(json).expect("deserialize");
        assert_eq!(config.seed, 0);
        assert!(config.roster[0].sensors.is_empty());
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
