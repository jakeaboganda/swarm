use serde::{Deserialize, Serialize};

/// Movement model an agent's entity is embodied with. Stays `Copy`: it is a
/// bare discriminant, so any per-embodiment configuration rides its own slot
/// field (see `AgentSlot::fmu` for `FmuVehicle`), never the enum.
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
    /// Vehicle dynamics computed by an external FMI 3.0 co-simulation FMU. The
    /// FMU integrates its own pose; the sim imposes it on a kinematic body. The
    /// FMU path + variable binding ride `AgentSlot::fmu`, required for this
    /// embodiment (validated at load, server-side).
    FmuVehicle,
}

/// The reserved device name every agent can read ground truth from without
/// declaring a sensor (the zero-friction safety-reflex source). A scenario
/// `SensorDef` may not reuse this name.
pub const GROUND_TRUTH_SENSOR: &str = "ground_truth";

/// The driver-actuator inputs every FMU vehicle exposes, named by the FMU's
/// own variable names. Mirrors `dynamics_fmi::binding::InputBinding` exactly,
/// so the `protocol` -> `dynamics-fmi` map is 1:1.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmuInputs {
    pub steer: String,
    pub throttle: String,
    pub brake: String,
}

/// The ground inputs -- the one-way road query the FMU pushes against. v1 is
/// single-point under the chassis. `height`/`normal_z` are required (a
/// vehicle-dynamics FMU has no traction without a surface); `friction` is the
/// one optional role, since not every FMU exposes it. Mirrors
/// `dynamics_fmi::binding::GroundBinding`'s field names.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmuGround {
    pub height: String,
    pub normal_z: String,
    #[serde(default)]
    pub friction: Option<String>,
}

/// The pose outputs stamped onto the kinematic body each tick. Mirrors
/// `dynamics_fmi::binding::OutputBinding`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmuOutputs {
    pub x: String,
    pub y: String,
    pub z: String,
    pub yaw: String,
}

/// A scenario's role -> variable-name map for an `FmuVehicle` slot, plus the
/// `.fmu` file path. Mirrors `dynamics_fmi::binding::BindingSpec` exactly (all
/// variable names `String`), so slice 4's protocol -> dynamics-fmi map is 1:1.
/// Required present iff the slot's `embodiment` is `FmuVehicle` (validated at
/// load, server-side -- not here).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmuConfig {
    /// Path to the `.fmu` archive.
    pub path: String,
    pub inputs: FmuInputs,
    pub ground: FmuGround,
    pub outputs: FmuOutputs,
}

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
    /// The `.fmu` path + variable binding, required iff `embodiment` is
    /// `FmuVehicle` (validated at load, server-side -- not here). Omitted for
    /// every other embodiment.
    #[serde(default)]
    pub fmu: Option<FmuConfig>,
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
    /// Half-angle of the *vertical* detection cone (radians): how far up/down
    /// from the sensor's horizontal plane a target may be and still be seen.
    /// Combined with `fov_half_angle` this makes a frustum (a target is culled
    /// if it exceeds either angle). `>= PI` (the default) means no vertical
    /// limit -- the vertically-unbounded wedge, i.e. today's behavior.
    #[serde(default = "full_angle")]
    pub vertical_fov_half_angle: f32,
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

/// The "no limit" angle used as the default for both FOV half-angles: at or
/// past `PI`, the corresponding cull is skipped entirely.
fn full_angle() -> f32 {
    std::f32::consts::PI
}

impl Default for SensorSpec {
    fn default() -> Self {
        Self {
            // Finite (not INFINITY: serde_json renders that as null) but far
            // larger than any arena.
            range: 1.0e6,
            fov_half_angle: full_angle(),
            vertical_fov_half_angle: full_angle(),
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

/// How fast sim-time advances relative to wall-clock. A property of the whole
/// run (all agents share one physics world), not per-agent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Pace {
    /// One sim-second per real second -- required for live viewing.
    #[default]
    Realtime,
    /// "As fast as possible": ticks run at CPU speed, wall-clock decoupled.
    /// Sim-time (and thus `duration`) is unchanged -- the deadline just
    /// arrives sooner in wall-clock. For headless batch runs.
    Afap,
}

/// The scenario's ownership of time: how long it runs and how fast time flows.
/// Omitted entirely (or field-by-field) falls back to the defaults: realtime
/// pace, unbounded duration (ends only when an agent disconnects).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct TimeConfig {
    /// Run length in *sim*-seconds. `None` = unbounded. Pace-independent: afap
    /// just reaches it sooner in wall-clock. The server converts it to a tick
    /// deadline at the fixed physics rate.
    #[serde(default)]
    pub duration: Option<f64>,
    #[serde(default)]
    pub pace: Pace,
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
    /// The scenario's time policy (duration + pace). Omitted = realtime,
    /// unbounded.
    #[serde(default)]
    pub time: TimeConfig,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_fmu_config() -> FmuConfig {
        FmuConfig {
            path: "fmus/VanDerPol.fmu".into(),
            inputs: FmuInputs {
                steer: "delta".into(),
                throttle: "ax".into(),
                brake: "brk".into(),
            },
            ground: FmuGround {
                height: "z_road".into(),
                normal_z: "n_z".into(),
                friction: Some("mu".into()),
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
                    fmu: None,
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
                                vertical_fov_half_angle: 0.3,
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
                    fmu: None,
                },
                AgentSlot {
                    name: "car-3".into(),
                    embodiment: Embodiment::FullVehicle,
                    sensors: vec![],
                    color: None,
                    scale: None,
                    fmu: None,
                },
                AgentSlot {
                    name: "car-4".into(),
                    embodiment: Embodiment::FmuVehicle,
                    sensors: vec![],
                    color: None,
                    scale: None,
                    fmu: Some(sample_fmu_config()),
                },
            ],
            seed: 42,
            map: Some("demo".into()),
            time: TimeConfig {
                duration: Some(30.0),
                pace: Pace::Afap,
            },
        };
        let json = serde_json::to_string_pretty(&config).expect("serialize");
        let back: ScenarioConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, back);
    }

    #[test]
    fn omitted_fmu_defaults_to_none() {
        // A slot with no `fmu` block (the common case: every non-FmuVehicle
        // embodiment) parses to `None`.
        let json = r#"{
            "arena": { "width": 50.0, "depth": 50.0 },
            "roster": [{ "name": "car-1", "embodiment": "holonomic" }]
        }"#;
        let config: ScenarioConfig = serde_json::from_str(json).expect("deserialize");
        assert_eq!(config.roster[0].fmu, None);
    }

    #[test]
    fn fmu_config_round_trips() {
        let config = sample_fmu_config();
        let json = serde_json::to_string(&config).expect("serialize");
        let back: FmuConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, back);
    }

    #[test]
    fn fmu_config_ground_friction_omits_cleanly() {
        // `ground.friction` is the one optional role; omitting it must still
        // parse (the required `height`/`normal_z` stay mandatory).
        let mut config = sample_fmu_config();
        config.ground.friction = None;
        let json = serde_json::to_string(&config).expect("serialize");
        let back: FmuConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, back);
        assert_eq!(back.ground.friction, None);
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
    fn omitted_time_defaults_to_realtime_unbounded() {
        // No `time` block: realtime pace, no duration limit -- today's behavior.
        let json = r#"{
            "arena": { "width": 50.0, "depth": 50.0 },
            "roster": [{ "name": "car-1", "embodiment": "holonomic" }]
        }"#;
        let config: ScenarioConfig = serde_json::from_str(json).expect("deserialize");
        assert_eq!(config.time, TimeConfig::default());
        assert_eq!(config.time.pace, Pace::Realtime);
        assert_eq!(config.time.duration, None);
    }

    #[test]
    fn partial_time_block_fills_defaults() {
        // Only `duration` given: pace defaults to realtime.
        let json = r#"{
            "arena": { "width": 50.0, "depth": 50.0 },
            "roster": [{ "name": "car-1", "embodiment": "holonomic" }],
            "time": { "duration": 12.5 }
        }"#;
        let config: ScenarioConfig = serde_json::from_str(json).expect("deserialize");
        assert_eq!(config.time.duration, Some(12.5));
        assert_eq!(config.time.pace, Pace::Realtime);
    }

    #[test]
    fn sensor_spec_round_trips() {
        let spec = SensorSpec {
            range: 20.0,
            fov_half_angle: 1.2,
            vertical_fov_half_angle: 0.3,
            position_noise: 0.3,
            velocity_noise: 0.1,
            latency_ticks: 4,
        };
        let json = serde_json::to_string(&spec).expect("serialize");
        let back: SensorSpec = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(spec, back);
    }

    #[test]
    fn omitted_vertical_fov_defaults_to_unbounded() {
        // A spec written before the vertical FOV existed (no such field) must
        // still parse -- to an unbounded vertical cone, i.e. the wedge.
        let json = r#"{
            "range": 20.0,
            "fov_half_angle": 1.2,
            "position_noise": 0.0,
            "velocity_noise": 0.0,
            "latency_ticks": 0
        }"#;
        let spec: SensorSpec = serde_json::from_str(json).expect("deserialize");
        assert_eq!(spec.vertical_fov_half_angle, std::f32::consts::PI);
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
