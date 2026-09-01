use std::collections::HashSet;

use anyhow::{bail, Context, Result};
use bevy::prelude::*;
use protocol::scenario::{
    AgentSlot, Embodiment, ScenarioConfig, SensorSource, GROUND_TRUTH_SENSOR,
};

#[derive(Resource)]
pub struct Roster(pub ScenarioConfig);

#[derive(Resource, Clone, Copy)]
pub struct ArenaBounds {
    pub half_width: f32,
    pub half_depth: f32,
}

pub fn load_scenario(path: &str) -> Result<ScenarioConfig> {
    let contents = std::fs::read_to_string(path)
        .with_context(|| format!("reading scenario file at {path}"))?;
    let config: ScenarioConfig = serde_json::from_str(&contents)
        .with_context(|| format!("parsing scenario file at {path}"))?;
    validate(&config).with_context(|| format!("invalid scenario file at {path}"))?;
    Ok(config)
}

/// Roster names are the agent identity and drive the "all joined" count, so
/// a duplicate would let two connections collide on one slot. Reject it at
/// load rather than fail confusingly at runtime.
fn validate(config: &ScenarioConfig) -> Result<()> {
    let mut seen = HashSet::new();
    for slot in &config.roster {
        if !seen.insert(slot.name.as_str()) {
            bail!("duplicate roster name: {}", slot.name);
        }
        validate_sensors(slot)?;
        validate_fmu(slot)?;
    }
    Ok(())
}

/// An `fmu` block is required for -- and only for -- an `FmuVehicle` slot. A
/// missing one leaves the FMU embodiment with nothing to load; a stray one on
/// any other embodiment is a config mistake (the binding would be silently
/// ignored). The FMU is not loaded here (that needs the fmi runtime and happens
/// at spawn); this is the cheap present-iff shape check at parse time.
fn validate_fmu(slot: &AgentSlot) -> Result<()> {
    match (slot.embodiment, slot.fmu.is_some()) {
        (Embodiment::FmuVehicle, false) => {
            bail!(
                "agent '{}' is an fmu_vehicle but has no `fmu` config",
                slot.name
            )
        }
        (embodiment, true) if embodiment != Embodiment::FmuVehicle => {
            bail!(
                "agent '{}' has an `fmu` config but embodiment is {:?}, not fmu_vehicle",
                slot.name,
                embodiment
            )
        }
        _ => Ok(()),
    }
}

/// Validate an agent's declared devices: unique, non-reserved names, and a
/// spec that's present-and-sane for `Simulated` / absent for `GroundTruth`.
/// A malformed spec (negative/NaN range, FOV, noise) would silently blind an
/// agent or disable a limit, so it's rejected at load like duplicate names.
fn validate_sensors(slot: &AgentSlot) -> Result<()> {
    let name = &slot.name;
    let mut seen = HashSet::new();
    for def in &slot.sensors {
        if def.name == GROUND_TRUTH_SENSOR {
            bail!("agent '{name}' sensor may not reuse the reserved name '{GROUND_TRUTH_SENSOR}'");
        }
        if !seen.insert(def.name.as_str()) {
            bail!("agent '{name}' has duplicate sensor name: {}", def.name);
        }
        match (def.source, &def.spec) {
            (SensorSource::GroundTruth, Some(_)) => {
                bail!(
                    "agent '{name}' ground-truth sensor '{}' must not carry a spec",
                    def.name
                );
            }
            (SensorSource::Simulated, None) => {
                bail!(
                    "agent '{name}' simulated sensor '{}' needs a spec",
                    def.name
                );
            }
            (SensorSource::Simulated, Some(spec)) => {
                // A latency the ring buffer cannot hold would be delivered
                // as the oldest frame it does hold -- a scenario asking for
                // 300 ticks would silently run at 126 and read as if the
                // request had been honoured.
                if spec.latency_ticks > crate::perception_router::MAX_LATENCY_TICKS {
                    bail!(
                        "agent '{name}' sensor '{}' latency_ticks {} exceeds the maximum of {}",
                        def.name,
                        spec.latency_ticks,
                        crate::perception_router::MAX_LATENCY_TICKS
                    );
                }
                for (field, value) in [
                    ("range", spec.range),
                    ("fov_half_angle", spec.fov_half_angle),
                    ("vertical_fov_half_angle", spec.vertical_fov_half_angle),
                    ("position_noise", spec.position_noise),
                    ("velocity_noise", spec.velocity_noise),
                ] {
                    if !value.is_finite() || value < 0.0 {
                        bail!(
                            "agent '{name}' sensor '{}' {field} must be finite and non-negative, got {value}",
                            def.name
                        );
                    }
                }
            }
            (SensorSource::GroundTruth, None) => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::scenario::{ArenaConfig, Embodiment, SensorDef, SensorSpec};

    fn simulated(name: &str, spec: SensorSpec) -> SensorDef {
        SensorDef {
            name: name.into(),
            source: SensorSource::Simulated,
            spec: Some(spec),
        }
    }

    fn config(names: &[&str]) -> ScenarioConfig {
        ScenarioConfig {
            arena: ArenaConfig {
                width: 50.0,
                depth: 50.0,
            },
            roster: names
                .iter()
                .map(|name| AgentSlot {
                    name: (*name).to_string(),
                    embodiment: Embodiment::Holonomic,
                    sensors: Default::default(),
                    color: None,
                    scale: None,
                    fmu: None,
                })
                .collect(),
            seed: 0,
            map: None,
            time: Default::default(),
        }
    }

    fn fmu_config() -> protocol::scenario::FmuConfig {
        use protocol::scenario::{FmuConfig, FmuGround, FmuInputs, FmuOutputs};
        FmuConfig {
            path: "car.fmu".into(),
            inputs: FmuInputs {
                steer: "delta".into(),
                throttle: "ax".into(),
                brake: "brk".into(),
            },
            ground: FmuGround {
                height: "z_road".into(),
                normal_z: Some("n_z".into()),
                friction: None,
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
    fn unique_roster_is_accepted() {
        assert!(validate(&config(&["car-1", "car-2"])).is_ok());
    }

    #[test]
    fn an_fmu_vehicle_with_a_config_is_accepted() {
        let mut c = config(&["car-1"]);
        c.roster[0].embodiment = Embodiment::FmuVehicle;
        c.roster[0].fmu = Some(fmu_config());
        assert!(validate(&c).is_ok());
    }

    #[test]
    fn an_fmu_vehicle_without_a_config_is_rejected() {
        let mut c = config(&["car-1"]);
        c.roster[0].embodiment = Embodiment::FmuVehicle;
        c.roster[0].fmu = None;
        assert!(validate(&c).is_err());
    }

    #[test]
    fn an_fmu_config_on_a_non_fmu_embodiment_is_rejected() {
        let mut c = config(&["car-1"]);
        // Embodiment stays Holonomic; a stray fmu block is a config mistake.
        c.roster[0].fmu = Some(fmu_config());
        assert!(validate(&c).is_err());
    }

    #[test]
    fn duplicate_roster_name_is_rejected() {
        assert!(validate(&config(&["car-1", "car-1"])).is_err());
    }

    #[test]
    fn no_sensors_is_valid() {
        assert!(validate(&config(&["car-1"])).is_ok());
    }

    #[test]
    fn a_valid_simulated_sensor_is_accepted() {
        let mut c = config(&["car-1"]);
        c.roster[0].sensors = vec![simulated("radar", SensorSpec::default())];
        assert!(validate(&c).is_ok());
    }

    #[test]
    fn negative_sensor_range_is_rejected() {
        let mut c = config(&["car-1"]);
        let spec = SensorSpec {
            range: -1.0,
            ..Default::default()
        };
        c.roster[0].sensors = vec![simulated("radar", spec)];
        assert!(validate(&c).is_err());
    }

    #[test]
    fn non_finite_sensor_field_is_rejected() {
        let mut c = config(&["car-1"]);
        let spec = SensorSpec {
            position_noise: f32::NAN,
            ..Default::default()
        };
        c.roster[0].sensors = vec![simulated("radar", spec)];
        assert!(validate(&c).is_err());
    }

    #[test]
    fn a_latency_the_buffer_cannot_hold_is_rejected_not_silently_capped() {
        use crate::perception_router::MAX_LATENCY_TICKS;

        let with_latency = |ticks: u32| {
            let mut c = config(&["car-1"]);
            let spec = SensorSpec {
                latency_ticks: ticks,
                ..Default::default()
            };
            c.roster[0].sensors = vec![simulated("radar", spec)];
            c
        };
        assert!(validate(&with_latency(0)).is_ok());
        assert!(validate(&with_latency(MAX_LATENCY_TICKS)).is_ok());
        // One past the buffer: the scenario would get a different sensor than
        // it wrote down, with nothing said about it.
        assert!(validate(&with_latency(MAX_LATENCY_TICKS + 1)).is_err());
        assert!(validate(&with_latency(300)).is_err());
    }

    #[test]
    fn simulated_sensor_without_spec_is_rejected() {
        let mut c = config(&["car-1"]);
        c.roster[0].sensors = vec![SensorDef {
            name: "radar".into(),
            source: SensorSource::Simulated,
            spec: None,
        }];
        assert!(validate(&c).is_err());
    }

    #[test]
    fn reserved_ground_truth_name_is_rejected() {
        let mut c = config(&["car-1"]);
        c.roster[0].sensors = vec![SensorDef {
            name: GROUND_TRUTH_SENSOR.into(),
            source: SensorSource::GroundTruth,
            spec: None,
        }];
        assert!(validate(&c).is_err());
    }
}
