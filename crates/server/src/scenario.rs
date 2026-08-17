use std::collections::HashSet;

use anyhow::{bail, Context, Result};
use bevy::prelude::*;
use protocol::scenario::{AgentSlot, ScenarioConfig, SensorSource, GROUND_TRUTH_SENSOR};

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
    }
    Ok(())
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
                })
                .collect(),
            seed: 0,
            map: None,
        }
    }

    #[test]
    fn unique_roster_is_accepted() {
        assert!(validate(&config(&["car-1", "car-2"])).is_ok());
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
