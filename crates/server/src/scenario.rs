use std::collections::HashSet;

use anyhow::{bail, Context, Result};
use bevy::prelude::*;
use protocol::scenario::{ScenarioConfig, SensorSpec};

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
        validate_sensors(&slot.name, &slot.sensors)?;
    }
    Ok(())
}

/// Reject sensor specs that would silently misbehave rather than fail loudly:
/// a negative or non-finite range/FOV/noise makes an agent perceive nothing
/// (or, for a NaN range, disables the range limit entirely) with no runtime
/// error. Catch it at load, like the duplicate-name check above.
fn validate_sensors(name: &str, spec: &SensorSpec) -> Result<()> {
    for (field, value) in [
        ("range", spec.range),
        ("fov_half_angle", spec.fov_half_angle),
        ("position_noise", spec.position_noise),
        ("velocity_noise", spec.velocity_noise),
    ] {
        if !value.is_finite() || value < 0.0 {
            bail!("agent '{name}' sensor {field} must be finite and non-negative, got {value}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::scenario::{AgentSlot, ArenaConfig, Embodiment};

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
                })
                .collect(),
            seed: 0,
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
    fn default_sensors_are_valid() {
        // The near-perfect default (range 1e6, etc.) must pass validation.
        assert!(validate(&config(&["car-1"])).is_ok());
    }

    #[test]
    fn negative_sensor_range_is_rejected() {
        let mut c = config(&["car-1"]);
        c.roster[0].sensors.range = -1.0;
        assert!(validate(&c).is_err());
    }

    #[test]
    fn non_finite_sensor_field_is_rejected() {
        let mut c = config(&["car-1"]);
        c.roster[0].sensors.position_noise = f32::NAN;
        assert!(validate(&c).is_err());
    }
}
