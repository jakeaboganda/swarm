use anyhow::{Context, Result};
use bevy::prelude::*;
use protocol::scenario::ScenarioConfig;

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
    serde_json::from_str(&contents).with_context(|| format!("parsing scenario file at {path}"))
}
