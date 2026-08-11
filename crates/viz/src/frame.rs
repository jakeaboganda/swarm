use serde::{Deserialize, Serialize};

use crate::math::{Transform, Vec3};
use crate::scene::EntityId;

/// The scene layer: the physical state of every dynamic entity this tick.
/// Full snapshot in v1 (every dynamic entity every frame); a delta variant
/// can slot in behind this type later.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub tick: u64,
    pub entities: Vec<EntityFrame>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityFrame {
    pub id: EntityId,
    pub transform: Transform,
}

/// The debug layer: non-physical annotations a viewer may render or ignore
/// — intent (the remaining plan) and diagnostics (reflex state). Sent as a
/// distinct message so scene-only consumers (recorders, USD export) can
/// skip it. Trails are *not* here — a viewer derives those from the frame
/// stream.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DebugFrame {
    pub tick: u64,
    pub entities: Vec<EntityDebug>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityDebug {
    pub id: EntityId,
    /// Remaining plan waypoints (positions the entity intends to visit).
    pub plan: Vec<Vec3>,
    /// Whether a reflex is currently overriding this entity's plan.
    pub reflex_active: bool,
    /// What this agent currently perceives through its simulated sensors —
    /// the delayed, noised set actually delivered on the sensor pathway.
    /// Debug overlay only (a viewer draws each as a "ghost"); empty for
    /// non-agents. `#[serde(default)]` keeps pre-v3 encodings decodable.
    #[serde(default)]
    pub detections: Vec<Blip>,
}

/// What a perceived blip is, so a viewer can color a peer differently from a
/// wall. Mirrors the perception pathway's kinds but kept viz-local so viz
/// stays independent of the sensor crates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DetectionKind {
    Agent,
    Static,
}

/// One entity as an agent perceives it right now: its identity, the noised
/// position the agent received, and what kind of thing it is. Debug-only.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Blip {
    pub id: EntityId,
    pub position: Vec3,
    pub kind: DetectionKind,
}
