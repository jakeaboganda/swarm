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
}
