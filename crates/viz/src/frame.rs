use serde::{Deserialize, Serialize};

use crate::math::{Transform, Vec3};
use crate::scene::{EntityId, NodePath};

/// The scene layer: what moved this tick.
///
/// Sparse by node: an entity contributes only the nodes whose transform
/// changed. A wall contributes nothing at all (static geometry is never in a
/// frame); a puck sends its root; a car sends its root and four wheels.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub tick: u64,
    pub entities: Vec<EntityFrame>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityFrame {
    pub id: EntityId,
    pub nodes: Vec<NodeUpdate>,
}

/// Where one node of an entity is now. The root's transform is the entity's
/// world pose; every other node's is relative to its parent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NodeUpdate {
    pub path: NodePath,
    pub transform: Transform,
}

/// The node names a vehicle's four wheels get, in the rig order
/// [`EntityDebug::wheels`] arrives in: front-left, front-right, rear-left,
/// rear-right.
///
/// Shared so the sim (which builds the nodes) and a viewer (which keys
/// per-wheel diagnostics onto them) cannot disagree. A stopgap: when the debug
/// layer grows per-node named values, the diagnostics address nodes directly
/// and this goes away with them.
pub const WHEEL_NODES: [&str; 4] = ["wheel.fl", "wheel.fr", "wheel.rl", "wheel.rr"];

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
    /// non-agents.
    pub detections: Vec<Blip>,
    /// Per-wheel diagnostics, in rig order. Empty for entities without wheels.
    /// Diagnostics rather than geometry, so they ride the debug layer: a
    /// scene-only consumer (a recorder, a USD export) has no use for them.
    /// Keyed onto nodes by [`WHEEL_NODES`].
    pub wheels: Vec<WheelDebug>,
}

/// What one wheel is doing, for the debug overlay: enough to tint a locked
/// wheel differently from a spinning one.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelDebug {
    /// `0` is free rolling, `-1` fully locked, positive is wheelspin.
    pub slip_ratio: f32,
    pub slip_angle: f32,
    pub contact: bool,
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
