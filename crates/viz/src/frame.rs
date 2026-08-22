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
    /// Each wheel's pose relative to the body, in rig order. Empty for
    /// entities without wheels. `#[serde(default)]` keeps earlier encodings
    /// decodable.
    #[serde(default)]
    pub wheels: Vec<WheelPose>,
}

/// One wheel this frame, relative to the chassis.
///
/// Only what a viewer cannot work out for itself. Spin is the notable one: a
/// locked wheel's spin is *not* its road speed, so a viewer that integrated
/// wheel angle from vehicle speed would draw a smoothly turning wheel on a car
/// that is sliding -- erasing the one thing worth seeing. Suspension travel is
/// not visible from the body transform at all.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelPose {
    /// Steering angle about the chassis up axis (radians).
    pub steer: f32,
    /// Accumulated spin about the axle, wrapped to `[0, 2pi)`. Sent as a
    /// scalar rather than folded into a rotation because a quaternion would
    /// alias: above about 35 m/s a wheel turns more than half a revolution
    /// between frames, and an interpolating viewer would draw it slowing and
    /// running backwards.
    pub spin: f32,
    /// Suspension compression from full extension (m). Positive is compressed.
    pub travel: f32,
    /// Vertical load carried by this wheel (N). Zero when airborne.
    pub load: f32,
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
    /// Per-wheel diagnostics, in rig order. Empty for entities without wheels.
    /// Diagnostics rather than geometry, so they ride the debug layer: a
    /// scene-only consumer (a recorder, a USD export) has no use for them.
    #[serde(default)]
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
