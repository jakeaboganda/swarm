use serde::{Deserialize, Serialize};

use crate::math::{Transform, Vec3};

/// Stable identity of a scene entity, consistent across scene-init,
/// lifecycle events, and frames. For agents this is their roster name;
/// static geometry gets a generated id (e.g. `wall-0`).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EntityId(pub String);

/// RGB color hint, each channel in `0.0..=1.0`. A viewer may honor or
/// ignore it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

/// Geometry of an entity. Primitives give half-extents / radii so a viewer can
/// build its own mesh at whatever resolution it likes; `Mesh` carries baked
/// triangle geometry (e.g. a road surface) outright.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum Shape {
    Capsule {
        radius: f32,
        half_length: f32,
    },
    Cuboid {
        half_extents: Vec3,
    },
    /// An explicit triangle mesh: per-vertex positions and up-normals, plus
    /// triangle indices (three per triangle). Y-up, meters.
    Mesh {
        positions: Vec<Vec3>,
        normals: Vec<Vec3>,
        indices: Vec<u32>,
    },
}

/// The movement model an agent is embodied with. Mirrors the sim's notion
/// but is defined here so `viz` stays independent of the agent protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Embodiment {
    Holonomic,
    CarLike,
    FullVehicle,
    RaycastVehicle,
}

/// What kind of thing an entity is. `Static` geometry never appears in
/// frames (it doesn't move); `Agent` entities do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum EntityKind {
    Static,
    Agent { embodiment: Embodiment },
}

impl EntityKind {
    /// Whether this entity's transform is streamed in frames.
    pub fn is_dynamic(&self) -> bool {
        matches!(self, EntityKind::Agent { .. })
    }
}

/// Everything static about an entity: its identity, shape, appearance, and
/// initial placement. Sent once (in scene-init or an `EntitySpawned`
/// event), never per frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityDescriptor {
    /// Stable key used everywhere (events, frames).
    pub id: EntityId,
    /// Human-facing display label, which a viewer may draw. Distinct from
    /// `id`; often equal for agents but free to differ (and differs for
    /// generated static geometry).
    pub name: String,
    pub kind: EntityKind,
    pub shape: Shape,
    pub color: Color,
    pub transform: Transform,
    /// The agent's sensing region, for the debug envelope overlay. `None` for
    /// static geometry and agents without simulated sensors.
    /// `#[serde(default)]` keeps pre-v3 encodings decodable.
    #[serde(default)]
    pub sensors: Option<SensorView>,
}

/// The few numbers a viewer needs to draw an agent's sensing region: max
/// range, horizontal FOV half-angle, and vertical FOV half-angle (radians;
/// `>= PI` means unbounded on that axis — full 360° horizontally, or a
/// vertically-unbounded wedge). Kept viz-local so viz doesn't depend on the
/// sensor crates' full `SensorSpec`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SensorView {
    pub range: f32,
    pub fov_half_angle: f32,
    pub vertical_fov_half_angle: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioState {
    WaitingForRoster,
    Running,
    Ended,
}

/// The full picture sent to a viewer on connect, so a late arrival is
/// immediately consistent before it starts following the live stream.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SceneInit {
    /// Schema version the sim speaks; a viewer can double-check it matches
    /// what it declared in `Hello`.
    pub protocol_version: u32,
    pub tick: u64,
    /// Ticks per second — how a frame's `tick` maps to real time, so a
    /// viewer can interpolate on a sim-time clock rather than jittery
    /// message-arrival times.
    pub tick_rate: f32,
    pub state: ScenarioState,
    pub arena: ArenaBounds,
    pub entities: Vec<EntityDescriptor>,
}

/// Arena extents, handy for a viewer to frame its camera even before any
/// entity exists.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ArenaBounds {
    pub width: f32,
    pub depth: f32,
}

/// A change to the scene between frames.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum SceneEvent {
    EntitySpawned(EntityDescriptor),
    EntityDespawned { id: EntityId },
    ScenarioState { state: ScenarioState },
}
