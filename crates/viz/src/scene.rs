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

/// Geometry of one node. Primitives give half-extents / radii so a viewer can
/// build its own mesh at whatever resolution it likes; `Mesh` carries baked
/// triangle geometry (e.g. a road surface) outright.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "geometry", rename_all = "snake_case")]
pub enum Geometry {
    Capsule {
        radius: f32,
        half_length: f32,
    },
    Cuboid {
        half_extents: Vec3,
    },
    /// Axis along +Y, `height` the full length (Bevy/glTF convention). A
    /// sender that wants it lying down says so in the node's rotation.
    Cylinder {
        radius: f32,
        height: f32,
    },
    Sphere {
        radius: f32,
    },
    /// An explicit triangle mesh: per-vertex positions and up-normals, plus
    /// triangle indices (three per triangle). Y-up, meters.
    Mesh {
        positions: Vec<Vec3>,
        normals: Vec<Vec3>,
        indices: Vec<u32>,
    },
    /// A *reference* to a model file, never its bytes — resolving it is the
    /// viewer's business, and nothing resolves one yet. A viewer that cannot
    /// load `uri` draws a `scale`-sized box instead, so it shows something.
    Asset {
        uri: String,
        scale: Vec3,
    },
}

/// Where a node sits in an entity's tree: node names from the root joined by
/// `/`, the empty path being the root itself.
///
/// `/` rather than `.`, because node names themselves contain dots
/// (`wheel.fl`) and a separator that appears inside names cannot be split on.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodePath(pub String);

impl NodePath {
    const SEPARATOR: char = '/';

    /// The entity's root node — the one whose transform is its world pose.
    pub fn root() -> Self {
        Self(String::new())
    }

    pub fn is_root(&self) -> bool {
        self.0.is_empty()
    }

    /// This path extended by a child's name.
    pub fn child(&self, name: &str) -> Self {
        if self.0.is_empty() {
            Self(name.to_string())
        } else {
            Self(format!("{}{}{}", self.0, Self::SEPARATOR, name))
        }
    }

    fn names(&self) -> impl Iterator<Item = &str> {
        self.0.split(Self::SEPARATOR)
    }
}

impl From<&str> for NodePath {
    fn from(path: &str) -> Self {
        Self(path.to_string())
    }
}

/// One node of an entity: a named, placed thing that may carry geometry and
/// may have children.
///
/// A car is a body node with four wheel children. A node without geometry is a
/// pure pivot. Same shape as Bevy/glTF/USD, so an authored asset drops in
/// later without a wire change.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityNode {
    /// Empty for the root; otherwise unique among its siblings, since it is
    /// how the node is addressed (see [`NodePath`]).
    pub name: String,
    /// Relative to the parent — or, for the root, the entity's world pose.
    pub transform: Transform,
    pub geometry: Option<Geometry>,
    pub children: Vec<EntityNode>,
}

impl EntityNode {
    /// A childless node with geometry, placed relative to its parent.
    pub fn new(name: impl Into<String>, transform: Transform, geometry: Geometry) -> Self {
        Self {
            name: name.into(),
            transform,
            geometry: Some(geometry),
            children: Vec::new(),
        }
    }

    /// A root node whose whole body is one shape. Its transform is filled in
    /// by the sender from the entity's live pose.
    pub fn body(geometry: Geometry) -> Self {
        Self::new("", Transform::IDENTITY, geometry)
    }

    pub fn with_children(mut self, children: Vec<EntityNode>) -> Self {
        self.children = children;
        self
    }

    /// The node at `path`, resolved from this node as the root.
    pub fn get(&self, path: &NodePath) -> Option<&EntityNode> {
        if path.is_root() {
            return Some(self);
        }
        let mut node = self;
        for name in path.names() {
            node = node.children.iter().find(|c| c.name == name)?;
        }
        Some(node)
    }

    /// Places the node at `path`. Returns `false` if there is no such node.
    pub fn set_transform(&mut self, path: &NodePath, transform: Transform) -> bool {
        if path.is_root() {
            self.transform = transform;
            return true;
        }
        let mut node = self;
        for name in path.names() {
            match node.children.iter_mut().find(|c| c.name == name) {
                Some(child) => node = child,
                None => return false,
            }
        }
        node.transform = transform;
        true
    }
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

/// Everything static about an entity: its identity, node tree, appearance,
/// and initial placement. Sent once (in scene-init or an `EntitySpawned`
/// event); frames afterwards carry only the nodes that moved.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityDescriptor {
    /// Stable key used everywhere (events, frames).
    pub id: EntityId,
    /// Human-facing display label, which a viewer may draw. Distinct from
    /// `id`; often equal for agents but free to differ (and differs for
    /// generated static geometry).
    pub name: String,
    pub kind: EntityKind,
    pub color: Color,
    /// The entity's tree. The root's transform is the entity's world pose, so
    /// the pose has exactly one home here and in frames alike.
    pub root: EntityNode,
    /// The agent's sensing region, for the debug envelope overlay. `None` for
    /// static geometry and agents without simulated sensors.
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
///
/// `EntitySpawned` is much the largest variant, and deliberately not boxed:
/// these are serialized and sent, not held in bulk, so one allocation per
/// spawn event would buy nothing and cost every construction and match site an
/// indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum SceneEvent {
    EntitySpawned(EntityDescriptor),
    EntityDespawned { id: EntityId },
    ScenarioState { state: ScenarioState },
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(y: f32) -> Transform {
        Transform {
            position: Vec3::new(0.0, y, 0.0),
            rotation: crate::math::Quat::IDENTITY,
        }
    }

    fn car() -> EntityNode {
        EntityNode::body(Geometry::Cuboid {
            half_extents: Vec3::new(0.8, 0.4, 1.4),
        })
        .with_children(vec![EntityNode::new(
            "wheel.fl",
            at(-0.55),
            Geometry::Cylinder {
                radius: 0.32,
                height: 0.22,
            },
        )
        .with_children(vec![EntityNode {
            name: "hub".into(),
            transform: at(1.0),
            geometry: None,
            children: Vec::new(),
        }])])
    }

    #[test]
    fn a_path_addresses_a_node_at_any_depth() {
        let car = car();
        assert_eq!(
            car.get(&NodePath::root()).map(|n| n.name.as_str()),
            Some("")
        );
        assert_eq!(
            car.get(&NodePath::from("wheel.fl"))
                .map(|n| n.name.as_str()),
            Some("wheel.fl"),
            "a name containing a dot is one node, not two"
        );
        assert_eq!(
            car.get(&NodePath::root().child("wheel.fl").child("hub")),
            car.get(&NodePath::from("wheel.fl/hub")),
        );
        assert_eq!(car.get(&NodePath::from("wheel.fr")), None);
    }

    #[test]
    fn setting_a_transform_moves_that_node_and_no_other() {
        let mut car = car();
        let moved = at(3.0);
        assert!(car.set_transform(&NodePath::from("wheel.fl"), moved));
        assert_eq!(
            car.get(&NodePath::from("wheel.fl")).unwrap().transform,
            moved
        );
        assert_eq!(car.transform, Transform::IDENTITY, "the root moved too");
        assert_eq!(
            car.get(&NodePath::from("wheel.fl/hub")).unwrap().transform,
            at(1.0),
            "the child moved too"
        );
        assert!(!car.set_transform(&NodePath::from("wheel.rr"), moved));
    }

    #[test]
    fn the_root_transform_is_the_entitys_world_pose() {
        let mut car = car();
        let pose = at(7.0);
        assert!(car.set_transform(&NodePath::root(), pose));
        assert_eq!(car.transform, pose);
    }
}
