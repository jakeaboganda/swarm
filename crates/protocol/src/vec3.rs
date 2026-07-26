use serde::{Deserialize, Serialize};

/// Wire-format 3D vector. Deliberately independent of any game-engine
/// vector type (e.g. `glam::Vec3`) so `protocol` has no dependency on
/// `bevy`. Consumers convert to/from their own vector type at the edge.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}
