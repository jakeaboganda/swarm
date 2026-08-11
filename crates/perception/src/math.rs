use serde::{Deserialize, Serialize};

/// Wire-format 3D vector. Independent of any engine vector type so the
/// pathway carries no `glam`/`bevy` dependency; the server converts at the
/// edge.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}
