//! Wire math types and the coordinate convention.
//!
//! **Coordinates: right-handed, Y-up, meters** (matching Bevy and glTF). A
//! USD adapter (Z-up) must convert. Positions are in meters; `Quat` follows
//! the usual `[x, y, z, w]` layout with `w` the scalar part.

use serde::{Deserialize, Serialize};

/// Wire-format 3D vector, independent of any engine's vector type.
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

/// Wire-format quaternion (orientation). `[0, 0, 0, 1]` is identity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }
}

/// A rigid placement: where an entity is and how it's oriented. The
/// canonical per-entity render state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Transform {
    pub const IDENTITY: Transform = Transform {
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };
}
