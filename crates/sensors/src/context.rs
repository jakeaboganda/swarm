use glam::Vec3;

/// A collidable thing a `TimeToCollision` reading is computed against —
/// another entity, or a static wall (represented with `velocity: Vec3::ZERO`
/// and a `radius` covering the nearest point on the wall). Gathered fresh
/// each tick by `server`'s ECS queries; this crate only ever sees the plain
/// data.
pub struct Obstacle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub radius: f32,
}

/// Ground-truth world state relevant to one entity's sensors for the
/// current tick.
pub struct SensorContext {
    pub self_position: Vec3,
    pub self_velocity: Vec3,
    /// The sensing entity's own collision radius. `TimeToCollision` adds
    /// this to each obstacle's radius so contact is predicted at the
    /// surfaces, not the centers.
    pub self_radius: f32,
    pub obstacles: Vec<Obstacle>,
}
