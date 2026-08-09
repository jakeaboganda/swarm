use bevy::ecs::component::Mutable;
use bevy::prelude::*;

/// The shared control contract: what velocity an entity should currently be
/// trying to reach. Populated each tick by `server`'s reflex-vs-plan
/// arbitration — reflex actions and plan waypoint-seeking both just supply a
/// value here, so movement models don't need to know which one is active.
///
/// `urgent` is set when a reflex (`brake`/`stop_and_hold`) is driving this
/// value rather than ordinary plan-following — movement models use it to
/// apply a higher force ceiling than cruising uses, since "brake as fast as
/// possible" must not be limited by the same cap tuned for smooth cornering.
#[derive(Component, Default, Clone, Copy, Debug)]
pub struct DesiredVelocity {
    pub value: Vec3,
    pub urgent: bool,
}

/// Read-back physical state of the body this tick, supplied to `drive`.
/// Rapier is the integrator — models return forces *given* this state, they
/// don't integrate it themselves.
#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    /// World-frame linear velocity.
    pub velocity: Vec3,
    /// Yaw rate about +Y (rad/s). Always 0 for yaw-locked models.
    pub yaw_rate: f32,
    /// Unit heading in the xz-plane (body forward). For fake-yaw models this
    /// is just the current facing; for physical-yaw models it is the real
    /// orientation.
    pub heading: Vec3,
}

/// What a model asks physics to apply this tick: a world-frame force and a
/// yaw torque about +Y. Rotation-locked models leave `yaw_torque` at 0 (and
/// Rapier discards torque about locked axes regardless).
#[derive(Clone, Copy, Debug, Default)]
pub struct Actuation {
    pub force: Vec3,
    pub yaw_torque: f32,
}

/// Marks a body whose yaw is a real physics DOF (e.g. `FullVehicle`), so the
/// cosmetic `face_velocity_direction` system leaves its orientation alone.
#[derive(Component, Default)]
pub struct PhysicalYaw;

/// A pluggable per-entity embodiment. Each implementation decides how to turn
/// a desired velocity into physical actuation (force + yaw torque) — the seam
/// where `Holonomic`, `CarLike`, and `FullVehicle` differ.
///
/// `&mut self` and `dt` let a model carry state that evolves over time (a
/// car's steering angle); `BodyState` supplies the read-back physics state
/// (velocity, yaw rate, heading) a dynamic model needs to compute slip.
pub trait MovementModel: Component<Mutability = Mutable> {
    fn drive(&mut self, desired: DesiredVelocity, body: BodyState, dt: f32) -> Actuation;
}
