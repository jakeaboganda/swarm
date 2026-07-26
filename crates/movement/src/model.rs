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

/// A pluggable per-entity embodiment. Each implementation decides how to
/// turn "desired velocity" into a physical force — this is the seam where
/// `Holonomic`, and later `CarLike`/`FullVehicle`, differ.
pub trait MovementModel: Component {
    fn compute_force(&self, desired: DesiredVelocity, current_velocity: Vec3) -> Vec3;
}
