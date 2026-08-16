use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use crate::carlike::CarLike;
use crate::fullvehicle::FullVehicle;
use crate::holonomic::Holonomic;
use crate::raycast_vehicle::drive_raycast_vehicles;
use crate::systems::{apply_movement_force, face_velocity_direction};

/// Lets other crates (`server`'s reflex-vs-plan arbitration) order their
/// own systems relative to movement's force application without depending
/// on its internal system functions directly.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum MovementSet {
    ApplyForce,
}

/// Registers movement dispatch for every embodiment. Adding a new one
/// (e.g. `FullVehicle`) means adding one more `apply_movement_force::<M>`
/// line here, nothing else — each is a separate monomorphized system that
/// only touches entities carrying that component.
pub struct MovementPlugin;

impl Plugin for MovementPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            (
                apply_movement_force::<Holonomic>,
                apply_movement_force::<CarLike>,
                apply_movement_force::<FullVehicle>,
                drive_raycast_vehicles,
            )
                .in_set(MovementSet::ApplyForce)
                .before(PhysicsSet::StepSimulation),
        )
        .add_systems(Update, face_velocity_direction);
    }
}
