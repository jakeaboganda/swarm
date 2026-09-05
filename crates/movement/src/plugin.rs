use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use crate::carlike::CarLike;
use crate::fmu_vehicle::{drive_fmu_vehicles, FmuStore};
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
/// line here, nothing else -- each is a separate monomorphized system that
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
                drive_fmu_vehicles,
            )
                .in_set(MovementSet::ApplyForce)
                // Before `SyncBackend`, not just `StepSimulation`: `SyncBackend`
                // is where Bevy state is pushed INTO Rapier (a kinematic body's
                // target pose, a dynamic body's forces). `drive_fmu_vehicles`
                // writes the FMU pose straight onto a `KinematicPositionBased`
                // Transform -- if that write lands after `SyncBackend`, Rapier
                // steps to the pre-write target and `Writeback` clobbers the FMU
                // pose, freezing the car at spawn. Running before `SyncBackend`
                // also lets the force-based models set `ExternalForce` a tick
                // earlier (correct, not just harmless).
                .before(PhysicsSet::SyncBackend),
        )
        .add_systems(Update, face_velocity_direction)
        // The FMU handle store is `NonSend` (a loaded FMU is `!Send`); ensure it
        // exists so `drive_fmu_vehicles` can take it even before any FMU spawns.
        .init_non_send::<FmuStore>();
    }
}
