//! Pluggable, per-entity vehicle movement. Each embodiment is an ECS component
//! selected by an agent's `embodiment`, dispatched by a generic system. Three
//! force-based models -- `Holonomic`, `CarLike`, `FullVehicle` -- implement the
//! `MovementModel::drive` seam (desired velocity -> a force + yaw torque); the
//! `RaycastVehicle` is the exception, applying its own per-wheel suspension /
//! drive / grip forces to ride the road's terrain. No networking or scenario
//! knowledge.

mod carlike;
mod fullvehicle;
mod holonomic;
mod model;
mod plugin;
mod raycast_vehicle;
mod systems;
mod tire;
mod wheel;

pub use carlike::CarLike;
pub use fullvehicle::FullVehicle;
pub use holonomic::{seek_force, Holonomic};
pub use model::{Actuation, BodyState, DesiredVelocity, MovementModel, PhysicalYaw};
pub use plugin::{MovementPlugin, MovementSet};
pub use raycast_vehicle::{wheel_offset, RaycastVehicle, VehicleControls};
pub use tire::{
    relax, slip_angle, slip_ratio, step_wheel, tire_force, TireForce, TireParams, WheelInput,
    WheelOutput, WheelSpec, MIN_RELAXATION_TIME, RELAXATION_LENGTH, SLIP_SPEED_FLOOR,
};
pub use wheel::{WheelState, Wheels};
