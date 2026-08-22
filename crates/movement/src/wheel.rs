//! The per-wheel state a wheeled vehicle carries between ticks.
//!
//! Split out from `RaycastVehicle` (which holds the tuning constants) so the
//! evolving state has one home: the drive system writes it, the viz stream
//! reads it, and a second wheeled embodiment could reuse it unchanged.

use bevy::prelude::*;

/// One wheel, as of the last tick.
///
/// `steer` lives here rather than on the vehicle because it is genuinely
/// per-wheel: today both front wheels share an angle, but Ackermann geometry
/// (where the inside wheel turns further) is then a change to how these are
/// filled in, not a change to where steering is stored.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelState {
    /// Spin rate about the axle (rad/s). The wheel's only real state.
    pub omega: f32,
    /// Accumulated spin angle, wrapped to `[0, 2pi)` -- for rendering. Wrapped
    /// because an unbounded accumulator loses visible precision in `f32`
    /// within a few minutes of driving.
    pub angle: f32,
    /// Steering angle about the chassis up axis (radians).
    pub steer: f32,
    /// Whether the wheel is touching the ground.
    pub contact: bool,
    /// Suspension compression from full extension (m).
    pub compression: f32,
    /// Vertical load the suspension is carrying (N). Zero when airborne; this
    /// is what scales the tire's grip, so weight transfer follows from it.
    pub load: f32,
    /// Instantaneous slip this tick -- what the wheel is doing right now.
    pub slip_ratio: f32,
    pub slip_angle: f32,
    /// The tire's relaxed (lagged) slips, carried between ticks. This is the
    /// carcass deflection the force is actually computed from; a real tire
    /// takes about half a metre of rolling to build its grip.
    pub relaxed_slip_ratio: f32,
    pub relaxed_slip_angle: f32,
}

/// A vehicle's four wheels, in the layout order the drive system uses:
/// front-left, front-right, rear-left, rear-right.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq)]
pub struct Wheels(pub [WheelState; 4]);

impl Wheels {
    /// Whether every wheel is on the ground.
    pub fn all_planted(&self) -> bool {
        self.0.iter().all(|w| w.contact)
    }

    /// Total vertical load across all wheels (N) -- the weight the suspension
    /// is currently carrying.
    pub fn total_load(&self) -> f32 {
        self.0.iter().map(|w| w.load).sum()
    }
}
