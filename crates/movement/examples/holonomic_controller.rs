//! Drives a unit mass toward a target velocity with the Holonomic force
//! controller, integrating by hand (semi-implicit Euler + linear damping,
//! as the sim does), and prints the approach to the commanded speed.
//!
//! Run: `cargo run -p movement --example holonomic_controller`

use bevy::math::Vec3;
use movement::{DesiredVelocity, Holonomic, MovementModel};

fn main() {
    let model = Holonomic::default();
    let desired = DesiredVelocity {
        value: Vec3::new(5.0, 0.0, 0.0),
        urgent: false,
    };

    let mass = 1.0;
    let dt = 1.0 / 60.0;
    let damping = 0.75;
    let mut velocity = Vec3::ZERO;

    println!("step   speed");
    for step in 0..=180 {
        if step % 20 == 0 {
            println!("{step:>4}   {:.3}", velocity.length());
        }
        let force = model.compute_force(desired, velocity);
        velocity += force / mass * dt;
        velocity *= 1.0 - damping * dt;
    }
    println!("\ncommanded speed = {:.3}", desired.value.length());
}
