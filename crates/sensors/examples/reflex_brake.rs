//! Shows the time-to-collision sensor driving a brake reflex as an obstacle
//! approaches head-on, including the hysteresis band that keeps the reflex
//! from chattering right at its threshold.
//!
//! Run: `cargo run -p sensors --example reflex_brake`

use glam::Vec3;
use protocol::messages::{Operator, ReflexAction, ReflexRule, SensorKind};
use sensors::{evaluate, ActiveRule, Obstacle, Sensor, SensorContext, TimeToCollision};

fn main() {
    let mut rules = vec![ActiveRule::new(ReflexRule {
        sensor: SensorKind::TimeToCollision,
        operator: Operator::LessThan,
        threshold: 2.0,
        action: ReflexAction::Brake,
        priority: 10,
    })];

    println!("gap    ttc    reflex");
    // Obstacle starts 12 units ahead and closes at 2 units/s (both radii 0.5).
    for step in 0..12 {
        let obstacle_x = 12.0 - step as f32;
        let ctx = SensorContext {
            self_position: Vec3::ZERO,
            self_velocity: Vec3::new(2.0, 0.0, 0.0),
            self_radius: 0.5,
            obstacles: vec![Obstacle {
                position: Vec3::new(obstacle_x, 0.0, 0.0),
                velocity: Vec3::ZERO,
                radius: 0.5,
            }],
        };

        let ttc = TimeToCollision.read(&ctx);
        let action = evaluate(&mut rules, &ctx);
        let label = match action {
            Some(a) => format!("{a:?}"),
            None => "-".to_string(),
        };
        println!("{obstacle_x:>4.1}  {ttc:>5.2}   {label}");
    }
}
