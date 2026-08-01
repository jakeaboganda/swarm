use std::collections::VecDeque;

use bevy::prelude::*;

use crate::agent::{AgentName, Plan};

/// Height above the ground at which path/trail gizmos are drawn, so they
/// float over the floor instead of z-fighting with it.
const VIZ_HEIGHT: f32 = 0.6;
/// Longest motion trail kept per entity (samples). At the fixed 60 Hz tick
/// this is a few seconds of history.
const TRAIL_MAX: usize = 240;

/// A breadcrumb of recent positions, drawn as the entity's motion trail.
#[derive(Component, Default)]
pub struct Trail(VecDeque<Vec3>);

/// A stable per-name color so each entity's plan and trail are
/// distinguishable and consistent across frames.
fn color_for(name: &str) -> Color {
    let hash = name
        .bytes()
        .fold(0u32, |acc, b| acc.wrapping_mul(31).wrapping_add(b as u32));
    Color::hsl((hash % 360) as f32, 0.8, 0.6)
}

fn on_viz_plane(x: f32, z: f32) -> Vec3 {
    Vec3::new(x, VIZ_HEIGHT, z)
}

/// Draws each entity's remaining plan: a line from its current position
/// through its upcoming waypoints, with a marker at each waypoint.
pub fn draw_plans(mut gizmos: Gizmos, query: Query<(&Transform, &Plan, &AgentName)>) {
    for (transform, plan, name) in &query {
        if plan.waypoints.is_empty() {
            continue;
        }
        let color = color_for(&name.0);
        let mut prev = on_viz_plane(transform.translation.x, transform.translation.z);
        for waypoint in &plan.waypoints {
            let point = on_viz_plane(waypoint.position.x, waypoint.position.z);
            gizmos.line(prev, point, color);
            gizmos.sphere(Isometry3d::from_translation(point), 0.35, color);
            prev = point;
        }
    }
}

/// Appends the current position to each entity's trail once per tick.
pub fn record_trails(mut query: Query<(&Transform, &mut Trail)>) {
    for (transform, mut trail) in &mut query {
        trail.0.push_back(on_viz_plane(
            transform.translation.x,
            transform.translation.z,
        ));
        if trail.0.len() > TRAIL_MAX {
            trail.0.pop_front();
        }
    }
}

/// Draws each entity's recent motion trail as a dimmer line.
pub fn draw_trails(mut gizmos: Gizmos, query: Query<(&Trail, &AgentName)>) {
    for (trail, name) in &query {
        if trail.0.len() < 2 {
            continue;
        }
        let color = color_for(&name.0).with_alpha(0.4);
        gizmos.linestrip(trail.0.iter().copied(), color);
    }
}
