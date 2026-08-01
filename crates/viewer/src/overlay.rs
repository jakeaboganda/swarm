use std::collections::VecDeque;

use bevy::prelude::*;

/// Height above the ground at which path/trail gizmos are drawn.
const VIZ_HEIGHT: f32 = 0.6;
/// Longest motion trail kept per entity.
const TRAIL_MAX: usize = 240;

/// Debug-layer state for a dynamic entity, updated from `DebugFrame`s.
#[derive(Component, Default)]
pub struct DebugData {
    pub plan: Vec<Vec3>,
    pub reflex_active: bool,
}

/// A breadcrumb of recent positions, accumulated by the viewer from the
/// frame stream (trails are not transmitted).
#[derive(Component, Default)]
pub struct Trail(VecDeque<Vec3>);

fn on_plane(x: f32, z: f32) -> Vec3 {
    Vec3::new(x, VIZ_HEIGHT, z)
}

/// Appends the current position to each entity's trail once per frame.
pub fn record_trails(mut query: Query<(&Transform, &mut Trail)>) {
    for (transform, mut trail) in &mut query {
        trail
            .0
            .push_back(on_plane(transform.translation.x, transform.translation.z));
        if trail.0.len() > TRAIL_MAX {
            trail.0.pop_front();
        }
    }
}

/// Draws each entity's remaining plan from its current position, turning
/// red while a reflex is overriding it.
pub fn draw_plans(mut gizmos: Gizmos, query: Query<(&Transform, &DebugData)>) {
    for (transform, debug) in &query {
        if debug.plan.is_empty() {
            continue;
        }
        let color = if debug.reflex_active {
            Color::srgb(1.0, 0.2, 0.1)
        } else {
            Color::srgb(0.2, 0.8, 1.0)
        };
        let mut prev = on_plane(transform.translation.x, transform.translation.z);
        for waypoint in &debug.plan {
            let point = on_plane(waypoint.x, waypoint.z);
            gizmos.line(prev, point, color);
            gizmos.sphere(Isometry3d::from_translation(point), 0.35, color);
            prev = point;
        }
    }
}

/// Draws each entity's recent motion trail as a dim line.
pub fn draw_trails(mut gizmos: Gizmos, query: Query<&Trail>) {
    for trail in &query {
        if trail.0.len() < 2 {
            continue;
        }
        gizmos.linestrip(
            trail.0.iter().copied(),
            Color::srgb(0.6, 0.6, 0.6).with_alpha(0.4),
        );
    }
}
