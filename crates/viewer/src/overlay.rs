use std::collections::VecDeque;

use bevy::prelude::*;
use viz::{DetectionKind, EntityId};

use crate::scene::EntityMap;

/// Height above the ground at which path/trail gizmos are drawn.
const VIZ_HEIGHT: f32 = 0.6;
/// Longest motion trail kept per entity.
const TRAIL_MAX: usize = 240;
/// A sensing range beyond this is treated as "effectively unlimited" and its
/// envelope isn't drawn (e.g. the near-perfect default's ~1e6 range, which
/// would be an absurd ring dwarfing the arena).
const MAX_ENVELOPE_RANGE: f32 = 500.0;
/// The sensor model has no vertical FOV, so the view frustum uses this fixed
/// modest vertical half-angle (~20°), clamped so the far plane stays above
/// ground.
const FRUSTUM_VFOV_HALF: f32 = 0.35;
/// Clamp on the horizontal half-angle used for the frustum's far-plane width,
/// so a very wide FOV can't blow it up via `tan()` near 90°.
const FRUSTUM_MAX_HFOV: f32 = 1.3;

/// Debug-layer state for a dynamic entity, updated from `DebugFrame`s.
#[derive(Component, Default)]
pub struct DebugData {
    pub plan: Vec<Vec3>,
    pub reflex_active: bool,
    /// What this agent currently perceives (delayed, noised) — drawn as
    /// ghosts by `draw_detections`.
    pub detections: Vec<PerceivedBlip>,
}

/// One entity as an agent perceives it, mirrored from the viz debug frame.
pub struct PerceivedBlip {
    pub id: EntityId,
    pub position: Vec3,
    pub kind: DetectionKind,
}

/// An agent's sensing region, for the envelope overlay. Attached at spawn
/// from the entity descriptor's `sensors`.
#[derive(Component, Clone, Copy)]
pub struct SensorEnvelope {
    pub range: f32,
    pub fov_half_angle: f32,
}

/// Which perception debug sub-layers are drawn. Both on by default; toggled
/// from the keyboard (see `toggle_overlays`).
#[derive(Resource)]
pub struct OverlayToggles {
    pub detections: bool,
    pub envelope: bool,
}

impl Default for OverlayToggles {
    fn default() -> Self {
        Self {
            detections: true,
            envelope: true,
        }
    }
}

/// A breadcrumb of recent positions, accumulated by the viewer from the
/// frame stream (trails are not transmitted).
#[derive(Component, Default)]
pub struct Trail(VecDeque<Vec3>);

fn on_plane(x: f32, z: f32) -> Vec3 {
    Vec3::new(x, VIZ_HEIGHT, z)
}

/// Appends the current position to each entity's trail, but only when it
/// actually moved. `record_trails` runs at render FPS while positions
/// update at the ~30 Hz frame rate, so deduping keeps the trail a
/// consistent length in time rather than in frames.
pub fn record_trails(mut query: Query<(&Transform, &mut Trail)>) {
    for (transform, mut trail) in &mut query {
        let point = on_plane(transform.translation.x, transform.translation.z);
        if trail.0.back() == Some(&point) {
            continue;
        }
        trail.0.push_back(point);
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

/// `P` toggles the perception detection overlay; `O` toggles the sensing
/// envelope.
pub fn toggle_overlays(keys: Res<ButtonInput<KeyCode>>, mut toggles: ResMut<OverlayToggles>) {
    if keys.just_pressed(KeyCode::KeyP) {
        toggles.detections = !toggles.detections;
    }
    if keys.just_pressed(KeyCode::KeyO) {
        toggles.envelope = !toggles.envelope;
    }
}

/// Draws what each agent perceives: a line from the agent to each detected
/// entity's noised ("ghost") position, a marker at the ghost, and a thin red
/// connector from the ghost to the entity's true position — that gap *is* the
/// perception error (noise + latency). Entities the agent can't see have no
/// ghost, so culling (range/FOV) reads as absence.
pub fn draw_detections(
    toggles: Res<OverlayToggles>,
    map: Res<EntityMap>,
    mut gizmos: Gizmos,
    observers: Query<(&Transform, &DebugData)>,
    transforms: Query<&Transform>,
) {
    if !toggles.detections {
        return;
    }
    for (observer, data) in &observers {
        let from = on_plane(observer.translation.x, observer.translation.z);
        for blip in &data.detections {
            let ghost = on_plane(blip.position.x, blip.position.z);
            let color = match blip.kind {
                DetectionKind::Agent => Color::srgb(1.0, 0.85, 0.2),
                DetectionKind::Static => Color::srgb(0.6, 0.7, 1.0),
            };
            gizmos.line(from, ghost, color.with_alpha(0.5));
            gizmos.sphere(Isometry3d::from_translation(ghost), 0.25, color);
            // Connector from the perceived ghost to the true position.
            if let Some(entity) = map.get(&blip.id) {
                if let Ok(truth) = transforms.get(entity) {
                    let true_pos = on_plane(truth.translation.x, truth.translation.z);
                    gizmos.line(ghost, true_pos, Color::srgb(1.0, 0.3, 0.3).with_alpha(0.8));
                }
            }
        }
    }
}

/// Draws each agent's sensing region: a range circle, plus a wedge around its
/// current facing when the FOV is limited. The wedge follows the rendered
/// facing (which tracks travel direction), so a stationary agent — which
/// senses 360° — shows just the ring. Skipped when range is effectively
/// unlimited (see `MAX_ENVELOPE_RANGE`).
pub fn draw_sensor_envelope(
    toggles: Res<OverlayToggles>,
    mut gizmos: Gizmos,
    query: Query<(&Transform, &SensorEnvelope)>,
) {
    if !toggles.envelope {
        return;
    }
    let color = Color::srgb(0.3, 0.9, 0.6).with_alpha(0.4);
    for (transform, env) in &query {
        if !env.range.is_finite() || env.range > MAX_ENVELOPE_RANGE {
            continue;
        }
        let center = on_plane(transform.translation.x, transform.translation.z);
        gizmos.circle(
            Isometry3d::new(center, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
            env.range,
            color,
        );
        if env.fov_half_angle >= std::f32::consts::PI {
            continue; // full 360°: the ring already says it
        }
        let forward = transform.forward();
        let heading = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
        if heading == Vec3::ZERO {
            continue; // stationary: senses 360°, ring only
        }
        draw_frustum(
            &mut gizmos,
            transform.translation,
            heading,
            env.range,
            env.fov_half_angle,
            color,
        );
    }
}

/// Draws a camera-style view frustum: apex at the agent's eye, four edges out
/// to a far rectangle at `range` along `heading`. The horizontal half-width is
/// the real FOV half-angle; the vertical is a fixed modest default (not in the
/// sensor model), clamped so the far plane stays above the ground.
fn draw_frustum(
    gizmos: &mut Gizmos,
    origin: Vec3,
    heading: Vec3,
    range: f32,
    hfov_half: f32,
    color: Color,
) {
    let eye = Vec3::new(origin.x, origin.y.max(0.2), origin.z);
    let right = heading.cross(Vec3::Y).normalize_or_zero();
    let half_w = range * hfov_half.min(FRUSTUM_MAX_HFOV).tan();
    let half_h = (range * FRUSTUM_VFOV_HALF.tan()).clamp(0.1, eye.y - 0.05);
    let far = eye + heading * range;
    let tl = far + Vec3::Y * half_h - right * half_w;
    let tr = far + Vec3::Y * half_h + right * half_w;
    let bl = far - Vec3::Y * half_h - right * half_w;
    let br = far - Vec3::Y * half_h + right * half_w;
    for corner in [tl, tr, bl, br] {
        gizmos.line(eye, corner, color);
    }
    gizmos.linestrip([tl, tr, br, bl, tl], color);
}
