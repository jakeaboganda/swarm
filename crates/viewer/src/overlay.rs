use std::collections::VecDeque;

use bevy::prelude::*;
use viz::{DetectionKind, EntityId};

use crate::scene::EntityMap;

/// Height above the ground at which path/trail gizmos are drawn.
const VIZ_HEIGHT: f32 = 0.6;
/// Radius of the filled sphere marking each plan waypoint. Small -- a dot on
/// the path, not a bubble over it.
const WAYPOINT_RADIUS: f32 = 0.1;
/// Longest motion trail kept per entity.
const TRAIL_MAX: usize = 240;
/// A sensing range beyond this is treated as "effectively unlimited" and its
/// envelope isn't drawn (e.g. the near-perfect default's ~1e6 range, which
/// would be an absurd ring dwarfing the arena).
const MAX_ENVELOPE_RANGE: f32 = 500.0;
/// The FOV prism is drawn this tall (~the arena wall height). The sensor culls
/// only horizontally -- it's vertically unbounded -- so this is a drawing
/// extent, not a sensing limit: the prism's top and bottom are identical.
const FOV_VOLUME_HEIGHT: f32 = 3.0;

/// Debug-layer state for a dynamic entity, updated from `DebugFrame`s.
#[derive(Component, Default)]
pub struct DebugData {
    pub plan: Vec<Vec3>,
    pub reflex_active: bool,
    /// What this agent currently perceives (delayed, noised) -- drawn as
    /// ghosts by `draw_detections`.
    pub detections: Vec<PerceivedBlip>,
    /// Per-wheel diagnostics, in `viz::WHEEL_NODES` order -- what `tint_wheels`
    /// colours the wheel nodes by.
    pub wheels: Vec<viz::WheelDebug>,
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
    /// Vertical FOV half-angle (radians). `>= PI` = unbounded: draw the
    /// vertically-unbounded wedge prism. Finite: draw an honest frustum that
    /// closes top and bottom at this elevation.
    pub vertical_fov_half_angle: f32,
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

/// Lift a real 3D point by the gizmo draw height, keeping its own elevation so
/// overlays sit just above a sloped road instead of flattening to y=0.
fn lift(p: Vec3) -> Vec3 {
    Vec3::new(p.x, p.y + VIZ_HEIGHT, p.z)
}

/// Appends the current position to each entity's trail, but only when it
/// actually moved. `record_trails` runs at render FPS while positions
/// update at the ~30 Hz frame rate, so deduping keeps the trail a
/// consistent length in time rather than in frames.
pub fn record_trails(mut query: Query<(&Transform, &mut Trail)>) {
    for (transform, mut trail) in &mut query {
        let point = lift(transform.translation);
        if trail.0.back() == Some(&point) {
            continue;
        }
        trail.0.push_back(point);
        if trail.0.len() > TRAIL_MAX {
            trail.0.pop_front();
        }
    }
}

/// Marks each plan waypoint. The wireframe sphere is replaced by a filled mesh
/// (`sync_waypoint_markers`); this tags those pooled marker entities.
#[derive(Component)]
pub struct WaypointMarker;

/// Reusable filled-sphere markers for plan waypoints, plus their shared mesh
/// and the two flat-colour materials (blue normally, red while a reflex is
/// overriding the plan). Pooled so the plan changing every tick doesn't churn
/// entities: we grow the pool as needed and hide the unused tail.
#[derive(Resource)]
pub struct WaypointMarkers {
    mesh: Handle<Mesh>,
    normal: Handle<StandardMaterial>,
    reflex: Handle<StandardMaterial>,
    pool: Vec<Entity>,
}

/// Creates the shared waypoint-marker mesh + materials once at startup.
pub fn setup_waypoint_markers(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let flat = |color: Color| StandardMaterial {
        base_color: color,
        // Unlit: a flat, readable marker colour regardless of scene lighting,
        // matching the old gizmo look but filled.
        unlit: true,
        ..default()
    };
    commands.insert_resource(WaypointMarkers {
        mesh: meshes.add(Sphere::new(WAYPOINT_RADIUS)),
        normal: materials.add(flat(Color::srgb(0.2, 0.8, 1.0))),
        reflex: materials.add(flat(Color::srgb(1.0, 0.2, 0.1))),
        pool: Vec::new(),
    });
}

/// Positions a filled sphere at every current plan waypoint, colouring it red
/// where a reflex is overriding that plan. Reuses the marker pool, growing it
/// when there are more waypoints than markers and hiding the leftovers.
pub fn sync_waypoint_markers(
    mut commands: Commands,
    mut markers: ResMut<WaypointMarkers>,
    plans: Query<&DebugData>,
    mut marker_q: Query<
        (
            &mut Transform,
            &mut MeshMaterial3d<StandardMaterial>,
            &mut Visibility,
        ),
        With<WaypointMarker>,
    >,
) {
    let mut points: Vec<(Vec3, bool)> = Vec::new();
    for debug in &plans {
        for waypoint in &debug.plan {
            points.push((lift(*waypoint), debug.reflex_active));
        }
    }
    // Grow the pool to cover this frame (a newly spawned marker is positioned
    // next frame, once its Commands have applied -- fine, the pool is stable).
    while markers.pool.len() < points.len() {
        let mesh = markers.mesh.clone();
        let material = markers.normal.clone();
        let entity = commands
            .spawn((
                Mesh3d(mesh),
                MeshMaterial3d(material),
                Transform::default(),
                Visibility::Hidden,
                WaypointMarker,
            ))
            .id();
        markers.pool.push(entity);
    }
    for (i, &entity) in markers.pool.iter().enumerate() {
        let Ok((mut transform, mut material, mut visibility)) = marker_q.get_mut(entity) else {
            continue;
        };
        match points.get(i) {
            Some(&(position, reflex)) => {
                transform.translation = position;
                material.0 = if reflex {
                    markers.reflex.clone()
                } else {
                    markers.normal.clone()
                };
                *visibility = Visibility::Visible;
            }
            None => *visibility = Visibility::Hidden,
        }
    }
}

/// Draws the line of each entity's remaining plan from its current position,
/// turning red while a reflex is overriding it. The waypoint dots themselves
/// are filled meshes drawn by `sync_waypoint_markers`.
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
        let mut prev = lift(transform.translation);
        for waypoint in &debug.plan {
            let point = lift(*waypoint);
            gizmos.line(prev, point, color);
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
/// connector from the ghost to the entity's true position -- that gap *is* the
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
        let from = lift(observer.translation);
        for blip in &data.detections {
            let ghost = lift(blip.position);
            let color = match blip.kind {
                DetectionKind::Agent => Color::srgb(1.0, 0.85, 0.2),
                DetectionKind::Static => Color::srgb(0.6, 0.7, 1.0),
            };
            gizmos.line(from, ghost, color.with_alpha(0.5));
            gizmos.sphere(Isometry3d::from_translation(ghost), 0.25, color);
            // Connector from the perceived ghost to the true position.
            if let Some(entity) = map.get(&blip.id) {
                if let Ok(truth) = transforms.get(entity) {
                    let true_pos = lift(truth.translation);
                    gizmos.line(ghost, true_pos, Color::srgb(1.0, 0.3, 0.3).with_alpha(0.8));
                }
            }
        }
    }
}

/// Draws each agent's sensing region: a range circle, plus -- when the FOV is
/// limited -- the sensed volume around its current facing. That volume is a
/// vertically-unbounded wedge prism by default, or an honest frustum when the
/// sensor has a finite vertical FOV. It follows the rendered facing (which
/// tracks travel direction), so a stationary agent -- which senses 360° --
/// shows just the ring. Skipped when range is effectively unlimited (see
/// `MAX_ENVELOPE_RANGE`).
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
        let center = lift(transform.translation);
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
        if env.vertical_fov_half_angle < std::f32::consts::PI {
            // A finite vertical FOV closes the wedge into a frustum.
            draw_fov_frustum(
                &mut gizmos,
                center,
                heading,
                env.range,
                env.fov_half_angle,
                env.vertical_fov_half_angle,
                color,
            );
        } else {
            draw_fov_prism(
                &mut gizmos,
                center,
                heading,
                env.range,
                env.fov_half_angle,
                color,
            );
        }
    }
}

/// Draws the FOV as an honest vertical prism: the horizontal wedge (apex at the
/// agent, arc at true radial `range`) extruded straight up with parallel walls.
/// The sensor culls only horizontally -- all heights within range+bearing are
/// seen -- so there is no vertical FOV: top and bottom are identical, and the
/// prism rises a fixed height from the agent's own elevation rather than
/// converging to a far plane.
fn draw_fov_prism(
    gizmos: &mut Gizmos,
    origin: Vec3,
    heading: Vec3,
    range: f32,
    hfov_half: f32,
    color: Color,
) {
    const SEGMENTS: usize = 16;
    // The horizontal FOV arc at radial `range`, at a given height.
    let arc = |y: f32| -> Vec<Vec3> {
        (0..=SEGMENTS)
            .map(|i| {
                let t = hfov_half * (2.0 * (i as f32 / SEGMENTS as f32) - 1.0);
                let dir = Quat::from_rotation_y(t) * heading;
                Vec3::new(origin.x, y, origin.z) + dir * range
            })
            .collect()
    };
    let base = origin;
    let top = base + Vec3::Y * FOV_VOLUME_HEIGHT;
    let floor = arc(origin.y);
    let ceil = arc(origin.y + FOV_VOLUME_HEIGHT);
    // Floor and ceiling wedges (apex + closing rays + arc), identical in size.
    for (apex, ring) in [(base, &floor), (top, &ceil)] {
        gizmos.line(apex, ring[0], color);
        gizmos.line(apex, ring[SEGMENTS], color);
        gizmos.linestrip(ring.iter().copied(), color);
    }
    // Vertical edges: the apex, plus the arc's ends and midpoint.
    gizmos.line(base, top, color);
    for i in [0, SEGMENTS / 2, SEGMENTS] {
        gizmos.line(floor[i], ceil[i], color);
    }
}

/// Draws the FOV as an honest frustum: bounded in azimuth (`hfov_half`) and
/// elevation (`vfov_half`) out to slant `range`, apex at the sensor. The top
/// and bottom arcs sit at `+/- vfov_half` elevation and converge back to the
/// apex, so unlike the wedge prism the volume closes vertically -- matching a
/// sensor that culls both horizontally and vertically.
#[allow(clippy::too_many_arguments)]
fn draw_fov_frustum(
    gizmos: &mut Gizmos,
    origin: Vec3,
    heading: Vec3,
    range: f32,
    hfov_half: f32,
    vfov_half: f32,
    color: Color,
) {
    const SEGMENTS: usize = 16;
    // A unit direction at azimuth `az` (about +Y from heading) and elevation
    // `el` (up from the horizontal plane), scaled to `range` from the apex.
    let point = |az: f32, el: f32| -> Vec3 {
        let h_dir = Quat::from_rotation_y(az) * heading;
        origin + (h_dir * el.cos() + Vec3::Y * el.sin()) * range
    };
    // The far-cap arc at a given elevation, swept across the azimuth FOV.
    let arc = |el: f32| -> Vec<Vec3> {
        (0..=SEGMENTS)
            .map(|i| {
                let az = hfov_half * (2.0 * (i as f32 / SEGMENTS as f32) - 1.0);
                point(az, el)
            })
            .collect()
    };
    let top = arc(vfov_half);
    let bottom = arc(-vfov_half);
    // The four corner rays from the apex.
    for corner in [top[0], top[SEGMENTS], bottom[0], bottom[SEGMENTS]] {
        gizmos.line(origin, corner, color);
    }
    // The far cap: top and bottom arcs, closed at the sides and middle.
    gizmos.linestrip(top.iter().copied(), color);
    gizmos.linestrip(bottom.iter().copied(), color);
    for i in [0, SEGMENTS / 2, SEGMENTS] {
        gizmos.line(bottom[i], top[i], color);
    }
}
