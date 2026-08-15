use std::collections::{HashMap, VecDeque};

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use viz::{EntityDescriptor, EntityId, SceneEvent, ServerToViewer, Shape};

use crate::client::VizStream;
use crate::overlay::{DebugData, PerceivedBlip, SensorEnvelope, Trail};

/// How many ticks behind the newest frame the render clock plays, so there
/// is always a next snapshot to interpolate toward and a late frame doesn't
/// cause a pause. ~62 ms at a 64 Hz tick.
const BUFFER_TICKS: f64 = 4.0;
/// Hard-snap the render clock back to its target depth if it drifts this far
/// (a long stall or a sim restart), rather than crawling back.
const REANCHOR_TICKS: f64 = 12.0;
/// Cap on per-entity snapshot history.
const HISTORY_MAX: usize = 32;
/// Low-pass weight for the render frame interval. Present timing is jittery
/// even under vsync (measured ~2-20 ms at a 6 ms refresh); advancing the
/// clock by the raw interval turns that jitter into motion judder, so we
/// advance by a smoothed interval instead.
const DT_SMOOTHING: f64 = 0.1;
/// Playback-speed correction per tick of buffer error (proportional gain),
/// clamped to `MAX_RATE_ADJUST`. Holds the buffer at `BUFFER_TICKS` with a
/// speed change too small to see, instead of leaving latency wherever a snap
/// happened to land.
const CATCHUP_GAIN: f64 = 0.02;
const MAX_RATE_ADJUST: f64 = 0.1;

/// Maps a viz `EntityId` to the Bevy entity rendering it.
#[derive(Resource, Default)]
pub struct EntityMap(HashMap<EntityId, Entity>);

impl EntityMap {
    /// The Bevy entity rendering `id`, if any.
    pub fn get(&self, id: &EntityId) -> Option<Entity> {
        self.0.get(id).copied()
    }
}

/// Latest scenario/arena info from the stream.
#[derive(Resource, Default)]
pub struct ViewerState {
    pub scenario: Option<viz::ScenarioState>,
    pub arena: Option<viz::ArenaBounds>,
}

/// Diagnostic accumulator (see `log_timing`). Records how often, and how
/// evenly, `apply_stream` actually sees a new frame.
#[derive(Resource)]
pub struct Diag {
    pub enabled: bool,
    last_seen: Option<f32>,
    win_min: f32,
    win_max: f32,
    win_count: u32,
}

impl Diag {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled,
            last_seen: None,
            win_min: f32::INFINITY,
            win_max: 0.0,
            win_count: 0,
        }
    }

    fn record_seen(&mut self, now_secs: f32) {
        if let Some(last) = self.last_seen {
            let gap = (now_secs - last) * 1000.0;
            self.win_min = self.win_min.min(gap);
            self.win_max = self.win_max.max(gap);
            self.win_count += 1;
        }
        self.last_seen = Some(now_secs);
    }
}

/// One timestamped pose from a frame.
#[derive(Clone, Copy)]
struct Sample {
    tick: f64,
    translation: Vec3,
    rotation: Quat,
}

/// A dynamic entity's recent poses, ordered by tick, that the render clock
/// interpolates through.
#[derive(Component, Default)]
pub struct History {
    samples: VecDeque<Sample>,
}

impl History {
    fn push(&mut self, sample: Sample) {
        self.samples.push_back(sample);
        while self.samples.len() > HISTORY_MAX {
            self.samples.pop_front();
        }
    }

    /// The two samples bracketing `tick`, if any.
    fn bracket(&self, tick: f64) -> Option<(Sample, Sample)> {
        for i in 1..self.samples.len() {
            let (a, b) = (self.samples[i - 1], self.samples[i]);
            if a.tick <= tick && tick <= b.tick {
                return Some((a, b));
            }
        }
        None
    }
}

/// A monotonic playback clock in sim-time (ticks). It advances at the sim's
/// tick rate and trails the newest received frame by `BUFFER_TICKS`, so
/// interpolation is smooth regardless of message-arrival jitter.
#[derive(Resource)]
pub struct RenderClock {
    tick: f64,
    tick_rate: f64,
    newest: f64,
    primed: bool,
}

impl Default for RenderClock {
    fn default() -> Self {
        Self {
            tick: 0.0,
            tick_rate: 64.0,
            newest: 0.0,
            primed: false,
        }
    }
}

fn to_transform(transform: &viz::Transform) -> Transform {
    Transform {
        translation: Vec3::new(
            transform.position.x,
            transform.position.y,
            transform.position.z,
        ),
        rotation: Quat::from_xyzw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
        scale: Vec3::ONE,
    }
}

fn mesh_for(shape: &Shape, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    match shape {
        Shape::Capsule {
            radius,
            half_length,
        } => meshes.add(Capsule3d::new(*radius, half_length * 2.0)),
        Shape::Cuboid { half_extents } => meshes.add(Cuboid::new(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        )),
        Shape::Mesh {
            positions,
            normals,
            indices,
        } => {
            // Baked triangle geometry (e.g. the road surface) straight from the
            // wire: positions + up-normals + triangle indices.
            let mut mesh = Mesh::new(
                PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            );
            let verts: Vec<[f32; 3]> = positions.iter().map(|v| [v.x, v.y, v.z]).collect();
            let norms: Vec<[f32; 3]> = normals.iter().map(|v| [v.x, v.y, v.z]).collect();
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, verts);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, norms);
            mesh.insert_indices(Indices::U32(indices.clone()));
            meshes.add(mesh)
        }
    }
}

fn material_for(
    descriptor: &EntityDescriptor,
    materials: &mut Assets<StandardMaterial>,
) -> Handle<StandardMaterial> {
    let color = descriptor.color;
    // Give dynamic entities a little emissive lift so they read as the
    // "actors" against the static geometry.
    let emissive = if descriptor.kind.is_dynamic() {
        LinearRgba::rgb(color.r * 0.25, color.g * 0.25, color.b * 0.25)
    } else {
        LinearRgba::BLACK
    };
    materials.add(StandardMaterial {
        base_color: Color::srgb(color.r, color.g, color.b),
        emissive,
        ..default()
    })
}

fn spawn_entity(
    commands: &mut Commands,
    map: &mut EntityMap,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    descriptor: &EntityDescriptor,
) {
    let transform = to_transform(&descriptor.transform);
    let mut entity = commands.spawn((
        Mesh3d(mesh_for(&descriptor.shape, meshes)),
        MeshMaterial3d(material_for(descriptor, materials)),
        transform,
    ));
    if descriptor.kind.is_dynamic() {
        entity.insert((DebugData::default(), Trail::default(), History::default()));
    }
    if let Some(sensors) = descriptor.sensors {
        entity.insert(SensorEnvelope {
            range: sensors.range,
            fov_half_angle: sensors.fov_half_angle,
        });
    }
    map.0.insert(descriptor.id.clone(), entity.id());
}

/// Drains the viz stream and applies it to the rendered world: (re)builds
/// the scene on `SceneInit`, adds/removes entities on lifecycle events, then
/// applies the latest frame / debug frame. Defensive against the stream:
/// frames for unknown ids are ignored, and spawns are idempotent.
#[allow(clippy::too_many_arguments)]
pub fn apply_stream(
    mut stream: ResMut<VizStream>,
    mut commands: Commands,
    mut map: ResMut<EntityMap>,
    mut state: ResMut<ViewerState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut histories: Query<&mut History>,
    mut debug: Query<&mut DebugData>,
    mut clock: ResMut<RenderClock>,
    time: Res<Time>,
    mut diag: ResMut<Diag>,
) {
    // Reliable, ordered messages first (scene-init resets before any frame).
    while let Ok(message) = stream.reliable.try_recv() {
        match message {
            ServerToViewer::SceneInit(init) => {
                // A scene-init is a full reset (also how a reconnect
                // re-syncs after a sim restart). Reset the render clock too:
                // it re-primes off the first frame with the sim's tick rate.
                for (_, entity) in map.0.drain() {
                    commands.entity(entity).despawn();
                }
                clock.tick_rate = init.tick_rate as f64;
                clock.primed = false;
                info!(
                    "scene-init: {} entities, state {:?}",
                    init.entities.len(),
                    init.state
                );
                state.arena = Some(init.arena);
                state.scenario = Some(init.state);
                for descriptor in &init.entities {
                    spawn_entity(
                        &mut commands,
                        &mut map,
                        &mut meshes,
                        &mut materials,
                        descriptor,
                    );
                }
            }
            ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor)) => {
                if !map.0.contains_key(&descriptor.id) {
                    spawn_entity(
                        &mut commands,
                        &mut map,
                        &mut meshes,
                        &mut materials,
                        &descriptor,
                    );
                }
            }
            ServerToViewer::Event(SceneEvent::EntityDespawned { id }) => {
                if let Some(entity) = map.0.remove(&id) {
                    commands.entity(entity).despawn();
                }
            }
            ServerToViewer::Event(SceneEvent::ScenarioState { state: scenario }) => {
                state.scenario = Some(scenario);
            }
            // Frames/debug frames are keep-latest (below), never reliable.
            ServerToViewer::Frame(_) | ServerToViewer::DebugFrame(_) => {}
        }
    }

    // Then the newest frame (keep-latest: intermediate frames are coalesced
    // away, so the queue can never grow). Each frame appends a timestamped
    // snapshot to the per-entity history; playback (advance_playback) reads
    // these on the sim-time render clock, so message-arrival jitter never
    // reaches the rendered motion.
    if stream.frame.has_changed().unwrap_or(false) {
        diag.record_seen(time.elapsed_secs());
        if let Some(frame) = stream.frame.borrow_and_update().clone() {
            clock.newest = clock.newest.max(frame.tick as f64);
            for entity_frame in &frame.entities {
                if let Some(&entity) = map.0.get(&entity_frame.id) {
                    if let Ok(mut history) = histories.get_mut(entity) {
                        let t = to_transform(&entity_frame.transform);
                        history.push(Sample {
                            tick: frame.tick as f64,
                            translation: t.translation,
                            rotation: t.rotation,
                        });
                    }
                }
            }
            // Prime the clock off the first frame, one buffer behind newest.
            if !clock.primed {
                clock.tick = clock.newest - BUFFER_TICKS;
                clock.primed = true;
            }
        }
    }

    if stream.debug.has_changed().unwrap_or(false) {
        if let Some(debug_frame) = stream.debug.borrow_and_update().clone() {
            for entity_debug in &debug_frame.entities {
                if let Some(&entity) = map.0.get(&entity_debug.id) {
                    if let Ok(mut data) = debug.get_mut(entity) {
                        data.plan = entity_debug
                            .plan
                            .iter()
                            .map(|p| Vec3::new(p.x, p.y, p.z))
                            .collect();
                        data.reflex_active = entity_debug.reflex_active;
                        data.detections = entity_debug
                            .detections
                            .iter()
                            .map(|b| PerceivedBlip {
                                id: b.id.clone(),
                                position: Vec3::new(b.position.x, b.position.y, b.position.z),
                                kind: b.kind,
                            })
                            .collect();
                    }
                }
            }
        }
    }
}

/// Diagnostic (set VIZ_DIAG=1): once a second, logs the viewer's render rate
/// and the interval between frames *as seen by apply_stream* (after display
/// sampling), so the beat between the 30 Hz stream and the display is visible.
pub fn log_timing(
    time: Res<Time>,
    clock: Res<RenderClock>,
    mut diag: ResMut<Diag>,
    // (elapsed, frame count, min frame dt ms, max frame dt ms)
    mut acc: Local<(f32, u32, f32, f32)>,
) {
    if !diag.enabled {
        return;
    }
    let dt = time.delta_secs();
    acc.0 += dt;
    acc.1 += 1;
    let dt_ms = dt * 1000.0;
    acc.2 = if acc.1 == 1 { dt_ms } else { acc.2.min(dt_ms) };
    acc.3 = acc.3.max(dt_ms);
    if acc.0 >= 1.0 {
        let (min, max, count) = (diag.win_min, diag.win_max, diag.win_count);
        info!(
            "render {:.0} fps (frame dt {:.1}-{:.1}ms) | seen-frame gap min {:.1}ms max {:.1}ms ({}/s) | buffer {:.1} ticks (rate {:.0})",
            acc.1 as f32 / acc.0,
            acc.2,
            acc.3,
            if min.is_finite() { min } else { 0.0 },
            max,
            count,
            clock.newest - clock.tick,
            clock.tick_rate,
        );
        *acc = (0.0, 0, 0.0, 0.0);
        diag.win_count = 0;
        diag.win_min = f32::INFINITY;
        diag.win_max = 0.0;
    }
}

/// Advances the sim-time render clock and poses each entity by interpolating
/// its snapshot history at the clock's tick. The clock plays `BUFFER_TICKS`
/// behind the newest frame, so it always has a bracketing pair to lerp
/// between and message-arrival jitter never reaches the rendered motion.
pub fn advance_playback(
    time: Res<Time>,
    mut clock: ResMut<RenderClock>,
    mut smoothed_dt: Local<f64>,
    mut query: Query<(&mut Transform, &History)>,
) {
    if !clock.primed {
        return;
    }
    // Advance by a low-passed frame interval, not the raw one, so present
    // jitter doesn't reach the motion.
    let dt = time.delta_secs_f64();
    *smoothed_dt = if *smoothed_dt <= 0.0 {
        dt
    } else {
        *smoothed_dt + (dt - *smoothed_dt) * DT_SMOOTHING
    };

    let target = clock.newest - BUFFER_TICKS;
    let error = target - clock.tick;
    if error.abs() > REANCHOR_TICKS {
        // Gross desync (startup warmup, a long stall, a sim restart): snap
        // to target rather than crawl there.
        clock.tick = target;
    } else {
        // Hold the buffer at BUFFER_TICKS by nudging playback speed a few
        // percent toward the set-point — closed-loop, so latency stays fixed
        // instead of parking wherever a snap left it.
        let adjust = (error * CATCHUP_GAIN).clamp(-MAX_RATE_ADJUST, MAX_RATE_ADJUST);
        clock.tick += *smoothed_dt * clock.tick_rate * (1.0 + adjust);
    }
    // Never extrapolate past the newest data we hold.
    if clock.tick > clock.newest {
        clock.tick = clock.newest;
    }
    let render_tick = clock.tick;
    for (mut transform, history) in &mut query {
        if let Some((a, b)) = history.bracket(render_tick) {
            let span = (b.tick - a.tick).max(1e-6);
            let alpha = ((render_tick - a.tick) / span).clamp(0.0, 1.0) as f32;
            transform.translation = a.translation.lerp(b.translation, alpha);
            transform.rotation = a.rotation.slerp(b.rotation, alpha);
        } else if let Some(last) = history.samples.back() {
            // Clock outside the sample range (start-up, or underrun after a
            // stall): hold at the newest known pose.
            transform.translation = last.translation;
            transform.rotation = last.rotation;
        }
    }
}

/// Frames the top-down camera to the arena once its bounds arrive (and if
/// they change between scenarios).
pub fn frame_camera(state: Res<ViewerState>, mut camera: Query<&mut Transform, With<Camera3d>>) {
    if !state.is_changed() {
        return;
    }
    let Some(arena) = state.arena else {
        return;
    };
    let height = arena.width.max(arena.depth) * 0.9;
    if let Ok(mut transform) = camera.single_mut() {
        *transform = Transform::from_xyz(0.0, height, height).looking_at(Vec3::ZERO, Vec3::Y);
    }
}

/// Spawns the camera and light. The camera frames a 50-unit arena from
/// above; scene geometry arrives over the stream.
pub fn setup_camera(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let height = 45.0;
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, height, height).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
