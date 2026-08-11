use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use sensors::{
    perceive, DetectionKind, Obstacle, PerceivedEntity, Rng, Sensor, SensorContext, TimeToCollision,
};

use crate::agent::AgentName;
use crate::arbitration::wall_obstacles;
use crate::scenario::ArenaBounds;
use crate::scenario_state::Tick;
use crate::world::AGENT_RADIUS;

/// Emit a perception frame every N physics ticks — ~32 Hz. Latency is
/// measured in these frames (see `Perceiver`), so this also sets the latency
/// unit.
const TICKS_PER_FRAME: u64 = 2;
/// Cap on the per-agent latency ring buffer. This bounds faithful latency at
/// `(MAX_BUFFER - 1) * TICKS_PER_FRAME` ticks; a larger `latency_ticks`
/// saturates to the oldest held frame rather than erroring.
const MAX_BUFFER: usize = 64;

/// The perception broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Perception(pub perception::PerceptionHandle);

/// Base seed for reproducible perception noise. Mixed with agent name + tick
/// per reading. Set from the scenario; 0 by default.
#[derive(Resource, Default)]
pub struct PerceptionSeed(pub u64);

/// Agents currently connected on the perception pathway, by name, with a
/// live-connection count. The count makes a same-name reconnect safe: if the
/// new connection's `AgentConnected` is observed before the old one's
/// `AgentDisconnected` (the perception server keeps the newer conn by token),
/// counting keeps the agent present instead of the disconnect evicting the
/// live connection. An entry exists only while its count is > 0.
#[derive(Resource, Default)]
pub struct PerceptionAgents(HashMap<String, u32>);

/// Per-agent ring buffer of recent perception frames, so `latency_frames` can
/// serve an older one.
#[derive(Resource, Default)]
pub struct PerceptionBuffers(HashMap<String, VecDeque<perception::PerceptionFrame>>);

/// Each agent's latest delivered perceived set (by name), for the viz debug
/// overlay. Holds the same delayed, noised detections sent on the sensor
/// pathway, so the overlay can never diverge from what the agent received.
/// Populated for every agent while Running — independent of whether anyone is
/// listening on :4002 — so a viewer can show perception with no perception
/// client connected.
#[derive(Resource, Default)]
pub struct PerceptionOverlay(HashMap<String, Vec<perception::Detection>>);

impl PerceptionOverlay {
    /// The agent's detections as viz overlay blips (empty if none).
    pub fn blips_for(&self, name: &str) -> Vec<viz::Blip> {
        self.0
            .get(name)
            .map(|dets| dets.iter().map(detection_to_blip).collect())
            .unwrap_or_default()
    }
}

fn detection_to_blip(d: &perception::Detection) -> viz::Blip {
    viz::Blip {
        id: viz::EntityId(d.id.clone()),
        position: viz::Vec3::new(d.position.x, d.position.y, d.position.z),
        kind: match d.kind {
            perception::DetectionKind::Agent => viz::DetectionKind::Agent,
            perception::DetectionKind::Static => viz::DetectionKind::Static,
        },
    }
}

/// An agent's perception config, attached at spawn. Wraps the protocol
/// `SensorSpec` so it can be an ECS component.
#[derive(Component, Default)]
pub struct Perceiver(pub protocol::scenario::SensorSpec);

/// Tracks perception connects/disconnects so the router delivers frames over
/// the wire only to agents that are listening (the overlay is computed for
/// all).
pub fn drain_perception_events(
    mut perception: ResMut<Perception>,
    mut agents: ResMut<PerceptionAgents>,
    mut buffers: ResMut<PerceptionBuffers>,
) {
    while let Ok(event) = perception.0.events.try_recv() {
        match event {
            perception::PerceptionEvent::AgentConnected { name } => {
                *agents.0.entry(name).or_insert(0) += 1;
            }
            perception::PerceptionEvent::AgentDisconnected { name } => {
                if let Some(count) = agents.0.get_mut(&name) {
                    *count -= 1;
                    if *count == 0 {
                        agents.0.remove(&name);
                        buffers.0.remove(&name);
                    }
                }
            }
        }
    }
}

/// Every `TICKS_PER_FRAME` ticks, builds every agent's simulated perception
/// (detections + impaired scalars) from ground truth, stashes it for the viz
/// overlay, and — delayed by the agent's latency — pushes it to any agent
/// listening on the perception port.
// Resources + query for the per-agent perception build; grouping them into a
// struct would obscure more than it saves.
#[allow(clippy::too_many_arguments)]
pub fn route_perception(
    mut last_emit: Local<u64>,
    perception: Res<Perception>,
    tick: Res<Tick>,
    seed: Res<PerceptionSeed>,
    bounds: Res<ArenaBounds>,
    agents: Res<PerceptionAgents>,
    mut buffers: ResMut<PerceptionBuffers>,
    mut overlay: ResMut<PerceptionOverlay>,
    query: Query<(&AgentName, &Transform, &Velocity, &Perceiver)>,
) {
    if tick.0 < *last_emit + TICKS_PER_FRAME {
        return;
    }
    *last_emit = tick.0;

    // Ground truth for every agent, gathered once.
    let all: Vec<(String, Vec3, Vec3)> = query
        .iter()
        .map(|(name, transform, velocity, _)| {
            (name.0.clone(), transform.translation, velocity.linear)
        })
        .collect();

    for (name, transform, velocity, perceiver) in &query {
        let spec = &perceiver.0;

        // Other agents, as ground-truth entities to be perceived.
        let others: Vec<PerceivedEntity> = all
            .iter()
            .filter(|(other, _, _)| other != &name.0)
            .map(|(id, position, vel)| PerceivedEntity {
                id: id.clone(),
                kind: DetectionKind::Agent,
                position: *position,
                velocity: *vel,
            })
            .collect();

        let heading = sensor_heading(transform, velocity.linear);
        let mut rng = perceiver_rng(seed.0, &name.0, tick.0);
        let detections = perceive(spec, transform.translation, heading, &others, &mut rng);

        // Impaired scalar time-to-collision: the existing sensor read over the
        // *detected* (noised, culled) obstacle set plus the walls.
        let mut obstacles: Vec<Obstacle> = detections
            .iter()
            .map(|d| Obstacle {
                position: d.position,
                velocity: d.velocity,
                radius: AGENT_RADIUS,
            })
            .collect();
        obstacles.extend(wall_obstacles(transform.translation, &bounds));
        let ttc = TimeToCollision.read(&SensorContext {
            self_position: transform.translation,
            self_velocity: velocity.linear,
            self_radius: AGENT_RADIUS,
            obstacles,
        });

        let frame = perception::PerceptionFrame {
            tick: tick.0,
            detections: detections.iter().map(to_wire).collect(),
            scalars: perception::Scalars {
                time_to_collision: ttc.is_finite().then_some(ttc),
                speed: velocity.linear.length(),
            },
        };

        // Buffer, then send the frame delayed by the agent's latency (or the
        // oldest available while the buffer is still filling).
        let buffer = buffers.0.entry(name.0.clone()).or_default();
        buffer.push_back(frame);
        while buffer.len() > MAX_BUFFER {
            buffer.pop_front();
        }
        // `latency_ticks` is a physics-tick delay, but the buffer holds one
        // entry per emitted frame (every TICKS_PER_FRAME ticks), so convert —
        // quantized down to a whole frame.
        let latency_frames = spec.latency_ticks as usize / TICKS_PER_FRAME as usize;
        let delayed = buffer.len().saturating_sub(1 + latency_frames);
        let send = buffer[delayed].clone();

        // Stash the delivered set for the viz debug overlay (every agent,
        // listener or not), then deliver over the wire only to agents actually
        // listening on the perception port.
        overlay.0.insert(name.0.clone(), send.detections.clone());
        if agents.0.contains_key(&name.0) {
            perception
                .0
                .send(&name.0, &perception::ServerToAgent::Perception(send));
        }
    }
}

/// The agent's sensor facing in the ground plane, or a zero vector when it is
/// (near-)stationary. `perceive` reads a zero-horizontal heading as "no FOV
/// cone", so a still agent senses all around — the intended design. While
/// moving, `movement::face_velocity_direction` has aligned the transform to
/// travel (and a `FullVehicle`'s yaw is real physics), so `forward` is the
/// true facing. Passing the un-updated spawn transform (facing `-Z`) here
/// instead would clamp a still agent to a `-Z` cone.
fn sensor_heading(transform: &Transform, velocity: Vec3) -> Vec3 {
    // Matches movement::face_velocity_direction's MIN_SPEED so "facing travel"
    // and "sensing a cone" switch on together.
    const MIN_SPEED: f32 = 0.05;
    if Vec3::new(velocity.x, 0.0, velocity.z).length() < MIN_SPEED {
        return Vec3::ZERO;
    }
    let forward = transform.forward();
    Vec3::new(forward.x, 0.0, forward.z)
}

/// Deterministic per-(agent, tick) RNG so simulated perception is
/// reproducible from the scenario seed.
fn perceiver_rng(base_seed: u64, name: &str, tick: u64) -> Rng {
    let mut hash = base_seed ^ 0xcbf2_9ce4_8422_2325;
    for byte in name.bytes() {
        hash = (hash ^ byte as u64).wrapping_mul(0x0000_0100_0000_01B3);
    }
    Rng::seed(hash ^ tick.wrapping_mul(0x9E37_79B9_7F4A_7C15))
}

fn to_wire(detection: &sensors::Detection) -> perception::Detection {
    perception::Detection {
        id: detection.id.clone(),
        kind: match detection.kind {
            DetectionKind::Agent => perception::DetectionKind::Agent,
            DetectionKind::Static => perception::DetectionKind::Static,
        },
        position: to_wire_vec(detection.position),
        velocity: to_wire_vec(detection.velocity),
        distance: detection.distance,
    }
}

fn to_wire_vec(v: Vec3) -> perception::Vec3 {
    perception::Vec3::new(v.x, v.y, v.z)
}
