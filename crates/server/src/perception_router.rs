use std::collections::{HashMap, HashSet, VecDeque};

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
/// Cap on the per-agent latency ring buffer.
const MAX_BUFFER: usize = 64;

/// The perception broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Perception(pub perception::PerceptionHandle);

/// Base seed for reproducible perception noise. Mixed with agent name + tick
/// per reading. Set from the scenario; 0 by default.
#[derive(Resource, Default)]
pub struct PerceptionSeed(pub u64);

/// Agents currently connected on the perception pathway (by name).
#[derive(Resource, Default)]
pub struct PerceptionAgents(HashSet<String>);

/// Per-agent ring buffer of recent perception frames, so `latency_frames` can
/// serve an older one.
#[derive(Resource, Default)]
pub struct PerceptionBuffers(HashMap<String, VecDeque<perception::PerceptionFrame>>);

/// An agent's perception config, attached at spawn. Wraps the protocol
/// `SensorSpec` so it can be an ECS component.
#[derive(Component, Default)]
pub struct Perceiver(pub protocol::scenario::SensorSpec);

/// Tracks perception connects/disconnects so the router only produces frames
/// for agents that are listening.
pub fn drain_perception_events(
    mut perception: ResMut<Perception>,
    mut agents: ResMut<PerceptionAgents>,
    mut buffers: ResMut<PerceptionBuffers>,
) {
    while let Ok(event) = perception.0.events.try_recv() {
        match event {
            perception::PerceptionEvent::AgentConnected { name } => {
                agents.0.insert(name);
            }
            perception::PerceptionEvent::AgentDisconnected { name } => {
                agents.0.remove(&name);
                buffers.0.remove(&name);
            }
        }
    }
}

/// Every `TICKS_PER_FRAME` ticks, builds each connected agent's simulated
/// perception (detections + impaired scalars) from ground truth and pushes it
/// — delayed by the agent's latency.
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
        if !agents.0.contains(&name.0) {
            continue;
        }
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

        let heading = flatten_forward(transform);
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
        let delayed = buffer.len().saturating_sub(1 + spec.latency_ticks as usize);
        let send = buffer[delayed].clone();
        perception
            .0
            .send(&name.0, &perception::ServerToAgent::Perception(send));
    }
}

/// Unit heading in the ground plane from an entity's transform, or +X if it
/// has no horizontal facing (so FOV still has a reference).
fn flatten_forward(transform: &Transform) -> Vec3 {
    let forward = transform.forward();
    Vec3::new(forward.x, 0.0, forward.z).normalize_or(Vec3::X)
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
