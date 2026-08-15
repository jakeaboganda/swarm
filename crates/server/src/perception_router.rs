use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use protocol::scenario::{SensorDef, SensorSource};
use sensors::{
    perceive, Detection, DetectionKind, Obstacle, PerceivedEntity, Rng, Sensor, SensorContext,
    TimeToCollision,
};

use crate::agent::AgentName;
use crate::arbitration::wall_obstacles;
use crate::scenario::ArenaBounds;
use crate::scenario_state::Tick;
use crate::world::AGENT_RADIUS;

/// Recompute perceived worlds every N physics ticks — ~32 Hz. Latency is
/// counted in these frames, so this also sets the latency quantum.
const TICKS_PER_FRAME: u64 = 2;
/// Cap on a device's latency ring buffer. Bounds faithful latency at
/// `(MAX_BUFFER - 1) * TICKS_PER_FRAME` ticks; a larger `latency_ticks`
/// saturates to the oldest held frame rather than erroring.
const MAX_BUFFER: usize = 64;

/// The perception broadcaster handle, driven from Bevy systems.
#[derive(Resource)]
pub struct Perception(pub perception::PerceptionHandle);

/// Base seed for reproducible perception noise. Mixed with agent + device name
/// + tick per reading. Set from the scenario; 0 by default.
#[derive(Resource, Default)]
pub struct PerceptionSeed(pub u64);

/// Agents currently connected on the perception pathway, by name, with a
/// live-connection count. The count makes a same-name reconnect safe: if the
/// new connection's `AgentConnected` is observed before the old one's
/// `AgentDisconnected`, counting keeps the agent present instead of the
/// disconnect evicting the live connection. An entry exists only while > 0.
#[derive(Resource, Default)]
pub struct PerceptionAgents(HashMap<String, u32>);

/// One device's perception for one agent: the latency buffer of past perceived
/// sets and the currently-delivered (delayed) one. Reflex evaluation and the
/// `:4002` stream both read `delivered`, so they can never disagree.
#[derive(Default)]
struct SensorState {
    buffer: VecDeque<Vec<Detection>>,
    delivered: Vec<Detection>,
}

/// Every agent's per-device delivered perceived world, computed in the fixed
/// loop before `arbitrate` and keyed `[agent][device]`. The single source of
/// truth read by reflex evaluation, the `:4002` sender, and the overlay.
#[derive(Resource, Default)]
pub struct PerceivedWorlds(HashMap<String, HashMap<String, SensorState>>);

impl PerceivedWorlds {
    /// The delivered (delayed, noised) detections one agent perceives through
    /// one device, or empty if that device produced nothing.
    pub fn delivered(&self, agent: &str, sensor: &str) -> &[Detection] {
        self.0
            .get(agent)
            .and_then(|devices| devices.get(sensor))
            .map(|state| state.delivered.as_slice())
            .unwrap_or(&[])
    }
}

/// Each agent's perceived set as viz overlay blips (the union across its
/// simulated devices), for the debug overlay. Populated for every agent while
/// Running, independent of `:4002` listeners, so a viewer shows perception
/// with no perception client connected.
#[derive(Resource, Default)]
pub struct PerceptionOverlay(HashMap<String, Vec<viz::Blip>>);

impl PerceptionOverlay {
    /// The agent's overlay blips (empty if none).
    pub fn blips_for(&self, name: &str) -> Vec<viz::Blip> {
        self.0.get(name).cloned().unwrap_or_default()
    }
}

/// The perceiving devices the world equipped this agent with (scenario
/// `SensorDef`s). Reflex rules reference them by name; the reserved
/// `ground_truth` device is implicit and not listed here.
#[derive(Component, Default)]
pub struct Perceiver(pub Vec<SensorDef>);

/// Tracks perception connects/disconnects so the router delivers frames over
/// the wire only to agents that are listening (perceived worlds are computed
/// for all agents regardless).
pub fn drain_perception_events(
    mut perception: ResMut<Perception>,
    mut agents: ResMut<PerceptionAgents>,
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
                    }
                }
            }
        }
    }
}

/// Every `TICKS_PER_FRAME` ticks, recomputes each agent's per-device simulated
/// perception from ground truth, delivers it (latency-delayed) into
/// `PerceivedWorlds` for reflex evaluation, streams it per device to any
/// listening agent on `:4002`, and stashes the overlay blips. Runs in the
/// fixed loop before `arbitrate`, so a `Simulated` reflex reads exactly what
/// was delivered this frame.
#[allow(clippy::too_many_arguments)]
pub fn route_perception(
    mut last_emit: Local<u64>,
    perception: Res<Perception>,
    tick: Res<Tick>,
    seed: Res<PerceptionSeed>,
    bounds: Res<ArenaBounds>,
    agents: Res<PerceptionAgents>,
    mut worlds: ResMut<PerceivedWorlds>,
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
        let mut overlay_blips: Vec<viz::Blip> = Vec::new();

        // Other agents, as ground-truth entities to be perceived (and to
        // occlude each other). The same set for every one of this agent's
        // devices, so build it once.
        let others: Vec<PerceivedEntity> = all
            .iter()
            .filter(|(other, _, _)| other != &name.0)
            .map(|(id, position, vel)| PerceivedEntity {
                id: id.clone(),
                kind: DetectionKind::Agent,
                position: *position,
                velocity: *vel,
                radius: AGENT_RADIUS,
            })
            .collect();

        for def in &perceiver.0 {
            let SensorSource::Simulated = def.source else {
                continue; // ground-truth devices are exact; nothing to compute
            };
            let Some(spec) = def.spec else {
                continue; // validated present at load; skip defensively
            };

            let heading = sensor_heading(transform, velocity.linear);
            let mut rng = perceiver_rng(seed.0, &name.0, &def.name, tick.0);
            let detections = perceive(&spec, transform.translation, heading, &others, &mut rng);

            // Buffer, then deliver the set delayed by the device's latency (or
            // the oldest available while the buffer is still filling).
            let state = worlds
                .0
                .entry(name.0.clone())
                .or_default()
                .entry(def.name.clone())
                .or_default();
            state.buffer.push_back(detections);
            while state.buffer.len() > MAX_BUFFER {
                state.buffer.pop_front();
            }
            // `latency_ticks` is a physics-tick delay; the buffer holds one
            // entry per emitted frame, so convert (quantized down to a frame).
            let latency_frames = spec.latency_ticks as usize / TICKS_PER_FRAME as usize;
            let idx = state.buffer.len().saturating_sub(1 + latency_frames);
            state.delivered = state.buffer[idx].clone();
            let delivered = state.delivered.clone();

            overlay_blips.extend(delivered.iter().map(detection_to_blip));

            if agents.0.contains_key(&name.0) {
                let frame = wire_frame(
                    &def.name,
                    tick.0,
                    &delivered,
                    transform.translation,
                    velocity.linear,
                    &bounds,
                );
                perception
                    .0
                    .send(&name.0, &perception::ServerToAgent::Perception(frame));
            }
        }

        overlay.0.insert(name.0.clone(), overlay_blips);
    }
}

/// Builds one device's wire frame: its delivered detections plus the impaired
/// scalars (time-to-collision over the delivered set + walls, own speed).
fn wire_frame(
    sensor: &str,
    tick: u64,
    delivered: &[Detection],
    self_position: Vec3,
    self_velocity: Vec3,
    bounds: &ArenaBounds,
) -> perception::PerceptionFrame {
    let mut obstacles: Vec<Obstacle> = delivered
        .iter()
        .map(|d| Obstacle {
            position: d.position,
            velocity: d.velocity,
            radius: AGENT_RADIUS,
        })
        .collect();
    obstacles.extend(wall_obstacles(self_position, bounds));
    let ttc = TimeToCollision.read(&SensorContext {
        self_position,
        self_velocity,
        self_radius: AGENT_RADIUS,
        obstacles,
    });
    perception::PerceptionFrame {
        sensor: sensor.to_string(),
        tick,
        detections: delivered.iter().map(to_wire).collect(),
        scalars: perception::Scalars {
            time_to_collision: ttc.is_finite().then_some(ttc),
            speed: self_velocity.length(),
        },
    }
}

/// The agent's sensor facing in the ground plane, or a zero vector when it is
/// (near-)stationary. `perceive` reads a zero-horizontal heading as "no FOV
/// cone", so a still agent senses all around. While moving,
/// `movement::face_velocity_direction` has aligned the transform to travel, so
/// `forward` is the true facing.
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

/// Deterministic per-(agent, device, tick) RNG so simulated perception is
/// reproducible from the scenario seed and independent across devices.
fn perceiver_rng(base_seed: u64, name: &str, sensor: &str, tick: u64) -> Rng {
    let mut hash = base_seed ^ 0xcbf2_9ce4_8422_2325;
    for byte in name
        .bytes()
        .chain(std::iter::once(b'/'))
        .chain(sensor.bytes())
    {
        hash = (hash ^ byte as u64).wrapping_mul(0x0000_0100_0000_01B3);
    }
    Rng::seed(hash ^ tick.wrapping_mul(0x9E37_79B9_7F4A_7C15))
}

fn detection_to_blip(d: &Detection) -> viz::Blip {
    viz::Blip {
        id: viz::EntityId(d.id.clone()),
        position: viz::Vec3::new(d.position.x, d.position.y, d.position.z),
        kind: match d.kind {
            DetectionKind::Agent => viz::DetectionKind::Agent,
            DetectionKind::Static => viz::DetectionKind::Static,
        },
    }
}

fn to_wire(detection: &Detection) -> perception::Detection {
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
