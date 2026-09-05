//! A real simulation, stood up on ephemeral ports and stepped by hand.
//!
//! `Sim` owns the tokio runtime, the three pathway servers, and the Bevy app
//! that `server::build_app` assembles -- the same graph the binary runs, minus
//! argument parsing. It always runs in *afap* pace, where virtual time
//! advances exactly one fixed step per `update()`, so one `step()` is one
//! physics tick and the tests are deterministic.
//!
//! `TestAgent` is a thin WebSocket client on the agent pathway. Its socket
//! lives on a tokio task so the test thread can stay synchronous: `send` and
//! `recv` hand messages to and from that task over channels, and the sim is
//! stepped in between.

#![allow(dead_code)] // Each integration test binary uses a different subset.

use std::net::SocketAddr;
use std::sync::mpsc as std_mpsc;
use std::time::{Duration, Instant};

use bevy::prelude::*;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration};
use futures_util::{SinkExt, StreamExt};
use protocol::messages::{ClientMessage, ServerMessage};
use protocol::scenario::{
    AgentSlot, ArenaConfig, Embodiment, Pace, ScenarioConfig, SensorDef, TimeConfig,
};
use server::agent::{AgentName, AgentRegistry, Plan};
use server::scenario_state::{ScenarioState, Tick};
use tokio::runtime::Runtime;
use tokio::sync::mpsc as tokio_mpsc;
use tokio_tungstenite::tungstenite::Message;

/// How long `expect` will keep stepping before giving up. Generous: it only
/// ever elapses on a genuine failure.
const EXPECT_TIMEOUT: Duration = Duration::from_secs(10);

/// Wall-clock pause per step, so the async side (socket reads, the accept
/// loop) makes progress between fixed steps. The sim itself is decoupled from
/// wall-clock in afap, so this only paces the test.
const STEP_PAUSE: Duration = Duration::from_millis(1);

/// A scenario with `names` as its roster, holonomic in the flat arena.
pub fn scenario(names: &[&str]) -> ScenarioConfig {
    ScenarioConfig {
        arena: ArenaConfig {
            width: 100.0,
            depth: 100.0,
        },
        roster: names
            .iter()
            .map(|name| AgentSlot {
                name: (*name).to_string(),
                embodiment: Embodiment::Holonomic,
                sensors: vec![],
                color: None,
                scale: None,
                fmu: None,
            })
            .collect(),
        seed: 0,
        map: None,
        time: TimeConfig {
            duration: None,
            pace: Pace::Afap,
        },
    }
}

/// Equips every roster slot with one device.
pub fn with_sensor(mut config: ScenarioConfig, sensor: SensorDef) -> ScenarioConfig {
    for slot in &mut config.roster {
        slot.sensors.push(sensor.clone());
    }
    config
}

pub struct Sim {
    pub app: App,
    pub agent_addr: SocketAddr,
    pub viz_addr: SocketAddr,
    pub perception_addr: SocketAddr,
    /// Declared last so it is dropped last: the app holds handles whose
    /// background tasks belong to this runtime.
    runtime: Runtime,
}

impl Sim {
    /// Binds the three pathways on ephemeral ports and builds the app.
    pub fn new(config: ScenarioConfig) -> Self {
        Self::with_transport(config, transport_config())
    }

    /// As [`Sim::new`], with the agent pathway configured explicitly (short
    /// heartbeat intervals, say).
    pub fn with_transport(config: ScenarioConfig, transport_config: transport::Config) -> Self {
        let runtime = Runtime::new().expect("build tokio runtime");
        let (transport, viz, perception) = runtime.block_on(async {
            let transport = transport::spawn(transport_config)
                .await
                .expect("bind agent pathway");
            let viz = viz::spawn(viz::VizConfig {
                addr: ([127, 0, 0, 1], 0).into(),
            })
            .await
            .expect("bind viz pathway");
            let perception = perception::spawn(perception::PerceptionConfig {
                addr: ([127, 0, 0, 1], 0).into(),
            })
            .await
            .expect("bind perception pathway");
            (transport, viz, perception)
        });
        let (agent_addr, viz_addr, perception_addr) =
            (transport.local_addr, viz.local_addr, perception.local_addr);

        let map = server::load_map(config.map.as_deref()).expect("load map");
        let guard = runtime.enter();
        let mut app = server::build_app(server::SimConfig {
            scenario: config,
            map,
            // Always afap: one `update()` is exactly one physics tick.
            afap: true,
            transport,
            viz,
            perception,
        });
        // `run()` would do this; a stepped app has to do it itself.
        app.finish();
        app.cleanup();
        drop(guard);

        Self {
            app,
            agent_addr,
            viz_addr,
            perception_addr,
            runtime,
        }
    }

    pub fn url(&self) -> String {
        format!("ws://{}", self.agent_addr)
    }

    /// Advances the sim by `count` ticks.
    pub fn step(&mut self, count: usize) {
        let _guard = self.runtime.enter();
        for _ in 0..count {
            self.app.update();
            std::thread::sleep(STEP_PAUSE);
        }
    }

    /// Advances the sim without pausing between ticks.
    ///
    /// The pause in [`Sim::step`] is there to let the socket side make
    /// progress; a long stretch of pure driving exchanges no messages, so it
    /// only costs wall-clock. Use this for those, and `step` around anything
    /// that has to cross the wire.
    pub fn step_quiet(&mut self, count: usize) {
        let _guard = self.runtime.enter();
        for _ in 0..count {
            self.app.update();
        }
    }

    /// Steps until `f` returns `Some`, or panics after [`EXPECT_TIMEOUT`].
    /// `what` names the thing being waited for, so a timeout reads as a claim
    /// that stopped being true.
    pub fn expect<T>(&mut self, what: &str, mut f: impl FnMut(&mut Sim) -> Option<T>) -> T {
        let deadline = Instant::now() + EXPECT_TIMEOUT;
        loop {
            if let Some(value) = f(self) {
                return value;
            }
            assert!(
                Instant::now() < deadline,
                "timed out after {EXPECT_TIMEOUT:?} waiting for {what}"
            );
            self.step(1);
        }
    }

    /// Steps until `agent` receives a message `f` accepts, and returns it.
    /// Messages that arrive first and are not accepted are discarded.
    pub fn expect_message<T>(
        &mut self,
        agent: &TestAgent,
        what: &str,
        f: impl Fn(&ServerMessage) -> Option<T>,
    ) -> T {
        self.expect(what, |_| agent.try_recv().as_ref().and_then(&f))
    }

    /// Steps until `agent` receives a `ServerMessage` of the same variant as
    /// `expected`, and returns it.
    pub fn expect_variant(&mut self, agent: &TestAgent, expected: &ServerMessage) -> ServerMessage {
        let want = std::mem::discriminant(expected);
        let what = format!("{expected:?}");
        self.expect_message(agent, &what, |message| {
            (std::mem::discriminant(message) == want).then(|| message.clone())
        })
    }

    /// Steps `count` ticks and asserts `agent` received nothing at all.
    pub fn expect_silence(&mut self, agent: &TestAgent, count: usize) {
        for _ in 0..count {
            self.step(1);
            if let Some(message) = agent.try_recv() {
                panic!("expected no message, got {message:?}");
            }
        }
    }

    pub fn state(&self) -> ScenarioState {
        *self.app.world().resource::<State<ScenarioState>>().get()
    }

    pub fn tick(&self) -> u64 {
        self.app.world().resource::<Tick>().0
    }

    /// Whether Rapier is stepping. `false` is the freeze-and-inspect end state.
    pub fn physics_active(&mut self) -> bool {
        self.app
            .world_mut()
            .query_filtered::<&RapierConfiguration, With<DefaultRapierContext>>()
            .single(self.app.world())
            .expect("the default Rapier context exists")
            .physics_pipeline_active
    }

    /// The entity currently registered under `name`, if any.
    pub fn entity_of(&self, name: &str) -> Option<Entity> {
        self.app.world().resource::<AgentRegistry>().by_name(name)
    }

    /// Every agent name that currently has an entity in the world, sorted.
    pub fn spawned_agents(&mut self) -> Vec<String> {
        let mut names: Vec<String> = self
            .app
            .world_mut()
            .query::<&AgentName>()
            .iter(self.app.world())
            .map(|n| n.0.clone())
            .collect();
        names.sort();
        names
    }

    /// The plan version driving `name`'s entity.
    pub fn plan_version(&mut self, name: &str) -> u64 {
        let entity = self.entity_of(name).expect("agent has an entity");
        self.app
            .world()
            .get::<Plan>(entity)
            .expect("entity has a plan")
            .version
    }

    /// The waypoints remaining in `name`'s plan.
    pub fn plan_waypoints(&mut self, name: &str) -> Vec<protocol::messages::Waypoint> {
        let entity = self.entity_of(name).expect("agent has an entity");
        self.app
            .world()
            .get::<Plan>(entity)
            .expect("entity has a plan")
            .waypoints
            .iter()
            .copied()
            .collect()
    }

    /// How many of `name`'s waypoints it has still to reach. The plan itself
    /// is never consumed, so this is strictly less than `plan_waypoints` once
    /// the body has driven past any of them.
    pub fn plan_remaining(&mut self, name: &str) -> usize {
        let entity = self.entity_of(name).expect("agent has an entity");
        self.app
            .world()
            .get::<Plan>(entity)
            .expect("entity has a plan")
            .remaining()
            .count()
    }

    pub fn position_of(&self, name: &str) -> Vec3 {
        let entity = self.entity_of(name).expect("agent has an entity");
        self.app
            .world()
            .get::<Transform>(entity)
            .expect("entity has a transform")
            .translation
    }

    /// Connects a new agent-pathway client.
    pub fn connect(&self) -> TestAgent {
        TestAgent::connect(&self.runtime, self.agent_addr)
    }

    /// Connects and joins as `name`, waiting for the `Joined` reply.
    pub fn join(&mut self, name: &str) -> TestAgent {
        let agent = self.connect();
        agent.send(ClientMessage::Join { name: name.into() });
        self.expect_message(&agent, &format!("{name} to be joined"), |message| {
            matches!(message, ServerMessage::Joined { agent_id, .. } if agent_id.0 == name)
                .then_some(())
        });
        agent
    }

    pub fn runtime(&self) -> &Runtime {
        &self.runtime
    }
}

/// The default agent-pathway config for tests: an ephemeral loopback port.
pub fn transport_config() -> transport::Config {
    transport::Config {
        addr: ([127, 0, 0, 1], 0).into(),
        ..transport::Config::default()
    }
}

/// A WebSocket client on the agent pathway, driven from a synchronous test.
pub struct TestAgent {
    outbound: tokio_mpsc::UnboundedSender<Message>,
    inbound: std_mpsc::Receiver<ServerMessage>,
}

impl TestAgent {
    pub fn connect(runtime: &Runtime, addr: SocketAddr) -> Self {
        let (out_tx, mut out_rx) = tokio_mpsc::unbounded_channel::<Message>();
        let (in_tx, in_rx) = std_mpsc::channel::<ServerMessage>();
        let url = format!("ws://{addr}");

        let ws = runtime.block_on(async move {
            let (ws, _) = tokio_tungstenite::connect_async(url)
                .await
                .expect("agent connects");
            ws
        });

        runtime.spawn(async move {
            let mut ws = ws;
            // Tungstenite queues a pong when it reads a ping, but only writes
            // it when the sink is next driven; flush on a timer so heartbeats
            // are answered even while the client is otherwise idle.
            let mut flush = tokio::time::interval(Duration::from_millis(50));
            loop {
                tokio::select! {
                    outgoing = out_rx.recv() => match outgoing {
                        Some(message) => {
                            if ws.send(message).await.is_err() {
                                break;
                            }
                        }
                        // The `TestAgent` was dropped: close cleanly, which
                        // the server sees as a disconnect.
                        None => {
                            let _ = ws.close(None).await;
                            break;
                        }
                    },
                    incoming = ws.next() => match incoming {
                        Some(Ok(Message::Text(text))) => {
                            match serde_json::from_str::<ServerMessage>(&text) {
                                Ok(message) => {
                                    if in_tx.send(message).is_err() {
                                        break;
                                    }
                                }
                                Err(err) => panic!("server sent unparseable JSON: {err}: {text}"),
                            }
                        }
                        Some(Ok(_)) => {}
                        Some(Err(_)) | None => break,
                    },
                    _ = flush.tick() => {
                        if ws.flush().await.is_err() {
                            break;
                        }
                    }
                }
            }
        });

        Self {
            outbound: out_tx,
            inbound: in_rx,
        }
    }

    pub fn send(&self, message: ClientMessage) {
        let text = serde_json::to_string(&message).expect("ClientMessage serializes");
        let _ = self.outbound.send(Message::text(text));
    }

    /// Sends a raw frame -- for payloads a `ClientMessage` can't express.
    pub fn send_raw(&self, frame: Message) {
        let _ = self.outbound.send(frame);
    }

    pub fn try_recv(&self) -> Option<ServerMessage> {
        self.inbound.try_recv().ok()
    }

    /// Drops the socket, which the server sees as a disconnect.
    pub fn disconnect(self) {
        drop(self);
    }
}

impl Sim {
    /// Teleports an agent's body below the world floor, the state
    /// `despawn_off_road` reacts to. Driving a vehicle off a real map edge
    /// takes hundreds of ticks and a road world; this reproduces the
    /// end state directly.
    pub fn put_below_floor(&mut self, name: &str) {
        let floor = self
            .app
            .world()
            .resource::<server::transport_bridge::Floor>()
            .0;
        let entity = self.entity_of(name).expect("agent has an entity");
        let mut transform = self
            .app
            .world_mut()
            .get_mut::<Transform>(entity)
            .expect("entity has a transform");
        transform.translation.y = floor - 1.0;
    }
}

impl Sim {
    /// A clone of one component of `name`'s entity.
    pub fn component<T: Component + Clone>(&self, name: &str) -> T {
        let entity = self.entity_of(name).expect("agent has an entity");
        self.app
            .world()
            .get::<T>(entity)
            .unwrap_or_else(|| panic!("{name} has no {}", std::any::type_name::<T>()))
            .clone()
    }

    /// The delivered (delayed, noised) detections `agent` perceives through
    /// `device` -- the exact set reflex evaluation reads this frame.
    pub fn delivered(&self, agent: &str, device: &str) -> Vec<sensors::Detection> {
        self.app
            .world()
            .resource::<server::perception_router::PerceivedWorlds>()
            .delivered(agent, device)
            .to_vec()
    }

    /// Connects a client to the perception pathway as `name`.
    pub fn watch_perception(&self, name: &str) -> PerceptionClient {
        PerceptionClient::connect(&self.runtime, self.perception_addr, name)
    }
}

/// A client on the perception pathway (`:4002`), same shape as [`TestAgent`].
pub struct PerceptionClient {
    inbound: std_mpsc::Receiver<perception::PerceptionFrame>,
    _outbound: tokio_mpsc::UnboundedSender<Message>,
}

impl PerceptionClient {
    pub fn connect(runtime: &Runtime, addr: SocketAddr, name: &str) -> Self {
        let (out_tx, mut out_rx) = tokio_mpsc::unbounded_channel::<Message>();
        let (in_tx, in_rx) = std_mpsc::channel::<perception::PerceptionFrame>();
        let url = format!("ws://{addr}");
        let hello = perception::encode(&perception::AgentToServer::Hello(perception::Hello {
            agent_name: name.to_string(),
            protocol_version: perception::PROTOCOL_VERSION,
        }));

        let mut ws = runtime.block_on(async move {
            let (mut ws, _) = tokio_tungstenite::connect_async(url)
                .await
                .expect("perception client connects");
            ws.send(Message::text(hello)).await.expect("send hello");
            ws
        });

        runtime.spawn(async move {
            loop {
                tokio::select! {
                    outgoing = out_rx.recv() => match outgoing {
                        Some(message) => { if ws.send(message).await.is_err() { break } }
                        None => { let _ = ws.close(None).await; break }
                    },
                    incoming = ws.next() => match incoming {
                        Some(Ok(Message::Text(text))) => {
                            match perception::decode::<perception::ServerToAgent>(&text) {
                                Ok(perception::ServerToAgent::Perception(frame)) => {
                                    if in_tx.send(frame).is_err() {
                                        break;
                                    }
                                }
                                Err(err) => panic!("unparseable perception frame: {err}: {text}"),
                            }
                        }
                        Some(Ok(_)) => {}
                        Some(Err(_)) | None => break,
                    },
                }
            }
        });

        Self {
            inbound: in_rx,
            _outbound: out_tx,
        }
    }

    pub fn try_recv(&self) -> Option<perception::PerceptionFrame> {
        self.inbound.try_recv().ok()
    }

    /// Every frame received so far, oldest first.
    pub fn drain(&self) -> Vec<perception::PerceptionFrame> {
        std::iter::from_fn(|| self.try_recv()).collect()
    }
}

/// A reflex rule on `device`.
pub fn rule(
    device: &str,
    measure: protocol::messages::SensorKind,
    operator: protocol::messages::Operator,
    threshold: f32,
    action: protocol::messages::ReflexAction,
) -> protocol::messages::ReflexRule {
    protocol::messages::ReflexRule {
        sensor: device.to_string(),
        measure,
        operator,
        threshold,
        action,
        priority: 0,
    }
}

/// A simulated device with `range` metres of reach and no other impairment.
pub fn short_range_sensor(name: &str, range: f32, latency_ticks: u32) -> SensorDef {
    SensorDef {
        name: name.to_string(),
        source: protocol::scenario::SensorSource::Simulated,
        spec: Some(protocol::scenario::SensorSpec {
            range,
            latency_ticks,
            ..Default::default()
        }),
    }
}

impl Sim {
    /// Lifts an agent's body clear of the ground, so its wheels have nothing
    /// to cast against. Driving off a real jump takes a road built for it;
    /// this reproduces the airborne state directly.
    pub fn lift(&mut self, name: &str, height: f32) {
        let entity = self.entity_of(name).expect("agent has an entity");
        let mut transform = self
            .app
            .world_mut()
            .get_mut::<Transform>(entity)
            .expect("entity has a transform");
        transform.translation.y += height;
    }
}

impl Sim {
    /// Connects a viewer to the viz pathway and completes its handshake.
    pub fn watch_viz(&self, subscribe_debug: bool) -> VizClient {
        VizClient::connect(&self.runtime, self.viz_addr, subscribe_debug)
    }
}

/// A viewer on the viz pathway (`:4001`), same shape as [`TestAgent`]. The
/// wire is MessagePack binary frames rather than JSON text.
pub struct VizClient {
    inbound: std_mpsc::Receiver<viz::ServerToViewer>,
    _outbound: tokio_mpsc::UnboundedSender<Message>,
}

impl VizClient {
    pub fn connect(runtime: &Runtime, addr: SocketAddr, subscribe_debug: bool) -> Self {
        let (out_tx, mut out_rx) = tokio_mpsc::unbounded_channel::<Message>();
        let (in_tx, in_rx) = std_mpsc::channel::<viz::ServerToViewer>();
        let url = format!("ws://{addr}");
        let hello = viz::encode(&viz::ViewerToServer::Hello(viz::Hello {
            protocol_version: viz::PROTOCOL_VERSION,
            subscribe_debug,
        }));

        let mut ws = runtime.block_on(async move {
            let (mut ws, _) = tokio_tungstenite::connect_async(url)
                .await
                .expect("viewer connects");
            ws.send(Message::binary(hello)).await.expect("send hello");
            ws
        });

        runtime.spawn(async move {
            loop {
                tokio::select! {
                    outgoing = out_rx.recv() => match outgoing {
                        Some(message) => { if ws.send(message).await.is_err() { break } }
                        None => { let _ = ws.close(None).await; break }
                    },
                    incoming = ws.next() => match incoming {
                        Some(Ok(Message::Binary(bytes))) => {
                            match viz::decode::<viz::ServerToViewer>(&bytes) {
                                Ok(message) => {
                                    if in_tx.send(message).is_err() {
                                        break;
                                    }
                                }
                                Err(err) => panic!("undecodable viz message: {err}"),
                            }
                        }
                        Some(Ok(_)) => {}
                        Some(Err(_)) | None => break,
                    },
                }
            }
        });

        Self {
            inbound: in_rx,
            _outbound: out_tx,
        }
    }

    pub fn try_recv(&self) -> Option<viz::ServerToViewer> {
        self.inbound.try_recv().ok()
    }
}

impl Sim {
    /// Steps until `viewer` receives a message `f` accepts, and returns it.
    pub fn expect_viz<T>(
        &mut self,
        viewer: &VizClient,
        what: &str,
        f: impl Fn(&viz::ServerToViewer) -> Option<T>,
    ) -> T {
        self.expect(what, |_| viewer.try_recv().as_ref().and_then(&f))
    }
}
