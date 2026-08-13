use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;

use crate::message::{decode, encode, AgentToServer, Hello, ServerToAgent, PROTOCOL_VERSION};

/// Per-agent queue depth. Bounded so a slow/dead agent can't grow memory:
/// perception frames are dropped when full, which is fine because each frame
/// is a complete snapshot — the next one resupplies the truth.
const QUEUE_CAPACITY: usize = 4;
/// How long an accepted connection has to send its `Hello` before we drop it.
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);
/// Backoff after a listener error so a persistent failure doesn't spin a core.
const ACCEPT_BACKOFF: Duration = Duration::from_millis(100);

pub struct PerceptionConfig {
    pub addr: SocketAddr,
}

impl Default for PerceptionConfig {
    fn default() -> Self {
        Self {
            addr: ([0, 0, 0, 0], 4002).into(),
        }
    }
}

/// An agent subscribing to / dropping its perception stream. The sim watches
/// `AgentConnected` to know which agents to route perception to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PerceptionEvent {
    AgentConnected { name: String },
    AgentDisconnected { name: String },
}

struct AgentConn {
    queue: mpsc::Sender<String>,
    /// Distinguishes this connection from a later one that reused the same
    /// agent name, so the older task's cleanup doesn't evict the newer conn.
    token: u64,
}

type Registry = Arc<Mutex<HashMap<String, AgentConn>>>;

/// Locks the registry, recovering from a poisoned mutex — one agent task
/// panicking must never take down the sim thread's next push.
fn lock(registry: &Registry) -> MutexGuard<'_, HashMap<String, AgentConn>> {
    registry.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What the sim uses to drive the perception pathway: watch agents connect,
/// and push each its own perception frame by name.
pub struct PerceptionHandle {
    pub events: mpsc::UnboundedReceiver<PerceptionEvent>,
    pub local_addr: SocketAddr,
    registry: Registry,
}

impl PerceptionHandle {
    /// Push a perception frame to one agent by name. Lossy: dropped if the
    /// agent's queue is full or it isn't connected.
    pub fn send(&self, agent_name: &str, message: &ServerToAgent) {
        let text = encode(message);
        let registry = lock(&self.registry);
        if let Some(conn) = registry.get(agent_name) {
            let _ = conn.queue.try_send(text);
        }
    }
}

/// Binds the perception listener and starts accepting agents on a background
/// task. Must be called from within a running tokio runtime.
pub async fn spawn(config: PerceptionConfig) -> std::io::Result<PerceptionHandle> {
    let listener = TcpListener::bind(config.addr).await?;
    let local_addr = listener.local_addr()?;

    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    let next_token = Arc::new(AtomicU64::new(0));

    let accept_registry = registry.clone();
    tokio::spawn(async move {
        loop {
            let stream = match listener.accept().await {
                Ok((stream, _peer)) => stream,
                Err(_) => {
                    tokio::time::sleep(ACCEPT_BACKOFF).await;
                    continue;
                }
            };
            let token = next_token.fetch_add(1, Ordering::Relaxed);
            tokio::spawn(handle_agent(
                stream,
                token,
                event_tx.clone(),
                accept_registry.clone(),
            ));
        }
    });

    Ok(PerceptionHandle {
        events: event_rx,
        local_addr,
        registry,
    })
}

/// Reads the `Hello` handshake within the timeout, returning it, or `None` if
/// the agent never (validly) said hello.
async fn read_handshake(ws: &mut tokio_tungstenite::WebSocketStream<TcpStream>) -> Option<Hello> {
    let first = tokio::time::timeout(HANDSHAKE_TIMEOUT, ws.next())
        .await
        .ok()?;
    match first {
        Some(Ok(Message::Text(text))) => match decode::<AgentToServer>(&text) {
            Ok(AgentToServer::Hello(hello)) => Some(hello),
            Err(_) => None,
        },
        _ => None,
    }
}

/// Owns one agent connection: reads the `Hello`, registers by name, forwards
/// perception frames, and cleans up on disconnect.
async fn handle_agent(
    stream: TcpStream,
    token: u64,
    event_tx: mpsc::UnboundedSender<PerceptionEvent>,
    registry: Registry,
) {
    let Ok(mut ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };
    let Some(hello) = read_handshake(&mut ws).await else {
        return;
    };
    if hello.protocol_version != PROTOCOL_VERSION {
        return;
    }
    let name = hello.agent_name;

    let (queue_tx, mut queue_rx) = mpsc::channel::<String>(QUEUE_CAPACITY);
    lock(&registry).insert(
        name.clone(),
        AgentConn {
            queue: queue_tx,
            token,
        },
    );
    let _ = event_tx.send(PerceptionEvent::AgentConnected { name: name.clone() });

    loop {
        tokio::select! {
            frame = queue_rx.recv() => {
                let Some(text) = frame else { break };
                if ws.send(Message::text(text)).await.is_err() {
                    break;
                }
            }
            incoming = ws.next() => {
                match incoming {
                    // Agents only consume perception here; ignore anything they
                    // send after the handshake, but notice a close/error.
                    Some(Ok(Message::Close(_))) | Some(Err(_)) | None => break,
                    Some(Ok(_)) => {}
                }
            }
        }
    }

    // Only evict if we're still the current connection for this name (a
    // reconnect under the same name may have replaced us).
    {
        let mut registry = lock(&registry);
        if registry.get(&name).is_some_and(|conn| conn.token == token) {
            registry.remove(&name);
        }
    }
    let _ = event_tx.send(PerceptionEvent::AgentDisconnected { name });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::{Detection, DetectionKind, PerceptionFrame, Scalars};
    use crate::Vec3;

    type Client = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

    async fn spawn_test_server() -> PerceptionHandle {
        spawn(PerceptionConfig {
            addr: ([127, 0, 0, 1], 0).into(),
        })
        .await
        .expect("bind test perception listener")
    }

    async fn connect(addr: SocketAddr, name: &str) -> Client {
        let (mut client, _) = tokio_tungstenite::connect_async(format!("ws://{addr}"))
            .await
            .expect("connect");
        let hello = AgentToServer::Hello(Hello {
            agent_name: name.into(),
            protocol_version: PROTOCOL_VERSION,
        });
        client
            .send(Message::text(encode(&hello)))
            .await
            .expect("send hello");
        client
    }

    async fn next_frame(client: &mut Client) -> ServerToAgent {
        let received = tokio::time::timeout(Duration::from_secs(5), client.next())
            .await
            .expect("timed out")
            .expect("stream closed")
            .expect("ws error");
        let Message::Text(text) = received else {
            panic!("expected text frame")
        };
        decode::<ServerToAgent>(&text).expect("decode")
    }

    fn sample_frame(tick: u64) -> ServerToAgent {
        ServerToAgent::Perception(PerceptionFrame {
            sensor: "radar".into(),
            tick,
            detections: vec![Detection {
                id: "car-2".into(),
                kind: DetectionKind::Agent,
                position: Vec3::new(1.0, 0.0, 2.0),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                distance: 5.0,
            }],
            scalars: Scalars {
                time_to_collision: Some(2.0),
                speed: 1.0,
            },
        })
    }

    /// Waits for the given connect event so a subsequent `send` can't race the
    /// registry insert.
    async fn await_connected(handle: &mut PerceptionHandle) -> String {
        match tokio::time::timeout(Duration::from_secs(5), handle.events.recv())
            .await
            .expect("timed out")
            .expect("events closed")
        {
            PerceptionEvent::AgentConnected { name } => name,
            other => panic!("expected AgentConnected, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn routes_a_frame_to_the_named_agent() {
        let mut handle = spawn_test_server().await;
        let mut client = connect(handle.local_addr, "car-1").await;
        assert_eq!(await_connected(&mut handle).await, "car-1");

        handle.send("car-1", &sample_frame(7));
        assert_eq!(next_frame(&mut client).await, sample_frame(7));
    }

    #[tokio::test]
    async fn a_frame_for_another_name_is_not_delivered() {
        let mut handle = spawn_test_server().await;
        let mut client = connect(handle.local_addr, "car-1").await;
        await_connected(&mut handle).await;

        // Addressed to a different agent: car-1 must not receive it, but a
        // frame addressed to car-1 right after must still arrive first.
        handle.send("car-2", &sample_frame(1));
        handle.send("car-1", &sample_frame(2));
        assert_eq!(next_frame(&mut client).await, sample_frame(2));
    }

    #[tokio::test]
    async fn wrong_protocol_version_is_dropped() {
        let handle = spawn_test_server().await;
        let (mut client, _) =
            tokio_tungstenite::connect_async(format!("ws://{}", handle.local_addr))
                .await
                .expect("connect");
        let bad = AgentToServer::Hello(Hello {
            agent_name: "car-1".into(),
            protocol_version: PROTOCOL_VERSION + 1,
        });
        client
            .send(Message::text(encode(&bad)))
            .await
            .expect("send hello");
        // The server closes the connection rather than serving a schema
        // mismatch, so the next read ends the stream.
        let closed = tokio::time::timeout(Duration::from_secs(5), client.next())
            .await
            .expect("timed out");
        assert!(matches!(
            closed,
            None | Some(Ok(Message::Close(_))) | Some(Err(_))
        ));
    }
}
