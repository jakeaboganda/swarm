use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;

use crate::message::{encode, Hello, ServerToViewer, ViewerToServer, PROTOCOL_VERSION};

/// Per-viewer *lossy* queue depth. Bounded so a slow or dead viewer can't
/// grow memory — frames are dropped when it's full, which is fine for the
/// scene/debug frame stream: the next frame is a complete snapshot that
/// resupplies the truth. Must-deliver messages use the reliable channel
/// instead.
const LOSSY_CAPACITY: usize = 8;
/// How long a newly accepted connection has to send its `Hello` before we
/// drop it. Guards against slowloris clients that connect and go silent.
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);
/// Backoff after a listener error so a persistent failure (e.g. EMFILE)
/// doesn't spin a core.
const ACCEPT_BACKOFF: Duration = Duration::from_millis(100);

/// Identifies one connected viewer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ViewerId(pub u64);

pub struct VizConfig {
    pub addr: SocketAddr,
}

impl Default for VizConfig {
    fn default() -> Self {
        Self {
            addr: ([0, 0, 0, 0], 4001).into(),
        }
    }
}

/// A viewer connecting or leaving. The sim watches `ViewerConnected` to send
/// that viewer a fresh scene-init so it catches up before the live stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VizEvent {
    ViewerConnected { id: ViewerId, subscribe_debug: bool },
    ViewerDisconnected { id: ViewerId },
}

struct ViewerConn {
    /// Must-deliver messages (scene-init, lifecycle events). Unbounded, so
    /// they are never dropped; low-rate, so growth is bounded in practice
    /// and a truly dead viewer is reclaimed when its socket errors.
    reliable: mpsc::UnboundedSender<Vec<u8>>,
    /// Droppable messages (frames, debug frames). Bounded; drops on full.
    lossy: mpsc::Sender<Vec<u8>>,
    subscribe_debug: bool,
}

type Registry = Arc<Mutex<HashMap<ViewerId, ViewerConn>>>;

/// Locks the registry, recovering from a poisoned mutex. A viewer task
/// panicking must never take down the sim thread on its next broadcast.
fn lock(registry: &Registry) -> MutexGuard<'_, HashMap<ViewerId, ViewerConn>> {
    registry.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What the sim uses to drive the broadcaster: watch viewers come and go,
/// and push messages out over the reliable or lossy path.
pub struct VizHandle {
    pub events: mpsc::UnboundedReceiver<VizEvent>,
    pub local_addr: SocketAddr,
    registry: Registry,
}

impl VizHandle {
    /// Reliable broadcast to every viewer — for must-deliver messages
    /// (lifecycle events: spawn/despawn, scenario-state changes).
    pub fn broadcast_reliable(&self, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = lock(&self.registry);
        for conn in registry.values() {
            let _ = conn.reliable.send(bytes.clone());
        }
    }

    /// Reliable send to one viewer — for the scene-init that catches a
    /// newly connected viewer up. Never dropped under frame backpressure.
    pub fn send_reliable(&self, id: ViewerId, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = lock(&self.registry);
        if let Some(conn) = registry.get(&id) {
            let _ = conn.reliable.send(bytes);
        }
    }

    /// Lossy broadcast to every viewer — for the scene frame stream.
    /// Dropped for any viewer whose queue is full.
    pub fn broadcast_frame(&self, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = lock(&self.registry);
        for conn in registry.values() {
            let _ = conn.lossy.try_send(bytes.clone());
        }
    }

    /// Lossy broadcast to viewers that opted into the debug layer.
    pub fn broadcast_debug(&self, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = lock(&self.registry);
        for conn in registry.values().filter(|c| c.subscribe_debug) {
            let _ = conn.lossy.try_send(bytes.clone());
        }
    }
}

/// Binds the viz listener and starts accepting viewers on a background
/// task. Must be called from within a running tokio runtime.
pub async fn spawn(config: VizConfig) -> std::io::Result<VizHandle> {
    let listener = TcpListener::bind(config.addr).await?;
    let local_addr = listener.local_addr()?;

    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    let next_id = Arc::new(AtomicU64::new(0));

    let accept_registry = registry.clone();
    tokio::spawn(async move {
        loop {
            let stream = match listener.accept().await {
                Ok((stream, _peer)) => stream,
                // Back off rather than busy-spin on a persistent error.
                Err(_) => {
                    tokio::time::sleep(ACCEPT_BACKOFF).await;
                    continue;
                }
            };
            let id = ViewerId(next_id.fetch_add(1, Ordering::Relaxed));
            tokio::spawn(handle_viewer(
                stream,
                id,
                event_tx.clone(),
                accept_registry.clone(),
            ));
        }
    });

    Ok(VizHandle {
        events: event_rx,
        local_addr,
        registry,
    })
}

/// Reads the `Hello` handshake within the timeout, returning it, or `None`
/// if the viewer never (validly) said hello.
async fn read_handshake(ws: &mut tokio_tungstenite::WebSocketStream<TcpStream>) -> Option<Hello> {
    let first = tokio::time::timeout(HANDSHAKE_TIMEOUT, ws.next())
        .await
        .ok()?;
    match first {
        Some(Ok(Message::Binary(bytes))) => {
            match crate::message::decode::<ViewerToServer>(&bytes) {
                Ok(ViewerToServer::Hello(hello)) => Some(hello),
                // Reachable/garbled hello: fall back to defaults.
                Err(_) => Some(Hello::default()),
            }
        }
        Some(Ok(_)) => Some(Hello::default()),
        _ => None, // closed/errored before saying hello
    }
}

/// Owns one viewer connection: reads the `Hello` handshake, registers the
/// connection, forwards reliable + lossy messages, and cleans up on
/// disconnect.
async fn handle_viewer(
    stream: TcpStream,
    id: ViewerId,
    event_tx: mpsc::UnboundedSender<VizEvent>,
    registry: Registry,
) {
    let Ok(mut ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };
    let Some(hello) = read_handshake(&mut ws).await else {
        return;
    };
    // Refuse a viewer speaking a schema we can't serve — better than
    // streaming bytes it will misdecode.
    if hello.protocol_version != PROTOCOL_VERSION {
        return;
    }
    let subscribe_debug = hello.subscribe_debug;

    let (reliable_tx, mut reliable_rx) = mpsc::unbounded_channel::<Vec<u8>>();
    let (lossy_tx, mut lossy_rx) = mpsc::channel::<Vec<u8>>(LOSSY_CAPACITY);
    lock(&registry).insert(
        id,
        ViewerConn {
            reliable: reliable_tx,
            lossy: lossy_tx,
            subscribe_debug,
        },
    );
    let _ = event_tx.send(VizEvent::ViewerConnected {
        id,
        subscribe_debug,
    });

    loop {
        tokio::select! {
            // Prefer reliable messages: a scene-init/lifecycle event must go
            // out before the frames that assume it.
            biased;
            reliable = reliable_rx.recv() => {
                let Some(bytes) = reliable else { break };
                if ws.send(Message::binary(bytes)).await.is_err() {
                    break;
                }
            }
            lossy = lossy_rx.recv() => {
                let Some(bytes) = lossy else { break };
                if ws.send(Message::binary(bytes)).await.is_err() {
                    break;
                }
            }
            incoming = ws.next() => {
                match incoming {
                    // Viewers are observers; ignore anything they send after
                    // the handshake, but notice a close/error.
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(_)) => break,
                    Some(Ok(_)) => {}
                }
            }
        }
    }

    lock(&registry).remove(&id);
    let _ = event_tx.send(VizEvent::ViewerDisconnected { id });
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::frame::Frame;
    use crate::message::decode;
    use crate::scene::{ArenaBounds, ScenarioState, SceneInit};

    async fn spawn_test_server() -> VizHandle {
        spawn(VizConfig {
            addr: ([127, 0, 0, 1], 0).into(),
        })
        .await
        .expect("bind test viz listener")
    }

    type Client = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

    async fn connect(addr: SocketAddr, hello: Hello) -> Client {
        let url = format!("ws://{addr}");
        let (mut client, _) = tokio_tungstenite::connect_async(url)
            .await
            .expect("connect");
        client
            .send(Message::binary(encode(&ViewerToServer::Hello(hello))))
            .await
            .expect("send hello");
        client
    }

    async fn next_message(client: &mut Client) -> ServerToViewer {
        let received = tokio::time::timeout(Duration::from_secs(1), client.next())
            .await
            .expect("timed out")
            .expect("stream closed")
            .expect("ws error");
        let Message::Binary(bytes) = received else {
            panic!("expected binary frame")
        };
        decode::<ServerToViewer>(&bytes).expect("decode")
    }

    fn scene_init() -> ServerToViewer {
        ServerToViewer::SceneInit(SceneInit {
            protocol_version: PROTOCOL_VERSION,
            tick: 0,
            state: ScenarioState::WaitingForRoster,
            arena: ArenaBounds {
                width: 50.0,
                depth: 50.0,
            },
            entities: vec![],
        })
    }

    #[tokio::test]
    async fn viewer_connect_emits_event_and_receives_broadcast() {
        let mut handle = spawn_test_server().await;
        let mut client = connect(handle.local_addr, Hello::default()).await;

        let event = tokio::time::timeout(Duration::from_secs(1), handle.events.recv())
            .await
            .expect("timed out")
            .expect("events closed");
        let VizEvent::ViewerConnected {
            subscribe_debug, ..
        } = event
        else {
            panic!("expected ViewerConnected, got {event:?}");
        };
        assert!(subscribe_debug);

        let frame = ServerToViewer::Frame(Frame {
            tick: 7,
            entities: vec![],
        });
        handle.broadcast_frame(&frame);
        assert_eq!(next_message(&mut client).await, frame);
    }

    #[tokio::test]
    async fn version_mismatch_is_rejected() {
        let mut handle = spawn_test_server().await;
        let _client = connect(
            handle.local_addr,
            Hello {
                protocol_version: PROTOCOL_VERSION + 1,
                subscribe_debug: true,
            },
        )
        .await;

        // A mismatched viewer must never register, so no event arrives.
        let event = tokio::time::timeout(Duration::from_millis(300), handle.events.recv()).await;
        assert!(event.is_err(), "mismatched viewer should not connect");
    }

    #[tokio::test]
    async fn debug_broadcast_skips_non_subscribers() {
        let mut handle = spawn_test_server().await;
        let mut watcher = connect(
            handle.local_addr,
            Hello {
                protocol_version: PROTOCOL_VERSION,
                subscribe_debug: false,
            },
        )
        .await;
        handle.events.recv().await;

        let debug = ServerToViewer::DebugFrame(crate::frame::DebugFrame {
            tick: 1,
            entities: vec![],
        });
        let scene = ServerToViewer::Frame(Frame {
            tick: 2,
            entities: vec![],
        });
        handle.broadcast_debug(&debug);
        handle.broadcast_frame(&scene);

        // The non-subscriber's first message must be the scene frame,
        // proving the debug frame was filtered out.
        assert_eq!(next_message(&mut watcher).await, scene);
    }

    #[tokio::test]
    async fn reliable_send_survives_frame_backpressure() {
        let mut handle = spawn_test_server().await;
        let mut client = connect(handle.local_addr, Hello::default()).await;
        let VizEvent::ViewerConnected { id, .. } = handle.events.recv().await.unwrap() else {
            panic!("expected ViewerConnected");
        };

        // Flood far more frames than the lossy queue can hold, without the
        // client reading, then send a reliable scene-init. The scene-init
        // must still arrive despite the frame flood being dropped.
        for tick in 0..100 {
            handle.broadcast_frame(&ServerToViewer::Frame(Frame {
                tick,
                entities: vec![],
            }));
        }
        handle.send_reliable(id, &scene_init());

        // Drain until we see the scene-init (biased select delivers it ahead
        // of queued frames).
        for _ in 0..LOSSY_CAPACITY + 2 {
            if matches!(
                next_message(&mut client).await,
                ServerToViewer::SceneInit(_)
            ) {
                return;
            }
        }
        panic!("reliable scene-init was not delivered");
    }
}
