use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use futures_util::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;

use crate::message::{encode, Hello, ServerToViewer, ViewerToServer};

/// Per-viewer outbound queue depth. Bounded so a slow or dead viewer can't
/// grow memory — frames are dropped when it's full, which is fine for viz:
/// the next frame is a complete snapshot that resupplies the truth.
const OUTBOUND_CAPACITY: usize = 8;

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
    sender: mpsc::Sender<Vec<u8>>,
    subscribe_debug: bool,
}

type Registry = Arc<Mutex<HashMap<ViewerId, ViewerConn>>>;

/// What the sim uses to drive the broadcaster: watch viewers come and go,
/// and push messages out to all / one / debug-subscribers.
pub struct VizHandle {
    pub events: mpsc::UnboundedReceiver<VizEvent>,
    pub local_addr: SocketAddr,
    registry: Registry,
}

impl VizHandle {
    /// Sends to every connected viewer. Use for scene-layer messages.
    pub fn broadcast(&self, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = self.registry.lock().expect("viz registry poisoned");
        for conn in registry.values() {
            let _ = conn.sender.try_send(bytes.clone());
        }
    }

    /// Sends only to viewers that opted into the debug layer.
    pub fn broadcast_debug(&self, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = self.registry.lock().expect("viz registry poisoned");
        for conn in registry.values().filter(|c| c.subscribe_debug) {
            let _ = conn.sender.try_send(bytes.clone());
        }
    }

    /// Sends to a single viewer (e.g. the scene-init on connect).
    pub fn send(&self, id: ViewerId, message: &ServerToViewer) {
        let bytes = encode(message);
        let registry = self.registry.lock().expect("viz registry poisoned");
        if let Some(conn) = registry.get(&id) {
            let _ = conn.sender.try_send(bytes);
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
            let Ok((stream, _peer)) = listener.accept().await else {
                continue;
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

/// Owns one viewer connection: reads the `Hello` handshake, registers the
/// connection, forwards outbound frames, and cleans up on disconnect.
async fn handle_viewer(
    stream: TcpStream,
    id: ViewerId,
    event_tx: mpsc::UnboundedSender<VizEvent>,
    registry: Registry,
) {
    let Ok(mut ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };

    // Handshake: the first message should be a Hello. Default to "wants
    // everything" if it's missing or unparseable rather than rejecting.
    let subscribe_debug = match ws.next().await {
        Some(Ok(Message::Binary(bytes))) => {
            match crate::message::decode::<ViewerToServer>(&bytes) {
                Ok(ViewerToServer::Hello(hello)) => hello.subscribe_debug,
                Err(_) => Hello::default().subscribe_debug,
            }
        }
        Some(Ok(_)) => Hello::default().subscribe_debug,
        _ => return, // closed/errored before saying hello
    };

    let (out_tx, mut out_rx) = mpsc::channel::<Vec<u8>>(OUTBOUND_CAPACITY);
    registry.lock().expect("viz registry poisoned").insert(
        id,
        ViewerConn {
            sender: out_tx,
            subscribe_debug,
        },
    );
    let _ = event_tx.send(VizEvent::ViewerConnected {
        id,
        subscribe_debug,
    });

    loop {
        tokio::select! {
            outgoing = out_rx.recv() => {
                let Some(bytes) = outgoing else { break };
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

    registry.lock().expect("viz registry poisoned").remove(&id);
    let _ = event_tx.send(VizEvent::ViewerDisconnected { id });
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::frame::Frame;
    use crate::message::decode;

    async fn spawn_test_server() -> VizHandle {
        spawn(VizConfig {
            addr: ([127, 0, 0, 1], 0).into(),
        })
        .await
        .expect("bind test viz listener")
    }

    async fn connect(
        addr: SocketAddr,
        hello: Hello,
    ) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>> {
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

    #[tokio::test]
    async fn viewer_connect_emits_event_and_receives_broadcast() {
        let mut handle = spawn_test_server().await;
        let mut client = connect(
            handle.local_addr,
            Hello {
                subscribe_debug: true,
            },
        )
        .await;

        let event = tokio::time::timeout(Duration::from_secs(1), handle.events.recv())
            .await
            .expect("timed out")
            .expect("events closed");
        let VizEvent::ViewerConnected {
            id,
            subscribe_debug,
        } = event
        else {
            panic!("expected ViewerConnected, got {event:?}");
        };
        assert!(subscribe_debug);

        let frame = ServerToViewer::Frame(Frame {
            tick: 7,
            entities: vec![],
        });
        handle.send(id, &frame);

        let received = tokio::time::timeout(Duration::from_secs(1), client.next())
            .await
            .expect("timed out")
            .expect("stream closed")
            .expect("ws error");
        let Message::Binary(bytes) = received else {
            panic!("expected binary frame")
        };
        assert_eq!(decode::<ServerToViewer>(&bytes).expect("decode"), frame);
    }

    #[tokio::test]
    async fn debug_broadcast_skips_non_subscribers() {
        let mut handle = spawn_test_server().await;
        let mut watcher = connect(
            handle.local_addr,
            Hello {
                subscribe_debug: false,
            },
        )
        .await;

        // Wait for the connection to register.
        tokio::time::timeout(Duration::from_secs(1), handle.events.recv())
            .await
            .expect("timed out")
            .expect("events closed");

        // A debug broadcast should not reach a non-subscriber, but a plain
        // broadcast should. Send debug first, then scene; the watcher must
        // receive only the scene message.
        let debug = ServerToViewer::DebugFrame(crate::frame::DebugFrame {
            tick: 1,
            entities: vec![],
        });
        let scene = ServerToViewer::Frame(Frame {
            tick: 2,
            entities: vec![],
        });
        handle.broadcast_debug(&debug);
        handle.broadcast(&scene);

        let received = tokio::time::timeout(Duration::from_secs(1), watcher.next())
            .await
            .expect("timed out")
            .expect("stream closed")
            .expect("ws error");
        let Message::Binary(bytes) = received else {
            panic!("expected binary frame")
        };
        // First (and only pending) message must be the scene frame, proving
        // the debug frame was filtered out.
        assert_eq!(decode::<ServerToViewer>(&bytes).expect("decode"), scene);
    }
}
