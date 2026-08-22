use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use protocol::messages::ServerMessage;
use tokio::net::TcpListener;
use tokio::sync::mpsc;

use crate::connection::handle_connection;
use crate::types::{ConnectionEvent, ConnectionId, Inbound};

/// Per-connection inbound queue capacity. Bounded so a flooding client
/// applies backpressure to its own socket (and eventually trips its
/// heartbeat) rather than growing server memory without limit; cooperative
/// agents never approach it.
const INBOUND_CAPACITY: usize = 256;

pub struct Config {
    pub addr: SocketAddr,
    pub heartbeat_interval: Duration,
    pub heartbeat_timeout: Duration,
    /// How long an accepted TCP peer has to complete the WebSocket upgrade
    /// before it is dropped. Without it, a peer that opens a socket and goes
    /// silent holds a task for the life of the process -- the slowloris case
    /// `viz` and `perception` already guard against.
    pub handshake_timeout: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            addr: ([0, 0, 0, 0], 4000).into(),
            heartbeat_interval: Duration::from_secs(2),
            heartbeat_timeout: Duration::from_secs(6),
            handshake_timeout: Duration::from_secs(5),
        }
    }
}

type OutboundRegistry = Arc<Mutex<HashMap<ConnectionId, mpsc::UnboundedSender<ServerMessage>>>>;

/// A connection task's timing policy, grouped so it travels as one value.
#[derive(Clone, Copy)]
pub(crate) struct Timings {
    pub heartbeat_interval: Duration,
    pub heartbeat_timeout: Duration,
    pub handshake_timeout: Duration,
}

/// Owns the receiving ends of transport's channels and lets `server` push
/// messages back out to specific connections. Constructed once by
/// [`spawn`]; polled from Bevy systems each tick.
pub struct TransportHandle {
    pub inbound: mpsc::Receiver<Inbound>,
    pub events: mpsc::UnboundedReceiver<ConnectionEvent>,
    pub local_addr: SocketAddr,
    outbound: OutboundRegistry,
}

impl TransportHandle {
    pub fn send(&self, connection: ConnectionId, message: ServerMessage) {
        let outbound = self.outbound.lock().expect("outbound registry poisoned");
        if let Some(sender) = outbound.get(&connection) {
            let _ = sender.send(message);
        }
    }

    pub fn broadcast(&self, message: ServerMessage) {
        let outbound = self.outbound.lock().expect("outbound registry poisoned");
        for sender in outbound.values() {
            let _ = sender.send(message.clone());
        }
    }
}

/// Binds the listening socket and starts the WebSocket accept loop on a
/// background tokio task, returning a handle for `server` to drain
/// messages from / send messages through. Must be called from within a
/// running tokio runtime.
pub async fn spawn(config: Config) -> std::io::Result<TransportHandle> {
    let listener = TcpListener::bind(config.addr).await?;
    let local_addr = listener.local_addr()?;

    let (inbound_tx, inbound_rx) = mpsc::channel(INBOUND_CAPACITY);
    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let outbound: OutboundRegistry = Arc::new(Mutex::new(HashMap::new()));
    let next_id = Arc::new(AtomicU64::new(0));
    let timings = Timings {
        heartbeat_interval: config.heartbeat_interval,
        heartbeat_timeout: config.heartbeat_timeout,
        handshake_timeout: config.handshake_timeout,
    };

    let accept_outbound = outbound.clone();
    tokio::spawn(async move {
        loop {
            let Ok((stream, _peer)) = listener.accept().await else {
                continue;
            };
            let id = ConnectionId(next_id.fetch_add(1, Ordering::Relaxed));
            tokio::spawn(handle_connection(
                stream,
                id,
                timings,
                inbound_tx.clone(),
                event_tx.clone(),
                accept_outbound.clone(),
            ));
        }
    });

    Ok(TransportHandle {
        inbound: inbound_rx,
        events: event_rx,
        local_addr,
        outbound,
    })
}
