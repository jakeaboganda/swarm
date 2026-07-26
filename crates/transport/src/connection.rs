use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use protocol::messages::{ClientMessage, ServerMessage};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::{Bytes, Message};

use crate::types::{ConnectionEvent, ConnectionId, Inbound};

type OutboundRegistry = Arc<Mutex<HashMap<ConnectionId, mpsc::UnboundedSender<ServerMessage>>>>;

/// Owns one connection end-to-end: accepting the WebSocket handshake,
/// heartbeat ping/pong with timeout detection, JSON (de)serialization, and
/// forwarding parsed messages / lifecycle events to the shared channels.
/// Runs as a single task per connection so heartbeat, reads, and writes
/// never fight over ownership of the socket.
pub async fn handle_connection(
    stream: TcpStream,
    id: ConnectionId,
    heartbeat_interval: Duration,
    heartbeat_timeout: Duration,
    inbound_tx: mpsc::UnboundedSender<Inbound>,
    event_tx: mpsc::UnboundedSender<ConnectionEvent>,
    outbound: OutboundRegistry,
) {
    let Ok(mut ws) = tokio_tungstenite::accept_async(stream).await else {
        return;
    };

    let (out_tx, mut out_rx) = mpsc::unbounded_channel::<ServerMessage>();
    outbound
        .lock()
        .expect("outbound registry poisoned")
        .insert(id, out_tx);
    let _ = event_tx.send(ConnectionEvent::Connected(id));

    let mut heartbeat = tokio::time::interval(heartbeat_interval);
    let mut last_pong = Instant::now();

    loop {
        tokio::select! {
            _ = heartbeat.tick() => {
                if last_pong.elapsed() > heartbeat_timeout {
                    break;
                }
                if ws.send(Message::Ping(Bytes::new())).await.is_err() {
                    break;
                }
            }
            outgoing = out_rx.recv() => {
                let Some(message) = outgoing else { break };
                let text = serde_json::to_string(&message).expect("ServerMessage always serializes");
                if ws.send(Message::text(text)).await.is_err() {
                    break;
                }
            }
            incoming = ws.next() => {
                match incoming {
                    Some(Ok(Message::Text(text))) => {
                        match serde_json::from_str::<ClientMessage>(&text) {
                            Ok(message) => {
                                let _ = inbound_tx.send(Inbound { connection: id, message });
                            }
                            Err(err) => {
                                // Malformed message: reply with an error and
                                // keep the connection open. Closing it here
                                // would be indistinguishable from a real
                                // disconnect, which ends the whole scenario.
                                let reply = ServerMessage::Error { message: err.to_string() };
                                let text = serde_json::to_string(&reply).expect("ServerMessage always serializes");
                                if ws.send(Message::text(text)).await.is_err() {
                                    break;
                                }
                            }
                        }
                    }
                    Some(Ok(Message::Pong(_))) => {
                        last_pong = Instant::now();
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(_)) => {}
                    Some(Err(_)) => break,
                }
            }
        }
    }

    outbound
        .lock()
        .expect("outbound registry poisoned")
        .remove(&id);
    let _ = event_tx.send(ConnectionEvent::Disconnected(id));
}
