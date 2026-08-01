use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use protocol::messages::{ClientMessage, ServerMessage};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::{Bytes, Message};
use tokio_tungstenite::WebSocketStream;

use crate::types::{ConnectionEvent, ConnectionId, Inbound};

type OutboundRegistry = Arc<Mutex<HashMap<ConnectionId, mpsc::UnboundedSender<ServerMessage>>>>;

/// Sends a `ServerMessage::Error` text frame to the client.
async fn send_error(
    ws: &mut WebSocketStream<TcpStream>,
    message: String,
) -> Result<(), tokio_tungstenite::tungstenite::Error> {
    let reply = ServerMessage::Error { message };
    let text = serde_json::to_string(&reply).expect("ServerMessage always serializes");
    ws.send(Message::text(text)).await
}

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
    inbound_tx: mpsc::Sender<Inbound>,
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

    // Delay the first tick by a full interval — `interval` would otherwise
    // fire immediately and ping the client the instant it connects.
    let mut heartbeat = tokio::time::interval_at(
        tokio::time::Instant::now() + heartbeat_interval,
        heartbeat_interval,
    );
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
                                // Bounded channel: awaiting here applies
                                // backpressure to a flooding client rather
                                // than growing memory. An error means the
                                // server dropped its receiver — shut down.
                                if inbound_tx.send(Inbound { connection: id, message }).await.is_err() {
                                    break;
                                }
                            }
                            Err(err) => {
                                // Malformed message: reply with an error and
                                // keep the connection open. Closing it here
                                // would be indistinguishable from a real
                                // disconnect, which ends the whole scenario.
                                if send_error(&mut ws, err.to_string()).await.is_err() {
                                    break;
                                }
                            }
                        }
                    }
                    Some(Ok(Message::Binary(_))) => {
                        // Same contract as malformed text: the protocol is
                        // JSON text frames only, but a stray binary frame is
                        // not a reason to end the scenario.
                        if send_error(&mut ws, "binary frames are not supported; send JSON text".into())
                            .await
                            .is_err()
                        {
                            break;
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
