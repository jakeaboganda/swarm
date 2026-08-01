use std::time::{Duration, Instant};

use bevy::prelude::*;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::{mpsc, watch};
use tokio_tungstenite::tungstenite::{Bytes, Message};
use viz::{decode, encode, DebugFrame, Frame, Hello, ServerToViewer, ViewerToServer};

/// How often the client pings, and how long without any traffic (message or
/// pong) before it treats the connection as dead and reconnects. The ping
/// keeps a legitimately-quiet stream (e.g. before a scenario starts) alive.
const PING_INTERVAL: Duration = Duration::from_secs(5);
const ACTIVITY_TIMEOUT: Duration = Duration::from_secs(15);

/// What Bevy drains each frame. Reliable, must-deliver messages (scene-init,
/// lifecycle events) go through an ordered queue; frames and debug frames
/// are keep-latest — only the newest matters, so buffering can't grow.
#[derive(Resource)]
pub struct VizStream {
    pub reliable: mpsc::UnboundedReceiver<ServerToViewer>,
    pub frame: watch::Receiver<Option<Frame>>,
    pub debug: watch::Receiver<Option<DebugFrame>>,
}

/// The sending ends, owned by the client task.
struct Senders {
    reliable: mpsc::UnboundedSender<ServerToViewer>,
    frame: watch::Sender<Option<Frame>>,
    debug: watch::Sender<Option<DebugFrame>>,
}

/// Builds the paired channels: the `VizStream` resource for Bevy and the
/// `Senders` for the client task.
pub fn channels() -> (VizStream, ClientChannels) {
    let (reliable_tx, reliable_rx) = mpsc::unbounded_channel();
    let (frame_tx, frame_rx) = watch::channel(None);
    let (debug_tx, debug_rx) = watch::channel(None);
    (
        VizStream {
            reliable: reliable_rx,
            frame: frame_rx,
            debug: debug_rx,
        },
        ClientChannels(Senders {
            reliable: reliable_tx,
            frame: frame_tx,
            debug: debug_tx,
        }),
    )
}

/// Opaque handle carrying the client task's sending ends.
pub struct ClientChannels(Senders);

/// Routes a decoded message to the right channel. Returns `false` if Bevy
/// has gone away (the app is shutting down).
fn route(senders: &Senders, message: ServerToViewer) -> bool {
    match message {
        ServerToViewer::Frame(frame) => senders.frame.send(Some(frame)).is_ok(),
        ServerToViewer::DebugFrame(debug) => senders.debug.send(Some(debug)).is_ok(),
        // Reliable delivery failing means the receiver was dropped.
        other => senders.reliable.send(other).is_ok(),
    }
}

/// Connects to the viz server and forwards messages to Bevy. Reconnects on
/// failure or disconnect (each reconnect yields a fresh scene-init that
/// resets the view), with a heartbeat so a half-open connection is detected
/// and retried rather than wedging the viewer.
pub async fn run_client(url: String, channels: ClientChannels) {
    let senders = channels.0;
    loop {
        if let Ok((mut ws, _)) = tokio_tungstenite::connect_async(url.as_str()).await {
            let hello = ViewerToServer::Hello(Hello::default());
            if ws.send(Message::binary(encode(&hello))).await.is_ok()
                && serve(&mut ws, &senders).await.is_break()
            {
                return; // Bevy went away.
            }
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}

/// Pumps one connection until it dies. `Break` means Bevy is gone (stop the
/// whole client); `Continue` means reconnect.
async fn serve(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    senders: &Senders,
) -> std::ops::ControlFlow<()> {
    let mut ping = tokio::time::interval(PING_INTERVAL);
    let mut last_activity = Instant::now();
    loop {
        tokio::select! {
            _ = ping.tick() => {
                if last_activity.elapsed() > ACTIVITY_TIMEOUT {
                    return std::ops::ControlFlow::Continue(()); // dead peer, reconnect
                }
                if ws.send(Message::Ping(Bytes::new())).await.is_err() {
                    return std::ops::ControlFlow::Continue(());
                }
            }
            incoming = ws.next() => {
                match incoming {
                    Some(Ok(message)) => {
                        last_activity = Instant::now();
                        if let Message::Binary(bytes) = message {
                            if let Ok(decoded) = decode::<ServerToViewer>(&bytes) {
                                if !route(senders, decoded) {
                                    return std::ops::ControlFlow::Break(());
                                }
                            }
                        } else if matches!(message, Message::Close(_)) {
                            return std::ops::ControlFlow::Continue(());
                        }
                    }
                    _ => return std::ops::ControlFlow::Continue(()),
                }
            }
        }
    }
}
