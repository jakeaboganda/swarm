use std::time::Duration;

use bevy::prelude::*;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;
use viz::{decode, encode, Hello, ServerToViewer, ViewerToServer};

/// The receiving end of the viz stream, drained by a Bevy system each frame.
#[derive(Resource)]
pub struct VizStream(pub mpsc::UnboundedReceiver<ServerToViewer>);

/// Connects to the viz server and forwards decoded messages to Bevy.
/// Reconnects on failure or disconnect, so the viewer can start before the
/// sim and survive a sim restart — each reconnect yields a fresh scene-init
/// that resets the view.
pub async fn run_client(url: String, tx: mpsc::UnboundedSender<ServerToViewer>) {
    loop {
        if let Ok((mut ws, _)) = tokio_tungstenite::connect_async(url.as_str()).await {
            let hello = ViewerToServer::Hello(Hello::default());
            if ws.send(Message::binary(encode(&hello))).await.is_ok() {
                while let Some(Ok(message)) = ws.next().await {
                    if let Message::Binary(bytes) = message {
                        if let Ok(decoded) = decode::<ServerToViewer>(&bytes) {
                            // The receiver was dropped: the app is shutting
                            // down, so stop the client.
                            if tx.send(decoded).is_err() {
                                return;
                            }
                        }
                    }
                }
            }
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}
