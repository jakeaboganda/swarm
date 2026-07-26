//! Runs the transport server and a client in one process: the client sends
//! a Join, the server observes it on its handle and replies with Joined.
//! Demonstrates the full inbound/outbound round trip without the Bevy
//! server.
//!
//! Run: `cargo run -p transport --example echo_server`

use futures_util::{SinkExt, StreamExt};
use protocol::messages::{AgentId, ClientMessage, ServerMessage};
use protocol::Vec3;
use tokio_tungstenite::tungstenite::Message;
use transport::{Config, ConnectionEvent};

#[tokio::main]
async fn main() {
    let config = Config {
        addr: ([127, 0, 0, 1], 0).into(),
        ..Config::default()
    };
    let mut handle = transport::spawn(config).await.expect("bind listener");
    println!("server listening on {}", handle.local_addr);

    let url = format!("ws://{}", handle.local_addr);
    let (mut client, _) = tokio_tungstenite::connect_async(url)
        .await
        .expect("client connect");

    let join = ClientMessage::Join {
        name: "car-1".into(),
    };
    client
        .send(Message::text(serde_json::to_string(&join).unwrap()))
        .await
        .expect("send join");

    if let Some(ConnectionEvent::Connected(id)) = handle.events.recv().await {
        println!("server: connection {id:?} opened");
    }
    let inbound = handle.inbound.recv().await.expect("inbound");
    println!("server received: {:?}", inbound.message);
    handle.send(
        inbound.connection,
        ServerMessage::Joined {
            agent_id: AgentId("car-1".into()),
            position: Vec3::ZERO,
        },
    );

    if let Some(Ok(Message::Text(text))) = client.next().await {
        let reply: ServerMessage = serde_json::from_str(&text).expect("parse reply");
        println!("client received: {reply:?}");
    }
}
