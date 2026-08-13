//! A minimal Rust agent: connects to a running server, joins, submits a
//! one-waypoint plan and a brake reflex, then prints every message the
//! server pushes back (Joined, ReflexFired, ScenarioEnded, ...).
//!
//! Start the server first, with a roster that includes the name you pass:
//!   cargo run --bin server -- scenario.json
//! then in another shell:
//!   cargo run -p server --example rust_agent -- car-1

use futures_util::{SinkExt, StreamExt};
use protocol::messages::{
    ClientMessage, Operator, ReflexAction, ReflexRule, SensorKind, ServerMessage, Waypoint,
};
use protocol::Vec3;
use tokio_tungstenite::tungstenite::Message;

#[tokio::main]
async fn main() {
    let name = std::env::args().nth(1).unwrap_or_else(|| "car-1".into());
    let (mut ws, _) = tokio_tungstenite::connect_async("ws://127.0.0.1:4000")
        .await
        .expect("connect to server");

    let outgoing = [
        ClientMessage::Join { name: name.clone() },
        ClientMessage::SubmitPlan {
            waypoints: vec![Waypoint {
                position: Vec3::new(15.0, 0.0, 0.0),
                speed: 5.0,
            }],
        },
        ClientMessage::RegisterReflexes {
            rules: vec![ReflexRule {
                sensor: "ground_truth".into(),
                measure: SensorKind::TimeToCollision,
                operator: Operator::LessThan,
                threshold: 2.0,
                action: ReflexAction::Brake,
                priority: 10,
            }],
        },
    ];
    for message in &outgoing {
        ws.send(Message::text(serde_json::to_string(message).unwrap()))
            .await
            .expect("send");
    }

    println!("[{name}] joined; printing server messages (Ctrl-C to stop)");
    while let Some(Ok(Message::Text(text))) = ws.next().await {
        if let Ok(message) = serde_json::from_str::<ServerMessage>(&text) {
            println!("[{name}] {message:?}");
        }
    }
}
