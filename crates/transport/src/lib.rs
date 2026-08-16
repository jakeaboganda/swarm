mod connection;
mod handle;
mod types;

pub use handle::{spawn, Config, TransportHandle};
pub use types::{ConnectionEvent, ConnectionId, Inbound};

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use futures_util::{SinkExt, StreamExt};
    use protocol::messages::{ClientMessage, ServerMessage};
    use tokio_tungstenite::tungstenite::Message;

    use super::*;

    async fn spawn_test_server() -> TransportHandle {
        let config = Config {
            addr: ([127, 0, 0, 1], 0).into(),
            ..Config::default()
        };
        spawn(config).await.expect("bind test listener")
    }

    #[tokio::test]
    async fn client_connect_and_message_round_trip() {
        let mut handle = spawn_test_server().await;

        let url = format!("ws://{}", handle.local_addr);
        let (mut client, _) = tokio_tungstenite::connect_async(url)
            .await
            .expect("client connect");

        let connected = tokio::time::timeout(Duration::from_secs(5), handle.events.recv())
            .await
            .expect("timed out waiting for Connected event")
            .expect("events channel closed");
        let ConnectionEvent::Connected(connection_id) = connected else {
            panic!("expected Connected event, got {connected:?}");
        };

        let join = ClientMessage::Join {
            name: "car-1".into(),
        };
        client
            .send(Message::text(serde_json::to_string(&join).unwrap()))
            .await
            .expect("send join");

        let inbound = tokio::time::timeout(Duration::from_secs(5), handle.inbound.recv())
            .await
            .expect("timed out waiting for inbound message")
            .expect("inbound channel closed");
        assert_eq!(inbound.connection, connection_id);
        assert_eq!(inbound.message, join);

        handle.send(
            connection_id,
            ServerMessage::Joined {
                agent_id: protocol::messages::AgentId("car-1".into()),
                position: protocol::Vec3::ZERO,
                map: None,
            },
        );

        let response = tokio::time::timeout(Duration::from_secs(5), client.next())
            .await
            .expect("timed out waiting for server reply")
            .expect("client stream closed")
            .expect("websocket error");
        let Message::Text(text) = response else {
            panic!("expected text frame")
        };
        let parsed: ServerMessage = serde_json::from_str(&text).expect("valid ServerMessage json");
        assert!(matches!(parsed, ServerMessage::Joined { .. }));
    }

    #[tokio::test]
    async fn malformed_message_gets_error_reply_and_connection_stays_open() {
        let mut handle = spawn_test_server().await;
        let url = format!("ws://{}", handle.local_addr);
        let (mut client, _) = tokio_tungstenite::connect_async(url)
            .await
            .expect("client connect");

        client
            .send(Message::text("not valid json"))
            .await
            .expect("send garbage");

        let response = tokio::time::timeout(Duration::from_secs(5), client.next())
            .await
            .expect("timed out waiting for error reply")
            .expect("client stream closed")
            .expect("websocket error");
        let Message::Text(text) = response else {
            panic!("expected text frame")
        };
        let parsed: ServerMessage = serde_json::from_str(&text).expect("valid ServerMessage json");
        assert!(matches!(parsed, ServerMessage::Error { .. }));

        // Connection must still be open: a well-formed message afterwards
        // should still go through, proving the bad message didn't get
        // treated as a disconnect.
        let join = ClientMessage::Join {
            name: "car-1".into(),
        };
        client
            .send(Message::text(serde_json::to_string(&join).unwrap()))
            .await
            .expect("send join");
        let inbound = tokio::time::timeout(Duration::from_secs(5), handle.inbound.recv())
            .await
            .expect("timed out waiting for inbound message after malformed one")
            .expect("inbound channel closed");
        assert_eq!(inbound.message, join);
    }

    #[tokio::test]
    async fn binary_frame_gets_error_reply_without_disconnecting() {
        let mut handle = spawn_test_server().await;
        let url = format!("ws://{}", handle.local_addr);
        let (mut client, _) = tokio_tungstenite::connect_async(url)
            .await
            .expect("client connect");

        client
            .send(Message::binary(vec![1, 2, 3]))
            .await
            .expect("send binary");

        let response = tokio::time::timeout(Duration::from_secs(5), client.next())
            .await
            .expect("timed out waiting for error reply")
            .expect("client stream closed")
            .expect("websocket error");
        let Message::Text(text) = response else {
            panic!("expected text frame")
        };
        let parsed: ServerMessage = serde_json::from_str(&text).expect("valid ServerMessage json");
        assert!(matches!(parsed, ServerMessage::Error { .. }));

        // Connection stays open: a following valid message still arrives.
        let join = ClientMessage::Join {
            name: "car-1".into(),
        };
        client
            .send(Message::text(serde_json::to_string(&join).unwrap()))
            .await
            .expect("send join");
        let inbound = tokio::time::timeout(Duration::from_secs(5), handle.inbound.recv())
            .await
            .expect("timed out waiting for inbound message after binary one")
            .expect("inbound channel closed");
        assert_eq!(inbound.message, join);
    }
}
