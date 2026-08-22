//! The agent pathway under conditions no cooperative client produces:
//! silence, flooding, a peer that never finishes its handshake, and sends to
//! a connection that is already gone.
//!
//! The premise of the project is that agents "may be LLM-driven, and
//! therefore slow and occasionally unreliable", so these are the ordinary
//! operating conditions of this port, not exotic ones.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use protocol::messages::{AgentId, ClientMessage, ServerMessage};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::MaybeTlsStream;
use transport::{Config, ConnectionEvent, ConnectionId, TransportHandle};

/// Heartbeats fast enough to observe inside a test. The intervals are already
/// parameterized on `Config`; nothing used that until now.
const INTERVAL: Duration = Duration::from_millis(50);
const TIMEOUT: Duration = Duration::from_millis(200);

async fn server_with(config: Config) -> TransportHandle {
    transport::spawn(Config {
        addr: ([127, 0, 0, 1], 0).into(),
        ..config
    })
    .await
    .expect("bind test listener")
}

async fn fast_heartbeat_server() -> TransportHandle {
    server_with(Config {
        heartbeat_interval: INTERVAL,
        heartbeat_timeout: TIMEOUT,
        ..Config::default()
    })
    .await
}

type Client = tokio_tungstenite::WebSocketStream<MaybeTlsStream<TcpStream>>;

async fn connect(handle: &TransportHandle) -> Client {
    let (ws, _) = tokio_tungstenite::connect_async(format!("ws://{}", handle.local_addr))
        .await
        .expect("client connects");
    ws
}

/// The next connection event, or a panic naming what we were waiting for.
async fn next_event(handle: &mut TransportHandle, what: &str) -> ConnectionEvent {
    tokio::time::timeout(Duration::from_secs(5), handle.events.recv())
        .await
        .unwrap_or_else(|_| panic!("timed out waiting for {what}"))
        .expect("events channel closed")
}

async fn connected_id(handle: &mut TransportHandle) -> ConnectionId {
    match next_event(handle, "a Connected event").await {
        ConnectionEvent::Connected(id) => id,
        other => panic!("expected Connected, got {other:?}"),
    }
}

#[tokio::test]
async fn a_silent_client_is_dropped_after_the_heartbeat_timeout() {
    let mut handle = fast_heartbeat_server().await;
    // Connect, then never poll the socket again: pings are never answered,
    // which is what a wedged agent process looks like from here. Scenario-end
    // detection is a heartbeat rather than a bare socket close precisely so
    // this case is detected at all.
    let _ws = connect(&handle).await;
    let id = connected_id(&mut handle).await;

    match next_event(&mut handle, "the silent client to time out").await {
        ConnectionEvent::Disconnected(gone) => assert_eq!(gone, id),
        other => panic!("expected Disconnected, got {other:?}"),
    }
}

#[tokio::test]
async fn a_client_that_pongs_stays_connected_well_past_the_timeout() {
    let mut handle = fast_heartbeat_server().await;
    let mut ws = connect(&handle).await;
    let _id = connected_id(&mut handle).await;

    // Answer pings for several multiples of the timeout. Tungstenite queues a
    // pong when it reads a ping but only writes it when the sink is driven,
    // so the client flushes as it polls.
    let deadline = tokio::time::Instant::now() + TIMEOUT * 5;
    while tokio::time::Instant::now() < deadline {
        tokio::select! {
            incoming = ws.next() => {
                if incoming.is_none() {
                    panic!("server closed a healthy connection");
                }
                ws.flush().await.expect("flush pong");
            }
            _ = tokio::time::sleep(Duration::from_millis(10)) => {
                ws.flush().await.expect("flush pong");
            }
        }
        if let Ok(event) = handle.events.try_recv() {
            panic!("a responsive client was dropped: {event:?}");
        }
    }
}

#[tokio::test]
async fn a_flooding_client_backpressures_its_own_socket() {
    // The inbound channel is bounded, and the connection task *awaits* on it
    // rather than dropping: a flooding agent stops being read, TCP stops
    // accepting its writes, and the pressure lands on the flooder instead of
    // on server memory. Nothing is lost -- it is throttled, not truncated.
    let mut handle = server_with(Config::default()).await;
    let mut ws = connect(&handle).await;
    let _id = connected_id(&mut handle).await;

    const FLOOD: usize = 400;
    let payload = serde_json::to_string(&ClientMessage::Join {
        name: "x".repeat(32 * 1024),
    })
    .expect("serialize");

    let sent = Arc::new(AtomicUsize::new(0));
    let counter = sent.clone();
    let flooder = tokio::spawn(async move {
        for _ in 0..FLOOD {
            if ws.send(Message::text(payload.clone())).await.is_err() {
                break;
            }
            counter.fetch_add(1, Ordering::Relaxed);
        }
        ws
    });

    // With the sim not draining, the flood stalls partway rather than being
    // swallowed whole.
    tokio::time::sleep(Duration::from_millis(300)).await;
    let stalled_at = sent.load(Ordering::Relaxed);
    assert!(
        stalled_at < FLOOD,
        "the whole flood was absorbed without backpressure ({stalled_at} sent)"
    );

    // Once the sim drains again, the client is released and every message it
    // did send arrives.
    let mut received = 0;
    while received < FLOOD {
        match tokio::time::timeout(Duration::from_secs(5), handle.inbound.recv()).await {
            Ok(Some(_)) => received += 1,
            Ok(None) => panic!("inbound channel closed"),
            Err(_) => panic!("stalled at {received} of {FLOOD} after draining resumed"),
        }
    }
    assert_eq!(sent.load(Ordering::Relaxed), FLOOD);
    flooder.await.expect("flooder task");
}

#[tokio::test]
async fn a_tcp_peer_that_never_upgrades_is_dropped_at_a_handshake_timeout() {
    // `viz` and `perception` both bound their handshake explicitly to defeat
    // slowloris clients; this port has to as well, or a peer that opens TCP
    // and says nothing holds a task for the life of the process.
    let handle = server_with(Config {
        handshake_timeout: Duration::from_millis(200),
        ..Config::default()
    })
    .await;

    let mut stream = TcpStream::connect(handle.local_addr)
        .await
        .expect("raw tcp connect");

    // Never sends the HTTP upgrade. The server must hang up on its own.
    let mut buffer = [0u8; 1];
    let read = tokio::time::timeout(Duration::from_secs(3), async {
        use tokio::io::AsyncReadExt;
        stream.read(&mut buffer).await
    })
    .await
    .expect("the server never dropped the silent peer");
    assert_eq!(read.expect("read"), 0, "expected the socket to be closed");
}

#[tokio::test]
async fn two_connections_get_distinct_ids_and_independent_outbound() {
    let mut handle = server_with(Config::default()).await;
    let mut first = connect(&handle).await;
    let first_id = connected_id(&mut handle).await;
    let mut second = connect(&handle).await;
    let second_id = connected_id(&mut handle).await;
    assert_ne!(first_id, second_id);

    handle.send(
        first_id,
        ServerMessage::Joined {
            agent_id: AgentId("car-1".into()),
            position: protocol::Vec3::ZERO,
            map: None,
        },
    );
    handle.send(
        second_id,
        ServerMessage::Error {
            message: "second".into(),
        },
    );

    let to_first = read_message(&mut first).await;
    let to_second = read_message(&mut second).await;
    assert!(
        matches!(&to_first, ServerMessage::Joined { agent_id, .. } if agent_id.0 == "car-1"),
        "first connection got {to_first:?}"
    );
    assert!(
        matches!(&to_second, ServerMessage::Error { message } if message == "second"),
        "second connection got {to_second:?}"
    );
}

#[tokio::test]
async fn sending_to_a_dropped_connection_is_a_noop() {
    let mut handle = server_with(Config::default()).await;
    let ws = connect(&handle).await;
    let id = connected_id(&mut handle).await;

    drop(ws);
    match next_event(&mut handle, "the client to disconnect").await {
        ConnectionEvent::Disconnected(gone) => assert_eq!(gone, id),
        other => panic!("expected Disconnected, got {other:?}"),
    }

    // The sim discovers a disconnect a tick or more after it happens, so it
    // will address messages to connections that are already gone. That has to
    // be a no-op, not a panic -- it happens on the sim thread.
    handle.send(
        id,
        ServerMessage::ScenarioEnded {
            reason: "gone".into(),
        },
    );
    handle.broadcast(ServerMessage::ScenarioEnded {
        reason: "gone".into(),
    });
}

async fn read_message(ws: &mut Client) -> ServerMessage {
    loop {
        let frame = tokio::time::timeout(Duration::from_secs(5), ws.next())
            .await
            .expect("timed out waiting for a server message")
            .expect("stream closed")
            .expect("websocket error");
        if let Message::Text(text) = frame {
            return serde_json::from_str(&text).expect("valid ServerMessage json");
        }
    }
}
