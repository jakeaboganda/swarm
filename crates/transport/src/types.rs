use protocol::messages::ClientMessage;

/// Identifies one live WebSocket connection. Not the same thing as an
/// `AgentId` -- a connection exists before an agent has sent `Join`, and an
/// agent that reconnects gets a new `ConnectionId` on the new socket.
/// Matching a reconnect back to the same logical agent (by name, during
/// the reconnect grace window) is `server`'s job, not transport's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConnectionId(pub u64);

/// A parsed message from an agent, tagged with which connection it came
/// from.
#[derive(Debug, Clone, PartialEq)]
pub struct Inbound {
    pub connection: ConnectionId,
    pub message: ClientMessage,
}

/// Raw connection lifecycle events. `Disconnected` fires once the
/// heartbeat timeout elapses with no pong (or the socket errors/closes) --
/// transport doesn't know or care about scenarios/rosters, it just reports
/// that this connection is gone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConnectionEvent {
    Connected(ConnectionId),
    Disconnected(ConnectionId),
}
