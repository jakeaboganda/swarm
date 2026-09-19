# transport

The async networking bridge. A tokio WebSocket server that connects external
agent processes to the synchronous Bevy world. Knows nothing about scenarios
or rosters; it only moves messages. Depends on `protocol`.

## Contents

- **`spawn(Config) -> TransportHandle`**. Binds the listener and starts the
  accept loop on a background tokio task.
- **`TransportHandle`**. What `server` polls each tick: `inbound` (parsed
  `ClientMessage`s tagged with their `ConnectionId`), `events`
  (`Connected`/`Disconnected`), plus `send`/`broadcast` for outbound
  `ServerMessage`s.
- **`ConnectionId`**. Identifies a live socket, distinct from an `AgentId`.
  A connection exists before `Join`, and a reconnect gets a new id.

Each connection runs one task handling the handshake, heartbeat ping/pong
with timeout, and JSON (de)serialization. A malformed message gets an
`Error` reply and the connection stays open. It is never treated as a
disconnect.
