# perception

The sensor pathway: simulated, **per-agent** perception the sim pushes to
agents, on its own WebSocket port (`:4002` by default). A separate channel
from both the agent control protocol and viz, so the producer stays
swappable: `provider → server-router → agent`. Today's provider is the
analytic one in `server` (ground truth → impairment); a rendered-sensor
provider (camera/LiDAR) is a later impl of the same boundary. Independent of
`protocol` and `viz`; its own JSON wire types only.

## Wire model

JSON text frames, mirroring the agent control channel so any-language clients
can decode with a plain JSON parser.

- **`Hello`** — an agent's connect-time declaration: which roster slot it is
  (so the server routes *that* agent's perception to it) + `PROTOCOL_VERSION`
  (dropped on mismatch).
- **`PerceptionFrame`** — one tick of one device's perception: the device
  `sensor` name (an agent may have several), `tick`, the perceived
  `Detection`s (id + kind + noised pose + true distance), and impaired
  `Scalars` (`time_to_collision`, `speed`).
- **`ServerToAgent` / `AgentToServer`** — the top-level messages. A forward
  pathway: the agent only ever sends the connect-time `Hello`.

Only *simulated* devices stream; a ground-truth device is a server-side
reflex fail-safe an agent never sees.

## Server

`spawn(PerceptionConfig) -> PerceptionHandle` starts the router. Unlike viz's
identical broadcast, delivery is **routed by agent name**: `handle.send(name,
frame)` pushes to just that agent (lossy: dropped if its queue is full). A
`PerceptionEvent` stream reports agents connecting/leaving so `server` knows
who to route to; a same-name reconnect is disambiguated by a connection token
so a stale disconnect can't evict the live connection.
