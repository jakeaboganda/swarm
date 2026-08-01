# viz

The visualization pathway: the semantic scene state the (headless) sim
streams to viewers, plus the broadcast server that fans it out. A forward,
observational channel — separate from the agent protocol, and carrying no
sensor data (sensing is a distinct pathway).

## Wire model

Two layers, sent as distinct message types so a scene-only consumer (a
recorder, a USD/glTF exporter) can ignore the rest:

- **Scene layer** (canonical/physical): `SceneInit` on connect → `SceneEvent`
  lifecycle changes (spawn/despawn, scenario state) → `Frame`s streamed at a
  fixed rate. Entities carry a `Shape` + `Transform`; static geometry is
  sent once and never appears in frames.
- **Debug layer** (optional): `DebugFrame` with per-entity plan paths and
  reflex flags. Trails are *not* transmitted — a viewer derives those from
  the frame stream.

`ServerToViewer` / `ViewerToServer` are the top-level messages. Viewers are
passive: the only thing they send is a connect-time `Hello`.

## Encoding

MessagePack (`rmp-serde`, named fields) via [`encode`]/[`decode`] — compact,
and decodable from the browser/Python. The same serde types also serialize
to JSON for debugging; tests assert both round-trip.

## Broadcast server

`spawn(VizConfig)` starts a WebSocket broadcaster on a separate port (4001
by default). `VizHandle` lets the sim `broadcast` scene messages to all
viewers, `broadcast_debug` to debug subscribers only, and `send` to one
viewer (the scene-init on connect). A `VizEvent` stream reports viewers
connecting/leaving. Per-viewer outbound queues are bounded and drop frames
when full — fine here, since each frame is a complete snapshot.
