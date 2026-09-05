# viz

The visualization pathway: the semantic scene state the (headless) sim
streams to viewers, plus the broadcast server that fans it out. A forward,
observational channel, separate from the agent protocol. The *agent-facing*
sensor pathway is a distinct channel (the `perception` crate); no agent ever
consumes sensor data from viz. viz's optional debug layer does carry a
**human-only** perception overlay (what each agent perceives), for the
viewer's benefit only (see below).

## Wire model

Two layers, sent as distinct message types so a scene-only consumer (a
recorder, a USD/glTF exporter) can ignore the rest:

- **Scene layer** (canonical/physical): `SceneInit` on connect → `SceneEvent`
  lifecycle changes (spawn/despawn, scenario state) → `Frame`s streamed at a
  fixed rate. Entities carry a `Shape` + `Transform`; static geometry is
  sent once and never appears in frames.
- **Debug layer** (optional): `DebugFrame` with per-entity plan paths, reflex
  flags, and the perception overlay: each agent's currently-perceived
  entities as `Blip`s (noised "ghost" positions). An `EntityDescriptor` also
  carries an optional `SensorView` (range + FOV) for the sensing-envelope
  overlay. All human-only diagnostics a viewer may render or ignore. Trails
  are *not* transmitted; a viewer derives those from the frame stream.

`ServerToViewer` / `ViewerToServer` are the top-level messages. Viewers are
passive: the only thing they send is a connect-time `Hello`.

## Conventions

- **Coordinates:** right-handed, **Y-up, meters** (matching Bevy and glTF).
  A USD adapter (Z-up) must convert.
- **Versioning:** `PROTOCOL_VERSION` is declared by the viewer in `Hello`
  and by the sim in `SceneInit`; the broadcaster drops a viewer on
  mismatch. Additive *fields* are backward compatible; a new message/enum
  variant (e.g. the future delta frame) is a breaking change; bump the
  version.

## Encoding

MessagePack (`rmp-serde`, named fields) via [`encode`]/[`decode`]: compact,
and decodable from the browser/Python. The same serde types also serialize
to JSON for debugging; tests assert both round-trip.

## Broadcast server

`spawn(VizConfig)` starts a WebSocket broadcaster on a separate port (4001
by default). Delivery is split into two paths per viewer:

- **reliable** (`broadcast_reliable`, `send_reliable`) for must-deliver
  messages, the scene-init and lifecycle events;
- **lossy** (`broadcast_frame`, `broadcast_debug`) for the frame streams,
  bounded and dropped when a viewer's queue is full (the next snapshot
  resupplies the truth).

A `VizEvent` stream reports viewers connecting/leaving so the sim can send
a fresh scene-init to each newcomer.

## Not yet transmitted (deferred)

Per-entity velocity (for viewer-side interpolation across dropped frames)
and delta/keyframe frames (the perf path); both slot in behind the same
message types later; v1 viewers render the latest full snapshot and may
snap under packet loss.
