# viewer

The reference visualizer: a Bevy app that subscribes to the sim's viz
stream and renders it. One of potentially many viewers (a browser viewer, a
recorder, ...) — it owns no simulation state, only what the stream tells it.

## Run

```
cargo run --bin viewer            # connects to ws://127.0.0.1:4001
cargo run --bin viewer -- ws://host:4001
```

Start it any time — before the sim, during a run, or after. It reconnects
on its own and each reconnect delivers a fresh scene-init.

## How it works

- **`client`** — a background tokio task connects to the viz WebSocket,
  sends the `Hello`, decodes MessagePack `ServerToViewer` messages, and
  forwards them to Bevy over a channel. Reconnects with backoff.
- **`scene`** — `apply_stream` drains that channel and mirrors it into the
  ECS: `SceneInit` rebuilds the world (a full reset, also how a reconnect
  re-syncs), lifecycle events add/remove entities, and frames update
  transforms. It's defensive — spawns are idempotent and frames for unknown
  ids are ignored. Meshes are built from each entity's `Shape`.
- **`overlay`** — draws the debug layer (plan paths, reflex highlight) and
  viewer-derived motion trails with gizmos.

The viewer never simulates: positions and orientations come straight off
the wire.
