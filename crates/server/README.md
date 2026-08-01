# server

The headless simulation binary: loads a scenario, runs Rapier physics,
drives the per-tick control loop, manages the scenario lifecycle, and
streams the world to viewers. No window or rendering of its own — that's the
`viewer`'s job. Depends on `protocol`, `movement`, `sensors`, `transport`,
and `viz`.

## Run

```
cargo run --bin server -- scenario.json
```

Binds the agent WebSocket port (4000), the viz stream port (4001), and waits
for the declared roster to connect before starting. It renders nothing on
its own — run a `viewer` (or any viz client) to watch.

## Contents

- **`world`** — spawns the arena (ground, four walls) and agent capsules as
  physics bodies, each tagged with a `VizEntity` describing it for viewers.
  Ground-constrained, rotation locked.
- **`scenario` / `scenario_state`** — scenario loading and the
  `WaitingForRoster → Running → Ended` state machine.
- **`transport_bridge`** — drains `transport`'s channels: spawns agents on
  `Join`, applies plans/reflexes, answers `GetState`, and handles
  disconnects.
- **`arbitration`** — the control loop's read-sensors → evaluate-reflexes →
  resolve-desired-velocity steps; force application is `movement`'s job,
  ordered after.
- **`viz_broadcast`** — drives the viz stream: sends a scene-init to each new
  viewer, announces spawns/despawns and state changes, and streams frames +
  debug frames at ~30 Hz.

The app runs on a bounded headless run-loop (`ScheduleRunnerPlugin`) with a
minimal Bevy plugin set — no `winit`/render. On any connected agent
dropping, physics freezes (after a reconnect grace window) and remaining
agents are notified; the process keeps running so viewers can inspect the
frozen world.
