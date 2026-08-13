# server

The headless simulation binary: loads a scenario, runs Rapier physics,
drives the per-tick control loop, computes simulated perception, manages the
scenario lifecycle, and streams the world to viewers. No window or rendering
of its own — that's the `viewer`'s job. Depends on `protocol`, `movement`,
`sensors`, `transport`, `viz`, and `perception`.

## Run

```
cargo run --bin server -- scenario.json
```

Binds the agent port (4000), the viz stream (4001), and the perception stream
(4002), then waits for the declared roster to connect before starting. It
renders nothing on its own — run a `viewer` (or any viz client) to watch.

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
  resolve-desired-velocity steps. Builds each agent's `{device →
  SensorContext}` map (the reserved `ground_truth` = exact others + walls;
  each simulated device = its delivered detections + walls) and evaluates
  rules against it. Force application is `movement`'s job, ordered after.
- **`perception_router`** — recomputes each agent's per-device perceived
  world in the fixed loop *before* arbitration (so a `Simulated` reflex reads
  exactly what was delivered), buffered for latency in a shared
  `PerceivedWorlds` resource. That one resource feeds reflex evaluation, the
  `:4002` per-device stream to listening agents, and the viz overlay — so
  they can never disagree.
- **`viz_broadcast`** — drives the viz stream: sends a scene-init to each new
  viewer, announces spawns/despawns and state changes, and streams frames +
  debug frames (plans, reflex flags, perception overlay) at ~30 Hz.

The app runs on a bounded headless run-loop (`ScheduleRunnerPlugin`) with a
minimal Bevy plugin set — no `winit`/render. On any connected agent
dropping, physics freezes (after a reconnect grace window) and remaining
agents are notified; the process keeps running so viewers can inspect the
frozen world.
