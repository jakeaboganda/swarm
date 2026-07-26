# server

The Bevy binary that wires everything together: loads a scenario, runs
Rapier physics, drives the per-tick control loop, and manages the scenario
lifecycle. Depends on `protocol`, `movement`, `sensors`, and `transport`.

## Run

```
cargo run --bin server -- scenario.json
```

Opens a window, binds the agent WebSocket port, and waits for the declared
roster to connect before starting.

## Contents

- **`world`** — spawns the arena (ground, four walls, light, camera) from
  the scenario JSON, and agent capsules (ground-constrained, rotation
  locked).
- **`scenario` / `scenario_state`** — scenario loading and the
  `WaitingForRoster → Running → Ended` state machine.
- **`transport_bridge`** — drains `transport`'s channels: spawns agents on
  `Join`, applies plans/reflexes, answers `GetState`, and ends the scenario
  on disconnect.
- **`arbitration`** — the control loop's read-sensors → evaluate-reflexes →
  resolve-desired-velocity steps; force application is `movement`'s job,
  ordered after.

On any connected agent dropping, physics freezes, remaining agents are
notified, and the window stays open for inspection.
