# sensors

Server-side sensing and reflex evaluation. Reflexes are the fast, reactive
layer: evaluated every tick from ground-truth world state, with zero
round-trip to the agent. Depends on `protocol` for rule/message shapes.

## Contents

- **`Sensor`** — a named reading produced each tick. v1 implementations:
  - `TimeToCollision` — nearest-by-closing-time (not by distance), against
    entities and walls, via linear extrapolation of relative velocity.
  - `DistanceTo` — distance to a fixed target.
  - `Speed` — the entity's own speed.
- **`SensorContext` / `Obstacle`** — the plain per-tick world state a sensor
  reads; gathered by `server` from its ECS queries.
- **`evaluate` / `ActiveRule`** — resolves an agent's reflex rules into at
  most one action, honoring agent-assigned `priority` (ties broken by
  registration order) and applying hysteresis so a rule near its threshold
  doesn't chatter on/off.

The `Sensor` trait is the extension point for later sensor simulation
(noise, limited range/FOV, latency) without changing reflex evaluation.
