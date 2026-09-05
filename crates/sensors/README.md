# sensors

Server-side sensing and reflex evaluation: the sensor readings a reflex
thresholds on, the impairment pipeline that turns ground truth into simulated
perception, and the rule engine that ties them together. Evaluated every tick
with zero round-trip to the agent. Depends on `protocol` for rule/message
shapes.

## Contents

- **`Sensor`** — a named reading produced each tick. Implementations:
  - `TimeToCollision` — nearest-by-closing-time (not by distance), against
    entities and walls, via linear extrapolation of relative velocity.
  - `DistanceTo` — distance to a fixed target.
  - `Speed` — the entity's own speed.
- **`SensorContext` / `Obstacle`** — the plain per-tick world state a sensor
  reads. `server` builds one *per device* per agent: for a `ground_truth`
  device the obstacles are every other entity, exact; for a `simulated`
  device they're that agent's culled/noised detections.
- **`perceive` / `Rng` / `Detection`** — the impairment pipeline. `perceive`
  culls a ground-truth entity list by range then field of view, then perturbs
  the survivors' pose with Gaussian noise (`Rng` is a hand-rolled seeded
  splitmix64 + Box–Muller, so runs are reproducible; latency lives in
  `server`'s delivery buffer, not here). A `simulated` sensor is then just an
  ordinary reading over the perceived obstacle set, with no per-sensor code.
- **`evaluate` / `ActiveRule`** — resolves an agent's reflex rules into at
  most one action. Each rule names a device (`rule.sensor`) and a predicate
  (`rule.measure`); `evaluate` looks the device up in a
  `HashMap<device, SensorContext>` and reads `measure` from it. Honors
  agent-assigned `priority` (ties broken by registration order) and applies
  hysteresis so a rule near its threshold doesn't chatter on/off. A rule
  naming an unknown device stays inactive.
