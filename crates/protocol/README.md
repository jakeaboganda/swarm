# protocol

Shared wire types for the swarm playground. Depends on nothing else in the
workspace; every other crate depends on it.

## Contents

- **`messages`** — the agent ↔ server WebSocket protocol (JSON):
  - `ClientMessage`: `Join`, `SubmitPlan`, `RegisterReflexes`, `GetState`.
  - `ServerMessage`: `Joined`, `State`, `ReflexFired`, `ScenarioEnded`, `Error`.
  - Supporting types: `Waypoint`, `ReflexRule` (names a `sensor` device to
    read + the `measure` predicate), `SensorKind`, `Operator`, `ReflexAction`,
    `AgentId`, `StateSnapshot`.
- **`scenario`** — the scenario file schema (`ScenarioConfig`): arena
  dimensions, the fixed agent roster, and a `seed` for reproducible
  perception noise. Each `AgentSlot` carries its `embodiment` and a list of
  `SensorDef` devices (`SensorSource::GroundTruth | Simulated` + a
  `SensorSpec` for simulated ones). The reserved `GROUND_TRUTH_SENSOR` device
  name is always available to rules without being declared.
- **`Vec3`** — a serde-friendly 3D vector kept independent of any game-engine
  vector type, so this crate stays free of `bevy`/`glam`.

All types derive `Serialize`/`Deserialize`; tests assert JSON round-trips.
