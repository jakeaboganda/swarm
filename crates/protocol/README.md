# protocol

Shared wire types for the swarm playground. Depends on nothing else in the
workspace; every other crate depends on it.

## Contents

- **`messages`**. The agent ↔ server WebSocket protocol (JSON):
  - `ClientMessage`: `Join`, `SubmitPlan`, `RegisterReflexes`, `GetState`,
    `RequestRoute`, `Subscribe`/`Ack` (the server-driven step clock).
  - `ServerMessage`: `Joined` (carries the delivered `map` in a road world),
    `State`, `ReflexFired`, `Route`, `Tick`, `OffRoad`, `ScenarioEnded`,
    `Error`.
  - Supporting types: `Waypoint`, `ReflexRule` (names a `sensor` device to
    read plus the `measure` predicate), `SensorKind`, `Operator`,
    `ReflexAction`, `AgentId`, `StateSnapshot`, `MapData`.
- **`scenario`**. The scenario file schema (`ScenarioConfig`): arena
  dimensions, an optional `map`, the fixed agent roster, a `seed` for
  reproducible perception noise, and an optional `time` block (run duration
  plus pace). Each `AgentSlot` carries its `embodiment`, a list of
  `SensorDef` devices (`SensorSource::GroundTruth | Simulated` plus a
  `SensorSpec` for simulated ones), and, for a `FmuVehicle`, its `fmu`
  binding. The reserved `GROUND_TRUTH_SENSOR` device name is always
  available to rules without being declared.
- **`Vec3`**. A serde-friendly 3D vector kept independent of any game-engine
  vector type, so this crate stays free of `bevy`/`glam`.

All types derive `Serialize`/`Deserialize`; tests assert JSON round-trips.
