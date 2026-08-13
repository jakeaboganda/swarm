# CLAUDE.md

Guidance for working in this repository. This file is self-contained by
design — it does not assume `DECISIONS.md` (gitignored, local-only) is
present. `DECISIONS.md` holds the *rationale* and rejected alternatives
behind each choice below, for whoever wants the "why"; this file holds the
facts and rules needed to work here day-to-day.

## What this is

A playground world for agentic swarms: a simple 3D physics simulation that
external agent processes (which may be LLM-driven, and therefore slow and
occasionally unreliable) connect to and control entities within. It's a dev
toy — an evolving project, not a fixed-spec deliverable.

## Architecture summary

- **Language: Rust. Engine: Bevy + `bevy_rapier3d`.** The **simulation is
  headless** (ECS + physics, no window/rendering); rendering lives in
  separate **viewer** processes that subscribe to a visualization stream.
- **Two independent pathways meet at the server:** agents connect over
  **WebSocket + JSON** (`transport`) to *control* entities; viewers connect
  over a separate **WebSocket + MessagePack** stream (`viz`) to *observe*.
  Viewers are passive and lifecycle-independent — connecting or dropping one
  never affects the sim.
- **Agents are external processes**, connecting over **WebSocket + JSON**.
- **Simulation is continuous real-time** (~60Hz Bevy/Rapier tick),
  independent of agent think-time. Each agent has a current action that
  stays in effect ("sticky") until replaced — the world never waits on a
  slow agent.
- **Agents control entities via two layers:**
  - A **plan**: an ordered path of waypoints, each `{position, speed}`.
    The server continuously steers the entity toward the current waypoint
    at its target speed, advancing to the next once within arrival
    tolerance.
  - **Reflexes**: declarative, agent-registered rules
    (`sensor` `measure` `operator` `threshold` → `action`) evaluated
    entirely server-side, every tick. `sensor` names a **device** to read —
    the always-available `ground_truth` (perfect, instant), or a scenario-
    equipped **simulated** device (see below) — and `measure` is the
    predicate (`time_to_collision`/…). So a reflex can react to impaired
    perception, not just the truth. They override plan-following when
    triggered — zero round-trip latency, since they never wait on the agent.
    Each rule carries an agent-assigned `priority` (higher wins on conflict;
    ties broken by registration order).
- **The world runs as a scenario**, not an open join-anytime space: a
  fixed roster of expected agents is declared upfront in a JSON scenario
  file, the server waits until every slot has connected, then runs. If any
  connected agent becomes unavailable, the *entire* scenario ends —
  physics freezes in place (for inspection), remaining agents are
  notified, and the process keeps running until manually killed.
- **Physics is ground-constrained**: gravity on, **roll and pitch locked**
  (no tipping/rolling). `Holonomic` and `CarLike` also lock yaw and steer via
  horizontal forces only; the invariant is relaxed to allow **yaw as a
  physical DOF for an embodiment that opts in** — the single-track
  `FullVehicle`, whose heading is real physics (tire-slip lateral forces + a
  yaw torque) rather than a cosmetic facing (see `DECISIONS.md`,
  "Higher-fidelity vehicle dynamics"). Movement model is a **pluggable,
  per-entity** component/trait selected by each agent's `embodiment`
  (`Holonomic`, `CarLike`, and `FullVehicle` ship). Models implement
  `drive(&mut self, desired, body, dt) -> Actuation` (a force plus a yaw
  torque), carrying state that evolves over time (a car's steering angle).
- **Sensors are first-class, world-equipped devices.** Each agent's roster
  slot declares `sensors: Vec<SensorDef>` — named devices, each `ground_truth`
  or `simulated` (with a `spec`: range, FOV half-angle, position/velocity
  Gaussian noise, latency). Reflex rules reference a device by name; the
  reserved `ground_truth` device is always available without declaration.
  Trust is the *device's* property, not the rule's — a rule is a dumb
  `measure op threshold`. The `sensors` crate holds both the predicate
  readings (`time_to_collision`/`distance_to`/`speed`, ground-truth math
  unchanged) and the impairment pipeline (`perceive`: range+FOV cull then
  Gaussian noise, seeded per `(scenario_seed, agent, device, tick)`); latency
  is a delivery-layer ring buffer. A `simulated` device's `time_to_collision`
  is just the same reading over the culled/noised obstacle set. **v1 impairs
  `time_to_collision` only**; `speed`/`distance_to` are self-referential and
  stay ground-truth (filters over them come later).
- **Perception is its own pathway** (`perception` crate, `:4002`): the sim
  computes each agent's per-device perceived world once per tick (before
  reflex arbitration, so a `Simulated` reflex reads exactly what was
  delivered) and pushes it to any agent listening — one frame per simulated
  device. `provider → server-router → agent`, provider-agnostic (analytic
  today, a rendered-sensor provider later). Ground-truth devices are
  reflex-only fail-safes — never streamed (an agent never *sees* ground
  truth). The **viz** debug layer separately carries a *human-only*
  perception overlay (see below); no agent consumes sensor data from viz.

## Crate layout

Cargo workspace, eight crates:

- **`protocol`** — shared `serde` types for the *agent* pathway: WebSocket
  messages (`join`, plan submission, reflex-rule registration,
  `get_state`/snapshot, `reflex_fired`/`scenario_ended`/`error` events), and
  the scenario JSON schema (world/walls + agent roster + per-agent
  `SensorDef`s + `seed`). Depends on nothing else in the workspace.
- **`movement`** — the pluggable embodiment trait +
  `Holonomic`/`CarLike`/`FullVehicle` implementations. No
  networking/scenario knowledge.
- **`sensors`** — sensor readings (`time_to_collision`/`distance_to`/`speed`),
  the perception-impairment pipeline (`perceive`: range/FOV cull + seeded
  Gaussian noise), and reflex-rule evaluation (`evaluate` resolves each
  rule's named device to a `SensorContext`, then applies threshold checks +
  hysteresis + priority). Depends on `protocol` for rule/message shapes.
- **`transport`** — the async *agent* WebSocket server: per-connection
  handling (heartbeat, malformed-message contract) and the bounded channels
  bridging async I/O into Bevy's synchronous ECS tick. Depends only on
  `protocol`.
- **`viz`** — the *visualization* pathway (`:4001`): the semantic scene wire
  types (MessagePack, versioned) and the WebSocket broadcast server that fans
  them out to viewers. Its debug layer also carries a human-only perception
  overlay (per-agent detections + a sensing envelope). Independent of
  `protocol` and `perception` (a separate pathway; viz-local types only).
- **`perception`** — the *sensor* pathway (`:4002`): the JSON wire types
  (`Hello`, per-device `PerceptionFrame`) and a per-agent-routed push server
  (each agent receives only its own perception, unlike viz's identical
  broadcast). Independent of `protocol`/`viz`.
- **`server`** — the headless simulation binary: loads the scenario, runs
  Rapier physics, dispatches movement per entity, computes per-device
  perceived worlds, resolves reflex-vs-plan arbitration each tick, manages
  scenario lifecycle, and drives the viz + perception broadcasts. Owns the
  tokio runtime; Bevy runs on the main thread.
- **`viewer`** — the reference 3D visualizer binary: a Bevy app that
  subscribes to the `viz` stream and renders the scene + debug overlays
  (plans, trails, and the perception overlay). One of potentially many
  viewers (a browser viewer, a recorder, ...); holds no simulation state.

## Conventions

- **Formatting/linting**: `rustfmt` defaults, `clippy` enforced at
  `-D warnings`. Anything clippy flags must be fixed or explicitly
  `#[allow]`'d with a comment explaining why.
- **Error handling**: `Result`-based throughout. Library crates
  (`protocol`, `movement`, `sensors`, `transport`, `viz`, `perception`) define
  their own error enums via `thiserror` where they surface errors; the
  `server` and `viewer` binaries may use `anyhow` for top-level error
  bubbling. Avoid `unwrap`/`expect` outside tests, except
  for genuinely infallible cases — and those get a short comment saying
  why. Panics are for programmer-error invariant violations, not for
  expected/recoverable conditions (malformed agent input, disconnects,
  scenario JSON errors all fall into the latter).
- **Test-Driven Development**: we do TDD in this repository. Write a
  failing test that pins down the behavior first, then the code that makes
  it pass, then refactor. When logic is entangled with Bevy/networking and
  can't be unit-tested directly, extract the decision into a pure function
  and drive *that* with tests (as `arbitration::planar_seek` and
  `AwaitingReconnect` are).
- **Testing**: required for the pure/deterministic logic crates —
  `protocol`/`viz`/`perception` (serialization round-trips), `movement`
  (seek-controller math), `sensors` (predicate evaluation, perception
  culling/noise, hysteresis, priority/tiebreak, device resolution). Networking
  crates (`transport`, `viz` broadcaster, `perception` server) also carry
  integration tests over a real socket. Not required for `server`/`viewer`
  (ECS wiring, scenario timing, rendering) — that code is better verified by
  running the app than by unit tests.
- **Dependencies**: free to add anything already implied by this file
  (`bevy`, `bevy_rapier3d`, `tokio`, `tokio-tungstenite`, `serde`/
  `serde_json`, `rmp-serde`, `thiserror`, `anyhow`) without asking. Anything
  outside that set — a new crate not implied by an existing decision — gets
  flagged before adding, even if minor.
- **Commit messages**: must follow [Conventional Commits](https://www.conventionalcommits.org/)
  (`type(scope): summary`, e.g. `feat(sensors): add hysteresis to
  time_to_collision`).
- **Doc comments**: clear and straight to the point. No flowery or
  roundabout phrasing. The audience is developers without much time —
  say the one thing they need to know and stop.

## Implementation guardrails

These came out of a pre-implementation design review and are correctness
requirements, not style preferences — getting them wrong reintroduces bugs
that were already found and fixed on paper:

- `bevy_rapier3d`'s `ExternalForce` must be **overwritten, not
  accumulated**, every tick by the control system, which must run ordered
  before Rapier's own physics step.
- Per-entity movement dispatch uses **generic systems monomorphized per
  concrete component type**, not `Box<dyn MovementModel>`.
- `get_state` snapshots and pushed events carry a **monotonic tick counter
  and a plan-version id**, so an agent can tell whether an event refers to
  its current plan or one already superseded.
- `time_to_collision` is **nearest-by-closing-time** (not
  nearest-by-distance — these diverge), **includes static walls**, and
  requires **hysteresis** on the threshold (e.g. fires at `< N`, clears at
  `< N + 0.5`) to avoid chattering at the boundary.
- `brake`/`stop_and_hold` use a **separate, higher `max_force` ceiling**
  than ordinary path-following — they must not share the cruising
  controller's force limit.
- Scenario-end detection uses a **heartbeat** (ping ~2s / timeout ~6s), not
  bare socket-close, plus a **short reconnect grace window** (~5-10s)
  before declaring the scenario over.
- A malformed agent message gets an `error` event reply; the **connection
  stays open** — it is never treated as a disconnect (which would
  otherwise end the whole scenario over a JSON typo).
- A new connection arriving after a scenario has already frozen is
  accepted and immediately gets the same `scenario_ended` reply.

## Definition of done

Before considering a change complete, all of the following must pass:

```
cargo fmt --check
cargo clippy --workspace -- -D warnings
cargo test --workspace
```

## Reference

Full rationale, alternatives considered, and the history of how this
design was reached: `DECISIONS.md` (local, gitignored, not part of this
repo's committed history).
