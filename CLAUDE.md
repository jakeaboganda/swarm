# CLAUDE.md

Guidance for working in this repository. This file is self-contained by
design: it does not assume `DECISIONS.md` (gitignored, local-only) is
present. `DECISIONS.md` holds the *rationale* and rejected alternatives
behind each choice below, for whoever wants the "why"; this file holds the
facts and rules needed to work here day-to-day.

## What this is

A playground for agentic driving: a 3D physics simulation that external agent
processes (which may be LLM-driven, and therefore slow and occasionally
unreliable) connect to and drive vehicles within: a flat, walled **arena**,
or a real **road network** (a hand-authored road or a loaded OpenDRIVE map).
It's a dev toy: an evolving project, not a fixed-spec deliverable. It grew from
a bare arena swarm into an agent-facing driving sandbox; both worlds coexist,
selected per scenario.

## Architecture summary

- **Language: Rust. Engine: Bevy + `bevy_rapier3d`.** The **simulation is
  headless** (ECS + physics, no window/rendering); rendering lives in
  separate **viewer** processes that subscribe to a visualization stream.
- **Three independent pathways meet at the server**, each on its own port:
  agents connect over **WebSocket + JSON** (`transport`, `:4000`) to *control*
  entities; viewers connect over a separate **WebSocket + MessagePack** stream
  (`viz`, `:4001`) to *observe*; and agents receive their **simulated
  perception** over a third stream (`perception`, JSON, `:4002`). Viewers are
  passive and lifecycle-independent: connecting or dropping one never affects
  the sim.
- **Agents are external processes**, connecting over **WebSocket + JSON**.
- **Simulation is continuous real-time** (64Hz Bevy/Rapier tick),
  independent of agent think-time. Each agent has a current action that
  stays in effect ("sticky") until replaced; the world never waits on a
  slow agent.
- **The scenario owns time.** A scenario's optional `time` block sets its
  run **duration** (in sim-seconds; omitted = unbounded, ends only on
  disconnect) and its **pace**: `realtime` (one sim-second per wall-second,
  for live viewing) or `afap` (ticks at CPU speed for headless batch runs;
  same sim-time, reached sooner in wall-clock). The physics tick rate
  (64Hz) is a fixed engine invariant, not scenario-owned. The `SIM_TIME`
  env var still overrides pace ad-hoc. The server ends the scenario at the
  duration deadline (same freeze-and-notify as a disconnect-end).
- **Agents may run on a server-driven step clock** instead of polling. An
  agent `Subscribe`s and the server pushes a `Tick { tick, dt, plan_version }`
  pulse carrying the sim-seconds elapsed since that agent's last pulse; the
  agent `Ack { tick }`s to release the next. It's **one-in-flight and never
  blocks the sim**: a fast agent approaches the 64Hz ceiling, a slow one
  gets fewer pulses with a larger `dt` (which is authoritative; the rate is
  best-effort). An agent that never subscribes just keeps polling `get_state`
  as before.
- **Agents control entities via two layers:**
  - A **plan**: an ordered path of waypoints, each `{position, speed}`.
    The server *tracks* it: each tick it measures how far along the path the
    body has got, aims at a point interpolated a speed-scaled lookahead
    **ahead of that**, and steers there at the speed the plan asked for. The
    waypoints are **never consumed**: they stay exactly as submitted until
    the path is driven or a reflex drops it, so an agent that re-plans
    freely never finds its own path and the server's idea of it drifting
    apart. Deciding *where* and *how fast* is the agent's; the server only
    executes.
  - **Reflexes**: declarative, agent-registered rules
    (`sensor` `measure` `operator` `threshold` → `action`) evaluated
    entirely server-side, every tick. `sensor` names a **device** to read:
    the always-available `ground_truth` (perfect, instant), or a scenario-
    equipped **simulated** device (see below). `measure` is the
    predicate (`time_to_collision`/…). So a reflex can react to impaired
    perception, not just the truth. They override plan-following when
    triggered, at zero round-trip latency, since they never wait on the agent.
    Each rule carries an agent-assigned `priority` (higher wins on conflict;
    ties broken by registration order).
- **The world runs as a scenario**, not an open join-anytime space: a
  fixed roster of expected agents is declared upfront in a JSON scenario
  file, the server waits until every slot has connected, then runs. If any
  connected agent becomes unavailable, the *entire* scenario ends:
  physics freezes in place (for inspection), remaining agents are
  notified, and the process keeps running until manually killed.
- **Physics is ground-constrained**: gravity on, **roll and pitch locked**
  (no tipping/rolling) by default. `Holonomic` and `CarLike` also lock yaw and
  steer via horizontal forces only. The invariant relaxes for embodiments that
  opt in: `FullVehicle` unlocks **yaw** as a physical DOF (its heading is real
  physics (tire-slip lateral forces + a yaw torque), not a cosmetic facing);
  `RaycastVehicle` unlocks **roll and pitch** too, so it leans on banking and
  follows the road's grade (see `DECISIONS.md`, "Higher-fidelity vehicle
  dynamics"). Movement model is a **pluggable, per-entity** component/trait
  selected by each agent's `embodiment` (`Holonomic`, `CarLike`, `FullVehicle`,
  `RaycastVehicle` ship). Force-based models implement
  `drive(&mut self, desired, body, dt) -> Actuation` (a force plus a yaw
  torque), carrying state that evolves over time (a car's steering angle); the
  raycast vehicle is the exception: it applies its own per-wheel suspension /
  drive / grip forces directly (it doesn't fit the `drive` seam).
- **Sensors are first-class, world-equipped devices.** Each agent's roster
  slot declares `sensors: Vec<SensorDef>`: named devices, each `ground_truth`
  or `simulated` (with a `spec`: range, FOV half-angle, position/velocity
  Gaussian noise, latency). Reflex rules reference a device by name; the
  reserved `ground_truth` device is always available without declaration.
  Trust is the *device's* property, not the rule's; a rule is a dumb
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
  delivered) and pushes it to any agent listening, one frame per simulated
  device. `provider → server-router → agent`, provider-agnostic (analytic
  today, a rendered-sensor provider later). Ground-truth devices are
  reflex-only fail-safes, never streamed (an agent never *sees* ground
  truth). The **viz** debug layer separately carries a *human-only*
  perception overlay (see below); no agent consumes sensor data from viz.
- **The world is arena or road**, chosen by the scenario's optional `map`
  field. No `map` builds the flat, walled arena. A `map` builds a **road world**
  from an internal, format-agnostic road-network model (`map::RoadNetwork`):
  `"demo"` is the built-in hand-authored road; a path ending in `.xodr` is baked
  from a real **OpenDRIVE** file by the `map-opendrive` importer. The road is
  one static trimesh collider the raycast vehicle drives on. Curves are baked to
  polylines at load, so consumers only ever sample points.
- **Roads are a known prior; traffic is perceived.** In a road world the static
  map is **delivered to the agent at join** (the `joined` message's `map`: lane
  centerlines, widths, and the connectivity graph), perfect and free. Dynamic
  obstacles are still perceived through the impaired `:4002` pathway. An agent
  lays a lane-following path from the delivered map, or asks the server to
  **route**: `request_route{from,to,speed}` runs `map`'s Dijkstra router over the
  connectivity graph (lane successors through junctions + lane-change neighbors)
  and returns a plan with the agent's speed stamped on. The plan (and its speed)
  stays the agent's; the server only does the mechanical pathfinding.

## Crate layout

Cargo workspace, ten crates:

- **`protocol`** — shared `serde` types for the *agent* pathway: WebSocket
  messages (`join`, plan submission, reflex-rule registration, `request_route`,
  `subscribe`/`ack` for the step clock; `get_state`/snapshot, `joined` (which
  carries the delivered `map` in a road world), `reflex_fired`/`route`/`tick`/
  `scenario_ended`/`error` events), and the scenario JSON schema (arena +
  optional `map` + agent roster + per-agent `SensorDef`s + `seed` + optional
  `time` block). Depends on nothing else in the workspace.
- **`movement`** — the pluggable embodiment trait +
  `Holonomic`/`CarLike`/`FullVehicle`/`RaycastVehicle` implementations. No
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
  them out to viewers. An entity is a **tree of nodes**: each a local
  transform, optional `Geometry` (primitives, a baked mesh, or an asset URI)
  and children. So a car is a body with four wheel children, and a frame
  carries only the nodes that moved, addressed by path. The **sim computes
  every node transform**: a viewer draws where it is told and composes nothing.
  Its debug layer also carries human-only diagnostics: a perception overlay
  (per-agent detections + a sensing envelope) and per-wheel slip and contact.
  Independent of `protocol` and `perception` (a separate pathway; viz-local
  types only).
- **`perception`** — the *sensor* pathway (`:4002`): the JSON wire types
  (`Hello`, per-device `PerceptionFrame`) and a per-agent-routed push server
  (each agent receives only its own perception, unlike viz's identical
  broadcast). Independent of `protocol`/`viz`.
- **`map`** — the format-agnostic **road-network model** (`RoadNetwork`): lanes
  (baked centerline polylines, width, travel direction), the drive-direction
  connectivity graph (`successors`/`predecessors`/`neighbors`), a surface-mesh
  tessellator, and the router (`route(from, to)` → a sampled path over the
  graph). Also the built-in hand-authored `demo_road`. Pure geometry; no
  networking, no OpenDRIVE knowledge.
- **`map-opendrive`** — the pure-Rust **OpenDRIVE (`.xodr`) importer** that
  bakes a real map into `map::RoadNetwork` at load: line/arc/spiral/paramPoly3/
  poly3 geometry, elevation, per-lane widths, lane offsets, multiple lane
  sections, and road/lane/junction connectivity. Depends on `map` + `roxmltree`;
  geometry cross-checked against the reference C++ libOpenDRIVE.
- **`server`** — the headless simulation: loads the scenario, builds the
  world (flat **arena**, or a **road** world: the road's trimesh collider +
  raycast-vehicle agents, from `demo_road` or an imported `.xodr`), runs Rapier
  physics, dispatches movement per entity, computes per-device perceived worlds,
  resolves reflex-vs-plan arbitration each tick, tracks each agent's plan
  (`tracker`: projected progress + a speed-scaled pure-pursuit aim point),
  answers `request_route`, manages scenario lifecycle, and drives the viz +
  perception broadcasts. Owns the tokio
  runtime; Bevy runs on the main thread. Split lib + bin: `app::build_app`
  assembles the whole system graph, and the binary is process concerns only
  (argument parsing, the runtime, binding the three servers), so a test can
  stand the real sim up on ephemeral ports and step it one tick at a time.
- **`viewer`** — the reference 3D visualizer binary: a Bevy app that
  subscribes to the `viz` stream and renders the scene + debug overlays
  (plans, trails, and the perception overlay). One of potentially many
  viewers (a browser viewer, a recorder, ...); holds no simulation state.

## Conventions

- **Formatting/linting**: `rustfmt` defaults, `clippy` enforced at
  `-D warnings`. Anything clippy flags must be fixed or explicitly
  `#[allow]`'d with a comment explaining why.
- **Error handling**: `Result`-based throughout. Library crates
  (`protocol`, `movement`, `sensors`, `transport`, `viz`, `perception`, `map`,
  `map-opendrive`) define their own error enums via `thiserror` where they
  surface errors (e.g. `map-opendrive`'s `ImportError`); the
  `server` and `viewer` binaries may use `anyhow` for top-level error
  bubbling. Avoid `unwrap`/`expect` outside tests, except
  for genuinely infallible cases, and those get a short comment saying
  why. Panics are for programmer-error invariant violations, not for
  expected/recoverable conditions (malformed agent input, disconnects,
  scenario JSON errors all fall into the latter).
- **Test-Driven Development**: we do TDD in this repository. Write a
  failing test that pins down the behavior first, then the code that makes
  it pass, then refactor. When logic is entangled with Bevy/networking and
  can't be unit-tested directly, extract the decision into a pure function
  and drive *that* with tests (as `arbitration::planar_seek` and
  `AwaitingReconnect` are).
- **Testing**: required for the pure/deterministic logic crates:
  `protocol`/`viz`/`perception` (serialization round-trips), `movement`
  (seek-controller math), `sensors` (predicate evaluation, perception
  culling/noise, hysteresis, priority/tiebreak, device resolution), `map`
  (geometry sampling, nearest-lane, routing/lane-changes), and `map-opendrive`
  (geometry per primitive, elevation/widths/laneOffset, multi-section,
  connectivity, plus real-`.xodr` smoke tests). Networking crates (`transport`,
  `viz` broadcaster, `perception` server) also carry integration tests over a
  real socket, including the adversarial cases the architecture invites (a
  silent client, a flooding one, a peer that never finishes its handshake).
  `server` is not unit-tested for ECS wiring, but its **invariants** are, from
  the outside: `crates/server/tests/` builds the real app via `build_app` on
  ephemeral ports and drives it through the real agent pathway: the scenario
  lifecycle, the control-loop guardrails below, and same-binary determinism.
  Untrusted agent input is tested at the pure boundary (`inbound`). `viewer`
  stays excluded from the *requirement* to have tests (rendering is verified by
  looking at the screen), but the gate runs whatever tests it does have, so
  anything pure that lands there (camera placement, wheel tinting) is not
  silently skipped.
- **Dependencies**: free to add anything already implied by this file
  (`bevy`, `bevy_rapier3d`, `tokio`, `tokio-tungstenite`, `serde`/
  `serde_json`, `rmp-serde`, `thiserror`, `anyhow`, `glam`, `roxmltree`; the
  last for the OpenDRIVE importer) without asking. Anything outside that set
  (a new crate not implied by an existing decision) gets flagged before adding,
  even if minor.
- **Commit messages**: must follow [Conventional Commits](https://www.conventionalcommits.org/)
  (`type(scope): summary`, e.g. `feat(sensors): add hysteresis to
  time_to_collision`).
- **Doc comments**: clear and straight to the point. No flowery or
  roundabout phrasing. The audience is developers without much time:
  say the one thing they need to know and stop.

## Implementation guardrails

These came out of a pre-implementation design review and are correctness
requirements, not style preferences; getting them wrong reintroduces bugs
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
  nearest-by-distance; these diverge), **includes static walls**, and
  requires **hysteresis** on the threshold (e.g. fires at `< N`, clears at
  `< N + 0.5`) to avoid chattering at the boundary.
- `brake`/`stop_and_hold` use a **separate, higher `max_force` ceiling**
  than ordinary path-following; they must not share the cruising
  controller's force limit.
- Scenario-end detection uses a **heartbeat** (ping ~2s / timeout ~6s), not
  bare socket-close, plus a **short reconnect grace window** (~5-10s)
  before declaring the scenario over.
- A malformed agent message gets an `error` event reply; the **connection
  stays open**: it is never treated as a disconnect (which would
  otherwise end the whole scenario over a JSON typo).
- A new connection arriving after a scenario has already frozen is
  accepted and immediately gets the same `scenario_ended` reply.

## Definition of done

Before considering a change complete, the gate must pass:

```
scripts/gate.sh
```

which is:

```
cargo fmt --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test -p protocol -p movement -p sensors -p transport \
           -p viz -p perception -p map -p map-opendrive
cargo test -p server -j 2
cargo test -p viewer -j 2
(cd clients/python && python3 -m shotgun.selftest)
python3 scripts/check_clients.py
```

The test step is split because a single `cargo test --workspace` links every
test binary at once and exhausts the linker on this machine (bevy's debug info
is enormous); `server` links alone with a reduced job count. CI runs exactly
`scripts/gate.sh`, so the gate and CI cannot drift apart.

The last two steps are the Python client side: `shotgun`'s self-test (the
co-driver toolkit's maths), and a check that every client in
`clients/python/*.py` byte-compiles *and imports*; importing is what
resolves `from shotgun import lane_plan`, so a renamed helper fails in the
gate instead of in front of a running sim.

## Reference

Full rationale, alternatives considered, and the history of how this
design was reached: `DECISIONS.md` (local, gitignored, not part of this
repo's committed history).
