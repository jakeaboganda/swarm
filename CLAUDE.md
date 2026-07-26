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

- **Language: Rust. Engine: Bevy + `bevy_rapier3d`.** One native codebase
  for ECS, 3D rendering, and physics — no browser frontend.
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
    (`sensor` `operator` `threshold` → `action`) evaluated entirely
    server-side, every tick, from ground-truth world state. They override
    plan-following when triggered — zero round-trip latency, since they
    never wait on the agent. Each rule carries an agent-assigned
    `priority` (higher wins on conflict; ties broken by registration
    order).
- **The world runs as a scenario**, not an open join-anytime space: a
  fixed roster of expected agents is declared upfront in a JSON scenario
  file, the server waits until every slot has connected, then runs. If any
  connected agent becomes unavailable, the *entire* scenario ends —
  physics freezes in place (for inspection), remaining agents are
  notified, and the process keeps running until manually killed.
- **Physics is ground-constrained**: gravity on, all rotation locked (no
  tipping/rolling), entities steered via horizontal forces only. Movement
  model is a **pluggable, per-entity** component/trait (`Holonomic` is the
  only implementation in v1; `CarLike`/`FullVehicle` are real future
  possibilities behind the same interface, not scope to avoid).
- **Sensors are a pluggable, per-predicate abstraction** (`sensors`
  crate): v1 ships ground-truth implementations only
  (`time_to_collision`, `distance_to`, `speed`), but the interface is
  shaped so sensor-simulation (noise, limited range/field-of-view,
  latency) can be added later as new implementations of the same trait.
  This is active mid-term work, not indefinitely deferred — don't treat
  the existence of the trait as a reason to avoid extending it.

## Crate layout

Cargo workspace, five crates:

- **`protocol`** — shared `serde` types: WebSocket messages (`join`, plan
  submission, reflex-rule registration, `get_state`/snapshot,
  `reflex_fired`/`scenario_ended`/`error` events), and the scenario JSON
  schema (world/walls + agent roster). Everything else depends on this;
  it depends on nothing else in the workspace.
- **`movement`** — the pluggable embodiment trait + `Holonomic`
  implementation. No networking/scenario knowledge.
- **`sensors`** — the named-sensor abstraction and reflex-rule evaluation
  (predicate readings → threshold checks → actions). Depends on
  `protocol` for rule/message shapes.
- **`transport`** — owns the tokio runtime, per-connection WebSocket
  handling (including heartbeat and reconnect-grace logic), and the
  bounded channels bridging async I/O into Bevy's synchronous ECS tick.
  Depends only on `protocol`.
- **`server`** — the Bevy binary: loads the scenario, runs Rapier physics,
  dispatches movement per entity, invokes sensor evaluation, resolves
  reflex-vs-plan arbitration each tick, manages scenario lifecycle.
  Consumes `transport`'s channels; does not touch tokio/async directly.

## Conventions

- **Formatting/linting**: `rustfmt` defaults, `clippy` enforced at
  `-D warnings`. Anything clippy flags must be fixed or explicitly
  `#[allow]`'d with a comment explaining why.
- **Error handling**: `Result`-based throughout. Library crates
  (`protocol`, `movement`, `sensors`, `transport`) define their own error
  enums via `thiserror`; the `server` binary may use `anyhow` for
  top-level error bubbling. Avoid `unwrap`/`expect` outside tests, except
  for genuinely infallible cases — and those get a short comment saying
  why. Panics are for programmer-error invariant violations, not for
  expected/recoverable conditions (malformed agent input, disconnects,
  scenario JSON errors all fall into the latter).
- **Testing**: required for the pure/deterministic logic crates —
  `protocol` (serialization round-trips), `movement` (seek-controller
  math), `sensors` (predicate evaluation, hysteresis, priority/tiebreak
  resolution). Not required for `server`/`transport` (networking glue, ECS
  wiring, scenario timing) — that code is better verified by running the
  app than by unit tests.
- **Dependencies**: free to add anything already implied by this file
  (`bevy`, `bevy_rapier3d`, `tokio`, `tokio-tungstenite`, `serde`/
  `serde_json`, `thiserror`, `anyhow`) without asking. Anything outside
  that set — a new crate not implied by an existing decision — gets
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
