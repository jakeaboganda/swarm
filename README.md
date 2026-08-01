# swarm

A playground for agentic swarms: a small 3D physics world that external
agent processes connect to and drive vehicles around. Built in Rust with
[Bevy](https://bevyengine.org) (ECS + rendering) and
[Rapier](https://rapier.rs) (physics), it opens a live 3D window so you can
watch agents move, plan, and react.

It's a dev toy — an evolving sandbox for experimenting with how independent
agents (scripted or LLM-driven) behave in a shared physical space, not a
finished product.

## The idea

An agent controls its vehicle through two layers:

- **Plan** (slow, deliberative) — a path of waypoints, each with a target
  speed. "Drive here, then here." The server follows it.
- **Reflexes** (fast, reactive) — declarative rules like *"if
  time-to-collision < 2s, brake"*, evaluated on the server every physics
  tick. They override the plan the instant they fire, so a vehicle reacts
  in one tick regardless of how slow its controlling agent is to think.

The world runs as a **scenario**: a fixed roster of vehicles declared up
front. The simulation waits for every declared agent to connect, runs, and
ends if any of them drops (after a short reconnect grace window).

Agents are external processes speaking **WebSocket + JSON**, so you can
write one in any language.

## Quick start

Needs a recent Rust toolchain (pinned in `rust-toolchain.toml`) and, on
Linux, the usual windowing/audio dev libraries Bevy requires.

```sh
# Terminal 1 — starts the simulation and opens the 3D window.
cargo run --bin server -- scenario.json

# Terminal 2 — drives two vehicles once the window is up (needs
# `pip install websockets`).
python3 clients/python/patrol_demo.py
```

You'll see two vehicles circle the arena, each trailing its recent path and
showing the waypoints ahead of it. `scenario.json` defines the arena size
and the roster.

## Writing an agent

An agent connects to `ws://<host>:4000` and exchanges JSON. The essentials:

```jsonc
// -> join the scenario under a declared roster name
{ "type": "join", "name": "car-1" }

// -> a plan: a path of waypoints (position + target speed)
{ "type": "submit_plan", "waypoints": [
    { "position": { "x": 15, "y": 0, "z": 0 }, "speed": 5 }
] }

// -> reflex rules, evaluated server-side every tick
{ "type": "register_reflexes", "rules": [
    { "sensor": { "kind": "time_to_collision" }, "operator": "less_than",
      "threshold": 2.0, "action": "brake", "priority": 10 }
] }

// -> ask for a world snapshot whenever you want to (re)plan
{ "type": "get_state" }
```

The server pushes back `joined`, `state` snapshots, `reflex_fired` events
(when a reflex overrides your plan), `scenario_ended`, and `error`. The
exact shapes live in [`crates/protocol`](crates/protocol/). Working clients:
[`clients/python/agent_smoke.py`](clients/python/agent_smoke.py) and the
Rust [`rust_agent`](crates/server/examples/rust_agent.rs) example.

## Layout

A Cargo workspace of five crates, each owning one concern:

```
protocol  ── wire types (JSON messages) + scenario schema; depends on nothing
movement  ── how a vehicle moves: the MovementModel trait + Holonomic
sensors   ── the reflex layer: sensors + rule evaluation
transport ── the async WebSocket server, bridged into the sync game loop
server    ── the Bevy binary that wires it all together and runs physics
```

Each crate has its own README and a runnable example under
`crates/<name>/examples/` (e.g. `cargo run -p sensors --example
reflex_brake`).

## Development

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

Conventions, architecture notes, and the correctness guardrails that aren't
obvious from the code are in [CLAUDE.md](CLAUDE.md). This repository uses
Test-Driven Development.

## Status

Ships two movement models — `Holonomic` (moves freely in any direction)
and `CarLike` (forward-only thrust, bounded turn rate, lateral grip) —
selected per agent via the roster's `embodiment` field
(`scenario_carlike.json` runs an all-car-like scenario). Plus ground-truth
sensors and a walled arena. The movement and sensor layers are extension
points: `FullVehicle` physics and simulated sensors (noise, limited range,
latency) are intended future work behind the same interfaces.
