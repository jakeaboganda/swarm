# swarm

A playground for agentic swarms: a small 3D physics world that external
agent processes connect to and drive vehicles around. Built in Rust with
[Bevy](https://bevyengine.org) (ECS) and [Rapier](https://rapier.rs)
(physics). The **simulation runs headless** and streams the world to
separate **viewer** processes — so you can watch it in a 3D window, run it on
a server with no display, or attach several viewers at once.

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
  in one tick regardless of how slow its controlling agent is to think. A
  rule reads a named **sensor device** — either perfect `ground_truth` or a
  scenario-equipped **simulated** device (limited range/FOV, noise, latency)
  — so an agent can react to *what it actually perceives*, not the truth.

The world runs as a **scenario**: a fixed roster of vehicles declared up
front. The simulation waits for every declared agent to connect, runs, and
ends if any of them drops (after a short reconnect grace window).

Agents are external processes speaking **WebSocket + JSON**, so you can
write one in any language. Three pathways meet at the server, each on its own
port: agents **control** entities (`:4000`), viewers **observe** a semantic
scene stream (MessagePack, `:4001`), and agents optionally subscribe to their
own **simulated perception** (`:4002`) — the impaired view their reflexes and
planning can run on. Viewers are passive: connecting or dropping one never
touches the sim.

## Quick start

Needs a recent Rust toolchain (pinned in `rust-toolchain.toml`) and, on
Linux, the usual windowing/audio dev libraries Bevy requires (for the
viewer). The one-command launcher starts the whole stack:

```sh
scripts/run.sh                 # headless sim + viewer + demo agents
```

Or run the pieces yourself:

```sh
# The headless simulation (no window). Binds :4000 (agents), :4001 (viz),
# and :4002 (perception).
cargo run --bin server -- scenario.json

# The viewer — opens the 3D window and renders the stream. Start it any
# time; it reconnects on its own.
cargo run --bin viewer

# Drive two vehicles (needs `pip install websockets`).
python3 clients/python/patrol_demo.py
```

You'll see two vehicles circle the arena, each trailing its recent path and
showing the waypoints ahead of it. `scenario.json` defines the arena size
and the roster.

> **Stuttery viewer on a hybrid-GPU laptop?** Some GPUs (e.g. NVIDIA
> Runtime-D3 laptops) suspend the dGPU between the viewer's frames and the
> resume latency shows up as periodic hitches. Run the viewer with
> `VIZ_GPU_KEEPALIVE=1` to render uncapped and keep the GPU awake (the
> portable stand-in for running `vkcube` alongside; costs some power).
> `scripts/run.sh` turns it on by default.

### A swarm avoiding itself

For something busier, run dozens of agents that all cross the arena at once
and steer around one another:

```sh
scripts/run.sh scenario_swarm.json clients/python/swarm_avoidance.py
```

![Two dozen agents crossing a shared arena, steering around each other](docs/assets/swarm_demo.gif)

Each agent orbits between a point on a big circle and its antipode, so they
all converge on the center together. The avoidance is the agents' own work,
not the sim's: reflexes can only brake (a safety net), so the actual
steering comes from each agent re-planning every tick against its neighbors
— boids-style separation plus a swirl bias to break head-on deadlocks. This
is the intended split — agents are the brains; the sim follows plans and
enforces reflexes. `--generate N` rewrites the scenario for a different
agent count:

```sh
python3 clients/python/swarm_avoidance.py --generate 36
```

## Writing an agent

An agent connects to `ws://<host>:4000` and exchanges JSON. The essentials:

```jsonc
// -> join the scenario under a declared roster name
{ "type": "join", "name": "car-1" }

// -> a plan: a path of waypoints (position + target speed)
{ "type": "submit_plan", "waypoints": [
    { "position": { "x": 15, "y": 0, "z": 0 }, "speed": 5 }
] }

// -> reflex rules, evaluated server-side every tick. `sensor` names the
//    device to read (the always-available `ground_truth`, or a simulated
//    device the scenario gave this agent); `measure` is the predicate.
{ "type": "register_reflexes", "rules": [
    { "sensor": "ground_truth", "measure": { "kind": "time_to_collision" },
      "operator": "less_than", "threshold": 2.0, "action": "brake",
      "priority": 10 }
] }

// -> ask for a world snapshot whenever you want to (re)plan
{ "type": "get_state" }
```

The server pushes back `joined`, `state` snapshots, `reflex_fired` events
(when a reflex overrides your plan), `scenario_ended`, and `error`. The
exact shapes live in [`crates/protocol`](crates/protocol/). Working clients:
[`agent_smoke.py`](clients/python/agent_smoke.py) (minimal),
[`swarm_avoidance.py`](clients/python/swarm_avoidance.py) (re-plans every
tick against neighbors), and the Rust
[`rust_agent`](crates/server/examples/rust_agent.rs) example.

## Layout

A Cargo workspace of eight crates, each owning one concern:

```
protocol   ── agent wire types (JSON messages) + scenario schema
movement   ── how a vehicle moves: MovementModel + Holonomic/CarLike/FullVehicle
sensors    ── the reflex layer: sensor readings, perception impairment, rules
transport  ── the async agent WebSocket server, bridged into the sync loop
viz        ── the visualization pathway: scene wire types + broadcast server
perception ── the sensor pathway: per-agent simulated-perception wire + server
server     ── the headless simulation binary that wires it all together
viewer     ── the reference 3D visualizer that renders the viz stream
```

Three pathways meet at the `server`: agents talk over `transport` (control,
`:4000`), viewers subscribe over `viz` (observation, `:4001`), and agents
receive their simulated perception over `perception` (`:4002`). The library
crates have READMEs and runnable examples under `crates/<name>/examples/`
(e.g. `cargo run -p sensors --example reflex_brake`).

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

Movement ships three embodiments, selected per agent via the roster's
`embodiment` field: `Holonomic` (moves freely in any direction), `CarLike`
(forward-only thrust, bounded turn rate, lateral grip), and `FullVehicle`
(single-track "bicycle" dynamics — real physical yaw with tire-slip lateral
forces, so understeer/oversteer emerge). `scenario_carlike.json` and
`scenario_bicycle.json` run those.

Sensors ship both ground-truth readings *and* **simulated perception**: the
world equips an agent with named devices (`AgentSlot.sensors`), each either
perfect `ground_truth` or `simulated` with a spec — limited range, field of
view, Gaussian position/velocity noise, and delivery latency. An agent
receives its impaired perception on the `:4002` pathway, and a reflex rule
reads whichever device it names, so imperfect perception has real
consequences. `scenario_sensors.json` shows the impairments; the
[reflex demo](clients/python/perception_reflex_demo.py)
(`scripts/run.sh scenario_reflex_demo.json clients/python/perception_reflex_demo.py`)
puts two identical chasers side by side — one on ground truth stops short,
one on a short-range radar rear-ends its target. The viewer draws each
agent's perception as a debug overlay (toggle detections with **P**, the
sensing envelope with **O**).
