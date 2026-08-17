# swarm

A playground for agentic driving: a small 3D physics world that external
agent processes connect to and drive vehicles around — in a bounded arena, or
on real road networks. Built in Rust with [Bevy](https://bevyengine.org) (ECS)
and [Rapier](https://rapier.rs) (physics). The **simulation runs headless** and
streams the world to separate **viewer** processes — so you can watch it in a
3D window, run it on a server with no display, or attach several viewers at
once.

It's a dev toy — an evolving sandbox for experimenting with how independent
agents (scripted or LLM-driven) behave in a shared physical space, not a
finished product.

## The idea

An agent controls its vehicle through two layers:

- **Plan** (slow, deliberative) — a path of waypoints, each with a target
  speed. "Drive here, then here." The server steers the vehicle along it.
- **Reflexes** (fast, reactive) — declarative rules like *"if
  time-to-collision < 2s, brake"*, evaluated on the server every physics
  tick. They override the plan the instant they fire, so a vehicle reacts in
  one tick regardless of how slow its controlling agent is to think. A rule
  reads a named **sensor device** — either perfect `ground_truth` or a
  scenario-equipped **simulated** device (limited range/FOV, noise, latency) —
  so an agent can react to *what it actually perceives*, not the truth.

This split is the whole point: **agents are the brains** (deciding where to go
and how to avoid trouble); **the sim is the body and the world** (following
plans, enforcing reflexes, running the physics).

The world runs as a **scenario**: a fixed roster of vehicles declared up
front. The simulation waits for every declared agent to connect, runs, and
ends if any of them drops (after a short reconnect grace window).

Agents are external processes speaking **WebSocket + JSON**, so you can write
one in any language. Three pathways meet at the server, each on its own port:
agents **control** entities (`:4000`), viewers **observe** a semantic scene
stream (MessagePack, `:4001`), and agents optionally subscribe to their own
**simulated perception** (`:4002`) — the impaired view their reflexes and
planning can run on. Viewers are passive: connecting or dropping one never
touches the sim.

## Two worlds

A scenario's `map` field selects which world it builds:

- **Arena** (no `map`) — a flat, walled box. Vehicles move on the ground; the
  arena's size and walls come from the scenario.
- **Road** (`"map": ...`) — a real road network: a graded, curved 3D road with
  lanes. The road is either the built-in hand-authored `"demo"` road or a real
  **OpenDRIVE** (`.xodr`) file baked into the same internal road model at load
  (`"map": "maps/e6mini.xodr"`). Vehicles drive it as a `RaycastVehicle` — four
  ray-cast wheels on spring-damper suspension, with real roll and pitch on the
  terrain. In a road world the agent is **handed the map at join** (lane
  centerlines plus a connectivity graph), so it can lay a lane-following path —
  or ask the server to **route** it from one point to another, across junctions
  and lane changes.

## Quick start

Needs a recent Rust toolchain (pinned in `rust-toolchain.toml`) and, on Linux,
the usual windowing/audio dev libraries Bevy requires (for the viewer). The
one-command launcher starts the whole stack — server, viewer, and the demo
agents — and tears it down on `Ctrl-C`:

```sh
scripts/run.sh                 # headless sim + viewer + two patrol agents
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
showing the waypoints ahead of it. `scenario.json` defines the arena size and
the roster.

> **Stuttery viewer on a hybrid-GPU laptop?** Some GPUs (e.g. NVIDIA
> Runtime-D3 laptops) suspend the dGPU between the viewer's frames and the
> resume latency shows up as periodic hitches. Run the viewer with
> `VIZ_GPU_KEEPALIVE=1` to render uncapped and keep the GPU awake (the portable
> stand-in for running `vkcube` alongside; costs some power). `scripts/run.sh`
> turns it on by default.

### More to try

`scripts/run.sh <scenario> <client>` runs any pairing. A tour of what the
sandbox can do:

```sh
# A swarm: dozens of agents crossing a shared arena, steering around each other
scripts/run.sh scenario_swarm.json clients/python/swarm_avoidance.py

# Drive a lane on the 3D road (through a 90-degree curve, up a 4% grade)
scripts/run.sh scenario_road_car.json clients/python/drive_road_demo.py

# Brake for a *perceived* obstacle: the car sees a barrier on radar and stops
scripts/run.sh scenario_road_obstacle.json clients/python/brake_road_demo.py

# Same, but the radar is a frustum (finite vertical FOV) instead of a wedge --
# press O in the viewer to see the sensing volume close top and bottom
scripts/run.sh scenario_road_frustum.json clients/python/brake_road_demo.py

# Perception matters: two identical chasers, one with a short-range radar,
# rear-ends its target because it "sees" it too late
scripts/run.sh scenario_reflex_demo.json clients/python/perception_reflex_demo.py

# Ask the server to route across CARLA's Town07 (234 roads, 31 junctions)
scripts/run.sh scenario_road_town.json clients/python/route_town_demo.py

# A whole fleet: 8 cars route across Town07 at once, each with a radar and a
# forward-collision reflex, braking to yield when a faster car closes on a
# slower one sharing a corridor
scripts/run.sh scenario_road_fleet.json clients/python/fleet_town_demo.py
```

In the viewer: **F** follows/unfollows the nearest vehicle (chase-cam), **Tab**
cycles vehicles, **P** toggles the perception overlay (what each agent detects),
**O** toggles the sensing envelope (its range/field-of-view).

## Writing an agent

An agent connects to `ws://<host>:4000` and exchanges JSON. The essentials:

```jsonc
// -> join the scenario under a declared roster name
{ "type": "join", "name": "car-1" }

// The server replies with `joined`, carrying your spawn position and — in a
// road world — the `map`: every lane's centerline, width, and connectivity
// (successors / predecessors / lane-change neighbors).

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

// -> ask the server to route between two points at a cruise speed (road
//    worlds). The reply is a `route` of waypoints — a ready-to-submit plan.
{ "type": "request_route",
  "from": { "x": 2, "y": 0, "z": -10 },
  "to":   { "x": 90, "y": 0, "z": -1094 },
  "speed": 6 }

// -> ask for a world snapshot whenever you want to (re)plan
{ "type": "get_state" }
```

The server pushes back `joined`, `state` snapshots, `reflex_fired` events
(when a reflex overrides your plan), `route` replies, `scenario_ended`, and
`error`. The exact shapes live in [`crates/protocol`](crates/protocol/).
Routing is your choice, not a mandate: the server does the pathfinding as a
convenience, but the returned plan — and its speed — is yours to submit,
edit, or ignore.

Working clients under [`clients/python/`](clients/python/):
[`agent_smoke.py`](clients/python/agent_smoke.py) (minimal),
[`swarm_avoidance.py`](clients/python/swarm_avoidance.py) (re-plans every tick
against neighbors), [`drive_road_demo.py`](clients/python/drive_road_demo.py)
(lane-following), [`route_demo.py`](clients/python/route_demo.py) (routing),
and the Rust [`rust_agent`](crates/server/examples/rust_agent.rs) example.

## Layout

A Cargo workspace of ten crates, each owning one concern:

```
protocol      ── agent wire types (JSON messages) + the scenario schema
movement      ── how a vehicle moves: MovementModel + the embodiments
sensors       ── the reflex layer: sensor readings, perception impairment, rules
transport     ── the async agent WebSocket server, bridged into the sync loop
viz           ── the visualization pathway: scene wire types + broadcast server
perception    ── the sensor pathway: per-agent simulated-perception wire + server
map           ── the road-network model: lanes, geometry, mesh, routing
map-opendrive ── the pure-Rust OpenDRIVE (.xodr) importer that bakes into `map`
server        ── the headless simulation binary that wires it all together
viewer        ── the reference 3D visualizer that renders the viz stream
```

Three pathways meet at the `server`: agents talk over `transport` (control,
`:4000`), viewers subscribe over `viz` (observation, `:4001`), and agents
receive their simulated perception over `perception` (`:4002`). The library
crates have runnable examples under `crates/<name>/examples/` (e.g.
`cargo run -p sensors --example reflex_brake`).

## Development

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

Conventions, architecture notes, and the correctness guardrails that aren't
obvious from the code are in [CLAUDE.md](CLAUDE.md). This repository uses
Test-Driven Development.

## What's in the box

**Movement** ships four embodiments, selected per agent via the roster's
`embodiment` field:

- `Holonomic` — moves freely in any direction (a puck/drone).
- `CarLike` — forward-only thrust, bounded turn rate, lateral grip.
- `FullVehicle` — single-track "bicycle" dynamics: real physical yaw with
  tire-slip lateral forces, so understeer/oversteer emerge.
- `RaycastVehicle` — the road-driving vehicle: four ray-cast wheels with
  spring-damper suspension and tire grip, roll and pitch as real physics, so it
  follows the terrain (grade, banking). `scenario_carlike.json`,
  `scenario_bicycle.json`, and the `scenario_road_*.json` files run these.

**Perception** ships both ground-truth readings *and* **simulated perception**:
the world equips an agent with named devices (`AgentSlot.sensors`), each either
perfect `ground_truth` or `simulated` with a spec — limited range, field of
view, Gaussian position/velocity noise, and delivery latency. An agent receives
its impaired perception on the `:4002` pathway, and a reflex rule reads
whichever device it names, so imperfect perception has real consequences.
`scenario_sensors.json` shows the impairments; the viewer draws each agent's
perception as a debug overlay (**P** for detections, **O** for the sensing
envelope).

**Roads** come from an internal, format-agnostic road-network model (`map`).
The built-in `demo` road is hand-authored; the `map-opendrive` crate is a pure-
Rust **OpenDRIVE importer** that bakes a real `.xodr` file into that same model
at load — line / arc / spiral / paramPoly3 / poly3 geometry, elevation,
per-lane widths, lane offsets, and multiple lane sections. Its geometry is
cross-checked against the reference C++ [libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE).
`scenario_road_real.json` drives a real esmini highway; `scenario_road_town.json`
loads CARLA's Town07.

**Routing** turns the road network into "drive from A to B". The importer
resolves road/lane links and junction connections into a **connectivity graph**
(each lane's drivable successors, plus lane-change neighbors); a router
(`map::RoadNetwork::route`) finds the shortest lane path over it and samples a
plan. The graph is delivered to the agent at join, so an agent can route itself;
the server also offers a `request_route` service that returns a ready plan.
`route_town_demo.py` routes one car across Town07's junctions;
`fleet_town_demo.py` routes a whole fleet of 8 at once — the server fans them
out across the map's forward lanes, and each perceives the others through an
impaired radar and brakes to yield when it closes on slower traffic.
