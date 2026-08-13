"""Dozens of agents crossing a shared arena while steering around each other.

Each agent orbits between a point on a big circle and its antipode, so every
agent is perpetually driving through the middle at the same time as everyone
else — a dense, continuous collision-avoidance stress test. Avoidance is
done *client-side*: the server's reflexes are brake-only (a safety net), so
real steering comes from each agent re-planning every control tick against
its neighbors (boids-style separation + a right-hand swirl bias to break
head-on deadlocks). That's the intended split — agents are the brains, the
sim just follows the plan and enforces the reflex.

Usage:
    # generate a matching scenario (roster + arena) sized for N agents:
    python3 clients/python/swarm_avoidance.py --generate 24

    # then run the stack (server first, or use scripts/run.sh):
    ./scripts/run.sh scenario_swarm.json clients/python/swarm_avoidance.py

Reads the roster + arena from the scenario file (default scenario_swarm.json)
so the two never disagree. Needs `pip install websockets`.
"""

import asyncio
import json
import math
import sys

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

SERVER_URL = "ws://127.0.0.1:4000"
DEFAULT_SCENARIO = "scenario_swarm.json"
SPAWN_SPACING = 3.0  # server spreads roster slots this far apart along X

# --- steering tunables ---------------------------------------------------
CONTROL_HZ = 15.0     # replan rate per agent
CRUISE = 6.0          # baseline speed toward the goal
MAX_SPEED = 9.0       # cap after avoidance is added
LOOKAHEAD = 5.0       # how far ahead to place the steering waypoint
ARRIVE_R = 5.0        # switch to the antipodal goal within this range
NEIGHBOR_R = 8.0      # only avoid neighbors closer than this
SEP_GAIN = 16.0       # separation strength
SWIRL = 0.4           # right-hand bias, so agents consistently pass on one side
WALL_MARGIN = 6.0     # start turning away this far from a wall
WALL_GAIN = 12.0
TTC_BRAKE = 1.0       # brake reflex fires under this time-to-collision (s)


def circle_radius(arena):
    return min(arena["width"], arena["depth"]) / 2.0 * 0.72


def generate_scenario(n):
    """Roster of N holonomic agents + an arena wide enough for the server's
    spawn line (roster slots spread SPAWN_SPACING apart along X) plus room to
    maneuver."""
    half = (n - 1) / 2.0 * SPAWN_SPACING + 12.0
    size = round(2.0 * half)
    return {
        "arena": {"width": float(size), "depth": float(size)},
        "roster": [
            {"name": f"agent-{i:02d}", "embodiment": "holonomic"} for i in range(n)
        ],
    }


def steer(name, world, arena, goal):
    """Compute the next steering waypoint for `name` from the shared world
    view. Returns (x, z, speed) or None if we don't know where we are yet."""
    me = world.get(name)
    if me is None:
        return None
    px, pz = me["pos"]

    gx, gz = goal["xz"]
    dx, dz = gx - px, gz - pz
    if math.hypot(dx, dz) < ARRIVE_R:  # reached it — head for the antipode
        goal["xz"] = (-gx, -gz)
        gx, gz = goal["xz"]
        dx, dz = gx - px, gz - pz
    dist = max(math.hypot(dx, dz), 1e-6)

    # Seek the goal.
    vx = dx / dist * CRUISE
    vz = dz / dist * CRUISE

    # Separate from close neighbors, with a perpendicular swirl so two agents
    # closing head-on veer the same way instead of stalling nose to nose.
    for other, o in world.items():
        if other == name:
            continue
        rx, rz = px - o["pos"][0], pz - o["pos"][1]
        d = math.hypot(rx, rz)
        if d < 1e-6 or d > NEIGHBOR_R:
            continue
        w = SEP_GAIN * (NEIGHBOR_R - d) / NEIGHBOR_R / d
        vx += rx * w + rz * w * SWIRL
        vz += rz * w - rx * w * SWIRL

    # Stay off the walls.
    half_w, half_d = arena["width"] / 2.0, arena["depth"] / 2.0
    if px > half_w - WALL_MARGIN:
        vx -= WALL_GAIN
    if px < -half_w + WALL_MARGIN:
        vx += WALL_GAIN
    if pz > half_d - WALL_MARGIN:
        vz -= WALL_GAIN
    if pz < -half_d + WALL_MARGIN:
        vz += WALL_GAIN

    speed = math.hypot(vx, vz)
    if speed < 1e-6:
        return None
    if speed > MAX_SPEED:
        vx, vz, speed = vx * MAX_SPEED / speed, vz * MAX_SPEED / speed, MAX_SPEED
    return px + vx / speed * LOOKAHEAD, pz + vz / speed * LOOKAHEAD, speed


async def read_loop(ws, world, stats, stop, name):
    try:
        async for raw in ws:
            msg = json.loads(raw)
            kind = msg.get("type")
            if kind == "state":
                for e in msg["entities"]:
                    p, v = e["position"], e["velocity"]
                    world[e["agent_id"]] = {"pos": (p["x"], p["z"]), "vel": (v["x"], v["z"])}
            elif kind == "reflex_fired":
                stats["brakes"] += 1
            elif kind == "error":
                # e.g. joining a name that isn't in the loaded scenario's
                # roster — surface it instead of hanging silently.
                print(f"[{name}] server error: {msg.get('message')}")
            elif kind == "scenario_ended":
                print(f"[{name}] scenario ended: {msg.get('reason')}")
                stop.set()
                return
    except websockets.ConnectionClosed:
        stop.set()


async def run_agent(name, angle, arena, world, stats, stop):
    r = circle_radius(arena)
    goal = {"xz": (r * math.cos(angle), r * math.sin(angle))}
    async with websockets.connect(SERVER_URL, ping_interval=None) as ws:
        await ws.send(json.dumps({"type": "join", "name": name}))
        await ws.send(json.dumps({"type": "register_reflexes", "rules": [
            {"sensor": "ground_truth", "measure": {"kind": "time_to_collision"},
             "operator": "less_than",
             "threshold": TTC_BRAKE, "action": "brake", "priority": 10}
        ]}))
        reader = asyncio.create_task(read_loop(ws, world, stats, stop, name))
        try:
            while not stop.is_set():
                await ws.send(json.dumps({"type": "get_state"}))
                cmd = steer(name, world, arena, goal)
                if cmd is not None:
                    x, z, speed = cmd
                    await ws.send(json.dumps({"type": "submit_plan", "waypoints": [
                        {"position": {"x": x, "y": 0.0, "z": z}, "speed": speed}
                    ]}))
                await asyncio.sleep(1.0 / CONTROL_HZ)
        finally:
            reader.cancel()


async def report(world, stats, names, stop):
    """Every 2s: how many agents are moving, their mean speed, brakes since."""
    while not stop.is_set():
        await asyncio.sleep(2.0)
        speeds = [math.hypot(*world[n]["vel"]) for n in names if n in world]
        moving = sum(1 for s in speeds if s > 0.2)
        mean = sum(speeds) / len(speeds) if speeds else 0.0
        print(f"agents={len(speeds)}/{len(names)} moving={moving} "
              f"mean_speed={mean:4.1f} brakes/2s={stats['brakes']}")
        stats["brakes"] = 0


async def main(scenario_path):
    with open(scenario_path) as f:
        cfg = json.load(f)
    arena = cfg["arena"]
    names = [slot["name"] for slot in cfg["roster"]]
    n = len(names)
    print(f"driving {n} agents on a {arena['width']:.0f}x{arena['depth']:.0f} arena, "
          f"antipodal swaps through the center")

    world, stats, stop = {}, {"brakes": 0}, asyncio.Event()
    tasks = [
        run_agent(name, 2 * math.pi * i / n, arena, world, stats, stop)
        for i, name in enumerate(names)
    ]
    tasks.append(report(world, stats, names, stop))
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--generate":
        count = int(args[1]) if len(args) > 1 else 24
        with open(DEFAULT_SCENARIO, "w") as f:
            json.dump(generate_scenario(count), f, indent=2)
            f.write("\n")
        print(f"wrote {DEFAULT_SCENARIO} with {count} agents")
    else:
        asyncio.run(main(args[0] if args else DEFAULT_SCENARIO))
