"""Shows a safety reflex firing on SIMULATED perception — too late.

Two identical chasers charge down separate lanes at a stationary target, each
armed with the same "stop if time-to-collision < threshold" reflex:

  * gt-chaser reads the reserved `ground_truth` device — it sees the target
    from any distance and stops with a clear margin.
  * sim-chaser reads a `radar` device the scenario made short-range (6 units).
    It doesn't perceive the target until it's almost on top of it — far too
    late to stop — so it rear-ends it.

Same rule, same speed, same threshold. The only difference is which device the
reflex reads, and that's a property of the sensor, not the rule.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_reflex_demo.json
    python3 clients/python/perception_reflex_demo.py

Needs the scenario_reflex_demo.json roster. `pip install websockets`.
"""

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

URL = "ws://127.0.0.1:4000"
LANE = {"gt-chaser": 8.0, "gt-target": 8.0, "sim-chaser": -8.0, "sim-target": -8.0}
TARGET_X = 16.0  # where each target parks
START_X = -20.0  # where each chaser lines up before charging
END_X = 26.0  # aim past the target so it's the reflex that stops the chase
LANE_SPEED = 6.0
CHASE_SPEED = 9.0
TTC_THRESHOLD = 1.2

world = {}  # agent_id -> (x, z)
fires = {"gt-chaser": 0, "sim-chaser": 0}


def wp(x, z, speed):
    return {"position": {"x": float(x), "y": 0.0, "z": float(z)}, "speed": float(speed)}


def plan(waypoints):
    return json.dumps({"type": "submit_plan", "waypoints": waypoints})


def brake_reflex(device):
    # stop_and_hold, not brake, so a triggered chaser parks cleanly where it
    # first "saw" trouble instead of creeping forward as ttc oscillates.
    return json.dumps(
        {
            "type": "register_reflexes",
            "rules": [
                {
                    "sensor": device,
                    "measure": {"kind": "time_to_collision"},
                    "operator": "less_than",
                    "threshold": TTC_THRESHOLD,
                    "action": "stop_and_hold",
                    "priority": 10,
                }
            ],
        }
    )


async def reader(ws, name):
    async for raw in ws:
        msg = json.loads(raw)
        kind = msg.get("type")
        if kind == "state":
            for e in msg["entities"]:
                world[e["agent_id"]] = (e["position"]["x"], e["position"]["z"])
        elif kind == "reflex_fired" and name in fires:
            fires[name] += 1
        elif kind == "scenario_ended":
            return


async def poll(ws):
    while True:
        await ws.send(json.dumps({"type": "get_state"}))
        await asyncio.sleep(0.2)


async def join(name):
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": name}))
    await ws.recv()  # Joined
    return ws


def gap(chaser, target):
    """Distance along the lane between a chaser and its target."""
    if chaser not in world or target not in world:
        return None
    return abs(world[target][0] - world[chaser][0])


def summary():
    print("\n=== result ===")
    for chaser, target in (("gt-chaser", "gt-target"), ("sim-chaser", "sim-target")):
        g = gap(chaser, target)
        g = f"{g:.1f}" if g is not None else "?"
        device = "ground_truth" if chaser == "gt-chaser" else "radar (range 4)"
        print(f"{chaser:<11} via {device:<16} stopped {g} units from its target "
              f"({fires[chaser]} reflex fires)")
    print("\ngt-chaser saw the target early and stopped short; sim-chaser's short-range")
    print("radar revealed it too late to stop — the same reflex, a worse sensor.")


async def main():
    names = ["gt-target", "sim-target", "gt-chaser", "sim-chaser"]
    ws = {n: await join(n) for n in names}
    tasks = [asyncio.create_task(reader(ws[n], n)) for n in names]

    # Targets roll out to their parking spots and idle there.
    for t in ("gt-target", "sim-target"):
        await ws[t].send(plan([wp(TARGET_X, LANE[t], 5.0)]))

    # Chasers line up at the far end of their lane (no reflex yet — the agents
    # spawn clustered, so an armed stop_and_hold would trip on a neighbor).
    for c in ("gt-chaser", "sim-chaser"):
        await ws[c].send(plan([wp(START_X, LANE[c], LANE_SPEED)]))
        tasks.append(asyncio.create_task(poll(ws[c])))

    print("targets parking, chasers lining up...")
    await asyncio.sleep(4.5)

    # Now arm the reflex and charge straight at the parked target.
    print("charge!")
    for c in ("gt-chaser", "sim-chaser"):
        device = "ground_truth" if c == "gt-chaser" else "radar"
        await ws[c].send(brake_reflex(device))
        await ws[c].send(plan([wp(END_X, LANE[c], CHASE_SPEED)]))
    await asyncio.sleep(8.0)

    for t in tasks:
        t.cancel()
    summary()
    for connection in ws.values():
        await connection.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
