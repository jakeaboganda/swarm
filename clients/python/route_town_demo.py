"""Route across a real town through junctions, and drive it.

Loads CARLA's Town07 (234 roads, 31 junctions) and shows the full routing
stack on a real map:

  1. The connectivity graph is delivered over the wire, so this client walks it
     itself (BFS over successors + lane-change neighbors from the car's lane) to
     find a *reachable* lane far from the spawn -- guaranteeing the destination
     is across junctions, not just down one road.
  2. It then asks the *server* routing service for the actual plan
     (request_route), which pathfinds the same graph and stamps the requested
     cruise speed onto every waypoint.
  3. It submits the plan and the path-tracking driver follows it across town.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_town.json
    python3 clients/python/route_town_demo.py
then watch it in the viewer (F to chase-cam the car through the junctions).

`pip install websockets`.
"""

import asyncio
import json
import sys
from collections import deque

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

from stepper import run_clock

URL = "ws://127.0.0.1:4000"
CRUISE = 6.0


def d2(a, b):
    return (a["x"] - b["x"]) ** 2 + (a["z"] - b["z"]) ** 2


def nearest_lane(lanes, p):
    """The lane whose centerline passes closest to point p."""
    return min(lanes, key=lambda l: min(d2(c, p) for c in l["centerline"]))


def farthest_reachable(lanes, start_id, spawn):
    """BFS the delivered graph from `start_id` over successors + neighbors;
    return the reachable lane whose middle is farthest from spawn."""
    by_id = {l["id"]: l for l in lanes}
    seen = {start_id}
    q = deque([start_id])
    while q:
        lid = q.popleft()
        lane = by_id.get(lid)
        if not lane:
            continue
        for nxt in lane["successors"] + lane["neighbors"]:
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    reachable = [by_id[i] for i in seen if i in by_id]

    def mid(lane):
        return lane["centerline"][len(lane["centerline"]) // 2]

    return max(reachable, key=lambda l: d2(mid(l), spawn)), len(reachable)


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    joined = json.loads(await ws.recv())
    map_data = joined.get("map")
    if not map_data:
        print("no map delivered -- is this the Town07 scenario?")
        await ws.close()
        return
    lanes = [l for l in map_data["lanes"] if l["kind"] == "driving"]
    spawn = joined["position"]
    print(f"Town07 loaded: {len(lanes)} driving lanes. car at x={spawn['x']:.0f},z={spawn['z']:.0f}")

    start = nearest_lane(lanes, spawn)
    dest_lane, reach = farthest_reachable(lanes, start["id"], spawn)
    dest = dest_lane["centerline"][len(dest_lane["centerline"]) // 2]
    straight = d2(dest, spawn) ** 0.5
    print(
        f"{reach} lanes reachable from lane {start['id']}; "
        f"routing to lane {dest_lane['id']} at x={dest['x']:.0f},z={dest['z']:.0f} "
        f"({straight:.0f} m away straight-line)"
    )

    await ws.send(
        json.dumps(
            {
                "type": "request_route",
                "from": spawn,
                "to": {"x": dest["x"], "y": dest["y"], "z": dest["z"]},
                "speed": CRUISE,
            }
        )
    )
    route = None
    while route is None:
        msg = json.loads(await ws.recv())
        if msg.get("type") == "route":
            route = msg["waypoints"]
    if not route:
        print("server found no route -- try re-running (spawn/destination vary).")
        await ws.close()
        return
    print(f"server route: {len(route)} waypoints -- driving it across town")

    world = {}
    await ws.send(json.dumps({"type": "submit_plan", "waypoints": route}))

    # Server-owned clock: report progress on each step pulse; the scenario's
    # duration ends the drive.
    async def report(_sim_time):
        await ws.send(json.dumps({"type": "get_state"}))

    async def on_message(msg):
        if msg.get("type") != "state":
            return
        for e in msg["entities"]:
            world[e["agent_id"]] = e["position"]
        p = world.get("car")
        if p:
            remaining = d2(p, dest) ** 0.5
            print(f"car x={p['x']:7.1f} z={p['z']:7.1f}   {remaining:6.0f} m to go")

    await run_clock(ws, on_step=report, on_message=on_message, report_dt=0.5)
    await ws.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
