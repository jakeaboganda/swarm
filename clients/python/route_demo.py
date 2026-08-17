"""Uses the server routing service: ask for a route, then drive it.

The agent asks the server to route from its spawn to a distant point at a
chosen cruise speed. The server snaps both ends to lanes, pathfinds over the
connectivity graph (successors through junctions + lane changes), stamps the
requested speed onto every waypoint, and returns a plan. The agent submits it
and the path-tracking driver follows it -- the agent owns the plan and the
speed; the server just does the mechanical pathfinding.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_real.json
    python3 clients/python/route_demo.py
then watch it in the viewer.

`pip install websockets`.
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
CRUISE = 6.0  # the agent's chosen cruise speed, stamped onto the route


def forward_lane(map_data):
    for lane in map_data["lanes"]:
        if lane["kind"] == "driving" and lane["direction"] == "forward":
            return lane
    return None


async def reader(ws, world):
    async for raw in ws:
        msg = json.loads(raw)
        if msg.get("type") == "state":
            for e in msg["entities"]:
                world[e["agent_id"]] = e["position"]
        elif msg.get("type") == "scenario_ended":
            return


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    joined = json.loads(await ws.recv())
    lane = forward_lane(joined.get("map") or {})
    if lane is None:
        print("no forward driving lane -- is this the automotive scenario?")
        await ws.close()
        return

    spawn = joined["position"]
    # Destination: a point far along the forward lane.
    center = lane["centerline"]
    dest = center[min(len(center) - 1, len(center) * 3 // 4)]
    print(
        f"requesting route from x={spawn['x']:.0f},z={spawn['z']:.0f} "
        f"to x={dest['x']:.0f},z={dest['z']:.0f} at {CRUISE} m/s"
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

    # Read until the route reply arrives.
    route = None
    while route is None:
        msg = json.loads(await ws.recv())
        if msg.get("type") == "route":
            route = msg["waypoints"]
    if not route:
        print("no route found (endpoints unreachable on this map)")
        await ws.close()
        return
    print(f"got a route: {len(route)} waypoints -- submitting and driving it")

    world = {}
    r = asyncio.create_task(reader(ws, world))
    await ws.send(json.dumps({"type": "submit_plan", "waypoints": route}))

    for _ in range(40):
        await ws.send(json.dumps({"type": "get_state"}))
        await asyncio.sleep(0.4)
        p = world.get("car")
        if p:
            print(f"car x={p['x']:8.1f} y={p['y']:5.2f} z={p['z']:8.1f}")

    r.cancel()
    await ws.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
