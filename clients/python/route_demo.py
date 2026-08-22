"""Uses the server routing service: ask for a route, then drive it.

The agent asks the server to route from its spawn to a distant point at a
chosen cruise speed. The server snaps both ends to lanes, pathfinds over the
connectivity graph (successors through junctions + lane changes), stamps the
requested speed onto every waypoint, and returns a plan.

That speed is flat -- one number on every waypoint, corners included -- because
the server does the pathfinding, not the driving. So the agent re-times the
route for its corners (`shotgun.retime`) before submitting it: the plan and its
speeds are the agent's, all the way down.

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

from shotgun import pick_driving_lane, retime
from stepper import run_clock

URL = "ws://127.0.0.1:4000"
CRUISE = 6.0  # the agent's chosen cruise speed, stamped onto the route


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    joined = json.loads(await ws.recv())
    lane = pick_driving_lane(joined.get("map"))
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
    print(f"got a route: {len(route)} waypoints")

    # The server stamps ONE speed on every waypoint -- it does the pathfinding,
    # not the driving. Re-time the route for its corners before submitting:
    # slow enough for each bend, and slowing early enough to arrive at it.
    plan = retime(route, CRUISE)
    slowest = min(wp["speed"] for wp in plan)
    print(
        f"re-timed for the corners: {slowest:.1f} to {CRUISE:.1f} m/s "
        "-- submitting and driving it"
    )

    world = {}
    await ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))

    # Server-owned clock: report on each step pulse; end at the scenario's
    # duration.
    async def report(_sim_time):
        await ws.send(json.dumps({"type": "get_state"}))

    async def on_message(msg):
        if msg.get("type") != "state":
            return
        for e in msg["entities"]:
            world[e["agent_id"]] = e["position"]
        p = world.get("car")
        if p:
            print(f"car x={p['x']:8.1f} y={p['y']:5.2f} z={p['z']:8.1f}")

    await run_clock(ws, on_step=report, on_message=on_message, report_dt=0.4)
    await ws.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
