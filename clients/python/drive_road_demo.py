"""Drives a raycast vehicle down its lane, following the map delivered at join.

The server hands the road's lanes in the `joined` message (P3). The agent picks
the forward driving lane, samples its centerline into a plan of waypoints from
where the car spawned to the lane's end, and submits it. The path-tracking
driver follows it -- around the curve and up the grade, staying in-lane.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_car.json
    python3 clients/python/drive_road_demo.py
then watch it in the viewer.

Needs the scenario_road_car.json roster. `pip install websockets`.
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

from stepper import run_clock

URL = "ws://127.0.0.1:4000"
CRUISE = 6.0  # target speed along the lane, m/s
pos = {}


def dist2(a, b):
    return (a["x"] - b["x"]) ** 2 + (a["z"] - b["z"]) ** 2


def lane_plan(lane, start):
    """Waypoints down `lane`'s centerline, from the point nearest `start` to the
    end. Skipping the points behind the car keeps it driving forward, not back
    to the lane's origin."""
    center = lane["centerline"]
    begin = min(range(len(center)), key=lambda i: dist2(center[i], start))
    return [
        {"position": {"x": p["x"], "y": p["y"], "z": p["z"]}, "speed": CRUISE}
        for p in center[begin:]
    ]


def forward_driving_lane(map_data):
    for lane in map_data["lanes"]:
        if lane["kind"] == "driving" and lane["direction"] == "forward":
            return lane
    return None


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    joined = json.loads(await ws.recv())

    map_data = joined.get("map")
    if not map_data:
        print("no map delivered -- is this the automotive scenario?")
        await ws.close()
        return
    lane = forward_driving_lane(map_data)
    if lane is None:
        print("no forward driving lane in the delivered map")
        await ws.close()
        return

    spawn = joined["position"]
    plan = lane_plan(lane, spawn)
    end = plan[-1]["position"]
    print(
        f"following lane {lane['id']} ({len(plan)} waypoints) "
        f"from x={spawn['x']:.1f},z={spawn['z']:.1f} "
        f"to x={end['x']:.1f},z={end['z']:.1f}"
    )

    await ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))

    # The server owns the clock: each step pulse, ask for state; report on the
    # reply. The run ends when the scenario's duration elapses.
    async def report(_sim_time):
        await ws.send(json.dumps({"type": "get_state"}))

    async def on_message(msg):
        if msg.get("type") != "state":
            return
        for e in msg["entities"]:
            pos[e["agent_id"]] = e["position"]
        p = pos.get("car")
        if p:
            # Lateral error off the lane end direction isn't computed here; just
            # show the track so you can eyeball in-lane progress round the curve.
            near = min(lane["centerline"], key=lambda c: dist2(c, p))
            off = math.sqrt(dist2(near, p))
            print(
                f"car  x={p['x']:6.1f}  y={p['y']:5.2f}  z={p['z']:6.2f}"
                f"   lane-offset={off:4.1f} m"
            )

    await run_clock(ws, on_step=report, on_message=on_message, report_dt=0.4)
    await ws.close()


if __name__ == "__main__":
    asyncio.run(main())
