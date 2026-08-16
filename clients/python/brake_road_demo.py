"""The automotive MVP: a car brakes for a PERCEIVED obstacle on the road.

An obstacle (a parked holonomic entity) sits in the forward lane ~30 m along.
The car follows the lane it's handed at join (P3), but with one safety reflex:

    time_to_collision(radar) < THRESHOLD  ->  stop_and_hold

`radar` is a simulated device (finite range, forward FOV, noise, latency), so
the car reacts to what it *perceives*, not ground truth. It detects the parked
obstacle as it closes, the reflex trips, and the car stops short of it -- no
round-trip to the agent, evaluated server-side every tick.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_obstacle.json
    python3 clients/python/brake_road_demo.py
then watch it in the viewer (press F to chase-cam the car).

Needs the scenario_road_obstacle.json roster. `pip install websockets`.
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

URL = "ws://127.0.0.1:4000"
CRUISE = 6.0  # car target speed along the lane, m/s
PARK_X = 30.0  # where the obstacle parks along the straight (x ~= arc length)
TTC_THRESHOLD = 2.0  # brake when perceived time-to-collision drops below this, s

world = {}  # agent_id -> position dict
fires = 0


def dist2(a, b):
    return (a["x"] - b["x"]) ** 2 + (a["z"] - b["z"]) ** 2


def forward_driving_lane(map_data):
    for lane in map_data["lanes"]:
        if lane["kind"] == "driving" and lane["direction"] == "forward":
            return lane
    return None


def wp(p, speed):
    return {"position": {"x": p["x"], "y": p["y"], "z": p["z"]}, "speed": speed}


def lane_plan(lane, start, speed):
    """Waypoints down the lane centerline from the point nearest `start` on."""
    center = lane["centerline"]
    begin = min(range(len(center)), key=lambda i: dist2(center[i], start))
    return [wp(p, speed) for p in center[begin:]]


def nearest_to_x(lane, x):
    """The centerline point closest to a given x -- the obstacle's park spot."""
    return min(lane["centerline"], key=lambda p: abs(p["x"] - x))


def brake_reflex():
    return {
        "type": "register_reflexes",
        "rules": [
            {
                "sensor": "radar",
                "measure": {"kind": "time_to_collision"},
                "operator": "less_than",
                "threshold": TTC_THRESHOLD,
                "action": "stop_and_hold",
                "priority": 10,
            }
        ],
    }


async def reader(ws, name):
    global fires
    async for raw in ws:
        msg = json.loads(raw)
        kind = msg.get("type")
        if kind == "state":
            for e in msg["entities"]:
                world[e["agent_id"]] = e["position"]
        elif kind == "reflex_fired" and name == "car":
            fires += 1
        elif kind == "scenario_ended":
            return


async def poll(ws):
    while True:
        await ws.send(json.dumps({"type": "get_state"}))
        await asyncio.sleep(0.2)


async def join(name):
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": name}))
    joined = json.loads(await ws.recv())
    return ws, joined


async def main():
    car_ws, car_joined = await join("car")
    obs_ws, obs_joined = await join("obstacle")

    lane = forward_driving_lane(car_joined.get("map") or {})
    if lane is None:
        print("no forward driving lane -- is this the automotive scenario?")
        for w in (car_ws, obs_ws):
            await w.close()
        return

    tasks = [
        asyncio.create_task(reader(car_ws, "car")),
        asyncio.create_task(reader(obs_ws, "obstacle")),
        asyncio.create_task(poll(car_ws)),
    ]

    # Obstacle drives into the lane and parks (keeps the plan, so it station-
    # keeps against the graded, frictionless surface instead of sliding).
    park = nearest_to_x(lane, PARK_X)
    await obs_ws.send(json.dumps({"type": "submit_plan", "waypoints": [wp(park, 4.0)]}))
    print(f"obstacle moving to park at x={park['x']:.1f}, z={park['z']:.1f} ...")
    await asyncio.sleep(5.0)

    # Arm the reflex, then send the car down its lane straight at the obstacle.
    plan = lane_plan(lane, car_joined["position"], CRUISE)
    await car_ws.send(json.dumps(brake_reflex()))
    await car_ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))
    print(f"car charging down lane {lane['id']} (radar TTC < {TTC_THRESHOLD}s -> stop)")

    for _ in range(30):
        await asyncio.sleep(0.4)
        car, obs = world.get("car"), world.get("obstacle")
        if car and obs:
            gap = math.sqrt(dist2(car, obs))
            print(
                f"car x={car['x']:6.1f} y={car['y']:5.2f} z={car['z']:5.2f}"
                f"   obstacle x={obs['x']:5.1f} y={obs['y']:5.2f} z={obs['z']:5.2f}"
                f"   gap={gap:5.1f}  fires={fires}"
            )

    car, obs = world.get("car"), world.get("obstacle")
    if car and obs:
        gap = math.sqrt(dist2(car, obs))
        print(f"\n=== result ===\ncar stopped {gap:.1f} m from the obstacle "
              f"({fires} reflex fires). Perceived it via radar and braked in time."
              if fires else
              f"\n=== result ===\ncar never braked (gap {gap:.1f} m) -- check radar range/FOV.")

    for t in tasks:
        t.cancel()
    for w in (car_ws, obs_ws):
        await w.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
