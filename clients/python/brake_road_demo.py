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
    """Connect and join. Returns (ws, reply); reply["type"] == "joined" on
    success, else an error reply (the caller decides how fatal that is)."""
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": name}))
    return ws, json.loads(await ws.recv())


WRONG_SCENARIO = (
    "Run the P4 obstacle scenario (2 slots: car + obstacle):\n"
    "    scripts/run.sh scenario_road_obstacle.json clients/python/brake_road_demo.py"
)


async def main():
    car_ws, car_joined = await join("car")
    if car_joined.get("type") != "joined":
        print(f"car join refused: {car_joined.get('message', car_joined)}\n{WRONG_SCENARIO}")
        await car_ws.close()
        return
    lane = forward_driving_lane(car_joined.get("map") or {})
    if lane is None:
        print(f"no forward driving lane -- not the automotive scenario?\n{WRONG_SCENARIO}")
        await car_ws.close()
        return

    # The obstacle is optional: without it the car just drives the lane. So a
    # refused obstacle join is a warning, never a reason to leave the car idle.
    obs_ws, obs_joined = await join("obstacle")
    have_obstacle = obs_joined.get("type") == "joined"
    if not have_obstacle:
        print(f"NOTE: no obstacle ({obs_joined.get('message', obs_joined)}); "
              f"car will just drive the lane.\n{WRONG_SCENARIO}")

    tasks = [asyncio.create_task(reader(car_ws, "car")), asyncio.create_task(poll(car_ws))]
    if have_obstacle:
        tasks.append(asyncio.create_task(reader(obs_ws, "obstacle")))
        # The obstacle spawns already parked in the lane (server-placed
        # downroad). Hold it there with a station-keep plan on its own spawn
        # point, so it doesn't creep down the graded, frictionless surface.
        spot = obs_joined["position"]
        await obs_ws.send(json.dumps({"type": "submit_plan", "waypoints": [wp(spot, 1.0)]}))
        print(f"obstacle parked at x={spot['x']:.1f}, z={spot['z']:.1f}")
        await asyncio.sleep(1.5)

    # Arm the reflex, then send the car down its lane straight at the obstacle.
    plan = lane_plan(lane, car_joined["position"], CRUISE)
    await car_ws.send(json.dumps(brake_reflex()))
    await car_ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))
    print(f"car charging down lane {lane['id']} (radar TTC < {TTC_THRESHOLD}s -> stop)")

    for _ in range(30):
        await asyncio.sleep(0.4)
        car, obs = world.get("car"), world.get("obstacle")
        if not car:
            continue
        line = f"car x={car['x']:6.1f} y={car['y']:5.2f} z={car['z']:5.2f}"
        if obs:
            line += f"   obstacle x={obs['x']:5.1f}   gap={math.sqrt(dist2(car, obs)):5.1f}"
        print(f"{line}   fires={fires}")

    car, obs = world.get("car"), world.get("obstacle")
    print("\n=== result ===")
    if car and obs:
        gap = math.sqrt(dist2(car, obs))
        print(f"car stopped {gap:.1f} m from the obstacle ({fires} reflex fires). "
              "Perceived it via radar and braked in time."
              if fires else
              f"car never braked (gap {gap:.1f} m) -- check radar range/FOV.")
    elif car:
        print(f"car drove to x={car['x']:.1f} (no obstacle in the scenario).")

    for t in tasks:
        t.cancel()
    for w in (car_ws, obs_ws):
        await w.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
