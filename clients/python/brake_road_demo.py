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
import sys

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

from shotgun import dist, lane_plan, pick_driving_lane, stop_on_ttc, waypoints
from stepper import run_clock

URL = "ws://127.0.0.1:4000"
CRUISE = 6.0  # car target speed along the lane, m/s
TTC_THRESHOLD = 2.0  # brake when perceived time-to-collision drops below this, s

world = {}  # agent_id -> position dict
fires = 0


async def drain(ws):
    """Keep a secondary connection (the obstacle) responsive until the scenario
    ends, so it doesn't drop and end the run early."""
    try:
        async for raw in ws:
            if json.loads(raw).get("type") == "scenario_ended":
                return
    except websockets.ConnectionClosed:
        return


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
    lane = pick_driving_lane(car_joined.get("map"))
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

    obstacle_task = None
    if have_obstacle:
        obstacle_task = asyncio.create_task(drain(obs_ws))
        # The obstacle spawns already parked in the lane (server-placed
        # downroad). Hold it there with a station-keep plan on its own spawn
        # point, so it doesn't creep down the graded, frictionless surface.
        spot = obs_joined["position"]
        await obs_ws.send(
            json.dumps({"type": "submit_plan", "waypoints": waypoints([spot], 1.0)})
        )
        print(f"obstacle parked at x={spot['x']:.1f}, z={spot['z']:.1f}")
        await asyncio.sleep(1.5)

    # Arm the reflex, then send the car down its lane straight at the obstacle.
    plan = lane_plan(lane, car_joined["position"], CRUISE)
    # Read from `radar`, not ground truth: the car stops for what it perceives.
    await car_ws.send(json.dumps({
        "type": "register_reflexes",
        "rules": [stop_on_ttc(TTC_THRESHOLD, sensor="radar")],
    }))
    await car_ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))
    print(f"car charging down lane {lane['id']} (radar TTC < {TTC_THRESHOLD}s -> stop)")

    # Server-owned clock: on each pulse, ask for state; count reflex fires from
    # the pushed events. The scenario's duration ends the run.
    async def report(_sim_time):
        await car_ws.send(json.dumps({"type": "get_state"}))
        car, obs = world.get("car"), world.get("obstacle")
        if not car:
            return
        line = f"car x={car['x']:6.1f} y={car['y']:5.2f} z={car['z']:5.2f}"
        if obs:
            line += f"   obstacle x={obs['x']:5.1f}   gap={dist(car, obs):5.1f}"
        print(f"{line}   fires={fires}")

    async def on_message(msg):
        global fires
        kind = msg.get("type")
        if kind == "state":
            for e in msg["entities"]:
                world[e["agent_id"]] = e["position"]
        elif kind == "reflex_fired":
            fires += 1

    await run_clock(car_ws, on_step=report, on_message=on_message, report_dt=0.4)

    car, obs = world.get("car"), world.get("obstacle")
    print("\n=== result ===")
    if car and obs:
        gap = dist(car, obs)
        print(f"car stopped {gap:.1f} m from the obstacle ({fires} reflex fires). "
              "Perceived it via radar and braked in time."
              if fires else
              f"car never braked (gap {gap:.1f} m) -- check radar range/FOV.")
    elif car:
        print(f"car drove to x={car['x']:.1f} (no obstacle in the scenario).")

    if obstacle_task:
        obstacle_task.cancel()
    for w in (car_ws, obs_ws):
        await w.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
