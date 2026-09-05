"""Drives the wheel/tire test track to a fixed script, for tuning by eye.

Every section of `maps/testtrack.xodr` targets something the vehicle model does
-- launch wheelspin, brake lockup, suspension travel over a crest, roll and load
transfer through a corner, understeer at the limit, and a decreasing-radius
spiral. This client drives them in order at the *same commanded numbers every
run*, so a difference on screen between two tuning passes is a difference in the
physics and not a difference in how it was driven. That is why it is scripted
rather than routed: a route would come out slightly different each run and muddy
the comparison.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_wheels.json
    python3 clients/python/wheels_demo.py
then watch it in the viewer.

Needs the scenario_wheels.json roster. `pip install websockets`.
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

from shotgun import (
    arc_lengths,
    nearest_index,
    pick_driving_lane,
    project_point,
    stop_and_hold_above,
    waypoints,
)
from stepper import run_clock

URL = "ws://127.0.0.1:4000"

# (arc length along the lane, commanded speed, what the section is for).
# Speeds are what the car is *asked* for; what it achieves is the measurement.
SCRIPT = [
    (0.0, 25.0, "launch: full throttle, wheelspin off the line"),
    (240.0, 0.0, "emergency stop: reflex brake, wheels lock, nose dives"),
    (300.0, 18.0, "crest and dip: suspension unloads then compresses"),
    (590.0, 15.0, "R60 corner: roll and lateral load transfer"),
    (740.0, 12.0, "R25 corner: limit cornering, understeer"),
    (830.0, 14.0, "spiral R100 to R30: decreasing radius, rollover margin"),
    (960.0, 8.0, "run-out: settle and stop"),
]

# Fires the emergency stop the moment the car is moving at all, so the stop
# happens at the station rather than wherever the car happened to speed up.
STOP_REFLEX = [stop_and_hold_above(0.5)]


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    joined = json.loads(await ws.recv())

    map_data = joined.get("map")
    if not map_data:
        print("no map delivered -- is this scenario_wheels.json?")
        await ws.close()
        return
    lane = pick_driving_lane(map_data)
    if lane is None:
        print("no forward driving lane in the delivered map")
        await ws.close()
        return

    center = lane["centerline"]
    lengths = arc_lengths(center)
    print(f"test track: lane {lane['id']}, {lengths[-1]:.0f} m, {len(center)} points")

    state = {"stage": -1, "braking": False, "pos": joined["position"], "speed": 0.0}

    async def advance_to(stage, index):
        station, speed, what = SCRIPT[stage]
        state["stage"] = stage
        print(f"\n[{station:6.0f} m] {what}   (commanding {speed:.0f} m/s)")
        if speed == 0.0:
            # A reflex stop, not a slow waypoint: this is the one that locks
            # the wheels, and only the reflex path brakes hard enough to.
            await ws.send(
                json.dumps({"type": "register_reflexes", "rules": STOP_REFLEX})
            )
            state["braking"] = True
        else:
            if state["braking"]:
                await ws.send(json.dumps({"type": "register_reflexes", "rules": []}))
                state["braking"] = False
            await ws.send(
                json.dumps(
                    {
                        "type": "submit_plan",
                        "waypoints": waypoints(center[index:], speed),
                    }
                )
            )

    async def report(_sim_time):
        await ws.send(json.dumps({"type": "get_state"}))

    async def on_message(msg):
        if msg.get("type") != "state":
            return
        car = next((e for e in msg["entities"] if e["agent_id"] == "car"), None)
        if car is None:
            return
        state["pos"] = car["position"]
        v = car["velocity"]
        state["speed"] = math.sqrt(v["x"] ** 2 + v["y"] ** 2 + v["z"] ** 2)

        index = nearest_index(center, state["pos"])
        travelled = lengths[index]
        _, _, _, off = project_point(center, state["pos"])
        print(
            f"  s={travelled:6.1f} m  speed={state['speed']:5.2f} m/s  "
            f"y={state['pos']['y']:5.2f}  off-lane={off:4.2f} m",
            end="\r",
        )

        # A braking stage holds until the car has actually stopped; the others
        # trigger on distance. Either way each stage fires exactly once.
        nxt = state["stage"] + 1
        if state["braking"]:
            if state["speed"] < 0.4 and nxt < len(SCRIPT):
                await advance_to(nxt, index)
        elif nxt < len(SCRIPT) and travelled >= SCRIPT[nxt][0]:
            await advance_to(nxt, index)

    await advance_to(0, nearest_index(center, state["pos"]))
    reason = await run_clock(ws, on_step=report, on_message=on_message, report_dt=0.25)
    print(f"\nrun ended: {reason}")
    await ws.close()


if __name__ == "__main__":
    asyncio.run(main())
