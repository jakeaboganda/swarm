"""Two Open-Car-Dynamics FMU cars lapping the banked oval.

The headline FMU demo: each car is an `fmu_vehicle` bound to the double-track
Open-Car-Dynamics model, and the server drapes it onto the superelevated oval so
it leans into the canted corners. This client just drives them: it reads the
loop lane the server delivers at join, lays an offset multi-lap plan down it (a
different lateral line per car, so the two run side by side), and submits it. The
banking is all server-side -- the agent only says where and how fast.

Watch it in the reference viewer (true 3D, the cars visibly tilt on the curves):

    ./scripts/run.sh scenario_banked_oval.json clients/python/banked_oval_demo.py

Reads the roster from the scenario file (default scenario_banked_oval.json) so
the client and server never disagree on who is driving. `pip install websockets`.
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

from shotgun import pick_driving_lane, waypoints
from stepper import run_clock

SERVER_URL = "ws://127.0.0.1:4000"
DEFAULT_SCENARIO = "scenario_banked_oval.json"
CRUISE = 14.0          # m/s -- brisk enough to lean hard through the 26 m curves
LAPS = 8               # how many times round before the plan runs out
OFFSETS = [-2.0, 2.0]  # lateral line per car index (m from the centreline)


def offset_loop(center, d):
    """Shift a closed centreline sideways by `d` metres (its left-hand normal),
    so two cars driving the same loop run on parallel lines instead of stacked."""
    n = len(center)
    out = []
    for i in range(n):
        a, b = center[(i - 1) % n], center[(i + 1) % n]
        hx, hz = b["x"] - a["x"], b["z"] - a["z"]
        hl = math.hypot(hx, hz) or 1e-6
        lx, lz = -hz / hl, hx / hl  # left normal of the tangent
        c = center[i]
        out.append(
            {"x": c["x"] + lx * d, "y": c.get("y", 0.0), "z": c["z"] + lz * d}
        )
    return out


async def run_car(name, offset):
    async with websockets.connect(SERVER_URL, ping_interval=None) as ws:
        await ws.send(json.dumps({"type": "join", "name": name}))
        joined = json.loads(await ws.recv())
        if joined.get("type") == "error":
            print(f"[{name}] join rejected: {joined.get('message')}")
            return
        lane = pick_driving_lane(joined.get("map"))
        if lane is None:
            print(f"[{name}] no driving lane in the map -- is this the banked-oval scenario?")
            return

        # One lap of the offset loop, repeated: the tracker's windowed search
        # follows a circuit that revisits its own points, so laps just work.
        loop = offset_loop(lane["centerline"], offset)
        plan = waypoints(loop * LAPS, CRUISE)

        pos = {}
        submitted = False

        async def on_step(_sim_time):
            # Submit once, on the first pulse -- i.e. once the scenario is
            # actually running. A plan sent earlier (while the server is still
            # waiting for the rest of the roster) lands before the body is
            # driving and is not picked up. Then just poll state.
            nonlocal submitted
            if not submitted:
                await ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))
                submitted = True
                print(f"[{name}] driving {LAPS} laps ({len(plan)} waypoints) at {CRUISE} m/s")
            await ws.send(json.dumps({"type": "get_state"}))

        async def on_message(msg):
            t = msg.get("type")
            if t == "error":
                print(f"[{name}] error: {msg.get('message')}")
            elif t == "state":
                for e in msg["entities"]:
                    if e["agent_id"] == name:
                        pos[name] = e["position"]
                p = pos.get(name)
                if p:
                    print(f"[{name}] x={p['x']:7.1f} y={p['y']:5.2f} z={p['z']:7.1f}")

        reason = await run_clock(ws, on_step=on_step, on_message=on_message, report_dt=1.0)
        print(f"[{name}] scenario ended: {reason}")


async def main(scenario_path):
    with open(scenario_path) as f:
        cfg = json.load(f)
    names = [slot["name"] for slot in cfg["roster"]]
    if not names:
        print("no roster in scenario")
        return
    tasks = [run_car(name, OFFSETS[i % len(OFFSETS)]) for i, name in enumerate(names)]
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    args = sys.argv[1:]
    try:
        asyncio.run(main(args[0] if args else DEFAULT_SCENARIO))
    except KeyboardInterrupt:
        pass
