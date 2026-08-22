"""End-to-end smoke test: connects two agents, fills the roster, submits
crossing plans plus a time-to-collision brake reflex, and prints their
positions as physics runs.

Time is server-owned: each agent subscribes to the step clock and reports on
its pulses, and the run ends when the scenario's `time.duration` elapses (no
client-side countdown).

Prerequisites: a running server (`cargo run --bin server -- scenario.json`)
with a two-slot roster (`car-1`, `car-2`), and `pip install websockets`.

Usage: python3 clients/python/agent_smoke.py
"""

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

from shotgun import brake_on_ttc
from stepper import run_clock

SERVER_URL = "ws://127.0.0.1:4000"


async def agent(name, waypoints):
    async with websockets.connect(SERVER_URL) as ws:
        await ws.send(json.dumps({"type": "join", "name": name}))
        joined = json.loads(await ws.recv())
        print(f"[{name}] joined ->", joined.get("type"), joined.get("position"))

        await ws.send(json.dumps({"type": "submit_plan", "waypoints": waypoints}))
        await ws.send(json.dumps({
            "type": "register_reflexes", "rules": [brake_on_ttc(2.0)]
        }))

        # On each ~0.6s step pulse, ask for state; print it when it arrives.
        async def report(_sim_time):
            await ws.send(json.dumps({"type": "get_state"}))

        async def on_message(msg):
            if msg.get("type") == "state":
                me = next((e for e in msg["entities"] if e["agent_id"] == name), None)
                print(f"[{name}] tick={msg['tick']} pos={me['position'] if me else '?'}")

        reason = await run_clock(ws, on_step=report, on_message=on_message, report_dt=0.6)
        print(f"[{name}] ended: {reason}")
        return name


async def main():
    # Crossing paths: the brake reflex should slow both as they close.
    wps1 = [{"position": {"x": 15.0, "y": 1.0, "z": 0.0}, "speed": 5.0}]
    wps2 = [{"position": {"x": -15.0, "y": 1.0, "z": 0.0}, "speed": 5.0}]
    results = await asyncio.gather(agent("car-1", wps1), agent("car-2", wps2))
    print("both agents ran:", results)


if __name__ == "__main__":
    asyncio.run(main())
