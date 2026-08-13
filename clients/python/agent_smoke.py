"""End-to-end smoke test: connects two agents, fills the roster, submits
crossing plans plus a time-to-collision brake reflex, and prints their
positions as physics runs.

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

SERVER_URL = "ws://127.0.0.1:4000"


async def agent(name, waypoints):
    async with websockets.connect(SERVER_URL) as ws:
        await ws.send(json.dumps({"type": "join", "name": name}))
        joined = json.loads(await ws.recv())
        print(f"[{name}] joined ->", joined.get("type"), joined.get("position"))

        await ws.send(json.dumps({"type": "submit_plan", "waypoints": waypoints}))
        await ws.send(json.dumps({"type": "register_reflexes", "rules": [
            {"sensor": "ground_truth", "measure": {"kind": "time_to_collision"},
             "operator": "less_than",
             "threshold": 2.0, "action": "brake", "priority": 10}
        ]}))

        # Let physics run, then pull state a few times.
        for _ in range(3):
            await asyncio.sleep(0.6)
            await ws.send(json.dumps({"type": "get_state"}))
            snap = json.loads(await ws.recv())
            if snap.get("type") == "state":
                me = next((e for e in snap["entities"] if e["agent_id"] == name), None)
                print(f"[{name}] tick={snap['tick']} pos={me['position'] if me else '?'}")
        return name


async def main():
    # Crossing paths: the brake reflex should slow both as they close.
    wps1 = [{"position": {"x": 15.0, "y": 1.0, "z": 0.0}, "speed": 5.0}]
    wps2 = [{"position": {"x": -15.0, "y": 1.0, "z": 0.0}, "speed": 5.0}]
    results = await asyncio.gather(agent("car-1", wps1), agent("car-2", wps2))
    print("both agents ran:", results)


if __name__ == "__main__":
    asyncio.run(main())
