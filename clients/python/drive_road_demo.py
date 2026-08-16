"""Drives a raycast vehicle straight down the demo road on a plan.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_car.json
    python3 clients/python/drive_road_demo.py
then watch it in the viewer.

Needs the scenario_road_car.json roster. `pip install websockets`.
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
pos = {}


async def reader(ws):
    async for raw in ws:
        msg = json.loads(raw)
        if msg.get("type") == "state":
            for e in msg["entities"]:
                pos[e["agent_id"]] = e["position"]
        elif msg.get("type") == "scenario_ended":
            return


async def main():
    ws = await websockets.connect(URL)
    await ws.send(json.dumps({"type": "join", "name": "car"}))
    await ws.recv()  # Joined
    r = asyncio.create_task(reader(ws))

    # Drive straight down the road (+X), accelerating to 6 m/s.
    plan = [
        {"position": {"x": x, "y": 0.0, "z": 0.0}, "speed": 6.0}
        for x in (15.0, 30.0, 38.0)
    ]
    await ws.send(json.dumps({"type": "submit_plan", "waypoints": plan}))

    for _ in range(20):
        await ws.send(json.dumps({"type": "get_state"}))
        await asyncio.sleep(0.4)
        p = pos.get("car")
        if p:
            print(f"car  x={p['x']:6.1f}  y={p['y']:5.2f}  z={p['z']:6.2f}")

    r.cancel()
    await ws.close()


if __name__ == "__main__":
    asyncio.run(main())
