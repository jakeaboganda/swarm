"""Visual demo: two cars patrol the arena on separate loops (opposite
directions, different radii) so there's sustained motion to watch, with a
time-to-collision brake reflex that fires on near-approaches.

Run the server first (it opens the 3D window):
    cargo run --bin server -- scenario.json
then, in another shell:
    python3 clients/python/patrol_demo.py

Needs a two-slot roster (car-1, car-2) and `pip install websockets`.
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
SPEED = 7.0
LAPS = 4


def square(radius, start_corner):
    """A clockwise square loop of waypoints at the given radius, rotated to
    begin at `start_corner` (0-3) so a car heads to its own side of the
    arena first instead of crossing the other car's path."""
    corners = [
        (-radius, radius),
        (radius, radius),
        (radius, -radius),
        (-radius, -radius),
    ]
    ordered = corners[start_corner:] + corners[:start_corner]
    return [
        {"position": {"x": float(x), "y": 0.0, "z": float(z)}, "speed": SPEED}
        for _ in range(LAPS)
        for (x, z) in ordered
    ]


async def agent(name, waypoints):
    async with websockets.connect(SERVER_URL) as ws:
        await ws.send(json.dumps({"type": "join", "name": name}))
        await ws.recv()  # Joined
        await ws.send(json.dumps({"type": "submit_plan", "waypoints": waypoints}))
        await ws.send(json.dumps({"type": "register_reflexes", "rules": [
            {"sensor": "ground_truth", "measure": {"kind": "time_to_collision"},
             "operator": "less_than",
             "threshold": 1.5, "action": "brake", "priority": 10}
        ]}))
        # Keep the connection open and report anything the server pushes.
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") == "reflex_fired":
                print(f"[{name}] brake (tick {msg['tick']})")
            elif msg.get("type") == "scenario_ended":
                print(f"[{name}] scenario ended: {msg.get('reason')}")
                return


async def main():
    print("driving car-1 (outer loop) and car-2 (inner loop), same direction")
    # car-1 spawns left of center, car-2 to the right; each starts toward
    # its own side (upper-left / lower-right corner) so they never cross.
    await asyncio.gather(
        agent("car-1", square(15.0, start_corner=0)),
        agent("car-2", square(8.0, start_corner=2)),
    )


if __name__ == "__main__":
    asyncio.run(main())
