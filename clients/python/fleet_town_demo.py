"""A fleet of 20 cars routing across a real town, each perceiving the others.

The multi-agent showcase on CARLA's Town07 (234 roads, 31 junctions). One
process opens 20 WebSocket connections -- one per car -- and drives them all in
parallel. Each car:

  1. spawns in its own forward lane (the server fans the fleet out across the
     map's forward lanes so nobody stacks);
  2. walks the delivered connectivity graph from its spawn lane to find a
     *reachable* destination far across town, then asks the server routing
     service (request_route) for the plan, stamped with its cruise speed;
  3. arms one safety reflex before driving:

         time_to_collision(radar) < 2.5s  ->  brake

     `radar` is a simulated device -- 40 m range, a ~70 deg forward cone,
     position/velocity noise, 2 ticks of latency -- so each car reacts to what
     it *perceives* of its neighbours, not ground truth. Where routes converge
     (junctions, shared roads) cars perceive each other and brake, all
     server-side with zero round-trip to this client.

Run the server first (or use scripts/run.sh):
    cargo run --bin server -- scenario_road_fleet.json
    python3 clients/python/fleet_town_demo.py
then watch it in the viewer (F to chase-cam a car through the junctions).

Needs the scenario_road_fleet.json roster (20 cars). `pip install websockets`.
"""

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    print("SKIP: websockets not installed (pip install websockets)")
    sys.exit(2)

from shotgun import brake_on_ttc, dist, driving_lanes, nearest_lane, reachable_lanes
from stepper import run_clock

URL = "ws://127.0.0.1:4000"
FLEET = [f"car-{i}" for i in range(20)]
# Each car cruises at a slightly different speed, so a faster car catches a
# slower one sharing a corridor -- real in-lane closing that trips the
# forward-collision reflex, instead of a fleet gliding at one uniform pace.
# Fewer entries than cars is fine: the Car ctor wraps this list by index.
CRUISE = [4.5, 5.5, 6.5, 5.0, 7.5, 6.0, 8.0, 5.5, 4.0, 7.0]
TTC_THRESHOLD = 2.5  # brake when perceived time-to-collision drops below this, s


def mid(lane):
    return lane["centerline"][len(lane["centerline"]) // 2]


def farthest_reachable(lanes, start_id, spawn):
    """Of the lanes reachable by driving from `start_id`, the one whose middle
    is farthest from spawn -- a destination across town, not down the road."""
    reachable = reachable_lanes(lanes, start_id)
    return max(reachable, key=lambda lane: dist(mid(lane), spawn)), len(reachable)


class Car:
    def __init__(self, name):
        self.name = name
        self.speed = CRUISE[int(name.rsplit("-", 1)[1]) % len(CRUISE)]
        self.ws = None
        self.spawn = None
        self.dest = None
        self.pos = None
        self.route_len = 0
        self.fires = 0
        self.status = "connecting"


async def join(car):
    """Connect and join. Fills spawn/map on success, raises on refusal."""
    car.ws = await websockets.connect(URL)
    await car.ws.send(json.dumps({"type": "join", "name": car.name}))
    joined = json.loads(await car.ws.recv())
    if joined.get("type") != "joined":
        raise RuntimeError(f"{car.name} join refused: {joined.get('message', joined)}")
    car.spawn = joined["position"]
    car.map = joined.get("map")
    car.status = "joined"


async def plan_route(car):
    """Pick a far reachable destination and get the server's route for it."""
    if not car.map:
        car.status = "no map"
        return None
    lanes = driving_lanes(car.map)
    start = nearest_lane(lanes, car.spawn)
    dest_lane, _ = farthest_reachable(lanes, start["id"], car.spawn)
    car.dest = mid(dest_lane)
    await car.ws.send(
        json.dumps(
            {
                "type": "request_route",
                "from": car.spawn,
                "to": {"x": car.dest["x"], "y": car.dest["y"], "z": car.dest["z"]},
                "speed": car.speed,
            }
        )
    )
    while True:
        msg = json.loads(await car.ws.recv())
        if msg.get("type") == "route":
            return msg["waypoints"]
        if msg.get("type") == "scenario_ended":
            return None


async def drive(car):
    route = await plan_route(car)
    if not route:
        car.status = "no route"
        return
    car.route_len = len(route)
    # One safety rule each, read from the car's own `radar` device -- so it
    # brakes for what it perceives of its neighbours, not for ground truth.
    await car.ws.send(json.dumps({
        "type": "register_reflexes",
        "rules": [brake_on_ttc(TTC_THRESHOLD, sensor="radar")],
    }))
    await car.ws.send(json.dumps({"type": "submit_plan", "waypoints": route}))
    car.status = "driving"


async def run(car):
    """Drive this car on the server-owned clock: ask for state each ~0.5s pulse,
    track its position and brake fires, and return when the scenario ends."""
    async def report(_sim_time):
        await car.ws.send(json.dumps({"type": "get_state"}))

    async def on_message(msg):
        kind = msg.get("type")
        if kind == "state":
            for e in msg["entities"]:
                if e["agent_id"] == car.name:
                    car.pos = e["position"]
        elif kind == "reflex_fired":
            car.fires += 1

    try:
        reason = await run_clock(car.ws, on_step=report, on_message=on_message, report_dt=0.5)
        if reason == "off_road":
            car.status = "off road"
    except websockets.ConnectionClosed:
        return


def report(cars):
    print("\n=== fleet ===")
    for car in cars:
        if car.pos and car.dest:
            gap = dist(car.pos, car.dest)
            print(
                f"  {car.name}: {car.status:8}  {car.route_len:3} wp  "
                f"{gap:6.0f} m to go   {car.fires} brake fires"
            )
        else:
            print(f"  {car.name}: {car.status}")


async def main():
    cars = [Car(name) for name in FLEET]

    # All 20 must connect before the scenario runs (fixed-roster). Join them
    # concurrently; if any slot is refused, the scenario is the wrong one.
    try:
        await asyncio.gather(*(join(c) for c in cars))
    except Exception as e:
        print(f"{e}\nRun the fleet scenario (20 cars):")
        print("    scripts/run.sh scenario_road_fleet.json clients/python/fleet_town_demo.py")
        for c in cars:
            if c.ws:
                await c.ws.close()
        return
    print(f"Town07 loaded, {len(FLEET)} cars joined. Routing each across town...")

    # Route + arm + submit each car, then run them all on the server clock
    # until the scenario's duration ends the run.
    await asyncio.gather(*(drive(c) for c in cars))
    routed = sum(1 for c in cars if c.status == "driving")
    print(f"{routed}/{len(FLEET)} cars found a route and are driving.\n")

    await asyncio.gather(*(run(c) for c in cars if c.status == "driving"))
    report(cars)

    total_fires = sum(c.fires for c in cars)
    print(
        f"\n{routed} cars drove their routes; {total_fires} total brake-reflex fires "
        "across the fleet (cars perceiving and yielding to each other)."
    )
    for c in cars:
        await c.ws.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
