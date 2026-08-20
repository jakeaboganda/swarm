"""Shared helper: run an agent on the *server-owned* clock.

Time ownership lives in the scenario now, not in the client. Instead of a
`for _ in range(N): sleep()` loop that decides how long to run, a client
`subscribe`s and the server pushes a `tick` pulse each step carrying `dt` (the
sim-seconds since that agent's last pulse). The client `ack`s each pulse to
release the next; the sim never waits, so a slow client just gets fewer, bigger-
`dt` pulses. The scenario's `time.duration` ends the run -- the client exits
when it sees `scenario_ended`.

`run_clock` captures that loop once so each demo can focus on its own logic:

    async def report(sim_time):
        await ws.send(json.dumps({"type": "get_state"}))  # reply -> on_message

    async def on_message(msg):
        if msg["type"] == "state":
            ...update/print...

    reason = await run_clock(ws, on_step=report, on_message=on_message)

`pip install websockets`.
"""

import json


async def run_clock(ws, *, on_step=None, on_message=None, report_dt=0.5):
    """Subscribe, then drive the server-owned clock until the scenario ends.

    For each `tick` pulse: `ack` it, accumulate `dt`, and every ~`report_dt`
    sim-seconds await `on_step(sim_time)` (the place to do periodic work like
    requesting state). Any other server message is passed to `on_message(msg)`
    -- including the `state` replies your `on_step` triggers. Returns the
    `scenario_ended` reason string (the server ends the run at its
    `time.duration`), or `"off_road"` if this agent's vehicle left the road and
    the server removed it (no more pulses will come).
    """
    await ws.send(json.dumps({"type": "subscribe"}))
    sim_time = 0.0
    since_report = report_dt  # report on the first pulse
    async for raw in ws:
        msg = json.loads(raw)
        kind = msg.get("type")
        if kind == "tick":
            sim_time += msg["dt"]
            since_report += msg["dt"]
            await ws.send(json.dumps({"type": "ack", "tick": msg["tick"]}))
            if since_report >= report_dt and on_step is not None:
                since_report = 0.0
                await on_step(sim_time)
        elif kind == "scenario_ended":
            return msg.get("reason")
        elif kind == "off_road":
            # Our vehicle is gone; no further pulses arrive. Stop cleanly.
            return "off_road"
        elif on_message is not None:
            await on_message(msg)
    return None
