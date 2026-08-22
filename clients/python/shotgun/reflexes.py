"""Building reflex rules -- the server-side safety net.

A rule is `sensor` `measure` `operator` `threshold` -> `action`, evaluated
every tick inside the sim, so it fires with no round-trip to the agent. These
constructors just build the dicts; the caller still sends them:

    await ws.send(json.dumps({"type": "register_reflexes",
                              "rules": [brake_on_ttc(2.0)]}))

`sensor` names a device. `ground_truth` is the reserved perfect one, always
available. Any other name must be a device the scenario equipped this agent
with -- and what it reports is range-limited, noisy and late. Which device a
rule reads is the only thing that separates "stopped in time" from "rear-ended
it"; the rule itself is the same either way.

Every constructor validates its enums and raises ValueError on a bad one. A
misspelt action or operator would otherwise be a rule that quietly never
fires.
"""

OPERATORS = ("less_than", "greater_than")
ACTIONS = ("brake", "stop_and_hold")

GROUND_TRUTH = "ground_truth"

# Higher wins when two of an agent's rules disagree. 10 is what every demo
# uses for its single safety rule -- room below for softer rules, room above
# for an override.
DEFAULT_PRIORITY = 10


def ttc():
    """Measure: seconds until collision with the nearest closing obstacle."""
    return {"kind": "time_to_collision"}


def speed():
    """Measure: the entity's own speed, m/s."""
    return {"kind": "speed"}


def distance_to(target):
    """Measure: distance from the entity to a fixed world point, in metres."""
    return {
        "kind": "distance_to",
        "target": {
            "x": float(target["x"]),
            "y": float(target.get("y", 0.0)),
            "z": float(target["z"]),
        },
    }


def rule(sensor, measure, operator, threshold, action, priority=DEFAULT_PRIORITY):
    """One reflex rule. The general form -- the named constructors below cover
    the common cases."""
    if operator not in OPERATORS:
        raise ValueError(f"operator must be one of {OPERATORS}, got {operator!r}")
    if action not in ACTIONS:
        raise ValueError(f"action must be one of {ACTIONS}, got {action!r}")
    if not isinstance(measure, dict) or "kind" not in measure:
        raise ValueError(f"measure must be a measure dict, got {measure!r}")
    if not sensor:
        raise ValueError("sensor must name a device (or 'ground_truth')")
    return {
        "sensor": sensor,
        "measure": measure,
        "operator": operator,
        "threshold": float(threshold),
        "action": action,
        "priority": int(priority),
    }


def brake_on_ttc(threshold, sensor=GROUND_TRUTH, priority=DEFAULT_PRIORITY):
    """Brake while time-to-collision is under `threshold` seconds. Releases
    once the threat clears, so the car slows for traffic and carries on."""
    return rule(sensor, ttc(), "less_than", threshold, "brake", priority)


def stop_on_ttc(threshold, sensor=GROUND_TRUTH, priority=DEFAULT_PRIORITY):
    """Stop dead and hold while time-to-collision is under `threshold`
    seconds. Use instead of `brake_on_ttc` when the car should park where it
    first saw trouble rather than creep forward as the reading oscillates."""
    return rule(sensor, ttc(), "less_than", threshold, "stop_and_hold", priority)


def stop_and_hold_above(speed_threshold, sensor=GROUND_TRUTH,
                        priority=DEFAULT_PRIORITY):
    """Stop the moment the entity is moving faster than `speed_threshold` m/s.
    An emergency stop on command: register it to brake hard right here (the
    reflex path has a higher force ceiling than plan-following), deregister to
    drive on."""
    return rule(sensor, speed(), "greater_than", speed_threshold,
                "stop_and_hold", priority)
