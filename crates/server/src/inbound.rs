//! Sanitization of agent-supplied payloads, at the boundary where untrusted
//! input first reaches the sim.
//!
//! Agents are external processes that "may be LLM-driven, and therefore slow
//! and occasionally unreliable" — a plan can arrive carrying `NaN`, `inf`, a
//! negative speed, or ten thousand waypoints, all of it valid JSON. These are
//! the pure decisions taken on that input, extracted from `drain_transport` so
//! they can be driven by tests.

use protocol::messages::{ReflexRule, Waypoint};

/// Cap on a single plan's waypoint count. A plan costs O(1) per tick (only the
/// front waypoint is read), so this bounds memory, not tick time — hence a
/// generous limit: a route across a city-sized map samples into the thousands.
pub const MAX_PLAN_WAYPOINTS: usize = 10_000;

/// Cap on one agent's reflex-rule count. `evaluate` is O(rules x obstacles)
/// per agent *per tick* at 64 Hz, so an unbounded set degrades the tick rate
/// for every other agent in the scenario. A declarative reflex set is a
/// handful of rules; 64 is already far past useful.
pub const MAX_REFLEX_RULES: usize = 64;

/// Why an inbound payload was refused outright. The connection stays open and
/// the agent gets the message as an `error` event — same contract as a
/// malformed frame.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum InboundError {
    #[error("plan rejected: none of its {0} waypoints are usable (non-finite position or speed)")]
    NoUsableWaypoints(usize),
    #[error("reflex set rejected: {0} rules exceeds the cap of {MAX_REFLEX_RULES}")]
    TooManyRules(usize),
}

fn usable(waypoint: &Waypoint) -> bool {
    let p = waypoint.position;
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite() && waypoint.speed.is_finite()
}

/// Cleans an agent's submitted plan, or refuses it.
///
/// Unusable waypoints (non-finite position or speed) are dropped and negative
/// speeds clamp to zero; an over-long plan is truncated at the cap, since a
/// long plan is plausibly a legitimate long route. A *non-empty* plan with
/// nothing usable left is refused rather than applied — silently replacing a
/// working plan with an empty one would coast the vehicle on stale forces and
/// look like a server bug. An empty submission is accepted as-is: that is the
/// deliberate way to clear a plan.
pub fn sanitize_plan(waypoints: Vec<Waypoint>) -> Result<Vec<Waypoint>, InboundError> {
    let submitted = waypoints.len();
    let cleaned: Vec<Waypoint> = waypoints
        .into_iter()
        .filter(usable)
        .map(|w| Waypoint {
            speed: w.speed.max(0.0),
            ..w
        })
        .take(MAX_PLAN_WAYPOINTS)
        .collect();
    if submitted > 0 && cleaned.is_empty() {
        return Err(InboundError::NoUsableWaypoints(submitted));
    }
    Ok(cleaned)
}

/// Checks an agent's reflex set against the per-tick cost cap.
///
/// Unlike a plan, an oversized rule set is refused rather than truncated: the
/// rules an agent keeps would be chosen by array position, so it would go on
/// believing it was protected by a rule the server had quietly dropped.
///
/// Individual rules are not otherwise filtered. A `NaN` threshold is inert by
/// construction (every comparison against it is false), which
/// `a_nan_threshold_never_activates_a_rule` in `sensors` pins deliberately.
pub fn sanitize_rules(rules: Vec<ReflexRule>) -> Result<Vec<ReflexRule>, InboundError> {
    if rules.len() > MAX_REFLEX_RULES {
        return Err(InboundError::TooManyRules(rules.len()));
    }
    Ok(rules)
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::messages::{Operator, ReflexAction, SensorKind};
    use protocol::Vec3;

    fn waypoint(x: f32, speed: f32) -> Waypoint {
        Waypoint {
            position: Vec3::new(x, 0.0, 0.0),
            speed,
        }
    }

    fn rule() -> ReflexRule {
        ReflexRule {
            sensor: "ground_truth".into(),
            measure: SensorKind::TimeToCollision,
            operator: Operator::LessThan,
            threshold: 1.0,
            action: ReflexAction::Brake,
            priority: 0,
        }
    }

    #[test]
    fn a_non_finite_waypoint_position_is_dropped_from_the_plan() {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let plan = vec![
                waypoint(1.0, 5.0),
                Waypoint {
                    position: Vec3::new(bad, 0.0, 0.0),
                    speed: 5.0,
                },
                Waypoint {
                    position: Vec3::new(0.0, bad, 0.0),
                    speed: 5.0,
                },
                Waypoint {
                    position: Vec3::new(0.0, 0.0, bad),
                    speed: 5.0,
                },
                waypoint(2.0, 5.0),
            ];
            let cleaned = sanitize_plan(plan).expect("two good waypoints remain");
            assert_eq!(cleaned, vec![waypoint(1.0, 5.0), waypoint(2.0, 5.0)]);
        }
    }

    #[test]
    fn a_non_finite_waypoint_speed_is_dropped_from_the_plan() {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let plan = vec![waypoint(1.0, bad), waypoint(2.0, 5.0)];
            let cleaned = sanitize_plan(plan).expect("one good waypoint remains");
            assert_eq!(cleaned, vec![waypoint(2.0, 5.0)]);
        }
    }

    #[test]
    fn a_negative_waypoint_speed_clamps_to_zero() {
        let cleaned = sanitize_plan(vec![waypoint(1.0, -12.0)]).expect("position is usable");
        assert_eq!(cleaned, vec![waypoint(1.0, 0.0)]);
    }

    #[test]
    fn an_oversized_plan_is_truncated_at_the_waypoint_cap() {
        let plan: Vec<Waypoint> = (0..MAX_PLAN_WAYPOINTS + 500)
            .map(|i| waypoint(i as f32, 5.0))
            .collect();
        let cleaned = sanitize_plan(plan).expect("plan is usable");
        assert_eq!(cleaned.len(), MAX_PLAN_WAYPOINTS);
        // Truncation keeps the front of the path -- the part the entity drives
        // next -- not an arbitrary window of it.
        assert_eq!(cleaned[0], waypoint(0.0, 5.0));
    }

    #[test]
    fn an_oversized_reflex_set_is_rejected_with_an_error_not_truncated() {
        let rules: Vec<ReflexRule> = (0..MAX_REFLEX_RULES + 1).map(|_| rule()).collect();
        assert_eq!(
            sanitize_rules(rules),
            Err(InboundError::TooManyRules(MAX_REFLEX_RULES + 1))
        );
        // Exactly at the cap is still fine.
        let at_cap: Vec<ReflexRule> = (0..MAX_REFLEX_RULES).map(|_| rule()).collect();
        assert_eq!(
            sanitize_rules(at_cap).map(|r| r.len()),
            Ok(MAX_REFLEX_RULES)
        );
    }

    #[test]
    fn a_rejected_plan_leaves_the_plan_version_unchanged() {
        // The version bump is the caller's, and it is gated on `Ok`: a plan
        // with nothing usable in it never reaches the entity, so an agent
        // watching `plan_version` sees its submission did not take.
        let all_bad = vec![waypoint(f32::NAN, 5.0), waypoint(1.0, f32::INFINITY)];
        assert_eq!(
            sanitize_plan(all_bad),
            Err(InboundError::NoUsableWaypoints(2))
        );
    }

    #[test]
    fn an_empty_plan_is_accepted_as_a_deliberate_clear() {
        assert_eq!(sanitize_plan(vec![]), Ok(vec![]));
    }
}
