use std::cmp::Reverse;
use std::collections::HashMap;

use protocol::messages::{Operator, ReflexAction, ReflexRule};

use crate::context::SensorContext;
use crate::sensor::sensor_for;

/// How much a reading has to move back past its threshold before a
/// triggered rule clears, so an entity sitting near the boundary doesn't
/// flicker the action on/off every tick.
const HYSTERESIS_MARGIN: f32 = 0.5;

/// A registered reflex rule plus its own trigger state, needed for
/// hysteresis (whether a rule is active depends on whether it *was*
/// active last tick, not just the current reading).
pub struct ActiveRule {
    rule: ReflexRule,
    active: bool,
}

impl ActiveRule {
    pub fn new(rule: ReflexRule) -> Self {
        Self {
            rule,
            active: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }
}

fn condition_met(rule: &ReflexRule, reading: f32, currently_active: bool) -> bool {
    let margin = if currently_active {
        HYSTERESIS_MARGIN
    } else {
        0.0
    };
    match rule.operator {
        Operator::LessThan => reading < rule.threshold + margin,
        Operator::GreaterThan => reading > rule.threshold - margin,
    }
}

/// Updates every rule's trigger state and returns the action of the
/// highest-priority active rule, if any. Each rule reads its `measure` from
/// the device it names (`rule.sensor`), resolved via `contexts` (keyed by
/// device name — e.g. `ground_truth` or a scenario sensor). A rule naming a
/// device with no context reads nothing and stays inactive. Ties are broken by
/// registration order (earlier wins); priority is per-agent.
pub fn evaluate(
    rules: &mut [ActiveRule],
    contexts: &HashMap<String, SensorContext>,
) -> Option<ReflexAction> {
    for active_rule in rules.iter_mut() {
        let Some(ctx) = contexts.get(&active_rule.rule.sensor) else {
            active_rule.active = false;
            continue;
        };
        let sensor = sensor_for(&active_rule.rule.measure);
        let reading = sensor.read(ctx);
        active_rule.active = condition_met(&active_rule.rule, reading, active_rule.active);
    }

    rules
        .iter()
        .enumerate()
        .filter(|(_, r)| r.active)
        .max_by_key(|(index, r)| (r.rule.priority, Reverse(*index)))
        .map(|(_, r)| r.rule.action)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::Obstacle;
    use glam::Vec3;

    fn rule(operator: Operator, threshold: f32, action: ReflexAction, priority: i32) -> ReflexRule {
        ReflexRule {
            sensor: "s".into(),
            measure: protocol::messages::SensorKind::Speed,
            operator,
            threshold,
            action,
            priority,
        }
    }

    /// A one-device context map (name "s") holding a context with `speed`.
    fn ctx_with_speed(speed: f32) -> HashMap<String, SensorContext> {
        HashMap::from([(
            "s".to_string(),
            SensorContext {
                self_position: Vec3::ZERO,
                self_velocity: Vec3::new(speed, 0.0, 0.0),
                self_radius: 0.0,
                obstacles: vec![],
            },
        )])
    }

    #[test]
    fn higher_priority_wins_when_both_active() {
        let mut rules = vec![
            ActiveRule::new(rule(Operator::GreaterThan, 1.0, ReflexAction::Brake, 1)),
            ActiveRule::new(rule(
                Operator::GreaterThan,
                1.0,
                ReflexAction::StopAndHold,
                5,
            )),
        ];
        let action = evaluate(&mut rules, &ctx_with_speed(10.0));
        assert_eq!(action, Some(ReflexAction::StopAndHold));
    }

    #[test]
    fn equal_priority_ties_broken_by_registration_order() {
        let mut rules = vec![
            ActiveRule::new(rule(Operator::GreaterThan, 1.0, ReflexAction::Brake, 5)),
            ActiveRule::new(rule(
                Operator::GreaterThan,
                1.0,
                ReflexAction::StopAndHold,
                5,
            )),
        ];
        let action = evaluate(&mut rules, &ctx_with_speed(10.0));
        assert_eq!(action, Some(ReflexAction::Brake));
    }

    #[test]
    fn no_action_when_nothing_active() {
        let mut rules = vec![ActiveRule::new(rule(
            Operator::GreaterThan,
            100.0,
            ReflexAction::Brake,
            0,
        ))];
        let action = evaluate(&mut rules, &ctx_with_speed(1.0));
        assert_eq!(action, None);
    }

    #[test]
    fn hysteresis_keeps_rule_active_past_the_trigger_threshold() {
        let mut rules = vec![ActiveRule::new(rule(
            Operator::LessThan,
            2.0,
            ReflexAction::Brake,
            0,
        ))];

        // Reading well below threshold: triggers.
        evaluate(&mut rules, &ctx_with_speed(1.0));
        assert!(rules[0].is_active());

        // Reading rises above the raw threshold but within the hysteresis
        // margin: should stay active (no chatter at the boundary).
        evaluate(&mut rules, &ctx_with_speed(2.2));
        assert!(rules[0].is_active());

        // Reading rises past threshold + margin: clears.
        evaluate(&mut rules, &ctx_with_speed(3.0));
        assert!(!rules[0].is_active());
    }

    #[test]
    fn time_to_collision_and_walls_are_reachable_through_evaluate() {
        let mut rules = vec![ActiveRule::new(ReflexRule {
            sensor: "s".into(),
            measure: protocol::messages::SensorKind::TimeToCollision,
            operator: Operator::LessThan,
            threshold: 5.0,
            action: ReflexAction::Brake,
            priority: 0,
        })];
        let contexts = HashMap::from([(
            "s".to_string(),
            SensorContext {
                self_position: Vec3::ZERO,
                self_velocity: Vec3::new(1.0, 0.0, 0.0),
                self_radius: 0.0,
                obstacles: vec![Obstacle {
                    position: Vec3::new(3.0, 0.0, 0.0),
                    velocity: Vec3::ZERO,
                    radius: 0.0,
                }],
            },
        )]);
        assert_eq!(evaluate(&mut rules, &contexts), Some(ReflexAction::Brake));
    }
}
