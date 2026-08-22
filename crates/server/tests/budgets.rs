//! What the inbound caps defend.
//!
//! Reflex evaluation runs for every agent on every one of the 64 ticks a
//! second, so its cost is multiplied by the whole fleet before it is felt.
//! The caps in `inbound` are the actual fix; this pins that they are set
//! somewhere useful.

use std::time::Instant;

use glam::Vec3;
use protocol::messages::{Operator, ReflexAction, ReflexRule, SensorKind};
use sensors::{evaluate, ActiveRule, Obstacle, SensorContext};
use server::inbound::{MAX_PLAN_WAYPOINTS, MAX_REFLEX_RULES};

/// One physics tick at 64 Hz.
const TICK: f64 = 1.0 / 64.0;
/// A large scenario: every agent perceiving every other one, plus walls.
const AGENTS: usize = 32;

fn rules(count: usize) -> Vec<ActiveRule> {
    (0..count)
        .map(|i| {
            ActiveRule::new(ReflexRule {
                sensor: "ground_truth".into(),
                measure: SensorKind::TimeToCollision,
                operator: Operator::LessThan,
                threshold: 0.5,
                action: ReflexAction::Brake,
                priority: i as i32,
            })
        })
        .collect()
}

fn context(obstacles: usize) -> std::collections::HashMap<String, SensorContext> {
    std::collections::HashMap::from([(
        "ground_truth".to_string(),
        SensorContext {
            self_position: Vec3::ZERO,
            self_velocity: Vec3::new(10.0, 0.0, 0.0),
            self_radius: 0.5,
            obstacles: (0..obstacles)
                .map(|i| Obstacle {
                    position: Vec3::new(20.0 + i as f32, 0.0, i as f32),
                    velocity: Vec3::new(-1.0, 0.0, 0.0),
                    radius: 0.5,
                })
                .collect(),
        },
    )])
}

#[test]
fn arbitration_cost_stays_linear_in_agents_times_rules() {
    // The shape of the cost, measured rather than asserted from the code:
    // doubling the rule count should roughly double the work, not square it.
    let contexts = context(AGENTS);
    let time = |rule_count: usize| {
        let mut set = rules(rule_count);
        // Warm up, then measure enough passes to be above timer noise.
        for _ in 0..100 {
            evaluate(&mut set, &contexts);
        }
        let start = Instant::now();
        for _ in 0..1000 {
            evaluate(&mut set, &contexts);
        }
        start.elapsed().as_secs_f64() / 1000.0
    };

    let small = time(MAX_REFLEX_RULES / 8);
    let full = time(MAX_REFLEX_RULES);
    // 8x the rules for well under 24x the time. Loose on purpose: the claim is
    // "not superlinear", and a shared machine's timer is not a stopwatch.
    assert!(
        full < small * 24.0 + 1e-6,
        "{MAX_REFLEX_RULES} rules cost {full:.9}s vs {:.9}s for an eighth -- \
         evaluation is growing faster than the rule count",
        small
    );

    // And the fleet-wide worst case fits inside a tick with room to spare: a
    // full grid of agents, each at the rule cap, each seeing every other.
    let mut set = rules(MAX_REFLEX_RULES);
    let start = Instant::now();
    for _ in 0..AGENTS {
        evaluate(&mut set, &contexts);
    }
    let fleet = start.elapsed().as_secs_f64();
    assert!(
        fleet < TICK / 4.0,
        "{AGENTS} agents at the {MAX_REFLEX_RULES}-rule cap cost {fleet:.6}s, \
         a quarter of the {TICK:.6}s tick"
    );
    println!("fleet-tick reflex cost at the caps: {fleet:.6}s (tick is {TICK:.6}s)");
}

#[test]
fn a_real_route_across_the_largest_shipped_map_fits_under_the_plan_cap() {
    // A cap an ordinary agent trips is a bug report waiting to happen. The
    // longest plan a cooperative agent submits is a server-generated route, so
    // measure one on the biggest map the repo ships rather than guessing.
    const TOWN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../maps/town07.xodr");
    let net = map_opendrive::load_file(TOWN).expect("load town07");
    let lanes: Vec<_> = net.driving_lanes().collect();

    let mut longest = 0;
    for i in 0..16 {
        for j in 0..16 {
            let from = lanes[i * lanes.len() / 16].center.point_at(0.0);
            let to = lanes[j * lanes.len() / 16].center.point_at(0.0);
            if let Some(route) = net.route(from, to) {
                longest = longest.max(route.len());
            }
        }
    }
    assert!(longest > 0, "no pair routed; the measurement is empty");
    assert!(
        longest * 4 < MAX_PLAN_WAYPOINTS,
        "the longest route sampled over town07 is {longest} waypoints, close \
         to the {MAX_PLAN_WAYPOINTS} cap -- the cap would start truncating \
         legitimate plans"
    );
    println!("longest town07 route: {longest} waypoints (cap is {MAX_PLAN_WAYPOINTS})");
}
