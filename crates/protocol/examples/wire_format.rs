//! Prints the JSON wire form of the main protocol messages, so you can see
//! exactly what an agent sends and what a scenario file looks like.
//!
//! Run: `cargo run -p protocol --example wire_format`

use protocol::messages::{
    ClientMessage, Operator, ReflexAction, ReflexRule, SensorKind, ServerMessage, Waypoint,
};
use protocol::scenario::{AgentSlot, ArenaConfig, Embodiment, ScenarioConfig};
use protocol::Vec3;

fn main() {
    let plan = ClientMessage::SubmitPlan {
        waypoints: vec![
            Waypoint {
                position: Vec3::new(10.0, 0.0, 0.0),
                speed: 5.0,
            },
            Waypoint {
                position: Vec3::new(10.0, 0.0, 10.0),
                speed: 3.0,
            },
        ],
    };

    let reflexes = ClientMessage::RegisterReflexes {
        rules: vec![ReflexRule {
            sensor: "ground_truth".into(),
            measure: SensorKind::TimeToCollision,
            operator: Operator::LessThan,
            threshold: 2.0,
            action: ReflexAction::Brake,
            priority: 10,
        }],
    };

    let ended = ServerMessage::ScenarioEnded {
        reason: "car-2 disconnected".into(),
    };

    let scenario = ScenarioConfig {
        arena: ArenaConfig {
            width: 50.0,
            depth: 50.0,
        },
        roster: vec![
            AgentSlot {
                name: "car-1".into(),
                embodiment: Embodiment::Holonomic,
                sensors: Default::default(),
            },
            AgentSlot {
                name: "car-2".into(),
                embodiment: Embodiment::Holonomic,
                sensors: Default::default(),
            },
        ],
        seed: 0,
        map: None,
    };

    for (label, json) in [
        (
            "client -> submit_plan",
            serde_json::to_string_pretty(&plan).unwrap(),
        ),
        (
            "client -> register_reflexes",
            serde_json::to_string_pretty(&reflexes).unwrap(),
        ),
        (
            "server -> scenario_ended",
            serde_json::to_string_pretty(&ended).unwrap(),
        ),
        (
            "scenario.json",
            serde_json::to_string_pretty(&scenario).unwrap(),
        ),
    ] {
        println!("// {label}\n{json}\n");
    }
}
