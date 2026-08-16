use serde::{Deserialize, Serialize};

use crate::map::MapData;
use crate::Vec3;

/// Identifies an agent. Equal to the `name` it registered under in the
/// scenario roster — roster names are already unique, so there's no need
/// for a separate generated id.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AgentId(pub String);

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    pub position: Vec3,
    pub speed: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SensorKind {
    TimeToCollision,
    DistanceTo { target: Vec3 },
    Speed,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Operator {
    LessThan,
    GreaterThan,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReflexAction {
    Brake,
    StopAndHold,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReflexRule {
    /// Named device to read from: a scenario `SensorDef` name, or the reserved
    /// `ground_truth` device (a perfect, instant fail-safe). Whether the
    /// reading is impaired is the device's property, not the rule's.
    pub sensor: String,
    /// The predicate read from that device.
    pub measure: SensorKind,
    pub operator: Operator,
    pub threshold: f32,
    pub action: ReflexAction,
    /// Higher wins on conflict. Ties broken by position in the
    /// registration array. Scope is per-agent: only ever compared against
    /// this same agent's other rules, never across agents.
    pub priority: i32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMessage {
    Join { name: String },
    SubmitPlan { waypoints: Vec<Waypoint> },
    RegisterReflexes { rules: Vec<ReflexRule> },
    GetState,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityState {
    pub agent_id: AgentId,
    pub position: Vec3,
    pub velocity: Vec3,
    /// Version of the plan currently driving this entity (incremented on
    /// every `SubmitPlan`). Lets an agent tell whether a snapshot reflects
    /// the plan it most recently submitted or an older one.
    pub plan_version: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StateSnapshot {
    pub tick: u64,
    pub entities: Vec<EntityState>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    Joined {
        agent_id: AgentId,
        position: Vec3,
        /// The static road prior, in the automotive world. `None` in the flat
        /// arena. Delivered once, at join -- the agent lays its path from this.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        map: Option<MapData>,
    },
    State(StateSnapshot),
    ReflexFired {
        tick: u64,
        plan_version: u64,
        action: ReflexAction,
    },
    ScenarioEnded {
        reason: String,
    },
    Error {
        message: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip<T>(value: &T)
    where
        T: Serialize + for<'de> Deserialize<'de> + PartialEq + std::fmt::Debug,
    {
        let json = serde_json::to_string(value).expect("serialize");
        let back: T = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(value, &back, "round-trip mismatch, json was: {json}");
    }

    #[test]
    fn client_messages_round_trip() {
        round_trip(&ClientMessage::Join {
            name: "car-1".into(),
        });
        round_trip(&ClientMessage::SubmitPlan {
            waypoints: vec![Waypoint {
                position: Vec3::new(1.0, 0.0, 2.0),
                speed: 3.0,
            }],
        });
        round_trip(&ClientMessage::RegisterReflexes {
            rules: vec![ReflexRule {
                sensor: "ground_truth".into(),
                measure: SensorKind::TimeToCollision,
                operator: Operator::LessThan,
                threshold: 2.0,
                action: ReflexAction::Brake,
                priority: 10,
            }],
        });
        round_trip(&ClientMessage::GetState);
    }

    #[test]
    fn server_messages_round_trip() {
        round_trip(&ServerMessage::Joined {
            agent_id: AgentId("car-1".into()),
            position: Vec3::ZERO,
            map: None,
        });
        round_trip(&ServerMessage::Joined {
            agent_id: AgentId("car-1".into()),
            position: Vec3::ZERO,
            map: Some(crate::map::MapData {
                lanes: vec![crate::map::LaneData {
                    id: 0,
                    kind: crate::map::LaneKind::Driving,
                    direction: crate::map::LaneDirection::Forward,
                    width: 3.5,
                    centerline: vec![Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0)],
                }],
            }),
        });
        round_trip(&ServerMessage::State(StateSnapshot {
            tick: 42,
            entities: vec![EntityState {
                agent_id: AgentId("car-1".into()),
                position: Vec3::ZERO,
                velocity: Vec3::ZERO,
                plan_version: 3,
            }],
        }));
        round_trip(&ServerMessage::ReflexFired {
            tick: 42,
            plan_version: 3,
            action: ReflexAction::StopAndHold,
        });
        round_trip(&ServerMessage::ScenarioEnded {
            reason: "car-2 disconnected".into(),
        });
        round_trip(&ServerMessage::Error {
            message: "bad json".into(),
        });
    }

    #[test]
    fn distance_to_sensor_carries_target() {
        let rule = ReflexRule {
            sensor: "ground_truth".into(),
            measure: SensorKind::DistanceTo {
                target: Vec3::new(5.0, 0.0, 5.0),
            },
            operator: Operator::LessThan,
            threshold: 1.0,
            action: ReflexAction::Brake,
            priority: 0,
        };
        round_trip(&rule);
    }
}
