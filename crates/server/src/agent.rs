use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;
use protocol::messages::Waypoint;
use sensors::ActiveRule;
use transport::ConnectionId;

#[derive(Component, Debug, Clone)]
pub struct AgentName(pub String);

/// The deliberative layer: a path of waypoints, advanced as each is
/// reached. `version` increments on every `SubmitPlan`, so agents can tell
/// whether a `reflex_fired` event refers to their current plan.
#[derive(Component, Debug, Default)]
pub struct Plan {
    pub waypoints: VecDeque<Waypoint>,
    pub version: u64,
}

/// The reactive layer: server-evaluated reflex rules for this entity.
#[derive(Component, Default)]
pub struct Reflexes(pub Vec<ActiveRule>);

/// Roster slots declared by the scenario file that haven't joined yet.
#[derive(Resource, Default)]
pub struct PendingRoster(pub Vec<String>);

/// Looks up an agent's entity by connection or by name.
#[derive(Resource, Default)]
pub struct AgentRegistry {
    by_connection: HashMap<ConnectionId, Entity>,
    by_name: HashMap<String, Entity>,
}

impl AgentRegistry {
    pub fn insert(&mut self, connection: ConnectionId, name: String, entity: Entity) {
        self.by_connection.insert(connection, entity);
        self.by_name.insert(name, entity);
    }

    pub fn by_connection(&self, connection: ConnectionId) -> Option<Entity> {
        self.by_connection.get(&connection).copied()
    }

    pub fn remove_connection(&mut self, connection: ConnectionId) -> Option<Entity> {
        self.by_connection.remove(&connection)
    }
}
