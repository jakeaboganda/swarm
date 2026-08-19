use std::collections::{HashMap, VecDeque};
use std::time::Instant;

use bevy::prelude::*;
use protocol::messages::Waypoint;
use sensors::ActiveRule;
use transport::ConnectionId;

#[derive(Component, Debug, Clone)]
pub struct AgentName(pub String);

/// The live connection currently controlling this entity. Used to route
/// per-entity server messages (e.g. `ReflexFired`) back to the right
/// socket.
#[derive(Component, Debug, Clone, Copy)]
pub struct Connection(pub ConnectionId);

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

    /// Every live `(connection, entity)` pair, for systems that push to each
    /// connected agent (e.g. step pulses).
    pub fn connections(&self) -> impl Iterator<Item = (ConnectionId, Entity)> + '_ {
        self.by_connection.iter().map(|(c, e)| (*c, *e))
    }

    pub fn by_name(&self, name: &str) -> Option<Entity> {
        self.by_name.get(name).copied()
    }

    pub fn remove_connection(&mut self, connection: ConnectionId) -> Option<Entity> {
        self.by_connection.remove(&connection)
    }

    pub fn remove_name(&mut self, name: &str) {
        self.by_name.remove(name);
    }
}

/// Agents that dropped mid-scenario and have until their deadline to
/// reconnect (re-`Join` by name) before the scenario ends. Their entity
/// keeps coasting on its last plan/reflexes during the window.
#[derive(Resource, Default)]
pub struct AwaitingReconnect {
    deadlines: HashMap<String, Instant>,
}

impl AwaitingReconnect {
    pub fn mark(&mut self, name: String, deadline: Instant) {
        self.deadlines.insert(name, deadline);
    }

    pub fn is_awaiting(&self, name: &str) -> bool {
        self.deadlines.contains_key(name)
    }

    /// Clears an agent's pending reconnect (it came back). Returns whether
    /// it was actually awaiting.
    pub fn reconnected(&mut self, name: &str) -> bool {
        self.deadlines.remove(name).is_some()
    }

    /// A name whose reconnect deadline has passed, if any.
    pub fn expired(&self, now: Instant) -> Option<String> {
        self.deadlines
            .iter()
            .find(|(_, deadline)| now >= **deadline)
            .map(|(name, _)| name.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn awaiting_is_tracked_and_cleared_on_reconnect() {
        let mut awaiting = AwaitingReconnect::default();
        let deadline = Instant::now() + Duration::from_secs(8);
        awaiting.mark("car-1".into(), deadline);
        assert!(awaiting.is_awaiting("car-1"));
        assert!(awaiting.reconnected("car-1"));
        assert!(!awaiting.is_awaiting("car-1"));
        assert!(!awaiting.reconnected("car-1"));
    }

    #[test]
    fn expired_reports_only_past_deadlines() {
        let mut awaiting = AwaitingReconnect::default();
        let now = Instant::now();
        awaiting.mark("future".into(), now + Duration::from_secs(8));
        assert_eq!(awaiting.expired(now), None);

        awaiting.mark("past".into(), now - Duration::from_secs(1));
        assert_eq!(awaiting.expired(now).as_deref(), Some("past"));
    }
}
