use bevy::prelude::*;
use protocol::messages::ReflexAction;

/// Emitted by arbitration when a reflex overrides an entity's plan on a
/// tick. The transport bridge forwards it to the owning agent as
/// `ServerMessage::ReflexFired`, carrying the tick and plan version so the
/// agent can tell which plan was interrupted.
#[derive(Message, Debug, Clone, Copy)]
pub struct ReflexFired {
    pub entity: Entity,
    pub tick: u64,
    pub plan_version: u64,
    pub action: ReflexAction,
}
