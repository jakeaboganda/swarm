use bevy::prelude::*;

/// A scenario is not an open join-anytime space: it waits for a fixed
/// roster to fully connect, runs, and ends permanently the moment any
/// connected agent becomes unavailable.
#[derive(States, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ScenarioState {
    #[default]
    WaitingForRoster,
    Running,
    Ended,
}

/// Why the scenario ended, set once alongside the transition to
/// `ScenarioState::Ended`. Reported to remaining agents via
/// `ServerMessage::ScenarioEnded`.
#[derive(Resource, Default)]
pub struct EndReason(pub Option<String>);

/// Ticks once per `FixedUpdate` step while the scenario is running; agents
/// use it to tell whether a snapshot/event is stale relative to a plan
/// they're about to submit.
#[derive(Resource, Default)]
pub struct Tick(pub u64);
