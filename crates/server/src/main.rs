mod agent;
mod arbitration;
mod events;
mod overlay;
mod scenario;
mod scenario_state;
mod transport_bridge;
mod viz_broadcast;
mod world;

use bevy::prelude::*;
use bevy_rapier3d::prelude::{NoUserData, RapierPhysicsPlugin};

use agent::{AgentRegistry, AwaitingReconnect, PendingRoster};
use events::ReflexFired;
use scenario::{ArenaBounds, Roster};
use scenario_state::{EndReason, ScenarioState, Tick};
use transport_bridge::{
    activate_physics, deactivate_physics, drain_transport, expire_reconnects, forward_reflex_fired,
    notify_scenario_ended, Transport,
};
use viz_broadcast::{
    broadcast_frames, broadcast_spawns, broadcast_state, drain_viz_events, Viz, VizFrameTimer,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let scenario_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenario.json".to_string());
    let scenario_config = scenario::load_scenario(&scenario_path)?;

    let transport_handle = transport::spawn(transport::Config::default()).await?;
    println!("listening for agents on {}", transport_handle.local_addr);

    let viz_handle = viz::spawn(viz::VizConfig::default()).await?;
    println!("streaming viz on {}", viz_handle.local_addr);

    let pending_roster = PendingRoster(
        scenario_config
            .roster
            .iter()
            .map(|slot| slot.name.clone())
            .collect(),
    );
    let arena_bounds = ArenaBounds {
        half_width: scenario_config.arena.width / 2.0,
        half_depth: scenario_config.arena.depth / 2.0,
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_plugins(movement::MovementPlugin)
        .init_state::<ScenarioState>()
        .add_message::<ReflexFired>()
        .insert_resource(Roster(scenario_config))
        .insert_resource(arena_bounds)
        .insert_resource(pending_roster)
        .insert_resource(AgentRegistry::default())
        .insert_resource(AwaitingReconnect::default())
        .insert_resource(EndReason::default())
        .insert_resource(Tick::default())
        .insert_resource(Transport(transport_handle))
        .insert_resource(Viz(viz_handle))
        .insert_resource(VizFrameTimer::default())
        .add_systems(Startup, (setup_arena, deactivate_physics))
        // Ingest agent messages in the same fixed cadence as physics so a
        // submitted plan is seen on the step it applies to, rather than at
        // render-frame rate. drain runs in all states (Join/disconnect);
        // arbitration only while Running.
        .add_systems(
            FixedUpdate,
            (drain_transport, expire_reconnects)
                .chain()
                .before(arbitration::arbitrate),
        )
        .add_systems(
            FixedUpdate,
            (arbitration::arbitrate, forward_reflex_fired)
                .chain()
                .before(movement::MovementSet::ApplyForce)
                .run_if(in_state(ScenarioState::Running)),
        )
        // Record motion trails at the physics cadence; draw plans and
        // trails every rendered frame.
        .add_systems(
            FixedUpdate,
            overlay::record_trails.run_if(in_state(ScenarioState::Running)),
        )
        .add_systems(Update, (overlay::draw_plans, overlay::draw_trails))
        // Viz broadcast: catch new viewers up, announce agent spawns, and
        // stream frames at the viz rate (decoupled from physics/render).
        .add_systems(
            Update,
            (drain_viz_events, broadcast_spawns, broadcast_frames),
        )
        .add_systems(
            OnEnter(ScenarioState::Running),
            (activate_physics, broadcast_state),
        )
        .add_systems(
            OnEnter(ScenarioState::Ended),
            (deactivate_physics, notify_scenario_ended, broadcast_state),
        )
        .run();

    Ok(())
}

fn setup_arena(
    mut commands: Commands,
    roster: Res<Roster>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    world::spawn_arena(&mut commands, &roster.0.arena, &mut meshes, &mut materials);
}
