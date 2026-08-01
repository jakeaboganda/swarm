mod agent;
mod arbitration;
mod events;
mod scenario;
mod scenario_state;
mod transport_bridge;
mod viz_broadcast;
mod world;

use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
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

fn main() -> anyhow::Result<()> {
    let scenario_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenario.json".to_string());
    let scenario_config = scenario::load_scenario(&scenario_path)?;

    // Own the tokio runtime explicitly. The transport and viz servers run on
    // its background workers; Bevy's blocking headless run-loop then owns the
    // main thread, so it never starves the async servers (as it would if it
    // parked a worker under `#[tokio::main]`). The runtime must outlive the
    // app, so it's held here for the whole run.
    let runtime = tokio::runtime::Runtime::new()?;
    let (transport_handle, viz_handle) = runtime.block_on(async {
        let transport_handle = transport::spawn(transport::Config::default()).await?;
        println!("listening for agents on {}", transport_handle.local_addr);
        let viz_handle = viz::spawn(viz::VizConfig::default()).await?;
        println!("streaming viz on {}", viz_handle.local_addr);
        anyhow::Ok((transport_handle, viz_handle))
    })?;
    let _runtime_guard = runtime.enter();

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

    let mut app = App::new();
    app
        // Headless: no window or rendering — rendering lives in the viewer.
        // A bounded run-loop drives the app instead of a window event loop.
        .add_plugins(
            MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
                1.0 / 120.0,
            ))),
        )
        .add_plugins(bevy::log::LogPlugin::default())
        .add_plugins(TransformPlugin)
        .add_plugins(StatesPlugin)
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
        // Viz broadcast. `broadcast_spawns` runs before `drain_viz_events`
        // so a viewer connecting the same frame an agent joins learns of
        // that agent only via its scene-init, never also via a duplicate
        // EntitySpawned (a not-yet-ready viewer is skipped by the spawn
        // broadcast). Frames stream only while Running — no dynamic state
        // to send otherwise.
        .add_systems(Update, (broadcast_spawns, drain_viz_events).chain())
        .add_systems(
            Update,
            broadcast_frames
                .after(drain_viz_events)
                .run_if(in_state(ScenarioState::Running)),
        )
        .add_systems(
            OnEnter(ScenarioState::Running),
            (activate_physics, broadcast_state),
        )
        .add_systems(
            OnEnter(ScenarioState::Ended),
            (deactivate_physics, notify_scenario_ended, broadcast_state),
        );

    app.run();

    Ok(())
}

fn setup_arena(mut commands: Commands, roster: Res<Roster>) {
    world::spawn_arena(&mut commands, &roster.0.arena);
}
