mod agent;
mod arbitration;
mod events;
mod perception_router;
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
use perception_router::{
    drain_perception_events, route_perception, PerceivedWorlds, Perception, PerceptionAgents,
    PerceptionOverlay, PerceptionSeed,
};
use scenario::{ArenaBounds, Roster};
use scenario_state::{EndReason, ScenarioState, Tick};
use transport_bridge::{
    activate_physics, deactivate_physics, drain_transport, expire_reconnects, forward_reflex_fired,
    notify_scenario_ended, Transport,
};
use viz_broadcast::{broadcast_frames, broadcast_spawns, broadcast_state, drain_viz_events, Viz};

/// Physics/sim tick rate. Pinned here (not left to Bevy's default) because
/// it's the `tick_rate` advertised to viewers, which key their playback
/// clock to it — a silent default change would desync every viewer.
const TICK_HZ: f64 = 64.0;

fn main() -> anyhow::Result<()> {
    let scenario_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenario.json".to_string());
    let scenario_config = scenario::load_scenario(&scenario_path)?;
    // Captured before the config is moved into the `Roster` resource below.
    let perception_seed = scenario_config.seed;

    // Own the tokio runtime explicitly. The transport and viz servers run on
    // its background workers; Bevy's blocking headless run-loop then owns the
    // main thread, so it never starves the async servers (as it would if it
    // parked a worker under `#[tokio::main]`). The runtime must outlive the
    // app, so it's held here for the whole run.
    let runtime = tokio::runtime::Runtime::new()?;
    let (transport_handle, viz_handle, perception_handle) = runtime.block_on(async {
        let transport_handle = transport::spawn(transport::Config::default()).await?;
        println!("listening for agents on {}", transport_handle.local_addr);
        let viz_handle = viz::spawn(viz::VizConfig::default()).await?;
        println!("streaming viz on {}", viz_handle.local_addr);
        let perception_handle = perception::spawn(perception::PerceptionConfig::default()).await?;
        println!("serving perception on {}", perception_handle.local_addr);
        anyhow::Ok((transport_handle, viz_handle, perception_handle))
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
    // Select the world: a road map, or the flat arena. Only the built-in "demo"
    // road exists today; loading a real OpenDRIVE file by path arrives at P5.
    let map_world = match scenario_config.map.as_deref() {
        None => None,
        Some("demo") => Some(map::demo_road()),
        Some(other) => anyhow::bail!(
            "unknown map {other:?}: only the built-in \"demo\" road exists \
             (OpenDRIVE file import arrives at P5)"
        ),
    };

    // Time model (env `SIM_TIME`):
    //   realtime (default) — the fixed step is paced to wall-clock, so one
    //     sim-second is one real second. Required for live viewing.
    //   afap — "as fast as possible": the run-loop spins without sleeping and
    //     virtual time advances one fixed step per iteration, decoupled from
    //     wall-clock, so the sim runs at CPU speed (headless batch runs). A
    //     realtime viewer can't keep pace with this.
    let afap = std::env::var("SIM_TIME")
        .map(|v| v.eq_ignore_ascii_case("afap"))
        .unwrap_or(false);
    let fixed_dt = Duration::from_secs_f64(1.0 / TICK_HZ);
    // realtime: drive Update at ~120 Hz (Fixed paces itself under it).
    // afap: no sleep — go as fast as the CPU allows.
    let loop_wait = if afap {
        Duration::ZERO
    } else {
        Duration::from_secs_f64(1.0 / 120.0)
    };
    println!("time mode: {}", if afap { "afap" } else { "realtime" });

    let mut app = App::new();
    app
        // Headless: no window or rendering — rendering lives in the viewer.
        // A bounded run-loop drives the app instead of a window event loop.
        .add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(loop_wait)))
        // Pin the fixed step to TICK_HZ rather than inheriting Bevy's default.
        .insert_resource(Time::<Fixed>::from_hz(TICK_HZ))
        .add_plugins(bevy::log::LogPlugin::default())
        .add_plugins(TransformPlugin)
        .add_plugins(StatesPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_plugins(movement::MovementPlugin)
        .init_state::<ScenarioState>()
        .add_message::<ReflexFired>()
        .insert_resource(Roster(scenario_config))
        .insert_resource(arena_bounds)
        .insert_resource(world::MapWorld(map_world))
        .insert_resource(pending_roster)
        .insert_resource(AgentRegistry::default())
        .insert_resource(AwaitingReconnect::default())
        .insert_resource(EndReason::default())
        .insert_resource(Tick::default())
        .insert_resource(Transport(transport_handle))
        .insert_resource(Viz(viz_handle))
        .insert_resource(Perception(perception_handle))
        .insert_resource(PerceptionSeed(perception_seed))
        .insert_resource(PerceptionAgents::default())
        .insert_resource(PerceivedWorlds::default())
        .insert_resource(PerceptionOverlay::default())
        .add_systems(Startup, (setup_world, deactivate_physics))
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
        // Recompute per-device perceived worlds before arbitration, so a
        // `Simulated` reflex reads exactly what was delivered this frame (and
        // the same set the agent gets on :4002).
        .add_systems(
            FixedUpdate,
            route_perception
                .before(arbitration::arbitrate)
                .run_if(in_state(ScenarioState::Running)),
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
            // The overlay is written each fixed step by route_perception, so by
            // Update it already carries this frame's perceived set.
            broadcast_frames
                .after(drain_viz_events)
                .run_if(in_state(ScenarioState::Running)),
        )
        // Register agents connecting on the perception port, in all states.
        .add_systems(Update, drain_perception_events)
        .add_systems(
            OnEnter(ScenarioState::Running),
            (activate_physics, broadcast_state),
        )
        .add_systems(
            OnEnter(ScenarioState::Ended),
            (deactivate_physics, notify_scenario_ended, broadcast_state),
        );

    if afap {
        // Advance virtual time by exactly one fixed step per Update, so the
        // Fixed accumulator releases one physics step per loop iteration
        // regardless of wall-clock — the loop then runs at CPU speed.
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(fixed_dt));
    }

    app.run();

    Ok(())
}

/// Builds the world at startup: the road map if the scenario selected one,
/// otherwise the flat arena.
fn setup_world(mut commands: Commands, roster: Res<Roster>, map: Res<world::MapWorld>) {
    match &map.0 {
        Some(road) => world::spawn_road(&mut commands, road),
        None => world::spawn_arena(&mut commands, &roster.0.arena),
    }
}
