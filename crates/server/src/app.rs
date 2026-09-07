//! The simulation's system graph, assembled in one place.
//!
//! `build_app` is the whole headless sim minus process concerns (argument
//! parsing, the tokio runtime, binding the three servers). Keeping it separate
//! from `main` is what lets an integration test stand a real sim up on
//! ephemeral ports and step it one tick at a time.

use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_rapier3d::prelude::{NoUserData, PhysicsSet, RapierPhysicsPlugin};
use protocol::scenario::ScenarioConfig;

use crate::agent::{AgentRegistry, AwaitingReconnect, PendingRoster};
use crate::arbitration;
use crate::events::ReflexFired;
use crate::perception_router::{
    drain_perception_events, route_perception, PerceivedWorlds, Perception, PerceptionAgents,
    PerceptionOverlay, PerceptionSeed,
};
use crate::pulse::PulseStates;
use crate::scenario::{ArenaBounds, Roster};
use crate::scenario_state::{EndReason, ScenarioState, Tick};
use crate::time_budget::{self, Deadline, TICK_HZ};
use crate::transport_bridge::{
    activate_physics, deactivate_physics, despawn_off_road, drain_transport, enforce_duration,
    expire_reconnects, floor_for, forward_reflex_fired, notify_scenario_ended, send_pulses,
    ReconnectGrace, Transport,
};
use crate::viz_broadcast::{
    broadcast_frames, broadcast_spawns, broadcast_state, drain_viz_events, Viz,
};
use crate::world;

/// Everything the sim needs that the process supplies: the loaded scenario,
/// its baked map, the pace, and the three already-bound pathway servers.
pub struct SimConfig {
    pub scenario: ScenarioConfig,
    /// The baked road network, or `None` for the flat arena world. Loaded by
    /// [`load_map`] from `scenario.map`.
    pub map: Option<map::RoadNetwork>,
    /// `true` ticks at CPU speed with virtual time advanced one fixed step per
    /// `Update`; `false` paces the fixed step to wall-clock.
    pub afap: bool,
    pub transport: transport::TransportHandle,
    pub viz: viz::VizHandle,
    pub perception: perception::PerceptionHandle,
}

/// Selects the world from a scenario's `map` field: a road map, or the flat
/// arena. `"demo"` is the built-in hand-authored road; a path ending in
/// `.xodr` is loaded via the OpenDRIVE importer and baked into the same
/// `RoadNetwork` (nothing downstream cares which source it came from).
pub fn load_map(spec: Option<&str>) -> anyhow::Result<Option<map::RoadNetwork>> {
    let network = match spec {
        None => return Ok(None),
        Some("demo") => map::demo_road(),
        // The banked oval is a plain canted RoadNetwork: its lane carries the
        // bank, so `surface_mesh` builds the canted collider/viz and
        // `sample_near` drapes FMU cars -- no bespoke track bundle.
        Some("banked_oval") => map::banked_oval(),
        Some(path) if path.ends_with(".xodr") => map_opendrive::load_file(path)
            .map_err(|e| anyhow::anyhow!("loading map {path:?}: {e}"))?,
        Some(other) => {
            anyhow::bail!(
                "unknown map {other:?}: use \"demo\", \"banked_oval\", or a path ending in .xodr"
            )
        }
    };
    // The road becomes one static trimesh collider at startup, inside a Bevy
    // system with nowhere to report a failure. An imported map's vertices trace
    // back to an untrusted file, so the mesh is checked here instead -- while
    // the file still has a name to put in the error.
    network
        .surface_mesh()
        .validate()
        .map_err(|e| anyhow::anyhow!("map {:?} is not drivable: {e}", spec.unwrap_or("<none>")))?;
    Ok(Some(network))
}

/// Builds the headless simulation: resources, the fixed-step control loop, and
/// the viz/perception broadcasts. The caller runs it (`app.run()`) or steps it
/// (`app.update()`).
pub fn build_app(config: SimConfig) -> App {
    let SimConfig {
        scenario,
        map: map_world,
        afap,
        transport,
        viz,
        perception,
    } = config;

    let perception_seed = scenario.seed;
    // Whether this map is superelevated: any lane carrying a bank profile. Drives
    // whether `conform_fmu_to_track` runs -- it drapes FMU cars onto a canted
    // surface, and is pure overhead on a flat map.
    let map_banked = map_world
        .as_ref()
        .is_some_and(|net| net.lanes.iter().any(|l| !l.bank.is_empty()));
    // The baked road surface, for the per-wheel FMU drape (sampled every tick).
    // Built once here rather than re-tessellated per tick.
    let road_mesh = map_world.as_ref().map(|net| net.surface_mesh());
    let pending_roster = PendingRoster(scenario.roster.iter().map(|s| s.name.clone()).collect());
    let arena_bounds = ArenaBounds {
        half_width: scenario.arena.width / 2.0,
        half_depth: scenario.arena.depth / 2.0,
    };
    // Run length: the scenario's `time.duration` (sim-seconds) as a tick
    // deadline, `None` if unbounded. Enforced by `enforce_duration`.
    let deadline = Deadline(time_budget::deadline_tick(scenario.time.duration, TICK_HZ));
    // The off-map fall floor, relative to this map's own elevation (so a road
    // that dips below zero isn't mistaken for freefall).
    let floor = floor_for(map_world.as_ref());
    let fixed_dt = Duration::from_secs_f64(1.0 / TICK_HZ);
    // realtime: drive Update at ~120 Hz (Fixed paces itself under it).
    // afap: no sleep -- go as fast as the CPU allows.
    let loop_wait = if afap {
        Duration::ZERO
    } else {
        Duration::from_secs_f64(1.0 / 120.0)
    };

    let mut app = App::new();
    app
        // Headless: no window or rendering -- rendering lives in the viewer.
        // A bounded run-loop drives the app instead of a window event loop.
        .add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(loop_wait)))
        // Pin the fixed step to TICK_HZ rather than inheriting Bevy's default.
        .insert_resource(Time::<Fixed>::from_hz(TICK_HZ))
        .add_plugins(TransformPlugin)
        .add_plugins(StatesPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_plugins(movement::MovementPlugin)
        .init_state::<ScenarioState>()
        .add_message::<ReflexFired>()
        .insert_resource(Roster(scenario))
        .insert_resource(arena_bounds)
        .insert_resource(world::MapWorld(map_world))
        .insert_resource(world::MapBanked(map_banked))
        .insert_resource(world::RoadMesh(road_mesh))
        .insert_resource(pending_roster)
        .insert_resource(AgentRegistry::default())
        .insert_resource(AwaitingReconnect::default())
        .insert_resource(EndReason::default())
        .insert_resource(Tick::default())
        .insert_resource(deadline)
        .insert_resource(floor)
        .insert_resource(ReconnectGrace::default())
        .insert_resource(PulseStates::default())
        .insert_resource(Transport(transport))
        .insert_resource(Viz(viz))
        .insert_resource(Perception(perception))
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
        // After arbitration (so `tick` and each plan's version are current for
        // this step): end the run if it's hit its duration, and push a step
        // pulse to each subscribed agent that isn't awaiting an ack.
        .add_systems(
            FixedUpdate,
            (enforce_duration, send_pulses)
                .after(arbitration::arbitrate)
                .run_if(in_state(ScenarioState::Running)),
        )
        // Drape FMU vehicles onto the banked surface: after the FMU writes its
        // flat pose (MovementSet::ApplyForce), before Rapier reads the kinematic
        // target (SyncBackend). Runs only on a superelevated map.
        .add_systems(
            FixedUpdate,
            world::conform_fmu_to_track
                .after(movement::MovementSet::ApplyForce)
                .before(PhysicsSet::SyncBackend)
                .run_if(|banked: Res<world::MapBanked>| banked.0),
        )
        // Viz broadcast. `broadcast_spawns` runs before `drain_viz_events`
        // so a viewer connecting the same frame an agent joins learns of
        // that agent only via its scene-init, never also via a duplicate
        // EntitySpawned (a not-yet-ready viewer is skipped by the spawn
        // broadcast). Frames stream only while Running -- no dynamic state
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
        // Remove any vehicle that has fallen off the road (Transform reflects
        // the latest physics writeback by Update); runs only while Running.
        .add_systems(
            Update,
            despawn_off_road.run_if(in_state(ScenarioState::Running)),
        )
        // Register agents connecting on the perception port, in all states.
        .add_systems(Update, drain_perception_events)
        // Free FMU handles for despawned vehicles, in all states (despawns
        // happen both pre-start and while Running). Drains RemovedComponents
        // every frame before the events age out.
        .add_systems(Update, world::free_despawned_fmus)
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
        // regardless of wall-clock -- the loop then runs at CPU speed. It also
        // makes a stepped app deterministic: one `update()` is one tick.
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(fixed_dt));
    }

    app
}

/// Builds the world at startup: the road map if the scenario selected one,
/// otherwise the flat arena.
fn setup_world(mut commands: Commands, roster: Res<Roster>, map: Res<world::MapWorld>) {
    match &map.0 {
        // A road (banked or flat): surface_mesh is the collider/viz, canted where
        // the lanes are.
        Some(road) => world::spawn_road(&mut commands, road),
        None => world::spawn_arena(&mut commands, &roster.0.arena),
    }
}

#[cfg(test)]
mod map_banked_tests {
    //! D4 wiring at the `load_map` boundary: the retire replaced the bespoke
    //! `BankedTrack`/`BankedTrackRes` with a plain canted `RoadNetwork` and a
    //! `MapBanked(bool)` gate. `MapBanked` is exactly this predicate over the
    //! loaded network, so pinning it here pins what actually decides whether
    //! `conform_fmu_to_track` runs. (`build_app` computes the identical
    //! expression; the full-app resource value is asserted in the server
    //! integration tests.)

    use super::load_map;

    /// The `MapBanked` predicate: any lane carries a (non-empty) bank profile.
    /// Mirrors `build_app`'s inline `map_banked` computation.
    fn is_banked(net: &map::RoadNetwork) -> bool {
        net.lanes.iter().any(|l| !l.bank.is_empty())
    }

    // A banked `.xodr` fixture with real superelevation, and a flat one from
    // before the feature (no `<superelevation>` -> every lane's bank is empty).
    const BANKED_XODR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../map-opendrive/tests/data/banked_sweeper.xodr"
    );
    const FLAT_XODR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../map-opendrive/tests/data/param_poly3.xodr"
    );

    #[test]
    fn banked_oval_loads_as_a_drivable_banked_lap() {
        // The retire re-expressed `banked_oval()` as a plain RoadNetwork; loading
        // it must still yield one canted, drivable, lappable lane.
        let net = load_map(Some("banked_oval"))
            .expect("banked_oval loads")
            .expect("banked_oval is a map");
        let lanes: Vec<_> = net.driving_lanes().collect();
        assert_eq!(lanes.len(), 1, "the oval is one driving lane");
        assert!(!lanes[0].bank.is_empty(), "the oval lane must carry a bank");
        // The lap wraps: driving off the exit re-enters the same lane, so
        // routing over the loop terminates.
        assert_eq!(lanes[0].successors, vec![lanes[0].id]);
        // The successor edge resolves through the network's routing lookup.
        assert_eq!(
            net.successors(lanes[0].id).map(|l| l.id).next(),
            Some(lanes[0].id),
            "the oval must route (successor wraps to itself)"
        );
        // load_map already ran surface_mesh().validate(); assert the cant is real
        // in the built collider mesh (a tilted normal exists).
        let mesh = net.surface_mesh();
        mesh.validate().expect("the oval collider mesh is valid");
        assert!(
            mesh.normals.iter().any(|n| n.y < 0.99),
            "the oval collider mesh is not canted"
        );
    }

    #[test]
    fn map_banked_is_true_for_banked_maps() {
        // The built-in oval and an imported banked `.xodr` both drive the conform.
        let oval = load_map(Some("banked_oval")).unwrap().unwrap();
        assert!(is_banked(&oval), "banked_oval must read as banked");

        let imported = load_map(Some(BANKED_XODR)).unwrap().unwrap();
        assert!(
            is_banked(&imported),
            "the imported banked sweeper must read as banked -- this is what makes \
             conform_fmu_to_track run on an imported map (feature goal b)"
        );
    }

    #[test]
    fn map_banked_is_false_for_flat_maps() {
        // The flat built-in road and a flat imported map: no lane carries a bank,
        // so conform_fmu_to_track is skipped and FMU cars are untouched, exactly
        // as before the retire.
        let demo = load_map(Some("demo")).unwrap().unwrap();
        assert!(!is_banked(&demo), "demo_road has no bank");

        let flat = load_map(Some(FLAT_XODR)).unwrap().unwrap();
        assert!(
            !is_banked(&flat),
            "a pre-superelevation imported map must read as flat"
        );
    }
}
