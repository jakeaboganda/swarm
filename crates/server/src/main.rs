//! Process entry point: parse arguments, own the tokio runtime, bind the three
//! pathway servers, and run the app that `server::build_app` assembles.

use bevy::prelude::*;
use server::{build_app, load_map, SimConfig};

fn main() -> anyhow::Result<()> {
    let scenario_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenario.json".to_string());
    let scenario = server::scenario::load_scenario(&scenario_path)?;

    // Own the tokio runtime explicitly. The transport and viz servers run on
    // its background workers; Bevy's blocking headless run-loop then owns the
    // main thread, so it never starves the async servers (as it would if it
    // parked a worker under `#[tokio::main]`). The runtime must outlive the
    // app, so it's held here for the whole run.
    let runtime = tokio::runtime::Runtime::new()?;
    let (transport, viz, perception) = runtime.block_on(async {
        let transport = transport::spawn(transport::Config::default()).await?;
        println!("listening for agents on {}", transport.local_addr);
        let viz = viz::spawn(viz::VizConfig::default()).await?;
        println!("streaming viz on {}", viz.local_addr);
        let perception = perception::spawn(perception::PerceptionConfig::default()).await?;
        println!("serving perception on {}", perception.local_addr);
        anyhow::Ok((transport, viz, perception))
    })?;
    let _runtime_guard = runtime.enter();

    let map = load_map(scenario.map.as_deref())?;

    // Pace (the scenario's `time.pace`, overridable by env `SIM_TIME`):
    //   realtime (default) -- the fixed step is paced to wall-clock, so one
    //     sim-second is one real second. Required for live viewing.
    //   afap -- "as fast as possible": the run-loop spins without sleeping and
    //     virtual time advances one fixed step per iteration, decoupled from
    //     wall-clock, so the sim runs at CPU speed (headless batch runs). A
    //     realtime viewer can't keep pace with this.
    // The scenario owns pace; `SIM_TIME` stays an ad-hoc override (set it to
    // force a pace without editing the file).
    let afap = match std::env::var("SIM_TIME") {
        Ok(v) => v.eq_ignore_ascii_case("afap"),
        Err(_) => matches!(scenario.time.pace, protocol::scenario::Pace::Afap),
    };
    println!("time mode: {}", if afap { "afap" } else { "realtime" });

    let mut app = build_app(SimConfig {
        scenario,
        map,
        afap,
        transport,
        viz,
        perception,
    });
    // Logging is the binary's concern: a test harness installs its own
    // subscriber, and a second `LogPlugin` would panic.
    app.add_plugins(bevy::log::LogPlugin::default());
    app.run();

    Ok(())
}
