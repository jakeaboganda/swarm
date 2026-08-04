mod client;
mod overlay;
mod scene;

use bevy::prelude::*;
use bevy::window::{PresentMode, Window, WindowPlugin};
use bevy::winit::{UpdateMode, WinitSettings};

use scene::{Diag, EntityMap, RenderClock, ViewerState};

#[tokio::main]
async fn main() {
    let url = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "ws://127.0.0.1:4001".to_string());
    println!("viewer connecting to {url}");

    let (stream, senders) = client::channels();
    tokio::spawn(client::run_client(url, senders));

    App::new()
        // Force FIFO (hard vsync): present blocks until the display refresh,
        // so frames are delivered at a uniform cadence. Without this the
        // uncapped render loop presents unevenly (measured 4-28ms between
        // frames), and since playback advances the render clock by each
        // frame's real dt, that unevenness shows up as motion judder.
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                present_mode: PresentMode::Fifo,
                ..default()
            }),
            ..default()
        }))
        // Update continuously even when the window isn't focused. The
        // viewer renders an external live stream, and Bevy's default
        // (`WinitSettings::game`) drops an unfocused window into a reactive
        // mode that only updates on input events — so without this, frames
        // (drained in `apply_stream`, an Update system) would only advance
        // when the mouse moves over the window.
        .insert_resource(WinitSettings {
            focused_mode: UpdateMode::Continuous,
            unfocused_mode: UpdateMode::Continuous,
        })
        .insert_resource(stream)
        .insert_resource(EntityMap::default())
        .insert_resource(ViewerState::default())
        .insert_resource(RenderClock::default())
        .insert_resource(Diag::new(std::env::var("VIZ_DIAG").is_ok()))
        .add_systems(Startup, scene::setup_camera)
        // Apply the stream, then advance the sim-time render clock to pose
        // entities; overlays read the posed transforms, so run after.
        .add_systems(
            Update,
            (
                scene::apply_stream,
                scene::advance_playback,
                scene::frame_camera,
            )
                .chain(),
        )
        .add_systems(Update, scene::log_timing.after(scene::apply_stream))
        .add_systems(
            Update,
            (
                overlay::record_trails,
                overlay::draw_plans,
                overlay::draw_trails,
            )
                .after(scene::advance_playback),
        )
        .run();
}
