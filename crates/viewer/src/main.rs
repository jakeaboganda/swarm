mod client;
mod overlay;
mod scene;

use bevy::prelude::*;
use bevy::winit::{UpdateMode, WinitSettings};

use scene::{EntityMap, ViewerState};

#[tokio::main]
async fn main() {
    let url = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "ws://127.0.0.1:4001".to_string());
    println!("viewer connecting to {url}");

    let (stream, senders) = client::channels();
    tokio::spawn(client::run_client(url, senders));

    App::new()
        .add_plugins(DefaultPlugins)
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
        .add_systems(Startup, scene::setup_camera)
        .add_systems(Update, (scene::apply_stream, scene::frame_camera).chain())
        // Overlays read the state applied above, so order them after it.
        .add_systems(
            Update,
            (
                overlay::record_trails,
                overlay::draw_plans,
                overlay::draw_trails,
            )
                .after(scene::apply_stream),
        )
        .run();
}
