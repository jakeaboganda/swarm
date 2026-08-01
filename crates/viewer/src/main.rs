mod client;
mod overlay;
mod scene;

use bevy::prelude::*;

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
