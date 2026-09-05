mod client;
mod follow;
mod overlay;
mod scene;
mod view;
mod wheels;

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

    // Present mode. FIFO (hard vsync) is the default: present blocks until the
    // display refresh, so frames arrive at a uniform cadence at low power.
    //
    // VIZ_GPU_KEEPALIVE=1 switches to uncapped (no-vsync) rendering. This is
    // the portable equivalent of running `vkcube` alongside the viewer: an
    // aggressive dynamic-power-management GPU (e.g. an NVIDIA hybrid laptop
    // with Runtime-D3) suspends the dGPU during the idle gaps FIFO leaves
    // between frames, and the ~250 ms resume latency shows up as a periodic
    // whole-process stall. Rendering uncapped never lets the GPU idle, so it
    // stays clocked up -- no vendor APIs or root needed. Motion stays smooth
    // regardless of the (now variable) frame rate because playback runs on the
    // sim-time render clock, not per-frame dt. Costs power/heat, hence opt-in.
    let present_mode = if std::env::var("VIZ_GPU_KEEPALIVE").is_ok() {
        PresentMode::AutoNoVsync
    } else {
        PresentMode::Fifo
    };

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                present_mode,
                ..default()
            }),
            ..default()
        }))
        // Update continuously even when the window isn't focused. The
        // viewer renders an external live stream, and Bevy's default
        // (`WinitSettings::game`) drops an unfocused window into a reactive
        // mode that only updates on input events -- so without this, frames
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
        .insert_resource(overlay::OverlayToggles::default())
        .insert_resource(follow::FollowCam::default())
        .insert_resource(view::CameraView::default())
        .add_systems(
            Startup,
            (
                scene::setup_camera,
                follow::setup_follow_label,
                overlay::setup_waypoint_markers,
            ),
        )
        // Apply the stream, then advance the sim-time render clock to pose
        // entities; the cameras and overlays read the posed transforms, so run
        // after. follow_camera drives the camera in follow mode; frame_camera
        // owns it in overview.
        .add_systems(
            Update,
            (
                scene::apply_stream,
                scene::advance_playback,
                follow::follow_camera,
                scene::frame_camera,
            )
                .chain(),
        )
        .add_systems(Update, scene::log_timing.after(scene::advance_playback))
        .add_systems(Update, wheels::tint_wheels.after(scene::apply_stream))
        .add_systems(
            Update,
            (
                overlay::toggle_overlays,
                follow::follow_input,
                view::view_input,
                follow::update_follow_label,
            ),
        )
        .add_systems(
            Update,
            (
                overlay::record_trails,
                overlay::draw_plans,
                overlay::sync_waypoint_markers,
                overlay::draw_trails,
                overlay::draw_detections,
                overlay::draw_sensor_envelope,
            )
                .after(scene::advance_playback),
        )
        .run();
}
