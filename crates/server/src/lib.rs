//! The headless simulation: ECS + physics, the scenario lifecycle, and the
//! three pathways (agent transport, viz, perception) wired into a Bevy app.
//!
//! The binary in `main.rs` is process concerns only -- argument parsing, the
//! tokio runtime, and binding the servers. Everything else lives here, so
//! integration tests can build the same app on ephemeral ports and step it.

pub mod agent;
pub mod app;
pub mod arbitration;
pub mod events;
pub mod fmu_setup;
pub mod inbound;
pub mod perception_router;
pub mod pulse;
pub mod scenario;
pub mod scenario_state;
pub mod time_budget;
pub mod tracker;
pub mod transport_bridge;
pub mod viz_broadcast;
pub mod viz_nodes;
pub mod world;

pub use app::{build_app, load_map, SimConfig};
