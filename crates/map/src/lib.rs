//! The pure-Rust road-network model — the "compiled map" every importer bakes
//! into and every consumer reads. Right-handed, **Y-up, meters** (matching viz
//! and Bevy). Format-agnostic: nothing here knows OpenDRIVE or OSM.
//!
//! Curves are baked to polylines, so consumers only ever sample points — an
//! importer (libOpenDRIVE today, a pure-Rust port later) does the clothoid math
//! once at load. Road grouping and a routing graph arrive with that importer.
//!
//! ## Importer contract
//! An importer baking external (possibly malformed) map data must:
//! - build lane geometry with [`Polyline::try_new`] and surface an error on
//!   degenerate input, rather than the panicking [`Polyline::new`];
//! - keep `LaneId`s opaque — never assume they index `RoadNetwork.lanes`;
//! - avoid lane curvature tighter than the half-width, or the `surface_mesh`
//!   ribs can self-intersect (the tessellator doesn't yet guard against it).

mod build;
mod geometry;
mod mesh;
mod network;
mod route;

pub use build::demo_road;
pub use geometry::{Polyline, Pose, Projection};
pub use mesh::Mesh;
pub use network::{Direction, Lane, LaneId, LaneKind, RoadNetwork};
