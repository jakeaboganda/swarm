//! Shared wire types for the *agent* pathway (WebSocket + JSON) plus the
//! scenario schema. `messages` holds the client/server messages (join, plan,
//! reflex rules, `request_route`; snapshots and `joined`/`reflex_fired`/`route`/
//! `scenario_ended`/`error` events); `map` is the road handed to an agent at
//! join; `scenario` is the roster / arena / `map` / sensor JSON. Depends on
//! nothing else in the workspace -- consumers convert to their own vector types
//! at the edge (`Vec3` here is deliberately engine-agnostic).

mod vec3;

pub mod map;
pub mod messages;
pub mod scenario;

pub use vec3::Vec3;
