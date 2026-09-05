//! The sensor pathway: simulated, per-agent perception the sim pushes to
//! agents. A separate pathway from both the agent control channel and viz --
//! `provider → server-router → agent` -- so the producer (analytic today, a
//! rendered-sensor provider later) stays swappable. JSON wire, mirroring the
//! agent control channel, for language-agnostic clients.

mod math;
mod message;
mod server;

pub use math::Vec3;
pub use message::{
    decode, encode, AgentToServer, Detection, DetectionKind, Hello, PerceptionFrame, Scalars,
    ServerToAgent, PROTOCOL_VERSION,
};
pub use server::{spawn, PerceptionConfig, PerceptionEvent, PerceptionHandle};
