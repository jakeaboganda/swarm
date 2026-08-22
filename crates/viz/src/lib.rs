//! The visualization pathway: semantic scene types the sim streams to
//! viewers. A forward/observational channel, separate from the agent
//! protocol.

mod broadcast;
mod frame;
mod math;
mod message;
mod scene;

pub use broadcast::{spawn, ViewerId, VizConfig, VizEvent, VizHandle};
pub use frame::{
    Blip, DebugFrame, DetectionKind, EntityDebug, EntityFrame, Frame, WheelDebug, WheelPose,
};
pub use math::{Quat, Transform, Vec3};
pub use message::{decode, encode, Hello, ServerToViewer, ViewerToServer, PROTOCOL_VERSION};
pub use scene::{
    ArenaBounds, Color, Embodiment, EntityDescriptor, EntityId, EntityKind, ScenarioState,
    SceneEvent, SceneInit, SensorView, Shape, WheelRig,
};
