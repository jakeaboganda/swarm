use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::math::Vec3;

/// Version of the perception wire schema. An agent declares it in `Hello`;
/// the server drops a connection on mismatch rather than stream bytes the
/// agent can't decode.
pub const PROTOCOL_VERSION: u32 = 1;

/// An agent's connect-time declaration on the perception port: which roster
/// slot it is (so the server routes that agent's perception to it) and the
/// schema it speaks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Hello {
    pub agent_name: String,
    pub protocol_version: u32,
}

/// What a detected entity is, so an agent can tell a peer from a wall.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DetectionKind {
    Agent,
    Static,
}

/// One entity as the agent perceives it: identity + kind carried through,
/// pose degraded by the agent's sensor spec.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    pub id: String,
    pub kind: DetectionKind,
    pub position: Vec3,
    pub velocity: Vec3,
    pub distance: f32,
    /// The perceived entity's body radius, so the agent's own avoidance can
    /// size the obstacle rather than assuming one. Size is a known property,
    /// so it isn't degraded by the sensor's position/velocity noise.
    pub radius: f32,
}

/// Impaired scalar readings computed over the same detected set -- the
/// existing named sensors, read through the agent's degraded perception.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Scalars {
    /// Time-to-collision over detected obstacles. `None` when no collision is
    /// predicted (would be infinite) -- JSON has no infinity.
    pub time_to_collision: Option<f32>,
    /// The agent's own speed.
    pub speed: f32,
}

/// One perception tick for one agent, through one of its named devices.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PerceptionFrame {
    /// Which device produced this frame (the agent's `SensorDef` name), so an
    /// agent with several sensors can tell them apart.
    pub sensor: String,
    pub tick: u64,
    pub detections: Vec<Detection>,
    pub scalars: Scalars,
}

/// Everything an agent sends the perception server. Deliberately tiny: this
/// is a forward pathway, so it's just the connect-time handshake.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum AgentToServer {
    Hello(Hello),
}

/// Everything the perception server pushes to an agent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerToAgent {
    Perception(PerceptionFrame),
}

/// Encodes a message as a JSON string -- the wire format is text frames, like
/// the agent control channel, so clients decode with a plain JSON parser.
pub fn encode<T: Serialize>(message: &T) -> String {
    serde_json::to_string(message).expect("perception messages always serialize")
}

/// Decodes a JSON message.
pub fn decode<T: DeserializeOwned>(text: &str) -> Result<T, serde_json::Error> {
    serde_json::from_str(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agent_hello_round_trips() {
        let hello = AgentToServer::Hello(Hello {
            agent_name: "car-1".into(),
            protocol_version: PROTOCOL_VERSION,
        });
        let back: AgentToServer = decode(&encode(&hello)).expect("decode");
        assert_eq!(hello, back);
    }

    #[test]
    fn perception_frame_round_trips() {
        let frame = ServerToAgent::Perception(PerceptionFrame {
            sensor: "radar".into(),
            tick: 42,
            detections: vec![Detection {
                id: "car-2".into(),
                kind: DetectionKind::Agent,
                position: Vec3::new(1.0, 0.0, 2.0),
                velocity: Vec3::new(-1.0, 0.0, 0.0),
                distance: 5.0,
                radius: 0.75,
            }],
            scalars: Scalars {
                time_to_collision: Some(1.5),
                speed: 3.0,
            },
        });
        let back: ServerToAgent = decode(&encode(&frame)).expect("decode");
        assert_eq!(frame, back);
    }

    #[test]
    fn infinite_ttc_is_none_not_infinity() {
        // Guards the JSON-has-no-infinity trap: no-collision must round-trip.
        let scalars = Scalars {
            time_to_collision: None,
            speed: 0.0,
        };
        let back: Scalars = decode(&encode(&scalars)).expect("decode");
        assert_eq!(scalars, back);
    }
}
