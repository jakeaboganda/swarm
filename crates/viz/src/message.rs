use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::frame::{DebugFrame, Frame};
use crate::scene::{SceneEvent, SceneInit};

/// Version of the viz wire schema. A viewer declares it in `Hello`; the sim
/// declares it in `SceneInit`. Bump it for any breaking change — notably a
/// new message/enum variant (e.g. the future delta/keyframe frame), which
/// an older internally-tagged decoder would otherwise fail on. Additive
/// *fields* are backward compatible and don't require a bump.
pub const PROTOCOL_VERSION: u32 = 1;

/// Everything the sim streams to a viewer. The scene layer (`SceneInit`,
/// `Event`, `Frame`) is canonical/physical; `DebugFrame` is the optional
/// annotation layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerToViewer {
    SceneInit(SceneInit),
    Event(SceneEvent),
    Frame(Frame),
    DebugFrame(DebugFrame),
}

/// Everything a viewer sends to the sim. Deliberately small: viz is a
/// forward/observational pathway, so this is just a connect-time handshake.
/// (Sensor data is a *separate* pathway, not viz — see DECISIONS.)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ViewerToServer {
    Hello(Hello),
}

/// A viewer's connect-time declaration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Hello {
    /// The schema version the viewer speaks. The sim drops the connection
    /// on mismatch rather than stream bytes the viewer can't decode.
    pub protocol_version: u32,
    /// Whether this viewer wants the debug/annotation layer in addition to
    /// the scene layer. Lets a scene-only consumer opt out of the extra
    /// traffic.
    pub subscribe_debug: bool,
}

impl Default for Hello {
    fn default() -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            subscribe_debug: true,
        }
    }
}

/// Encodes a message to MessagePack — the wire format. Uses named fields so
/// the bytes are self-describing and decodable from other languages
/// (browser, Python).
pub fn encode<T: Serialize>(message: &T) -> Vec<u8> {
    rmp_serde::to_vec_named(message).expect("viz messages always serialize")
}

/// Decodes a MessagePack message.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> Result<T, rmp_serde::decode::Error> {
    rmp_serde::from_slice(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{EntityDebug, EntityFrame};
    use crate::math::{Quat, Transform, Vec3};
    use crate::scene::*;

    fn sample_descriptor() -> EntityDescriptor {
        EntityDescriptor {
            id: EntityId("car-1".into()),
            name: "car-1".into(),
            kind: EntityKind::Agent {
                embodiment: Embodiment::CarLike,
            },
            shape: Shape::Capsule {
                radius: 0.5,
                half_length: 0.5,
            },
            color: Color {
                r: 0.9,
                g: 0.4,
                b: 0.1,
            },
            transform: Transform::IDENTITY,
        }
    }

    fn samples() -> Vec<ServerToViewer> {
        vec![
            ServerToViewer::SceneInit(SceneInit {
                protocol_version: PROTOCOL_VERSION,
                tick: 0,
                state: ScenarioState::WaitingForRoster,
                arena: ArenaBounds {
                    width: 50.0,
                    depth: 50.0,
                },
                entities: vec![
                    sample_descriptor(),
                    EntityDescriptor {
                        id: EntityId("wall-0".into()),
                        name: "wall-0".into(),
                        kind: EntityKind::Static,
                        shape: Shape::Cuboid {
                            half_extents: Vec3::new(25.0, 1.5, 0.25),
                        },
                        color: Color {
                            r: 0.45,
                            g: 0.47,
                            b: 0.52,
                        },
                        transform: Transform::IDENTITY,
                    },
                ],
            }),
            ServerToViewer::Event(SceneEvent::EntitySpawned(sample_descriptor())),
            ServerToViewer::Event(SceneEvent::EntityDespawned {
                id: EntityId("car-2".into()),
            }),
            ServerToViewer::Event(SceneEvent::ScenarioState {
                state: ScenarioState::Running,
            }),
            ServerToViewer::Frame(Frame {
                tick: 42,
                entities: vec![EntityFrame {
                    id: EntityId("car-1".into()),
                    transform: Transform {
                        position: Vec3::new(1.0, 1.0, 2.0),
                        rotation: Quat::new(0.0, 0.6, 0.0, 0.8),
                    },
                }],
            }),
            ServerToViewer::DebugFrame(DebugFrame {
                tick: 42,
                entities: vec![EntityDebug {
                    id: EntityId("car-1".into()),
                    plan: vec![Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 10.0)],
                    reflex_active: true,
                }],
            }),
        ]
    }

    #[test]
    fn server_messages_round_trip_through_msgpack() {
        for message in samples() {
            let bytes = encode(&message);
            let back: ServerToViewer = decode(&bytes).expect("decode");
            assert_eq!(message, back);
        }
    }

    #[test]
    fn messages_also_round_trip_through_json_for_debugging() {
        for message in samples() {
            let json = serde_json::to_string(&message).expect("to json");
            let back: ServerToViewer = serde_json::from_str(&json).expect("from json");
            assert_eq!(message, back);
        }
    }

    #[test]
    fn viewer_handshake_round_trips() {
        let hello = ViewerToServer::Hello(Hello {
            protocol_version: PROTOCOL_VERSION,
            subscribe_debug: false,
        });
        let bytes = encode(&hello);
        let back: ViewerToServer = decode(&bytes).expect("decode");
        assert_eq!(hello, back);
    }
}
