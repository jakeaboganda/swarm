use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::frame::{DebugFrame, Frame};
use crate::scene::{SceneEvent, SceneInit};

/// Version of the viz wire schema. A viewer declares it in `Hello`; the sim
/// declares it in `SceneInit`. Bump it for any breaking change — notably a
/// new message/enum variant (e.g. the future delta/keyframe frame), which
/// an older internally-tagged decoder would otherwise fail on. Additive
/// *fields* are backward compatible and don't require a bump.
pub const PROTOCOL_VERSION: u32 = 4;

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
    use crate::frame::{Blip, DetectionKind, EntityDebug, EntityFrame};
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
            sensors: Some(SensorView {
                range: 20.0,
                fov_half_angle: 1.2,
                vertical_fov_half_angle: 0.5,
            }),
            wheels: None,
        }
    }

    fn samples() -> Vec<ServerToViewer> {
        vec![
            ServerToViewer::SceneInit(SceneInit {
                protocol_version: PROTOCOL_VERSION,
                tick: 0,
                tick_rate: 64.0,
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
                        sensors: None,
                        wheels: None,
                    },
                    EntityDescriptor {
                        id: EntityId("road".into()),
                        name: "road".into(),
                        kind: EntityKind::Static,
                        shape: Shape::Mesh {
                            positions: vec![
                                Vec3::new(0.0, 0.0, -2.0),
                                Vec3::new(0.0, 0.0, 2.0),
                                Vec3::new(10.0, 0.0, 2.0),
                            ],
                            normals: vec![Vec3::new(0.0, 1.0, 0.0); 3],
                            indices: vec![0, 1, 2],
                        },
                        color: Color {
                            r: 0.2,
                            g: 0.2,
                            b: 0.22,
                        },
                        transform: Transform::IDENTITY,
                        sensors: None,
                        wheels: None,
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
                    wheels: vec![],
                }],
            }),
            ServerToViewer::DebugFrame(DebugFrame {
                tick: 42,
                entities: vec![EntityDebug {
                    id: EntityId("car-1".into()),
                    plan: vec![Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 10.0)],
                    reflex_active: true,
                    detections: vec![Blip {
                        id: EntityId("car-2".into()),
                        position: Vec3::new(3.0, 0.0, -1.0),
                        kind: DetectionKind::Agent,
                    }],
                    wheels: vec![],
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
    fn wheel_rig_and_poses_round_trip() {
        use crate::frame::{EntityFrame, Frame, WheelPose};
        use crate::scene::WheelRig;

        let rig = WheelRig {
            radius: 0.32,
            width: 0.22,
            rest: 0.20,
            offsets: [
                Vec3::new(0.8, -0.35, -1.3),
                Vec3::new(-0.8, -0.35, -1.3),
                Vec3::new(0.8, -0.35, 1.3),
                Vec3::new(-0.8, -0.35, 1.3),
            ],
        };
        let frame = ServerToViewer::Frame(Frame {
            tick: 9,
            entities: vec![EntityFrame {
                id: crate::scene::EntityId("car".into()),
                transform: Transform::IDENTITY,
                wheels: vec![
                    WheelPose {
                        steer: 0.12,
                        spin: 4.71,
                        travel: 0.058,
                        load: 3188.0,
                    };
                    4
                ],
            }],
        });
        assert_eq!(
            decode::<ServerToViewer>(&encode(&frame)).expect("decode"),
            frame
        );

        // The rig travels on the descriptor, once.
        let mut descriptor = crate::scene::EntityDescriptor {
            id: crate::scene::EntityId("car".into()),
            name: "car".into(),
            kind: crate::scene::EntityKind::Agent {
                embodiment: crate::scene::Embodiment::RaycastVehicle,
            },
            shape: crate::scene::Shape::Cuboid {
                half_extents: Vec3::new(0.8, 0.4, 1.4),
            },
            color: crate::scene::Color {
                r: 0.8,
                g: 0.2,
                b: 0.2,
            },
            transform: Transform::IDENTITY,
            sensors: None,
            wheels: Some(rig),
        };
        let event = ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor.clone()));
        assert_eq!(
            decode::<ServerToViewer>(&encode(&event)).expect("decode"),
            event
        );

        descriptor.wheels = None;
        let event = ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor));
        assert_eq!(
            decode::<ServerToViewer>(&encode(&event)).expect("decode"),
            event
        );
    }

    #[test]
    fn wheel_diagnostics_round_trip_on_the_debug_layer() {
        use crate::frame::{DebugFrame, EntityDebug, WheelDebug};

        let debug = ServerToViewer::DebugFrame(DebugFrame {
            tick: 3,
            entities: vec![EntityDebug {
                id: crate::scene::EntityId("car".into()),
                plan: vec![],
                reflex_active: true,
                detections: vec![],
                wheels: vec![
                    WheelDebug {
                        slip_ratio: -1.0,
                        slip_angle: 0.08,
                        contact: true,
                    },
                    WheelDebug {
                        slip_ratio: 0.0,
                        slip_angle: 0.0,
                        contact: false,
                    },
                ],
            }],
        });
        assert_eq!(
            decode::<ServerToViewer>(&encode(&debug)).expect("decode"),
            debug
        );
    }

    #[test]
    fn a_frame_without_wheels_still_decodes() {
        // The whole reason the wheel fields are additive and the protocol
        // version did not move: an encoding that predates them, or a sender
        // that has nothing with wheels in it, must still decode. `to_vec_named`
        // writes field-name maps, so a missing key takes its serde default.
        use crate::frame::{EntityFrame, Frame};

        #[derive(serde::Serialize)]
        struct OldEntityFrame {
            id: crate::scene::EntityId,
            transform: Transform,
        }
        #[derive(serde::Serialize)]
        struct OldFrame {
            tick: u64,
            entities: Vec<OldEntityFrame>,
        }

        let bytes = rmp_serde::to_vec_named(&OldFrame {
            tick: 1,
            entities: vec![OldEntityFrame {
                id: crate::scene::EntityId("puck".into()),
                transform: Transform::IDENTITY,
            }],
        })
        .expect("encode without the wheel field");
        let decoded: Frame = rmp_serde::from_slice(&bytes).expect("decode");
        assert_eq!(
            decoded.entities[0],
            EntityFrame {
                id: crate::scene::EntityId("puck".into()),
                transform: Transform::IDENTITY,
                wheels: vec![],
            }
        );
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
