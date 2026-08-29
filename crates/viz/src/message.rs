use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::frame::{DebugFrame, Frame};
use crate::scene::{SceneEvent, SceneInit};

/// Version of the viz wire schema. A viewer declares it in `Hello`; the sim
/// declares it in `SceneInit`, and drops a viewer that declared anything else.
/// Bump it for any change to the shape of these types: nothing outside this
/// repo consumes this wire, so the schema is broken cleanly rather than kept
/// compatible, and the failure mode is a viewer that refuses to connect.
pub const PROTOCOL_VERSION: u32 = 6;

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
    use crate::frame::{
        Blip, DetectionKind, EntityDebug, EntityFrame, NodeUpdate, WheelDebug, WHEEL_NODES,
    };
    use crate::math::{Quat, Transform, Vec3};
    use crate::scene::*;

    fn at(x: f32, y: f32, z: f32) -> Transform {
        Transform {
            position: Vec3::new(x, y, z),
            rotation: Quat::IDENTITY,
        }
    }

    /// A car: a cuboid body with four wheel children, the tree the sim builds
    /// for a `RaycastVehicle`.
    fn car_tree() -> EntityNode {
        let wheels = WHEEL_NODES
            .iter()
            .enumerate()
            .map(|(index, name)| {
                EntityNode::new(
                    *name,
                    at(if index % 2 == 0 { 0.8 } else { -0.8 }, -0.55, -1.3),
                    Geometry::Cylinder {
                        radius: 0.32,
                        height: 0.22,
                    },
                )
            })
            .collect();
        EntityNode::body(Geometry::Cuboid {
            half_extents: Vec3::new(0.8, 0.4, 1.4),
        })
        .with_children(wheels)
    }

    fn sample_descriptor() -> EntityDescriptor {
        EntityDescriptor {
            id: EntityId("car-1".into()),
            name: "car-1".into(),
            kind: EntityKind::Agent {
                embodiment: Embodiment::CarLike,
            },
            color: Color {
                r: 0.9,
                g: 0.4,
                b: 0.1,
            },
            root: EntityNode::body(Geometry::Capsule {
                radius: 0.5,
                half_length: 0.5,
            }),
            sensors: Some(SensorView {
                range: 20.0,
                fov_half_angle: 1.2,
                vertical_fov_half_angle: 0.5,
            }),
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
                        color: Color {
                            r: 0.45,
                            g: 0.47,
                            b: 0.52,
                        },
                        root: EntityNode::body(Geometry::Cuboid {
                            half_extents: Vec3::new(25.0, 1.5, 0.25),
                        }),
                        sensors: None,
                    },
                    EntityDescriptor {
                        id: EntityId("road".into()),
                        name: "road".into(),
                        kind: EntityKind::Static,
                        color: Color {
                            r: 0.2,
                            g: 0.2,
                            b: 0.22,
                        },
                        root: EntityNode::body(Geometry::Mesh {
                            positions: vec![
                                Vec3::new(0.0, 0.0, -2.0),
                                Vec3::new(0.0, 0.0, 2.0),
                                Vec3::new(10.0, 0.0, 2.0),
                            ],
                            normals: vec![Vec3::new(0.0, 1.0, 0.0); 3],
                            indices: vec![0, 1, 2],
                        }),
                        sensors: None,
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
                    nodes: vec![NodeUpdate {
                        path: NodePath::root(),
                        transform: Transform {
                            position: Vec3::new(1.0, 1.0, 2.0),
                            rotation: Quat::new(0.0, 0.6, 0.0, 0.8),
                        },
                    }],
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
    fn a_nested_tree_round_trips() {
        // The tree is recursive, and every geometry variant has to survive the
        // wire -- including `Asset`, which is a reference a viewer resolves
        // (or falls back from), never bytes.
        let mut root = car_tree();
        root.children.push(
            EntityNode {
                name: "mount".into(),
                transform: at(0.0, 0.5, -1.0),
                // A pure pivot: no geometry of its own.
                geometry: None,
                children: Vec::new(),
            }
            .with_children(vec![
                EntityNode::new(
                    "lidar",
                    at(0.0, 0.1, 0.0),
                    Geometry::Asset {
                        uri: "models/lidar.glb".into(),
                        scale: Vec3::new(1.0, 1.0, 1.0),
                    },
                ),
                EntityNode::new("dome", at(0.0, 0.2, 0.0), Geometry::Sphere { radius: 0.1 }),
            ]),
        );

        let mut descriptor = sample_descriptor();
        descriptor.root = root;
        let event = ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor.clone()));
        let back: ServerToViewer = decode(&encode(&event)).expect("decode");
        assert_eq!(event, back);

        // And the decoded tree is still addressable at depth.
        let ServerToViewer::Event(SceneEvent::EntitySpawned(decoded)) = back else {
            panic!("wrong variant");
        };
        assert_eq!(
            decoded.root.get(&NodePath::from("mount/lidar")).unwrap(),
            descriptor.root.get(&NodePath::from("mount/lidar")).unwrap()
        );
        assert_eq!(decoded.root.children.len(), 5);
    }

    #[test]
    fn a_frame_carries_only_the_nodes_that_moved() {
        // The point of the sparse frame: a puck sends one update, a car sends
        // five, and a wall sends no entry at all.
        let frame = ServerToViewer::Frame(Frame {
            tick: 9,
            entities: vec![
                EntityFrame {
                    id: EntityId("puck".into()),
                    nodes: vec![NodeUpdate {
                        path: NodePath::root(),
                        transform: at(1.0, 0.5, 2.0),
                    }],
                },
                EntityFrame {
                    id: EntityId("car".into()),
                    nodes: std::iter::once(NodeUpdate {
                        path: NodePath::root(),
                        transform: at(4.0, 0.5, 0.0),
                    })
                    .chain(WHEEL_NODES.iter().map(|name| NodeUpdate {
                        path: NodePath::root().child(name),
                        transform: Transform {
                            position: Vec3::new(0.8, -0.5, -1.3),
                            // A quarter turn about Z: the axle laid across the
                            // body, as the sim sends it.
                            rotation: Quat::new(
                                0.0,
                                0.0,
                                std::f32::consts::FRAC_1_SQRT_2,
                                std::f32::consts::FRAC_1_SQRT_2,
                            ),
                        },
                    }))
                    .collect(),
                },
            ],
        });
        let back: ServerToViewer = decode(&encode(&frame)).expect("decode");
        assert_eq!(frame, back);
        let ServerToViewer::Frame(decoded) = back else {
            panic!("wrong variant");
        };
        assert_eq!(decoded.entities[0].nodes.len(), 1);
        assert_eq!(decoded.entities[1].nodes.len(), 5);
        assert!(decoded.entities[1].nodes[0].path.is_root());
        assert_eq!(
            decoded.entities[1].nodes[1].path,
            NodePath::from("wheel.fl")
        );
    }

    #[test]
    fn wheel_diagnostics_round_trip_on_the_debug_layer() {
        let debug = ServerToViewer::DebugFrame(DebugFrame {
            tick: 3,
            entities: vec![EntityDebug {
                id: EntityId("car".into()),
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
    fn fmu_vehicle_embodiment_round_trips() {
        // FmuVehicle is drawn as a car, same tree shape as any other vehicle
        // embodiment; only the `embodiment` tag differs.
        let mut descriptor = sample_descriptor();
        descriptor.kind = EntityKind::Agent {
            embodiment: Embodiment::FmuVehicle,
        };
        let event = ServerToViewer::Event(SceneEvent::EntitySpawned(descriptor.clone()));
        let back: ServerToViewer = decode(&encode(&event)).expect("decode");
        assert_eq!(event, back);
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
