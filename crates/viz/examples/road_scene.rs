//! Puts a baked road mesh on the viz wire. Builds a small road-surface strip,
//! wraps it in a scene-init as a `Shape::Mesh` entity (the same way the server
//! sends a road to viewers), round-trips it through MessagePack, and prints a
//! summary.
//!
//! Run: `cargo run -p viz --example road_scene`

use viz::{
    decode, encode, ArenaBounds, Color, EntityDescriptor, EntityId, EntityKind, ScenarioState,
    SceneInit, ServerToViewer, Shape, Transform, Vec3, PROTOCOL_VERSION,
};

fn main() {
    // A short two-quad road strip climbing along +X, Y-up.
    let positions = vec![
        Vec3::new(0.0, 0.0, -2.0),
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(6.0, 0.5, -2.0),
        Vec3::new(6.0, 0.5, 2.0),
        Vec3::new(12.0, 1.0, -2.0),
        Vec3::new(12.0, 1.0, 2.0),
    ];
    let normals = vec![Vec3::new(0.0, 1.0, 0.0); positions.len()];
    // Two triangles per quad, wound to face up.
    let indices = vec![0, 1, 3, 0, 3, 2, 2, 3, 5, 2, 5, 4];

    let road = EntityDescriptor {
        id: EntityId("road".into()),
        name: "road".into(),
        kind: EntityKind::Static,
        shape: Shape::Mesh {
            positions,
            normals,
            indices,
        },
        color: Color {
            r: 0.2,
            g: 0.2,
            b: 0.22,
        },
        transform: Transform::IDENTITY,
        sensors: None,
        wheels: None,
    };

    let scene = ServerToViewer::SceneInit(SceneInit {
        protocol_version: PROTOCOL_VERSION,
        tick: 0,
        tick_rate: 64.0,
        state: ScenarioState::WaitingForRoster,
        arena: ArenaBounds {
            width: 50.0,
            depth: 50.0,
        },
        entities: vec![road],
    });

    let bytes = encode(&scene);
    let back: ServerToViewer = decode(&bytes).expect("decode");
    assert_eq!(
        scene, back,
        "the mesh scene must survive the wire round-trip"
    );

    if let ServerToViewer::SceneInit(init) = &back {
        for entity in &init.entities {
            if let Shape::Mesh {
                positions, indices, ..
            } = &entity.shape
            {
                println!(
                    "road mesh '{}': {} vertices, {} triangles, {} MessagePack bytes",
                    entity.name,
                    positions.len(),
                    indices.len() / 3,
                    bytes.len(),
                );
            }
        }
    }
}
