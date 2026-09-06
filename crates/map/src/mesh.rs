use glam::{Quat, Vec3};

use crate::geometry::left_normal;
use crate::network::RoadNetwork;

/// A triangle mesh (Y-up, meters): per-vertex positions and up-normals, plus
/// triangle indices. The road surface, shared by the physics collider (which
/// needs only positions) and the viewer (which needs normals for lighting).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub indices: Vec<u32>,
}

/// Why a mesh cannot be turned into a physics trimesh.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MeshError {
    #[error("mesh has no triangles")]
    Empty,
    #[error("vertex {0} is not finite")]
    NonFiniteVertex(usize),
    #[error("triangle {0} indexes vertex {1}, past the {2} vertices present")]
    IndexOutOfRange(usize, u32, usize),
    #[error("triangle {0} is degenerate (it repeats a vertex)")]
    DegenerateTriangle(usize),
}

impl Mesh {
    /// Whether this mesh can back a physics collider.
    ///
    /// The collider builder answers the same question, but only by failing at
    /// spawn time -- deep inside a Bevy startup system, where there is nothing
    /// to do but panic. Checking here lets an untrusted map be rejected at
    /// *load*, with the file named, which is what a bad `.xodr` deserves.
    pub fn validate(&self) -> Result<(), MeshError> {
        if self.indices.is_empty() {
            return Err(MeshError::Empty);
        }
        if let Some(i) = self.vertices.iter().position(|v| !v.is_finite()) {
            return Err(MeshError::NonFiniteVertex(i));
        }
        for (t, triangle) in self.indices.chunks_exact(3).enumerate() {
            for &index in triangle {
                if index as usize >= self.vertices.len() {
                    return Err(MeshError::IndexOutOfRange(t, index, self.vertices.len()));
                }
            }
            if triangle[0] == triangle[1]
                || triangle[1] == triangle[2]
                || triangle[0] == triangle[2]
            {
                return Err(MeshError::DegenerateTriangle(t));
            }
        }
        Ok(())
    }
}

impl RoadNetwork {
    /// Tessellate every driving lane into one surface mesh-- a quad strip per
    /// lane, each rib offset +/-width/2 from the centerline along the per-vertex
    /// cross axis, carrying the centerline's elevation. On a superelevated lane
    /// the cross axis is tilted about the tangent by the local `bank`, so the
    /// outer edge of a banked curve rides above the inner one. Winding is
    /// consistent (triangles face up).
    ///
    /// Note: ribs use the vertex bisector normal, which keeps width consistent
    /// across vertices but does not guard against self-intersection on curves
    /// tighter than the half-width. An importer contract to enforce at bake.
    pub fn surface_mesh(&self) -> Mesh {
        let mut mesh = Mesh::default();
        for lane in self.driving_lanes() {
            let points = lane.center.points();
            let tangents = lane.center.tangents();
            let half = lane.width * 0.5;
            let base = mesh.vertices.len() as u32;
            for i in 0..points.len() {
                let along = tangents[i];
                // Cross axis, rolled about the (stored) tangent by the local
                // bank: +bank raises the left rib. Flat lanes (empty bank) leave
                // it the horizontal left normal, so the mesh is unchanged.
                let bank = lane.bank.get(i).copied().unwrap_or(0.0);
                let lateral = Quat::from_axis_angle(along, bank) * left_normal(along);
                // Surface up-normal: along × lateral is +Y for a flat road, and
                // tilts with grade/bank.
                let up = along.cross(lateral).normalize_or(Vec3::Y);
                mesh.vertices.push(points[i] + lateral * half);
                mesh.normals.push(up);
                mesh.vertices.push(points[i] - lateral * half);
                mesh.normals.push(up);
            }
            // Two triangles per segment, over the [left, right] rib pairs.
            for i in 0..points.len() as u32 - 1 {
                let l0 = base + i * 2;
                let (r0, l1, r1) = (l0 + 1, l0 + 2, l0 + 3);
                mesh.indices.extend_from_slice(&[l0, r0, r1, l0, r1, l1]);
            }
        }
        mesh
    }
}

#[cfg(test)]
mod tests {
    use crate::{demo_road, Direction, Lane, LaneId, LaneKind, Mesh, Polyline, RoadNetwork};
    use glam::Vec3;

    #[test]
    fn one_straight_lane_tessellates_to_two_quads() {
        let net = RoadNetwork {
            lanes: vec![Lane {
                id: LaneId(0),
                kind: LaneKind::Driving,
                direction: Direction::Forward,
                center: Polyline::new(vec![
                    Vec3::new(0.0, 0.0, 0.0),
                    Vec3::new(5.0, 0.0, 0.0),
                    Vec3::new(10.0, 0.0, 0.0),
                ]),
                width: 4.0,
                bank: Vec::new(),
                successors: Vec::new(),
                predecessors: Vec::new(),
                neighbors: Vec::new(),
            }],
        };
        let mesh = net.surface_mesh();
        // 3 ribs × 2 vertices; 2 segments × 2 triangles × 3 indices.
        assert_eq!(mesh.vertices.len(), 6);
        assert_eq!(mesh.normals.len(), 6);
        assert_eq!(mesh.indices.len(), 12);
        // The first rib straddles the centerline by +/-half-width in Z.
        assert!((mesh.vertices[0].z - mesh.vertices[1].z).abs() > 3.9);
        // A flat road faces straight up.
        assert!(mesh.normals[0].abs_diff_eq(Vec3::Y, 1e-5));
    }

    #[test]
    fn every_shipped_map_tessellates_into_a_valid_trimesh() {
        // The built-in road, here; the imported ones are checked in
        // `map-opendrive`, which is the crate that can load them. This is the
        // cheapest guard against the startup panic: the server builds one
        // static trimesh collider from exactly this mesh.
        demo_road()
            .surface_mesh()
            .validate()
            .expect("the built-in demo road tessellates");
    }

    #[test]
    fn validate_rejects_what_a_collider_cannot_build() {
        use crate::MeshError;
        assert_eq!(Mesh::default().validate(), Err(MeshError::Empty));

        let sound = Mesh {
            vertices: vec![Vec3::ZERO, Vec3::X, Vec3::Z],
            normals: vec![Vec3::Y; 3],
            indices: vec![0, 1, 2],
        };
        assert_eq!(sound.validate(), Ok(()));

        let mut nan = sound.clone();
        nan.vertices[1].x = f32::NAN;
        assert_eq!(nan.validate(), Err(MeshError::NonFiniteVertex(1)));

        let mut past_end = sound.clone();
        past_end.indices = vec![0, 1, 7];
        assert_eq!(
            past_end.validate(),
            Err(MeshError::IndexOutOfRange(0, 7, 3))
        );

        let mut degenerate = sound.clone();
        degenerate.indices = vec![0, 1, 1];
        assert_eq!(degenerate.validate(), Err(MeshError::DegenerateTriangle(0)));
    }

    #[test]
    fn a_banked_lane_tilts_its_ribs() {
        // A straight lane heading +X, canted a constant 0.2 rad: +bank raises
        // the left rib, so the left (first) vertex of each pair rides above the
        // right, and the up-normal leans off vertical.
        let net = RoadNetwork {
            lanes: vec![Lane {
                bank: vec![0.2; 3],
                ..Lane {
                    id: LaneId(0),
                    kind: LaneKind::Driving,
                    direction: Direction::Forward,
                    center: Polyline::new(vec![
                        Vec3::new(0.0, 0.0, 0.0),
                        Vec3::new(5.0, 0.0, 0.0),
                        Vec3::new(10.0, 0.0, 0.0),
                    ]),
                    width: 4.0,
                    bank: Vec::new(),
                    successors: Vec::new(),
                    predecessors: Vec::new(),
                    neighbors: Vec::new(),
                }
            }],
        };
        let mesh = net.surface_mesh();
        mesh.validate().expect("a banked lane is a valid trimesh");
        // Left (index 0) above right (index 1) of the first rib pair.
        assert!(
            mesh.vertices[0].y - mesh.vertices[1].y > 0.5,
            "outer/left {} should ride above inner/right {}",
            mesh.vertices[0].y,
            mesh.vertices[1].y
        );
        // The up-normal is tilted but still points up.
        assert!(mesh.normals[0].y < 0.99 && mesh.normals[0].y > 0.9);
        assert!(
            (mesh.normals[0] - Vec3::Y).length() > 0.05,
            "normal should tilt"
        );
    }

    // Build a one-lane network from centerline points and a per-vertex bank.
    fn banked_net(points: Vec<Vec3>, bank: Vec<f32>, width: f32) -> RoadNetwork {
        RoadNetwork {
            lanes: vec![Lane {
                id: LaneId(0),
                kind: LaneKind::Driving,
                direction: Direction::Forward,
                center: Polyline::new(points),
                width,
                bank,
                successors: Vec::new(),
                predecessors: Vec::new(),
                neighbors: Vec::new(),
            }],
        }
    }

    // A CURVED banked lane: a right-hand arc (curving toward +Z), so the left
    // edge is the OUTER edge. With a constant positive bank the outer edge must
    // ride above the inner one all the way round, every normal still points up,
    // and the mesh is a valid trimesh.
    #[test]
    fn a_banked_curve_lifts_the_outer_edge_and_stays_valid() {
        let r = 20.0_f32;
        // Circle centered at (0,0,r): point = (r sinθ, 0, r - r cosθ). θ=0 -> +X
        // heading, z grows -> a right turn, so left_normal (-Z at start) is outer.
        let points: Vec<Vec3> = (0..=8)
            .map(|k| {
                let th = (k as f32) * (std::f32::consts::FRAC_PI_4 / 8.0);
                Vec3::new(r * th.sin(), 0.0, r - r * th.cos())
            })
            .collect();
        let net = banked_net(points.clone(), vec![0.15; points.len()], 5.0);
        let mesh = net.surface_mesh();
        mesh.validate().expect("a banked curve tessellates");

        // Every normal still points generally up.
        assert!(
            mesh.normals.iter().all(|n| n.y > 0.9),
            "some normal fell below 0.9: {:?}",
            mesh.normals.iter().map(|n| n.y).fold(1.0_f32, f32::min)
        );
        // Outer (left, even index) rib rides above the inner (right, odd) rib at
        // every rib pair -- the physically-correct banked-curve profile.
        for i in 0..points.len() {
            let outer = mesh.vertices[2 * i].y;
            let inner = mesh.vertices[2 * i + 1].y;
            assert!(
                outer - inner > 0.5,
                "rib {i}: outer {outer} should ride above inner {inner}"
            );
        }
        // The normals are genuinely tilted, not vertical.
        assert!(mesh.normals.iter().any(|n| (n.y - 1.0).abs() > 0.005));
    }

    // Negative bank rolls the surface the other way: the RIGHT rib rides above
    // the left, and the normal leans toward -Z -- the mirror of positive bank.
    #[test]
    fn negative_bank_raises_the_opposite_rib() {
        let pts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
        ];
        let neg = banked_net(pts.clone(), vec![-0.2; 3], 4.0).surface_mesh();
        let pos = banked_net(pts, vec![0.2; 3], 4.0).surface_mesh();
        neg.validate().expect("valid trimesh");
        // Right (index 1) above left (index 0) for negative bank.
        assert!(
            neg.vertices[1].y - neg.vertices[0].y > 0.5,
            "negative bank should raise the right rib: left {} right {}",
            neg.vertices[0].y,
            neg.vertices[1].y
        );
        // Exact mirror of the positive-bank mesh.
        assert!((neg.vertices[0].y + pos.vertices[0].y).abs() < 1e-5);
        assert!((neg.vertices[1].y + pos.vertices[1].y).abs() < 1e-5);
        // Normal leans toward -Z (positive bank leans +Z).
        assert!(neg.normals[0].z < -0.05, "neg normal {:?}", neg.normals[0]);
        assert!(pos.normals[0].z > 0.05, "pos normal {:?}", pos.normals[0]);
        assert!(neg.normals.iter().all(|n| n.y > 0.9));
    }

    // The two independent consumers of `bank` must agree: at each centerline
    // vertex the mesh's per-vertex up-normal equals the up-normal
    // `Lane::sample_at` derives at that vertex's arc length. Both roll +Y about
    // the same stored tangent by the same bank, so they cannot diverge.
    #[test]
    fn mesh_normal_matches_sample_at_up_at_each_vertex() {
        let pts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(8.0, 0.0, 0.0),
            Vec3::new(16.0, 0.0, 6.0),
            Vec3::new(20.0, 0.0, 16.0),
        ];
        let bank = vec![0.05, 0.12, 0.18, 0.1];
        let net = banked_net(pts.clone(), bank, 5.0);
        let mesh = net.surface_mesh();
        let lane = &net.lanes[0];
        // Cumulative arc length at each vertex.
        let mut s = 0.0_f32;
        for i in 0..pts.len() {
            if i > 0 {
                s += (pts[i] - pts[i - 1]).length();
            }
            let mesh_up = mesh.normals[2 * i]; // left and right share the vertex normal
            let sampled_up = lane.sample_at(s).up;
            assert!(
                mesh_up.abs_diff_eq(sampled_up, 1e-5),
                "vertex {i} (s={s}): mesh normal {mesh_up:?} != sample_at up {sampled_up:?}"
            );
        }
    }

    #[test]
    fn demo_road_mesh_is_non_degenerate_and_faces_up() {
        let mesh = demo_road().surface_mesh();
        assert!(!mesh.vertices.is_empty());
        assert_eq!(mesh.normals.len(), mesh.vertices.len());
        assert_eq!(mesh.indices.len() % 3, 0);
        assert!(mesh
            .indices
            .iter()
            .all(|&i| (i as usize) < mesh.vertices.len()));
        // The graded road tilts slightly but every normal still points upward.
        assert!(mesh.normals.iter().all(|n| n.y > 0.9));
    }
}
