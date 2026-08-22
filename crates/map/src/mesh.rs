use glam::Vec3;

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
    /// (bisector) normal, carrying the centerline's elevation. Winding is
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
                let left = left_normal(along);
                // Surface up-normal: along × left is +Y for a flat road, and
                // tilts with grade/bank.
                let up = along.cross(left).normalize_or_zero();
                mesh.vertices.push(points[i] + left * half);
                mesh.normals.push(up);
                mesh.vertices.push(points[i] - left * half);
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
