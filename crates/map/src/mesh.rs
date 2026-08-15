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
    use crate::{demo_road, Direction, Lane, LaneId, LaneKind, Polyline, RoadNetwork};
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
