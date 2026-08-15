use glam::Vec3;

/// A position plus a horizontal heading along a lane. Y is up (elevation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: Vec3,
    /// Unit tangent in the XZ (ground) plane — the direction of travel.
    pub heading: Vec3,
}

/// The result of projecting a world point onto a polyline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projection {
    /// Arc length of the nearest point along the polyline.
    pub s: f32,
    /// The nearest point on the polyline itself.
    pub point: Vec3,
    /// Signed lateral distance from the polyline, in the ground plane.
    /// Positive is to the **left** of travel; negative is to the right.
    pub offset: f32,
}

/// A polyline in 3D (Y-up, meters), queried by arc length. This is the baked
/// form every curve reduces to: an importer samples clothoids/arcs into points;
/// consumers only ever see the points. At least two points.
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline {
    points: Vec<Vec3>,
    /// Cumulative arc length at each point; `cumulative[0] == 0`.
    cumulative: Vec<f32>,
}

impl Polyline {
    /// Panics if given fewer than two points — a degenerate polyline is a
    /// map-construction bug, not a recoverable runtime condition.
    pub fn new(points: Vec<Vec3>) -> Self {
        assert!(points.len() >= 2, "a polyline needs at least two points");
        let mut cumulative = Vec::with_capacity(points.len());
        let mut acc = 0.0;
        cumulative.push(0.0);
        for pair in points.windows(2) {
            acc += (pair[1] - pair[0]).length();
            cumulative.push(acc);
        }
        Self { points, cumulative }
    }

    pub fn points(&self) -> &[Vec3] {
        &self.points
    }

    pub fn length(&self) -> f32 {
        self.cumulative.last().copied().unwrap_or(0.0)
    }

    /// The segment index containing arc length `s` (clamped to a valid segment).
    fn segment(&self, s: f32) -> usize {
        let last = self.points.len() - 2;
        self.cumulative
            .partition_point(|&c| c <= s)
            .saturating_sub(1)
            .min(last)
    }

    /// Position at arc length `s` (clamped to `[0, length]`).
    pub fn point_at(&self, s: f32) -> Vec3 {
        let s = s.clamp(0.0, self.length());
        let i = self.segment(s);
        let seg_len = self.cumulative[i + 1] - self.cumulative[i];
        let t = if seg_len > 1e-6 {
            (s - self.cumulative[i]) / seg_len
        } else {
            0.0
        };
        self.points[i].lerp(self.points[i + 1], t)
    }

    /// Position and horizontal heading at arc length `s`.
    pub fn pose_at(&self, s: f32) -> Pose {
        let s = s.clamp(0.0, self.length());
        let i = self.segment(s);
        Pose {
            position: self.point_at(s),
            heading: horizontal(self.points[i + 1] - self.points[i]).normalize_or_zero(),
        }
    }

    /// Nearest point on the polyline to `point`, with its arc length and signed
    /// lateral offset. Nearest is by full 3D distance; the offset is measured in
    /// the ground plane.
    pub fn project(&self, point: Vec3) -> Projection {
        let mut best = Projection {
            s: 0.0,
            point: self.points[0],
            offset: 0.0,
        };
        let mut best_dist = f32::INFINITY;
        for i in 0..self.points.len() - 1 {
            let a = self.points[i];
            let ab = self.points[i + 1] - a;
            let len2 = ab.length_squared();
            let t = if len2 > 1e-9 {
                ((point - a).dot(ab) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let closest = a + ab * t;
            let dist = (point - closest).length_squared();
            if dist < best_dist {
                best_dist = dist;
                let heading = horizontal(ab).normalize_or_zero();
                best = Projection {
                    s: self.cumulative[i] + ab.length() * t,
                    point: closest,
                    offset: horizontal(point - closest).dot(left_normal(heading)),
                };
            }
        }
        best
    }
}

/// Drop a vector onto the XZ ground plane.
fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z)
}

/// Unit left-hand normal of a horizontal `heading` (about +Y up). For heading
/// +X this is −Z. Zero if the heading has no horizontal extent.
pub(crate) fn left_normal(heading: Vec3) -> Vec3 {
    Vec3::Y.cross(horizontal(heading)).normalize_or_zero()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(points: &[[f32; 3]]) -> Polyline {
        Polyline::new(points.iter().map(|p| Vec3::from_array(*p)).collect())
    }

    #[test]
    fn length_sums_the_segments() {
        let l = line(&[[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [3.0, 0.0, 4.0]]);
        assert!((l.length() - 7.0).abs() < 1e-5);
    }

    #[test]
    fn point_at_interpolates_and_clamps() {
        let l = line(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]]);
        assert!((l.point_at(5.0) - Vec3::new(5.0, 0.0, 0.0)).length() < 1e-5);
        // Past the end clamps to the last point.
        assert!((l.point_at(100.0) - Vec3::new(10.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn pose_heading_follows_the_segment() {
        let l = line(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 0.0, 10.0]]);
        assert!(l.pose_at(1.0).heading.abs_diff_eq(Vec3::X, 1e-5));
        assert!(l.pose_at(15.0).heading.abs_diff_eq(Vec3::Z, 1e-5));
    }

    #[test]
    fn project_gives_arc_length_and_signed_offset() {
        let l = line(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]]);
        // A point to the left of +X travel (toward −Z) is a positive offset.
        let left = l.project(Vec3::new(5.0, 0.0, -3.0));
        assert!((left.s - 5.0).abs() < 1e-4);
        assert!((left.offset - 3.0).abs() < 1e-4, "offset {}", left.offset);
        // A point to the right (+Z) is negative.
        let right = l.project(Vec3::new(5.0, 0.0, 3.0));
        assert!((right.offset + 3.0).abs() < 1e-4, "offset {}", right.offset);
    }

    #[test]
    fn left_normal_of_plus_x_is_minus_z() {
        assert!(left_normal(Vec3::X).abs_diff_eq(Vec3::new(0.0, 0.0, -1.0), 1e-5));
    }
}
