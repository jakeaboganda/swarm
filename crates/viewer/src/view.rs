//! Which way the camera looks at whatever it is framing.
//!
//! Orthogonal to *what* it frames: `follow::follow_camera` owns the camera
//! while a vehicle is followed and `scene::frame_camera` owns it otherwise, and
//! both place it with the same offset — one in the vehicle's frame, one in the
//! world's.

use bevy::prelude::*;

/// The camera's angle on its subject. A viewer-local choice; nothing about it
/// is streamed.
#[derive(Resource, Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum CameraView {
    /// Behind and above: the chase view when following, the three-quarter
    /// overview when not.
    #[default]
    Chase,
    /// Ahead, looking back at it.
    Front,
    /// Off its left flank, looking at its left side.
    Left,
    /// Off its right flank, looking at its right side. Either flank view is
    /// the one for watching a wheel lock or the suspension compress; which
    /// one you want depends on which way the corner goes.
    Right,
    /// Straight down.
    Top,
}

impl CameraView {
    /// Whether this view looks straight down. It needs its own framing
    /// distance: from directly above, the camera sees only what fits in the
    /// frustum at that height.
    pub fn is_top(self) -> bool {
        matches!(self, CameraView::Top)
    }

    /// What to call this view on screen.
    pub fn label(self) -> &'static str {
        match self {
            CameraView::Chase => "chase",
            CameraView::Front => "front",
            CameraView::Left => "left",
            CameraView::Right => "right",
            CameraView::Top => "top",
        }
    }
}

/// Where the camera sits relative to what it frames, and which way is up on
/// screen — both in the subject's own frame (forward `-Z`, right `+X`, up
/// `+Y`), so the caller rotates them by whatever basis it is framing in.
pub fn view_placement(view: CameraView, distance: f32, height: f32) -> (Vec3, Vec3) {
    match view {
        CameraView::Chase => (Vec3::new(0.0, height, distance), Vec3::Y),
        CameraView::Front => (Vec3::new(0.0, height, -distance), Vec3::Y),
        // Named for the flank you are looking at, so the camera sits on that
        // side: the left view is off the subject's left, seeing its left side.
        CameraView::Left => (Vec3::new(-distance, height, 0.0), Vec3::Y),
        CameraView::Right => (Vec3::new(distance, height, 0.0), Vec3::Y),
        // Looking straight down, `+Y` is the view direction and useless as an
        // up vector, so the subject's own forward becomes up on screen.
        CameraView::Top => (Vec3::new(0.0, distance, 0.0), Vec3::NEG_Z),
    }
}

/// `1`/`2`/`3`/`4` select the front, left, right and top views; pressing the
/// one already active returns to the default. They apply whether or not a
/// vehicle is followed — following just changes what the view is a view *of*.
pub fn view_input(keys: Res<ButtonInput<KeyCode>>, mut view: ResMut<CameraView>) {
    let pressed = [
        (KeyCode::Digit1, CameraView::Front),
        (KeyCode::Digit2, CameraView::Left),
        (KeyCode::Digit3, CameraView::Right),
        (KeyCode::Digit4, CameraView::Top),
    ]
    .into_iter()
    .find(|(key, _)| keys.just_pressed(*key));

    if let Some((_, chosen)) = pressed {
        let next = if *view == chosen {
            CameraView::default()
        } else {
            chosen
        };
        // Assigning through `ResMut` always marks the resource changed, and
        // the camera systems reframe on that — so only write a real change.
        if *view != next {
            *view = next;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: f32 = 12.0;
    const H: f32 = 6.0;

    #[test]
    fn each_view_looks_from_the_side_of_the_subject_it_names() {
        // Bevy's forward is -Z, so "behind" is +Z and "ahead" is -Z.
        let (chase, _) = view_placement(CameraView::Chase, D, H);
        assert_eq!(chase, Vec3::new(0.0, H, D), "chase must sit behind");
        let (front, _) = view_placement(CameraView::Front, D, H);
        assert_eq!(front.z, -D, "the front view must sit ahead of the subject");
        assert_eq!(front.x, 0.0);
        let (left, _) = view_placement(CameraView::Left, D, H);
        assert_eq!(
            left,
            Vec3::new(-D, H, 0.0),
            "the left view must sit to port"
        );
        let (right, _) = view_placement(CameraView::Right, D, H);
        assert_eq!(
            right,
            Vec3::new(D, H, 0.0),
            "the right view must sit to starboard"
        );
        let (top, _) = view_placement(CameraView::Top, D, H);
        assert_eq!(top, Vec3::new(0.0, D, 0.0), "the top view must sit above");
    }

    #[test]
    fn the_two_flank_views_are_opposite_each_other() {
        // Not just "both abeam": a left view that sat on the right would still
        // frame the car, and would silently be the same view twice.
        let (left, _) = view_placement(CameraView::Left, D, H);
        let (right, _) = view_placement(CameraView::Right, D, H);
        assert_eq!(left.x, -right.x, "the flanks are on the same side");
        assert_eq!((left.y, left.z), (right.y, right.z));
        assert!(left.x < 0.0, "left must be the subject's -X");
    }

    #[test]
    fn no_view_puts_the_camera_inside_its_subject() {
        for view in [
            CameraView::Chase,
            CameraView::Front,
            CameraView::Left,
            CameraView::Right,
            CameraView::Top,
        ] {
            let (offset, _) = view_placement(view, D, H);
            assert!(offset.length() >= D, "{view:?} is too close: {offset:?}");
        }
    }

    #[test]
    fn every_up_vector_is_usable_from_where_the_camera_sits() {
        // `look_at` needs an up that is not parallel to the view direction --
        // straight down with up = +Y is the degenerate case, and the one a
        // top view walks straight into.
        for view in [
            CameraView::Chase,
            CameraView::Front,
            CameraView::Left,
            CameraView::Right,
            CameraView::Top,
        ] {
            let (offset, up) = view_placement(view, D, H);
            let direction = (-offset).normalize();
            assert!(
                direction.cross(up).length() > 0.1,
                "{view:?} looks along {direction:?} with up {up:?}"
            );
        }
    }
}
