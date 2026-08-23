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
    /// Behind, looking forward at it. Distinct from `Chase`: this is a level
    /// rear elevation, where `Chase` is the raised three-quarter view you
    /// drive from.
    Rear,
    /// Off its left flank, looking at its left side.
    Left,
    /// Off its right flank, looking at its right side. Either flank view is
    /// the one for watching a wheel lock or the suspension compress; which
    /// one you want depends on which way the corner goes.
    Right,
    /// Straight down.
    Top,
}

/// How far out the top view sits, against the caller's base distance. Looking
/// straight down, the camera sees only what fits in its frustum at that
/// height, so it needs more room than a view that looks across at its subject.
const TOP_DISTANCE: f32 = 2.0;

/// How high the four cardinal views sit, against the caller's base height.
/// They are *elevations* -- low and looking across at the subject, which is
/// where a wheel locking up or a spring compressing actually reads. `Chase`
/// keeps the full height because it is the view you drive from.
const ELEVATION_HEIGHT: f32 = 0.25;

impl CameraView {
    /// This view's distance and height, scaled from the caller's base pair.
    fn scaled(self, distance: f32, height: f32) -> (f32, f32) {
        match self {
            CameraView::Chase => (distance, height),
            CameraView::Top => (distance * TOP_DISTANCE, height),
            _ => (distance, height * ELEVATION_HEIGHT),
        }
    }

    /// What to call this view on screen.
    pub fn label(self) -> &'static str {
        match self {
            CameraView::Chase => "chase",
            CameraView::Front => "front",
            CameraView::Rear => "rear",
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
    let (distance, height) = view.scaled(distance, height);
    match view {
        CameraView::Chase => (Vec3::new(0.0, height, distance), Vec3::Y),
        CameraView::Front => (Vec3::new(0.0, height, -distance), Vec3::Y),
        CameraView::Rear => (Vec3::new(0.0, height, distance), Vec3::Y),
        // Named for the flank you are looking at, so the camera sits on that
        // side: the left view is off the subject's left, seeing its left side.
        CameraView::Left => (Vec3::new(-distance, height, 0.0), Vec3::Y),
        CameraView::Right => (Vec3::new(distance, height, 0.0), Vec3::Y),
        // Looking straight down, `+Y` is the view direction and useless as an
        // up vector, so the subject's own forward becomes up on screen.
        CameraView::Top => (Vec3::new(0.0, distance, 0.0), Vec3::NEG_Z),
    }
}

/// `1`-`5` select the front, rear, left, right and top views; pressing the one
/// already active returns to the default. They apply whether or not a
/// vehicle is followed — following just changes what the view is a view *of*.
pub fn view_input(keys: Res<ButtonInput<KeyCode>>, mut view: ResMut<CameraView>) {
    let pressed = [
        (KeyCode::Digit1, CameraView::Front),
        (KeyCode::Digit2, CameraView::Rear),
        (KeyCode::Digit3, CameraView::Left),
        (KeyCode::Digit4, CameraView::Right),
        (KeyCode::Digit5, CameraView::Top),
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
        // The flanks are abeam: the height they sit at is the elevation
        // claim, tested on its own below.
        let (left, _) = view_placement(CameraView::Left, D, H);
        assert_eq!(
            (left.x, left.z),
            (-D, 0.0),
            "the left view must sit to port"
        );
        let (right, _) = view_placement(CameraView::Right, D, H);
        assert_eq!(
            (right.x, right.z),
            (D, 0.0),
            "the right view must sit to starboard"
        );
        // Directly overhead. How far overhead is the stand-off claim, tested
        // on its own below.
        let (top, _) = view_placement(CameraView::Top, D, H);
        assert_eq!((top.x, top.z), (0.0, 0.0), "the top view must sit above");
        assert!(top.y > 0.0);
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
    fn front_and_rear_are_opposite_each_other() {
        let (front, _) = view_placement(CameraView::Front, D, H);
        let (rear, _) = view_placement(CameraView::Rear, D, H);
        assert_eq!(front.z, -rear.z, "both ends are on the same side");
        assert_eq!((front.x, front.y), (rear.x, rear.y));
        assert!(front.z < 0.0, "front must be the subject's -Z");
    }

    #[test]
    fn the_rear_view_is_not_just_the_chase_view_again() {
        // Chase already sits behind the subject, so a rear view sharing its
        // height would be a second key for the same picture. Rear is a level
        // elevation; chase is the raised view you drive from.
        let (rear, _) = view_placement(CameraView::Rear, D, H);
        let (chase, _) = view_placement(CameraView::Chase, D, H);
        assert_eq!(rear.z, chase.z, "both sit behind");
        assert!(
            rear.y < chase.y,
            "rear sits at {} and chase at {}: the same picture twice",
            rear.y,
            chase.y
        );
    }

    #[test]
    fn the_cardinal_views_look_across_at_the_subject_not_down_at_it() {
        // A flank view exists to show a wheel locking and a spring
        // compressing. From 6 m up at 12 m out you are looking at the roof.
        for view in [
            CameraView::Front,
            CameraView::Rear,
            CameraView::Left,
            CameraView::Right,
        ] {
            let (offset, _) = view_placement(view, D, H);
            let horizontal = offset.x.hypot(offset.z);
            assert!(
                offset.y < horizontal * 0.5,
                "{view:?} looks down at {:.0} degrees",
                offset.y.atan2(horizontal).to_degrees()
            );
        }
    }

    #[test]
    fn the_top_view_stands_off_further_than_the_rest() {
        // Straight down, the camera frames only what fits in its frustum at
        // that height, so the same base distance shows far less than it does
        // from across the subject.
        let (top, _) = view_placement(CameraView::Top, D, H);
        let (chase, _) = view_placement(CameraView::Chase, D, H);
        assert!(
            top.y > chase.length(),
            "top sits at {} but chase is already {} away",
            top.y,
            chase.length()
        );
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
