//! The purpose-built wheel/tire test track imports as the road it describes.
//!
//! `testtrack.xodr` exists so vehicle behaviour is exercised deliberately
//! rather than by whatever a city map happens to contain. That only works if
//! the sections it documents actually survive import -- a straight really
//! straight, a tight corner really tight, a crest that really rises and falls.
//! Otherwise a tuning session is reading a road that isn't there.

use map_opendrive::load_file;

const TRACK: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../maps/testtrack.xodr");

/// Curvature at arc length `s`, from the change in heading over a short span.
fn curvature(lane: &map::Lane, s: f32) -> f32 {
    const SPAN: f32 = 4.0;
    let (a, b) = (
        lane.center.pose_at(s - SPAN * 0.5).heading,
        lane.center.pose_at(s + SPAN * 0.5).heading,
    );
    // Signed angle between the two headings, in the ground plane.
    let cross = a.z * b.x - a.x * b.z;
    let dot = a.x * b.x + a.z * b.z;
    (cross.atan2(dot) / SPAN).abs()
}

#[test]
fn the_test_track_has_the_sections_it_claims() {
    let net = load_file(TRACK).expect("the test track loads");
    assert_eq!(net.driving_lanes().count(), 2, "one lane each direction");

    let lane = net
        .driving_lanes()
        .max_by(|a, b| a.center.length().total_cmp(&b.center.length()))
        .expect("a driving lane");
    let length = lane.center.length();
    assert!(
        (length - 1053.0).abs() < 40.0,
        "track baked to {length:.1} m, expected ~1053"
    );

    // The straight really is straight, and the tight corner really is tight.
    // R=25 is the sharpest thing on the track outside the spiral's end.
    let straight = curvature(lane, 150.0);
    assert!(straight < 1e-3, "the opening straight curves at {straight}");
    let tight = curvature(lane, 764.0);
    assert!(
        tight > 1.0 / 40.0,
        "the R25 corner came out at radius {:.0} m",
        1.0 / tight.max(1e-6)
    );

    // The spiral tightens: curvature must grow along it, not sit constant.
    let (entry, exit) = (curvature(lane, 860.0), curvature(lane, 940.0));
    assert!(
        exit > entry * 1.5,
        "the spiral did not tighten: {entry:.5} -> {exit:.5} per metre"
    );

    // It climbs and comes back down -- the crest and dip are the point of the
    // elevation profile, so a flat import would be a silent loss.
    let heights: Vec<f32> = lane.center.points().iter().map(|p| p.y).collect();
    let low = heights.iter().copied().fold(f32::INFINITY, f32::min);
    let high = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    assert!(
        high - low > 3.0,
        "the track is nearly flat: {low:.2} m to {high:.2} m"
    );
    let crest = lane.center.point_at(405.0).y;
    assert!(
        crest > lane.center.point_at(300.0).y + 2.0,
        "no crest: {crest:.2} m against the climb's start"
    );
    assert!(
        crest > lane.center.point_at(470.0).y + 1.0,
        "the crest does not fall away into the dip"
    );

    assert!(lane.center.points().iter().all(|p| p.is_finite()));
}
