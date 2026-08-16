//! Smoke test: the importer survives a real, tool-exported OpenDRIVE map.
//!
//! `real_roads.xodr` is CARLA's Town07 (MIT-licensed; header preserved in the
//! file) -- 234 roads of lines + arcs + `laneOffset` + many lane sections, no
//! spirals. It pins the "loads without panicking, produces finite geometry"
//! bar for real files, and is the driver for P5b multi-lane-section support.

use map_opendrive::load_file;

const REAL: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/real_roads.xodr");

#[test]
fn real_map_loads_with_finite_geometry() {
    let net = load_file(REAL).expect("real OpenDRIVE map should load");
    assert!(net.driving_lanes().count() > 0, "no driving lanes imported");
    for lane in &net.lanes {
        let pts = lane.center.points();
        assert!(pts.len() >= 2, "lane {:?} has < 2 points", lane.id);
        assert!(
            pts.iter().all(|p| p.is_finite()),
            "lane {:?} has a non-finite point",
            lane.id
        );
        let len = lane.center.length();
        assert!(len.is_finite() && len > 0.0, "lane {:?} length {len}", lane.id);
    }
}
