//! Independent test pass for the per-wheel FMU road-conform ("option C").
//!
//! These drive the pure `server::world::wheel_drape` geometry through the public
//! API only, pinning the behaviours the design says C must satisfy:
//!   * the chassis float is actually removed (settled below full extension), and
//!     on the flat every wheel's contact patch lands on the surface;
//!   * an *elevated* banked road (the risk the reviewer flagged) neither sinks
//!     nor floats the car -- the chassis tracks the local grade and the wheels
//!     sit on the canted surface;
//!   * the drape is genuinely per-wheel (a bank tilts the body so the wheels sit
//!     at different world heights), versus a flat road where it is level;
//!   * robustness: no road -> `None`; a car off to the side still drapes,
//!     finite, without NaNs.
//!
//! Test-only file (added by the test pass); no non-test code is touched.

use glam::Vec3;
use map::{Direction, Lane, LaneId, LaneKind, Polyline, RoadNetwork};
use movement::{wheel_offset, RaycastVehicle};
use server::world::{car_ride_height, conformed_ride_height, wheel_drape, WheelDrape, CAR_MASS};

/// A single straight driving lane along +X, centred at the origin, with an
/// optional per-vertex bank (empty = flat).
fn straight(bank: Vec<f32>) -> RoadNetwork {
    RoadNetwork {
        lanes: vec![Lane {
            id: LaneId(0),
            kind: LaneKind::Driving,
            direction: Direction::Forward,
            center: Polyline::new(vec![Vec3::new(-60.0, 0.0, 0.0), Vec3::new(60.0, 0.0, 0.0)]),
            width: 6.0,
            bank,
            successors: Vec::new(),
            predecessors: Vec::new(),
            neighbors: Vec::new(),
        }],
    }
}

/// Surface height (world Y) under `xz` on `net`, computed the same tangent-plane
/// way `wheel_drape` does -- the reference for "the wheel sits on the road".
fn surface_y(net: &RoadNetwork, xz: Vec3) -> f32 {
    let s = net.sample_near(xz).expect("a lane under the point");
    let n = s.up;
    let d = xz - s.point;
    s.point.y - (n.x * d.x + n.z * d.z) / n.y
}

/// World position of wheel `i`'s contact patch for `drape` at planar `pos`, per
/// the formula in the task brief:
///   bottom = chassis.transform_point(offset) - up*(rest - compression + radius)
fn wheel_bottom(pos: Vec3, yaw: f32, drape: &WheelDrape, rig: &RaycastVehicle, i: usize) -> Vec3 {
    // Mirror `wheel_drape`'s chassis pose exactly, yaw included -- dropping the
    // yaw only hides on a level or planar surface, and misplaces the wheels
    // (wrong station) on a grade.
    let chassis = bevy::prelude::Transform {
        translation: Vec3::new(pos.x, drape.chassis_y, pos.z),
        rotation: bevy::prelude::Quat::from_rotation_arc(Vec3::Y, drape.up)
            * bevy::prelude::Quat::from_rotation_y(yaw),
        scale: Vec3::ONE,
    };
    let reach = rig.suspension_rest - drape.compression[i] + rig.wheel_radius;
    chassis.transform_point(wheel_offset(i, rig)) - drape.up * reach
}

/// yaw such that the car's local forward (-Z) aligns with horizontal `heading`.
fn yaw_for(heading: Vec3) -> f32 {
    (-heading.x).atan2(-heading.z)
}

// --- Priority 1: the float is removed, numerically -------------------------

#[test]
fn conformed_ride_height_is_a_static_sag_below_full_extension() {
    let full = car_ride_height();
    let settled = conformed_ride_height();
    assert!(
        settled < full,
        "conformed height {settled} must sit below full extension {full}"
    );
    // The gap is exactly a quarter of the car's weight on the springs.
    let sag = full - settled;
    let expected = (CAR_MASS * 9.81 / 4.0) / RaycastVehicle::default().suspension_stiffness;
    assert!(
        (sag - expected).abs() < 1e-4,
        "sag {sag} should equal quarter-weight/stiffness {expected}"
    );
    assert!(sag > 0.03 && sag < 0.12, "sag {sag} out of a sane range");
}

#[test]
fn flat_drape_sits_at_the_conformed_height_with_every_wheel_on_the_surface() {
    let rig = RaycastVehicle::default();
    let net = straight(Vec::new());
    let pos = Vec3::ZERO;
    let drape = wheel_drape(&net, pos, yaw_for(Vec3::X), &rig).expect("road under the car");

    assert!(drape.up.abs_diff_eq(Vec3::Y, 1e-5), "up {:?}", drape.up);
    // Surface is at y=0 here, so the chassis rides exactly the settled height.
    assert!(
        (drape.chassis_y - conformed_ride_height()).abs() < 1e-4,
        "chassis {} should equal conformed ride height {}",
        drape.chassis_y,
        conformed_ride_height()
    );
    let sag = car_ride_height() - conformed_ride_height();
    for i in 0..4 {
        let b = wheel_bottom(pos, yaw_for(Vec3::X), &drape, &rig, i);
        assert!(b.y.abs() < 1e-4, "wheel {i} bottom {} not on the road", b.y);
        assert!(
            (drape.compression[i] - sag).abs() < 1e-4,
            "wheel {i} compression {} should equal static sag {sag}",
            drape.compression[i]
        );
    }
}

// --- Priority 2: elevated + banked road (the reviewer's flagged risk) -------

/// A road that BOTH climbs (5% grade) and banks (0.2 rad constant), imported
/// through the real OpenDRIVE path -- the case that would sink or float a car if
/// the conform pinned the chassis near y=0 instead of tracking the local grade.
const CLIMBING_BANKED: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="7" name="climbing_banked" version="1.00"/>
  <road name="climbing_banked" length="100.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="0.0" b="0.05" c="0.0" d="0.0"/>
    </elevationProfile>
    <lateralProfile>
      <superelevation s="0.0" a="0.2" b="0.0" c="0.0" d="0.0"/>
    </lateralProfile>
    <lanes>
      <laneSection s="0.0">
        <left><lane id="1" type="driving"><width sOffset="0.0" a="4.0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="4.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

#[test]
fn elevated_banked_drape_tracks_the_grade_and_cants_the_car() {
    let rig = RaycastVehicle::default();
    let net = map_opendrive::load_str(CLIMBING_BANKED).expect("climbing_banked imports");

    // Sample a forward driving lane at mid-road (an elevated, canted station).
    let lane = net
        .lanes
        .iter()
        .find(|l| l.kind == LaneKind::Driving && l.direction == Direction::Forward)
        .expect("a forward driving lane");
    let mid = lane.center.length() * 0.5;
    let s = lane.sample_at(mid);
    let pos = Vec3::new(s.point.x, 0.0, s.point.z);
    let yaw = yaw_for(s.heading);

    let drape = wheel_drape(&net, pos, yaw, &rig).expect("road under the elevated car");

    // The station is genuinely elevated (~2.5 m of climb at mid-road); the
    // chassis must ride the *local* surface, not be pinned near zero.
    let surf = surface_y(&net, pos);
    assert!(surf > 2.0, "mid-road surface {surf} should be elevated");
    assert!(
        (drape.chassis_y - (surf + conformed_ride_height())).abs() < 5e-2,
        "chassis {} should track local surface {surf} + settled height {}",
        drape.chassis_y,
        conformed_ride_height()
    );
    assert!(
        drape.chassis_y.is_finite() && drape.chassis_y > 2.0,
        "chassis {} sank/floated off the elevated grade",
        drape.chassis_y
    );

    // The body leans onto the 0.2 rad cant.
    assert!(
        drape.up.y > 0.95 && drape.up.y < 0.999,
        "up {:?} should tilt onto the ~0.2 rad cant",
        drape.up
    );
    assert!((drape.bank.abs() - 0.2).abs() < 1e-2, "bank {}", drape.bank);

    // Every wheel sits on the canted surface under it (self-consistent with the
    // same tangent-plane sample), and the body is tilted (heights spread).
    let mut ys = Vec::new();
    for i in 0..4 {
        let b = wheel_bottom(pos, yaw, &drape, &rig, i);
        assert!(b.is_finite(), "wheel {i} bottom not finite");
        assert!(
            (b.y - surface_y(&net, b)).abs() < 1e-2,
            "wheel {i} bottom {} off the surface {}",
            b.y,
            surface_y(&net, b)
        );
        ys.push(b.y);
    }
    let spread = ys.iter().copied().fold(f32::NEG_INFINITY, f32::max)
        - ys.iter().copied().fold(f32::INFINITY, f32::min);
    assert!(
        spread > 0.2,
        "car did not cant on the bank (spread {spread})"
    );
}

// --- Priority 3: the drape is genuinely per-wheel --------------------------

#[test]
fn flat_is_level_and_equal_while_a_bank_tilts_the_wheels_apart() {
    let rig = RaycastVehicle::default();

    // Flat: all four wheels at the same world height, equal compression.
    let flat = straight(Vec::new());
    let fd = wheel_drape(&flat, Vec3::ZERO, yaw_for(Vec3::X), &rig).expect("flat road");
    let f_ys: Vec<f32> = (0..4)
        .map(|i| wheel_bottom(Vec3::ZERO, yaw_for(Vec3::X), &fd, &rig, i).y)
        .collect();
    let f_spread = f_ys.iter().copied().fold(f32::NEG_INFINITY, f32::max)
        - f_ys.iter().copied().fold(f32::INFINITY, f32::min);
    assert!(
        f_spread < 1e-3,
        "flat wheels should be level (spread {f_spread})"
    );
    for i in 1..4 {
        assert!(
            (fd.compression[i] - fd.compression[0]).abs() < 1e-4,
            "flat compressions should be equal"
        );
    }

    // Bank: the body tilts, so the wheels sit at clearly different world heights,
    // each still on the surface under it.
    let banked = straight(vec![0.15, 0.15]);
    let bd = wheel_drape(&banked, Vec3::ZERO, yaw_for(Vec3::X), &rig).expect("banked road");
    let b_ys: Vec<f32> = (0..4)
        .map(|i| {
            let b = wheel_bottom(Vec3::ZERO, yaw_for(Vec3::X), &bd, &rig, i);
            assert!(
                (b.y - surface_y(&banked, b)).abs() < 5e-3,
                "banked wheel {i} bottom {} off the surface {}",
                b.y,
                surface_y(&banked, b)
            );
            b.y
        })
        .collect();
    let b_spread = b_ys.iter().copied().fold(f32::NEG_INFINITY, f32::max)
        - b_ys.iter().copied().fold(f32::INFINITY, f32::min);
    // yaw-aligned, the 0.15 rad cant spreads the ~1.6 m track by 1.6*sin(0.15)
    // ~= 0.24 m -- clearly canted, not level.
    assert!(
        b_spread > 0.2,
        "the bank did not tilt the wheels apart (spread {b_spread})"
    );
}

// --- Priority 5: robustness ------------------------------------------------

#[test]
fn empty_network_drapes_to_none() {
    let rig = RaycastVehicle::default();
    assert!(wheel_drape(&RoadNetwork::default(), Vec3::ZERO, 0.0, &rig).is_none());
}

#[test]
fn a_car_off_to_the_side_still_drapes_finitely() {
    let rig = RaycastVehicle::default();
    // Lane runs along X through z=0, width 6; put the car far off to the side.
    let net = straight(vec![0.1, 0.1]);
    let pos = Vec3::new(0.0, 0.0, 40.0);
    let drape = wheel_drape(&net, pos, yaw_for(Vec3::X), &rig)
        .expect("sample_near finds the nearest lane even off to the side");

    assert!(drape.chassis_y.is_finite(), "chassis_y not finite");
    assert!(
        drape.up.is_finite() && (drape.up.length() - 1.0).abs() < 1e-4,
        "up {:?} not a finite unit vector",
        drape.up
    );
    assert!(drape.bank.is_finite(), "bank not finite");
    for (i, c) in drape.compression.iter().enumerate() {
        assert!(c.is_finite(), "compression {i} not finite");
        assert!(
            *c >= 0.0 && *c <= rig.suspension_rest + 1e-6,
            "compression {i} = {c} out of [0, rest]"
        );
    }
}
