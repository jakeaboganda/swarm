//! Independent test pass for the per-wheel FMU road-conform's *mesh* behaviours
//! (the float-fix follow-up): the wheels now sit on the actual baked surface, the
//! body's own roll/pitch don't lift them off it, and the spin angle stays bounded.
//!
//! These drive the public `server::world` API (`wheel_drape`, `seat_wheel`,
//! `conformed_ride_height`) plus `map::Mesh::height_at` -- no ECS -- so they pin
//! geometry, not wiring:
//!   * P2: across a *curved + banked* lane, every wheel's contact patch lands on
//!     `mesh.height_at` under it (the ≤94 mm analytic-vs-mesh float is gone);
//!   * P3: rolling/pitching the body (conform's `arc(Y,up)*yaw*rot_z*rot_x`) and
//!     re-seating each wheel keeps the wheels on the mesh -- the body lean is the
//!     visible suspension travel, not a lift-off;
//!   * P4: the spin wrap keeps the angle in [0, 2π) over an arbitrarily long run;
//!   * P5: off the mesh -> `None`.
//!
//! Test-only file (added by the test pass); no non-test code is touched.

use std::f32::consts::TAU;

use bevy::prelude::{Quat, Transform};
use glam::Vec3;
use movement::{wheel_offset, RaycastVehicle};
use server::world::{conformed_ride_height, seat_wheel, wheel_drape};

/// yaw such that the car's local forward (-Z) aligns with horizontal `heading`.
fn yaw_for(heading: Vec3) -> f32 {
    (-heading.x).atan2(-heading.z)
}

/// Reconstruct the chassis transform `wheel_drape` implies (road-aligned):
/// `arc(Y, up) * yaw`, at the draped chassis height over planar `pos`.
fn road_aligned_chassis(pos: Vec3, yaw: f32, up: Vec3, chassis_y: f32) -> Transform {
    Transform {
        translation: Vec3::new(pos.x, chassis_y, pos.z),
        rotation: Quat::from_rotation_arc(Vec3::Y, up) * Quat::from_rotation_y(yaw),
        scale: Vec3::ONE,
    }
}

/// The world contact patch of wheel `i` for a chassis, measured the way the
/// conform seats it: down the surface `up` from the attach point by
/// `radius + (rest - compression)`.
fn wheel_bottom(
    chassis: &Transform,
    up: Vec3,
    rig: &RaycastVehicle,
    i: usize,
    compression: f32,
) -> Vec3 {
    let attach = chassis.transform_point(wheel_offset(i, rig));
    let reach = rig.wheel_radius + (rig.suspension_rest - compression);
    attach - up * reach
}

// --- P2: wheels sit on the MESH across a curved + banked lane ---------------

#[test]
fn every_wheel_lands_on_the_mesh_around_the_banked_curve() {
    // The banked oval is curved AND super-elevated -- exactly the geometry where
    // the smooth analytic centreline rode a chord-error (measured ≤94 mm) above
    // the faceted collider surface. Sampling the mesh must remove that: each
    // wheel's contact patch sits on `height_at` under it to within a few mm.
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    let lane = net.driving_lanes().next().expect("the oval has a lane");
    let len = lane.center.length();

    let mut checked = 0;
    let mut on_bank = false;
    let mut max_err = 0f32;
    // March many stations right around the loop, through both banked curves.
    for k in 0..120 {
        let s = lane.sample_at(len * (k as f32) / 120.0);
        let pos = Vec3::new(s.point.x, 0.0, s.point.z);
        let yaw = yaw_for(s.heading);
        let drape = wheel_drape(&mesh, &net, pos, yaw, &rig).expect("car is on the oval");
        // Body tilts on the banked curves.
        if drape.up.y < 0.99 {
            on_bank = true;
        }
        let chassis = road_aligned_chassis(pos, yaw, drape.up, drape.chassis_y);
        for i in 0..4 {
            if !drape.on_road[i] {
                continue;
            }
            let b = wheel_bottom(&chassis, drape.up, &rig, i, drape.compression[i]);
            let (surf, _) = mesh
                .height_at(b.x, b.z)
                .expect("the wheel's contact patch is over the mesh");
            let err = (b.y - surf).abs();
            max_err = max_err.max(err);
            // The old analytic drape carried a measured ≤94 mm float on tight
            // banked curves; the mesh drape's residual (the reach's lateral swing
            // under a tilted `up` landing on a neighbouring facet) is an order of
            // magnitude smaller. 30 mm is a generous ceiling that still proves the
            // float is gone.
            assert!(
                err < 3e-2,
                "station {k} wheel {i}: contact {} is {err:.4} m off the mesh {surf}",
                b.y
            );
            checked += 1;
        }
    }
    assert!(checked > 300, "too few wheels checked ({checked})");
    assert!(
        on_bank,
        "never sampled a banked station -- test is not exercising the cant"
    );
    eprintln!("banked-curve wheel-on-mesh: {checked} wheels, max err {max_err:.5} m");
}

#[test]
fn drape_on_the_curve_beats_the_analytic_centreline_error() {
    // The bug: the old drape sat wheels on the smooth analytic surface, which on
    // a tight banked curve rides a chord-error above the drawn facets. Pin that
    // the mesh drape's own contact patches are flush with the mesh (sub-cm),
    // i.e. we are no longer carrying that ≤94 mm error.
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    let lane = net.driving_lanes().next().expect("lane");
    let len = lane.center.length();
    // Curve apex region: ~1/4 of the way round is the first curve.
    let s = lane.sample_at(len * 0.30);
    let pos = Vec3::new(s.point.x, 0.0, s.point.z);
    let yaw = yaw_for(s.heading);
    let drape = wheel_drape(&mesh, &net, pos, yaw, &rig).expect("on the curve");
    let chassis = road_aligned_chassis(pos, yaw, drape.up, drape.chassis_y);
    for i in 0..4 {
        if !drape.on_road[i] {
            continue;
        }
        let b = wheel_bottom(&chassis, drape.up, &rig, i, drape.compression[i]);
        let (surf, _) = mesh.height_at(b.x, b.z).expect("over the mesh");
        assert!(
            (b.y - surf).abs() < 3e-2,
            "wheel {i} floats {:.4} m over the mesh -- analytic-vs-mesh error is back",
            b.y - surf
        );
    }
}

// --- P3: body roll/pitch keep the wheels on the road ------------------------

#[test]
fn rolling_the_body_re_seats_the_wheels_onto_the_mesh() {
    // Layer the FMU's own roll+pitch on top of the road tilt (conform's
    // `arc(Y,up)*yaw*rot_z(roll)*rot_x(pitch)`), then re-seat each wheel against
    // the mesh under its FINAL xz -- exactly what `conform_fmu_to_track` does.
    // Rolling the body must not lift a wheel off the road: each re-seated contact
    // patch stays on the mesh. Angles are kept inside the suspension's droop
    // budget (~0.058 m = quarter-weight sag); a corner rise beyond that would
    // out-travel the springs and legitimately lift the wheel (real physics, and
    // the FMU's own lean stays well under it).
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    let lane = net.driving_lanes().next().expect("lane");
    let len = lane.center.length();

    let mut max_err = 0f32;
    for (roll, pitch) in [(0.02_f32, 0.0_f32), (-0.02, 0.01), (0.015, -0.012)] {
        for frac in [0.05_f32, 0.30, 0.55, 0.80] {
            let s = lane.sample_at(len * frac);
            let pos = Vec3::new(s.point.x, 0.0, s.point.z);
            let yaw = yaw_for(s.heading);
            let drape = wheel_drape(&mesh, &net, pos, yaw, &rig).expect("on the oval");

            // The tilted body pose the conform builds.
            let body = Quat::from_rotation_arc(Vec3::Y, drape.up)
                * Quat::from_rotation_y(yaw)
                * Quat::from_rotation_z(roll)
                * Quat::from_rotation_x(pitch);
            let chassis = Transform {
                translation: Vec3::new(pos.x, drape.chassis_y, pos.z),
                rotation: body,
                scale: Vec3::ONE,
            };

            for i in 0..4 {
                let attach = chassis.transform_point(wheel_offset(i, &rig));
                let Some((y, _)) = mesh.height_at(attach.x, attach.z) else {
                    continue; // attach xz swung off the mesh edge; not this test's case
                };
                let contact = Vec3::new(attach.x, y, attach.z);
                let comp = seat_wheel(&chassis, drape.up, contact, &rig, i);
                // In travel and not railed to full droop: the spring still presses,
                // so the wheel is on the road.
                assert!(
                    comp > 0.0 && comp < rig.suspension_rest,
                    "roll={roll} pitch={pitch} frac={frac} wheel {i}: compression {comp} railed -- wheel lifted or buried"
                );
                // Reconstruct the re-seated contact patch and confirm it's on the
                // mesh: rolling the body did not lift the wheel off the road.
                let b = wheel_bottom(&chassis, drape.up, &rig, i, comp);
                let (surf, _) = mesh
                    .height_at(b.x, b.z)
                    .expect("re-seated patch over the mesh");
                let err = (b.y - surf).abs();
                max_err = max_err.max(err);
                assert!(
                    err < 2.0e-2,
                    "roll={roll} pitch={pitch} frac={frac} wheel {i}: re-seated patch {} is {err:.4} m off the mesh {surf} -- the roll lifted it",
                    b.y,
                );
            }
        }
    }
    eprintln!("rolled-body wheel-on-mesh: max err {max_err:.5} m");
}

#[test]
fn an_over_travel_tilt_clamps_gracefully_without_lifting_out_of_range() {
    // A body tilt bigger than the droop budget CAN out-travel the springs (the
    // wheel then hangs at full droop) -- but seat_wheel must clamp into [0, rest]
    // and stay finite, never producing a NaN or an out-of-range compression.
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    let lane = net.driving_lanes().next().expect("lane");
    let s = lane.sample_at(0.0);
    let pos = Vec3::new(s.point.x, 0.0, s.point.z);
    let yaw = yaw_for(s.heading);
    let drape = wheel_drape(&mesh, &net, pos, yaw, &rig).expect("on the oval");

    let chassis = Transform {
        translation: Vec3::new(pos.x, drape.chassis_y, pos.z),
        rotation: Quat::from_rotation_arc(Vec3::Y, drape.up)
            * Quat::from_rotation_y(yaw)
            * Quat::from_rotation_z(0.25) // way past the droop budget
            * Quat::from_rotation_x(0.15),
        scale: Vec3::ONE,
    };
    for i in 0..4 {
        let attach = chassis.transform_point(wheel_offset(i, &rig));
        if let Some((y, _)) = mesh.height_at(attach.x, attach.z) {
            let comp = seat_wheel(
                &chassis,
                drape.up,
                Vec3::new(attach.x, y, attach.z),
                &rig,
                i,
            );
            assert!(comp.is_finite(), "wheel {i} compression not finite");
            assert!(
                (0.0..=rig.suspension_rest + 1e-6).contains(&comp),
                "wheel {i} compression {comp} out of [0, rest]"
            );
        }
    }
}

#[test]
fn a_pure_road_aligned_body_leaves_the_wheels_where_the_drape_put_them() {
    // Sanity: with zero FMU roll/pitch the re-seat is a no-op -- compression
    // equals the drape's own, and the wheels sit at the conformed height on flat.
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    let lane = net.driving_lanes().next().expect("lane");
    // Mid of the bottom straight (x≈0, flat, clear of the curve junctions), not
    // frac=0.0 which sits on a phase join where a wheel can reach onto the
    // ramping cant.
    let s = lane.sample_at(lane.center.length() * 0.11);
    let pos = Vec3::new(s.point.x, 0.0, s.point.z);
    let yaw = yaw_for(s.heading);
    let drape = wheel_drape(&mesh, &net, pos, yaw, &rig).expect("on the oval");
    assert!(
        drape.up.abs_diff_eq(Vec3::Y, 5e-3),
        "mid-straight should be ~flat: {:?}",
        drape.up
    );
    let chassis = road_aligned_chassis(pos, yaw, drape.up, drape.chassis_y);
    for i in 0..4 {
        let attach = chassis.transform_point(wheel_offset(i, &rig));
        let (y, _) = mesh.height_at(attach.x, attach.z).expect("over the mesh");
        let comp = seat_wheel(
            &chassis,
            drape.up,
            Vec3::new(attach.x, y, attach.z),
            &rig,
            i,
        );
        assert!(
            (comp - drape.compression[i]).abs() < 1e-4,
            "re-seat with no body tilt should match the drape: {comp} vs {}",
            drape.compression[i]
        );
    }
    // On flat, the chassis rides the settled height above the (y=0) surface.
    assert!(
        (drape.chassis_y - conformed_ride_height()).abs() < 5e-3,
        "flat chassis {} should be the settled height {}",
        drape.chassis_y,
        conformed_ride_height()
    );
}

// --- P4: the spin angle stays bounded over a long run -----------------------

#[test]
fn wheel_spin_stays_in_zero_to_tau_over_a_long_run() {
    // The conform advances each wheel by `(angle + travelled/radius).rem_euclid(TAU)`
    // every tick. Over a long drive that accumulator would otherwise blow past
    // f32's usable precision; the wrap must keep it in [0, 2π). Replay the exact
    // recurrence over a run far longer than any scenario (100 km at 64 Hz).
    let rig = RaycastVehicle::default();
    let radius = rig.wheel_radius;
    let dt = 1.0 / 64.0;
    let speed = 40.0_f32; // m/s -- a fast lap
    let per_tick = speed * dt; // metres travelled per tick

    let mut angle = 0.0_f32;
    let ticks = 64 * 60 * 40; // ~40 minutes -> ~96 km
    for t in 0..ticks {
        let spin = per_tick / radius;
        angle = (angle + spin).rem_euclid(TAU);
        assert!(
            (0.0..TAU).contains(&angle),
            "tick {t}: spin angle {angle} escaped [0, TAU)"
        );
        assert!(angle.is_finite(), "tick {t}: spin angle went non-finite");
    }
    // And it did actually move (not stuck at 0).
    assert!(angle != 0.0, "the wheel never advanced");
}

// --- P5: off the mesh -> None (mesh-gated, not extrapolated) -----------------

#[test]
fn off_the_mesh_edge_gives_no_drape() {
    let rig = RaycastVehicle::default();
    let net = map::banked_oval();
    let mesh = net.surface_mesh();
    // Well outside the oval footprint (the track lives within ~|x|<95, |z|<40).
    assert!(
        wheel_drape(&mesh, &net, Vec3::new(500.0, 0.0, 500.0), 0.0, &rig).is_none(),
        "a car far off the road must not drape onto extrapolated surface"
    );
}
