//! Real-ABI smoke test: load the committed VanDerPol Reference FMU, step it,
//! and confirm the state advances and every `StepOutcome` is clean. This is the
//! only test that exercises the `fmi` crate against a real shared library; the
//! pure logic is unit-tested against an in-memory fake elsewhere.
//!
//! VanDerPol is a continuous ODE with no events / early return, so the
//! `StepOutcome` event path stays untested-by-design in v1.

use std::path::PathBuf;

use dynamics_fmi::{BaseType, Causality, Fmu, FmuInstance};

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/VanDerPol.fmu")
}

const DT: f64 = 1.0 / 64.0;

#[test]
fn vanderpol_loads_and_advances() {
    let mut fmu = Fmu::load(fixture(), 0.0).expect("load VanDerPol.fmu");

    // x0 is a Float64 output that starts at 2.0 (per the model description).
    let x0 = fmu
        .model_description()
        .variable("x0")
        .expect("x0 in model description")
        .value_reference;

    let start = fmu.get_output(x0).expect("read x0");
    assert!((start - 2.0).abs() < 1e-9, "x0 start = {start}");

    // Step ~1 second. VanDerPol (mu=1) oscillates with period ~6.6 s, so x0
    // moves well clear of its start over this window.
    //
    // Note (a real finding for the coupling layer): this FMU has a fixed
    // internal step of 0.01 s, so it reports `last_successful_time` on its own
    // grid, which LAGS the requested communication point -- it is not
    // `current_time + dt`. So we assert only what actually holds: the outcome
    // flags are clean, and the time is finite, monotonic, and never ahead of
    // what we asked. Slices 4/5 must treat `last_successful_time` as advisory.
    let mut t = 0.0;
    let mut prev_time = 0.0;
    for _ in 0..64 {
        let outcome = fmu.do_step(t, DT).expect("do_step");
        assert!(!outcome.early_return, "unexpected early return at t={t}");
        assert!(
            !outcome.terminate_simulation,
            "unexpected terminate at t={t}"
        );
        assert!(!outcome.event_handling_needed, "unexpected event at t={t}");
        assert!(outcome.last_successful_time.is_finite());
        assert!(
            outcome.last_successful_time >= prev_time,
            "time went backwards: {} < {prev_time}",
            outcome.last_successful_time
        );
        assert!(
            outcome.last_successful_time <= t + DT + 1e-9,
            "advanced past the requested point: {} > {}",
            outcome.last_successful_time,
            t + DT
        );
        prev_time = outcome.last_successful_time;
        t += DT;
    }

    let end = fmu.get_output(x0).expect("read x0");
    assert!(
        (end - start).abs() > 1e-3,
        "x0 did not advance: start={start} end={end}"
    );
}

#[test]
fn model_description_maps_types_and_causalities() {
    // Confirms real XML parsing + our schema mapping produced the right shapes.
    let fmu = Fmu::load(fixture(), 0.0).expect("load");
    let md = fmu.model_description();

    let mu = md.variable("mu").expect("mu");
    assert_eq!(mu.base_type, BaseType::Float64);
    assert_eq!(mu.causality, Causality::Parameter);

    let x1 = md.variable("x1").expect("x1");
    assert_eq!(x1.base_type, BaseType::Float64);
    assert_eq!(x1.causality, Causality::Output);
}
