use glam::Vec3;

use crate::binding::ResolvedOutputs;
use crate::instance::{FmuError, FmuInstance};

/// The vehicle pose read out of the FMU each tick and stamped onto the kinematic
/// body. Position in metres (world); `yaw` in radians about +Y.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    pub position: Vec3,
    pub yaw: f32,
    /// Body roll + pitch (rad). Zero unless the FMU binds roll/pitch outputs
    /// (a 3D model like double-track OCD); a planar FMU leaves them 0.
    pub roll: f32,
    pub pitch: f32,
}

/// Read the bound pose outputs off the FMU. Values are FMI Float64, narrowed to
/// the engine's `f32`. Takes `&mut dyn` so a boxed, type-erased instance (the
/// per-entity FMU store in `movement`) can be read; a concrete `&mut T` still
/// coerces in. Roll/pitch are read only when bound (0 otherwise).
pub fn read_pose(fmu: &mut dyn FmuInstance, outputs: &ResolvedOutputs) -> Result<Pose, FmuError> {
    let read_opt = |fmu: &mut dyn FmuInstance, vr: Option<crate::instance::ValueReference>| {
        vr.map(|vr| fmu.get_output(vr)).transpose()
    };
    Ok(Pose {
        position: Vec3::new(
            fmu.get_output(outputs.x)? as f32,
            fmu.get_output(outputs.y)? as f32,
            fmu.get_output(outputs.z)? as f32,
        ),
        yaw: fmu.get_output(outputs.yaw)? as f32,
        roll: read_opt(fmu, outputs.roll)?.unwrap_or(0.0) as f32,
        pitch: read_opt(fmu, outputs.pitch)?.unwrap_or(0.0) as f32,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instance::{StepOutcome, ValueReference};
    use std::collections::HashMap;

    /// In-memory `FmuInstance` for the pure tests: a value store keyed by value
    /// reference, plus a step counter. `do_step` here does no dynamics -- pose
    /// tests set the outputs directly.
    #[derive(Default)]
    struct FakeFmu {
        values: HashMap<ValueReference, f64>,
        steps: u32,
    }

    impl FmuInstance for FakeFmu {
        fn set_input(&mut self, vr: ValueReference, value: f64) -> Result<(), FmuError> {
            self.values.insert(vr, value);
            Ok(())
        }
        fn do_step(&mut self, current_time: f64, step_size: f64) -> Result<StepOutcome, FmuError> {
            self.steps += 1;
            Ok(StepOutcome {
                event_handling_needed: false,
                terminate_simulation: false,
                early_return: false,
                last_successful_time: current_time + step_size,
            })
        }
        fn get_output(&mut self, vr: ValueReference) -> Result<f64, FmuError> {
            self.values
                .get(&vr)
                .copied()
                .ok_or(FmuError::GetOutput { vr })
        }
    }

    fn outputs() -> ResolvedOutputs {
        ResolvedOutputs {
            x: 10,
            y: 11,
            z: 12,
            yaw: 13,
            roll: None,
            pitch: None,
        }
    }

    #[test]
    fn read_pose_pulls_the_bound_outputs() {
        let mut fmu = FakeFmu::default();
        fmu.values.insert(10, 1.5);
        fmu.values.insert(11, 0.0);
        fmu.values.insert(12, -3.0);
        fmu.values.insert(13, 0.25);
        let pose = read_pose(&mut fmu, &outputs()).expect("pose");
        assert_eq!(pose.position, Vec3::new(1.5, 0.0, -3.0));
        assert_eq!(pose.yaw, 0.25);
        // Unbound roll/pitch default to 0.
        assert_eq!(pose.roll, 0.0);
        assert_eq!(pose.pitch, 0.0);
    }

    #[test]
    fn read_pose_reads_bound_roll_and_pitch() {
        let mut fmu = FakeFmu::default();
        for (vr, v) in [
            (10, 0.0),
            (11, 0.0),
            (12, 0.0),
            (13, 0.0),
            (14, 0.1),
            (15, -0.05),
        ] {
            fmu.values.insert(vr, v);
        }
        let outs = ResolvedOutputs {
            roll: Some(14),
            pitch: Some(15),
            ..outputs()
        };
        let pose = read_pose(&mut fmu, &outs).expect("pose");
        assert!((pose.roll - 0.1).abs() < 1e-6);
        assert!((pose.pitch - (-0.05)).abs() < 1e-6);
    }

    #[test]
    fn read_pose_errors_on_a_missing_output() {
        let mut fmu = FakeFmu::default();
        assert!(matches!(
            read_pose(&mut fmu, &outputs()),
            Err(FmuError::GetOutput { vr: 10 })
        ));
    }

    #[test]
    fn set_input_round_trips_and_do_step_advances() {
        let mut fmu = FakeFmu::default();
        fmu.set_input(1, 0.3).expect("set");
        assert_eq!(fmu.get_output(1).expect("get"), 0.3);
        let outcome = fmu.do_step(0.0, 1.0 / 64.0).expect("step");
        assert_eq!(
            outcome,
            StepOutcome {
                event_handling_needed: false,
                terminate_simulation: false,
                early_return: false,
                last_successful_time: 1.0 / 64.0,
            }
        );
        fmu.do_step(1.0 / 64.0, 1.0 / 64.0).expect("step");
        assert_eq!(fmu.steps, 2);
    }
}
