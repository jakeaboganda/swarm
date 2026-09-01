use glam::Vec3;

use crate::pose::Pose;

/// The coordinate frame an FMU emits its pose in. The sim is Bevy's Y-up frame
/// (+Y up, forward = -Z, left = -X). [`to_sim_local`] maps an FMU-frame pose into
/// a sim-local pose -- still expressed relative to the FMU's OWN origin; the
/// caller rebases that onto the spawn pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FmuFrame {
    /// Already the sim's Y-up frame -- identity.
    #[default]
    SimYUp,
    /// Open-Car-Dynamics: x forward, y left, z up (right-handed, z-up). Its yaw
    /// is about z, left-positive -- the same sense as the sim's yaw about +Y.
    OcdZUp,
}

// The OcdZUp -> sim-local yaw sign, isolated so it is trivial to flip if an
// empirical drive shows turns going the wrong way. Both frames are left-positive
// about their up axis by derivation, so this is +1; the axis remap in
// `to_sim_local` likewise has each sign on its own line for the same reason.
const OCD_YAW_SIGN: f32 = 1.0;

/// Map an FMU-frame pose to a sim-local (Bevy Y-up) pose. The result is still
/// relative to the FMU's origin -- `drive_fmu_vehicles` composes it onto the
/// spawn pose.
pub fn to_sim_local(frame: FmuFrame, pose: Pose) -> Pose {
    match frame {
        FmuFrame::SimYUp => pose,
        FmuFrame::OcdZUp => {
            // OCD position is (forward, left, up). Sim-local basis is
            // forward = -Z, left = -X, up = +Y, so:
            //   OCD forward (+x) -> sim -Z
            //   OCD left    (+y) -> sim -X
            //   OCD up      (+z) -> sim +Y
            let p = pose.position;
            Pose {
                position: Vec3::new(-p.y, p.z, -p.x),
                yaw: OCD_YAW_SIGN * pose.yaw,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_y_up_is_identity() {
        let pose = Pose {
            position: Vec3::new(1.0, 2.0, 3.0),
            yaw: 0.4,
        };
        assert_eq!(to_sim_local(FmuFrame::SimYUp, pose), pose);
    }

    #[test]
    fn ocd_forward_maps_to_sim_forward_minus_z() {
        // 10 m forward in OCD (its +x) -- sim forward is -Z.
        let out = to_sim_local(
            FmuFrame::OcdZUp,
            Pose {
                position: Vec3::new(10.0, 0.0, 0.0),
                yaw: 0.0,
            },
        );
        assert_eq!(out.position, Vec3::new(0.0, 0.0, -10.0));
    }

    #[test]
    fn ocd_left_maps_to_sim_left_minus_x() {
        // 5 m left in OCD (its +y) -- sim left is -X.
        let out = to_sim_local(
            FmuFrame::OcdZUp,
            Pose {
                position: Vec3::new(0.0, 5.0, 0.0),
                yaw: 0.0,
            },
        );
        assert_eq!(out.position, Vec3::new(-5.0, 0.0, 0.0));
    }

    #[test]
    fn ocd_up_maps_to_sim_up_plus_y() {
        // OCD z (up) -- sim up is +Y.
        let out = to_sim_local(
            FmuFrame::OcdZUp,
            Pose {
                position: Vec3::new(0.0, 0.0, 2.0),
                yaw: 0.0,
            },
        );
        assert_eq!(out.position, Vec3::new(0.0, 2.0, 0.0));
    }

    #[test]
    fn ocd_left_yaw_stays_left_positive() {
        // A positive (left) OCD yaw stays a positive (left) sim yaw.
        let out = to_sim_local(
            FmuFrame::OcdZUp,
            Pose {
                position: Vec3::ZERO,
                yaw: 0.3,
            },
        );
        assert_eq!(out.yaw, 0.3);
    }
}
