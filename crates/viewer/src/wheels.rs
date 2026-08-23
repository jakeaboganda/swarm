//! Tinting a vehicle's wheels by what they are doing.
//!
//! Placing them is not this module's job any more, nor the viewer's: the sim
//! sends each wheel as a node with its transform already composed, and
//! `scene::apply_stream` applies it. All that is left here is the debug-layer
//! tint, which is the one thing about a wheel a viewer must decide for itself.

use bevy::prelude::*;
use viz::NodePath;

use crate::overlay::DebugData;
use crate::scene::NodeIndex;

/// Colour of a wheel that is rolling normally.
const ROLLING: Color = Color::srgb(0.10, 0.10, 0.12);
/// A locked wheel: stopped while the car is still moving.
const LOCKED: Color = Color::srgb(0.85, 0.15, 0.15);
/// A spinning wheel: turning faster than the road is passing.
const SPINNING: Color = Color::srgb(0.20, 0.45, 0.95);
/// Off the ground, so doing nothing at all.
const AIRBORNE: Color = Color::srgb(0.35, 0.35, 0.40);

/// Slip at or below this is a locked wheel; at or above the positive one it is
/// wheelspin. Well inside the +/-1 extremes, so a tint means "clearly
/// slipping" rather than "rounding error".
const LOCKED_SLIP: f32 = -0.5;
const SPINNING_SLIP: f32 = 0.5;

/// What one wheel's diagnostics say it should look like.
fn tint(diagnostic: &viz::WheelDebug) -> Color {
    if !diagnostic.contact {
        AIRBORNE
    } else if diagnostic.slip_ratio <= LOCKED_SLIP {
        LOCKED
    } else if diagnostic.slip_ratio >= SPINNING_SLIP {
        SPINNING
    } else {
        ROLLING
    }
}

/// Tints each wheel node by what its wheel is doing, so lockup and wheelspin
/// are visible rather than inferred. Without it the only window into the tire
/// model is a per-tick dump on stderr.
///
/// The diagnostics arrive as a rig-ordered list, so they are keyed onto nodes
/// through `viz::WHEEL_NODES`.
pub fn tint_wheels(
    vehicles: Query<(&DebugData, &NodeIndex)>,
    nodes: Query<&MeshMaterial3d<StandardMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (debug, index) in &vehicles {
        for (slot, diagnostic) in debug.wheels.iter().enumerate() {
            let Some(name) = viz::WHEEL_NODES.get(slot) else {
                continue;
            };
            let Some(node) = index.get(&NodePath::root().child(name)) else {
                continue;
            };
            let Ok(handle) = nodes.get(node) else {
                continue;
            };
            let Some(mut material) = materials.get_mut(&handle.0) else {
                continue;
            };
            material.base_color = tint(diagnostic);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diagnostic(slip_ratio: f32, contact: bool) -> viz::WheelDebug {
        viz::WheelDebug {
            slip_ratio,
            slip_angle: 0.0,
            contact,
        }
    }

    #[test]
    fn a_wheel_is_tinted_by_what_it_is_doing() {
        assert_eq!(tint(&diagnostic(0.0, true)), ROLLING);
        assert_eq!(tint(&diagnostic(-1.0, true)), LOCKED);
        assert_eq!(tint(&diagnostic(1.0, true)), SPINNING);
        // Airborne wins: a wheel off the ground has no meaningful slip.
        assert_eq!(tint(&diagnostic(-1.0, false)), AIRBORNE);
    }
}
