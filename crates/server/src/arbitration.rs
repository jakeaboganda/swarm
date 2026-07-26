use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use movement::DesiredVelocity;
use protocol::messages::ReflexAction;
use sensors::{evaluate, Obstacle, SensorContext};

use crate::agent::{Plan, Reflexes};
use crate::events::ReflexFired;
use crate::scenario::ArenaBounds;
use crate::scenario_state::Tick;
use crate::world::AGENT_RADIUS;

/// A waypoint is reached once the entity is within this horizontal
/// distance of it.
const ARRIVAL_TOLERANCE: f32 = 0.5;
/// Inside this horizontal distance of a waypoint, commanded speed scales
/// down linearly toward zero ("arrive" behavior) so the entity settles on
/// the point instead of overshooting and orbiting it.
const ARRIVE_RADIUS: f32 = 2.0;

/// Horizontal (xz-plane) steering toward a waypoint. Motion is
/// ground-constrained, so both the arrival test and the steering direction
/// ignore the vertical axis — otherwise the capsule's resting height above
/// the ground (center well above y=0) would keep every ground-plane
/// waypoint permanently "unreached". Returns `None` once arrived (the
/// caller should advance to the next waypoint), else the desired velocity
/// with arrive-slowdown applied.
fn planar_seek(current: Vec3, target: Vec3, speed: f32) -> Option<Vec3> {
    let delta = Vec3::new(target.x - current.x, 0.0, target.z - current.z);
    let distance = delta.length();
    if distance < ARRIVAL_TOLERANCE {
        return None;
    }
    let scaled_speed = speed * (distance / ARRIVE_RADIUS).min(1.0);
    Some(delta / distance * scaled_speed)
}

/// Nearest point on each of the four walls to `position`, treated as
/// static (zero-velocity) obstacles for `time_to_collision`. Clamping to
/// each wall's span gives the true nearest point on that wall segment for
/// an axis-aligned arena, not just an approximation.
fn wall_obstacles(position: Vec3, bounds: &ArenaBounds) -> [Obstacle; 4] {
    let x = position.x.clamp(-bounds.half_width, bounds.half_width);
    let z = position.z.clamp(-bounds.half_depth, bounds.half_depth);
    [
        Obstacle {
            position: Vec3::new(x, position.y, bounds.half_depth),
            velocity: Vec3::ZERO,
            radius: 0.0,
        },
        Obstacle {
            position: Vec3::new(x, position.y, -bounds.half_depth),
            velocity: Vec3::ZERO,
            radius: 0.0,
        },
        Obstacle {
            position: Vec3::new(bounds.half_width, position.y, z),
            velocity: Vec3::ZERO,
            radius: 0.0,
        },
        Obstacle {
            position: Vec3::new(-bounds.half_width, position.y, z),
            velocity: Vec3::ZERO,
            radius: 0.0,
        },
    ]
}

/// The per-tick control loop's steps 1-3: read sensors, evaluate reflexes,
/// and resolve reflex-vs-plan arbitration into a `DesiredVelocity`. Step 4
/// (force application) is `movement`'s job, ordered to run after this.
pub fn arbitrate(
    mut tick: ResMut<Tick>,
    bounds: Res<ArenaBounds>,
    mut reflex_fired: MessageWriter<ReflexFired>,
    mut query: Query<(
        Entity,
        &Transform,
        &Velocity,
        &mut Plan,
        &mut Reflexes,
        &mut DesiredVelocity,
    )>,
) {
    tick.0 += 1;

    let others: Vec<(Entity, Obstacle)> = query
        .iter()
        .map(|(entity, transform, velocity, ..)| {
            (
                entity,
                Obstacle {
                    position: transform.translation,
                    velocity: velocity.linear,
                    radius: AGENT_RADIUS,
                },
            )
        })
        .collect();

    for (entity, transform, velocity, mut plan, mut reflexes, mut desired) in &mut query {
        let mut obstacles: Vec<Obstacle> = others
            .iter()
            .filter(|(other, _)| *other != entity)
            .map(|(_, obstacle)| Obstacle {
                position: obstacle.position,
                velocity: obstacle.velocity,
                radius: obstacle.radius,
            })
            .collect();
        obstacles.extend(wall_obstacles(transform.translation, &bounds));

        let ctx = SensorContext {
            self_position: transform.translation,
            self_velocity: velocity.linear,
            self_radius: AGENT_RADIUS,
            obstacles,
        };

        if let Some(action) = evaluate(&mut reflexes.0, &ctx) {
            *desired = DesiredVelocity {
                value: Vec3::ZERO,
                urgent: true,
            };
            reflex_fired.write(ReflexFired {
                entity,
                tick: tick.0,
                plan_version: plan.version,
                action,
            });
            if action == ReflexAction::StopAndHold {
                plan.waypoints.clear();
            }
            continue;
        }

        if let Some(waypoint) = plan.waypoints.front() {
            let target = Vec3::new(
                waypoint.position.x,
                waypoint.position.y,
                waypoint.position.z,
            );
            match planar_seek(transform.translation, target, waypoint.speed) {
                Some(value) => {
                    *desired = DesiredVelocity {
                        value,
                        urgent: false,
                    };
                    continue;
                }
                None => {
                    plan.waypoints.pop_front();
                }
            }
        }

        *desired = DesiredVelocity {
            value: Vec3::ZERO,
            urgent: false,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrived_when_horizontally_close_despite_vertical_offset() {
        // Capsule rests with its center ~1 unit above a y=0 ground-plane
        // waypoint; horizontally it is within tolerance, so it has arrived.
        let current = Vec3::new(0.2, 1.0, 0.1);
        let target = Vec3::new(0.0, 0.0, 0.0);
        assert_eq!(planar_seek(current, target, 5.0), None);
    }

    #[test]
    fn full_speed_far_from_target_in_the_horizontal_plane() {
        let desired = planar_seek(Vec3::new(0.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 5.0)
            .expect("not arrived");
        assert!((desired.x - 5.0).abs() < 1e-4);
        assert_eq!(desired.y, 0.0);
        assert_eq!(desired.z, 0.0);
    }

    #[test]
    fn speed_scales_down_inside_the_arrive_radius() {
        // 1 unit out with ARRIVE_RADIUS 2.0 -> half the commanded speed.
        let desired = planar_seek(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 5.0)
            .expect("not arrived");
        assert!((desired.length() - 2.5).abs() < 1e-4);
    }
}
