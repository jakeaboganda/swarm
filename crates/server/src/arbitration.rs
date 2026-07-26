use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use movement::DesiredVelocity;
use protocol::messages::ReflexAction;
use sensors::{evaluate, Obstacle, SensorContext};

use crate::agent::{Plan, Reflexes};
use crate::scenario::ArenaBounds;
use crate::scenario_state::Tick;
use crate::world::AGENT_RADIUS;

const ARRIVAL_TOLERANCE: f32 = 0.5;

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
            let to_target = target - transform.translation;
            if to_target.length() < ARRIVAL_TOLERANCE {
                plan.waypoints.pop_front();
            } else {
                *desired = DesiredVelocity {
                    value: to_target.normalize() * waypoint.speed,
                    urgent: false,
                };
                continue;
            }
        }

        *desired = DesiredVelocity {
            value: Vec3::ZERO,
            urgent: false,
        };
    }
}
