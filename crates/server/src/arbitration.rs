use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::prelude::Velocity;
use movement::DesiredVelocity;
use protocol::messages::ReflexAction;
use protocol::scenario::{SensorSource, GROUND_TRUTH_SENSOR};
use sensors::{evaluate, Obstacle, SensorContext};

use crate::agent::{AgentName, Plan, Reflexes};
use crate::events::ReflexFired;
use crate::perception_router::{PerceivedWorlds, Perceiver};
use crate::scenario::ArenaBounds;
use crate::scenario_state::Tick;
use crate::tracker::{PlanPath, Tracking};
use crate::world::Radius;

/// Nearest point on each of the four walls to `position`, treated as
/// static (zero-velocity) obstacles for `time_to_collision`. Clamping to
/// each wall's span gives the true nearest point on that wall segment for
/// an axis-aligned arena, not just an approximation.
pub(crate) fn wall_obstacles(position: Vec3, bounds: &ArenaBounds) -> [Obstacle; 4] {
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
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub fn arbitrate(
    mut tick: ResMut<Tick>,
    bounds: Res<ArenaBounds>,
    worlds: Res<PerceivedWorlds>,
    mut reflex_fired: MessageWriter<ReflexFired>,
    mut query: Query<(
        Entity,
        &AgentName,
        &Transform,
        &Velocity,
        &Radius,
        &Perceiver,
        &mut Plan,
        &mut Reflexes,
        &mut DesiredVelocity,
    )>,
) {
    tick.0 += 1;

    let others: Vec<(Entity, Obstacle)> = query
        .iter()
        .map(|(entity, _, transform, velocity, radius, ..)| {
            (
                entity,
                Obstacle {
                    position: transform.translation,
                    velocity: velocity.linear,
                    radius: radius.0,
                },
            )
        })
        .collect();

    for (
        entity,
        name,
        transform,
        velocity,
        radius,
        perceiver,
        mut plan,
        mut reflexes,
        mut desired,
    ) in &mut query
    {
        // Ground-truth context: every other agent, exact, plus the walls. This
        // is the reserved `ground_truth` device — a perfect, instant fail-safe.
        let mut gt_obstacles: Vec<Obstacle> = others
            .iter()
            .filter(|(other, _)| *other != entity)
            .map(|(_, obstacle)| Obstacle {
                position: obstacle.position,
                velocity: obstacle.velocity,
                radius: obstacle.radius,
            })
            .collect();
        gt_obstacles.extend(wall_obstacles(transform.translation, &bounds));

        let context = |obstacles| SensorContext {
            self_position: transform.translation,
            self_velocity: velocity.linear,
            self_radius: radius.0,
            obstacles,
        };

        let mut contexts: HashMap<String, SensorContext> = HashMap::new();
        contexts.insert(GROUND_TRUTH_SENSOR.to_string(), context(gt_obstacles));

        // Each simulated device: its delivered (delayed, noised) detections as
        // obstacles, plus walls (walls are perceived as static ground truth).
        for def in &perceiver.0 {
            if def.source != SensorSource::Simulated {
                continue;
            }
            let mut obstacles: Vec<Obstacle> = worlds
                .delivered(&name.0, &def.name)
                .iter()
                .map(|d| Obstacle {
                    position: d.position,
                    velocity: d.velocity,
                    radius: d.radius,
                })
                .collect();
            obstacles.extend(wall_obstacles(transform.translation, &bounds));
            contexts.insert(def.name.clone(), context(obstacles));
        }

        if let Some(action) = evaluate(&mut reflexes.0, &contexts) {
            *desired = DesiredVelocity {
                value: Vec3::ZERO,
                urgent: true,
                lookahead: 0.0,
            };
            reflex_fired.write(ReflexFired {
                entity,
                tick: tick.0,
                plan_version: plan.version,
                action,
            });
            if action == ReflexAction::StopAndHold {
                plan.waypoints.clear();
                plan.clear_progress();
            }
            continue;
        }

        // Path tracking. The plan is measured against, not consumed: progress
        // is re-projected every tick from where it was last tick, so the
        // waypoints stay exactly as the agent submitted them.
        if let Some(path) = PlanPath::new(&plan.waypoints) {
            let ground_speed = Vec3::new(velocity.linear.x, 0.0, velocity.linear.z).length();
            let tracked = path.track(
                transform.translation,
                ground_speed,
                plan.progress().map(|p| p.s),
            );
            match tracked.tracking {
                Tracking::Drive {
                    velocity,
                    lookahead,
                } => {
                    plan.set_progress(tracked.progress);
                    *desired = DesiredVelocity {
                        value: velocity,
                        urgent: false,
                        lookahead,
                    };
                    continue;
                }
                // The path has been driven. Dropping it here is what keeps
                // "the plan ran out" observable as an empty plan, for agents
                // and for the viz overlay alike.
                Tracking::Arrived => {
                    plan.waypoints.clear();
                    plan.clear_progress();
                }
                Tracking::Hold => plan.set_progress(tracked.progress),
            }
        }

        *desired = DesiredVelocity {
            value: Vec3::ZERO,
            urgent: false,
            lookahead: 0.0,
        };
    }
}
