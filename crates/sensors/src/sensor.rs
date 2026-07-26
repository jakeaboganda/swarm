use glam::Vec3;
use protocol::messages::SensorKind;

use crate::context::{Obstacle, SensorContext};

/// A named, pluggable sensor reading. Ground-truth implementations
/// (below) are all v1 ships — the trait exists so sensor-simulation
/// (noise, limited range/field-of-view, latency) can slot in later as new
/// implementations without changing the reflex-evaluation logic.
pub trait Sensor {
    fn read(&self, ctx: &SensorContext) -> f32;
}

pub struct TimeToCollision;

impl Sensor for TimeToCollision {
    fn read(&self, ctx: &SensorContext) -> f32 {
        ctx.obstacles
            .iter()
            .map(|obstacle| {
                closing_time(
                    ctx.self_position,
                    ctx.self_velocity,
                    ctx.self_radius,
                    obstacle,
                )
            })
            .fold(f32::INFINITY, f32::min)
    }
}

pub struct DistanceTo {
    pub target: Vec3,
}

impl Sensor for DistanceTo {
    fn read(&self, ctx: &SensorContext) -> f32 {
        ctx.self_position.distance(self.target)
    }
}

pub struct Speed;

impl Sensor for Speed {
    fn read(&self, ctx: &SensorContext) -> f32 {
        ctx.self_velocity.length()
    }
}

/// Builds the sensor implementation named by a wire-format `SensorKind`.
pub fn sensor_for(kind: &SensorKind) -> Box<dyn Sensor> {
    match kind {
        SensorKind::TimeToCollision => Box::new(TimeToCollision),
        SensorKind::DistanceTo { target } => Box::new(DistanceTo {
            target: Vec3::new(target.x, target.y, target.z),
        }),
        SensorKind::Speed => Box::new(Speed),
    }
}

/// Nearest-by-closing-time (not nearest-by-distance — these diverge: a far,
/// fast-closing obstacle can have a lower time-to-collision than a near,
/// slow one). Treats both parties as spheres and solves for the smallest
/// non-negative `t` at which their surfaces touch (center distance equals
/// the sum of radii), via linear extrapolation of current relative
/// velocity. Returns `f32::INFINITY` if they never come within contact on
/// their current trajectories.
fn closing_time(
    self_position: Vec3,
    self_velocity: Vec3,
    self_radius: f32,
    obstacle: &Obstacle,
) -> f32 {
    let relative_position = obstacle.position - self_position;
    let relative_velocity = obstacle.velocity - self_velocity;

    let contact_distance = self_radius + obstacle.radius;
    let c = relative_position.length_squared() - contact_distance * contact_distance;
    if c <= 0.0 {
        return 0.0;
    }

    let a = relative_velocity.length_squared();
    if a <= f32::EPSILON {
        return f32::INFINITY;
    }

    let b = 2.0 * relative_position.dot(relative_velocity);
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return f32::INFINITY;
    }

    let sqrt_d = discriminant.sqrt();
    let t1 = (-b - sqrt_d) / (2.0 * a);
    let t2 = (-b + sqrt_d) / (2.0 * a);
    if t1 >= 0.0 {
        t1
    } else if t2 >= 0.0 {
        t2
    } else {
        f32::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(self_position: Vec3, self_velocity: Vec3, obstacles: Vec<Obstacle>) -> SensorContext {
        SensorContext {
            self_position,
            self_velocity,
            self_radius: 0.0,
            obstacles,
        }
    }

    #[test]
    fn head_on_approach_has_finite_decreasing_ttc() {
        let obstacle = Obstacle {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 1.0,
        };
        let far = TimeToCollision.read(&ctx(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), vec![obstacle]));

        let obstacle = Obstacle {
            position: Vec3::new(5.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 1.0,
        };
        let near = TimeToCollision.read(&ctx(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), vec![obstacle]));

        assert!(near < far);
        assert!(near.is_finite());
    }

    #[test]
    fn moving_apart_is_infinite() {
        let obstacle = Obstacle {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 1.0,
        };
        let ttc = TimeToCollision.read(&ctx(Vec3::ZERO, Vec3::new(-1.0, 0.0, 0.0), vec![obstacle]));
        assert_eq!(ttc, f32::INFINITY);
    }

    #[test]
    fn already_overlapping_is_zero() {
        let obstacle = Obstacle {
            position: Vec3::new(0.5, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 1.0,
        };
        let ttc = TimeToCollision.read(&ctx(Vec3::ZERO, Vec3::ZERO, vec![obstacle]));
        assert_eq!(ttc, 0.0);
    }

    #[test]
    fn static_wall_gives_distance_over_speed() {
        // Wall approximated as a zero-radius point 8 units away, closed at 2 units/s.
        let obstacle = Obstacle {
            position: Vec3::new(8.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 0.0,
        };
        let ttc = TimeToCollision.read(&ctx(Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0), vec![obstacle]));
        assert!((ttc - 4.0).abs() < 1e-4);
    }

    #[test]
    fn nearest_by_closing_time_not_by_distance() {
        // Near obstacle, but stationary relative to self (never closes).
        let near_but_static = Obstacle {
            position: Vec3::new(2.0, 0.0, 0.0),
            velocity: Vec3::new(1.0, 0.0, 0.0),
            radius: 0.5,
        };
        // Far obstacle, closing fast.
        let far_but_closing = Obstacle {
            position: Vec3::new(20.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 0.5,
        };
        let ttc = TimeToCollision.read(&ctx(
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            vec![near_but_static, far_but_closing],
        ));
        // The near-but-matching-velocity obstacle never closes (infinite);
        // the far one does, in finite time.
        assert!(ttc.is_finite());
    }

    #[test]
    fn self_radius_shortens_time_to_collision() {
        // Point obstacle 8 units away, closing at 2 units/s. With a
        // point self (radius 0) contact is at t=4; with self radius 2 the
        // surfaces meet 2 units sooner, at t=3.
        let obstacle = Obstacle {
            position: Vec3::new(8.0, 0.0, 0.0),
            velocity: Vec3::ZERO,
            radius: 0.0,
        };
        let with_radius = TimeToCollision.read(&SensorContext {
            self_position: Vec3::ZERO,
            self_velocity: Vec3::new(2.0, 0.0, 0.0),
            self_radius: 2.0,
            obstacles: vec![obstacle],
        });
        assert!((with_radius - 3.0).abs() < 1e-4);
    }

    #[test]
    fn distance_to_reads_euclidean_distance() {
        let target = Vec3::new(3.0, 0.0, 4.0);
        let reading = DistanceTo { target }.read(&ctx(Vec3::ZERO, Vec3::ZERO, vec![]));
        assert!((reading - 5.0).abs() < 1e-4);
    }

    #[test]
    fn speed_reads_velocity_magnitude() {
        let reading = Speed.read(&ctx(Vec3::ZERO, Vec3::new(3.0, 0.0, 4.0), vec![]));
        assert!((reading - 5.0).abs() < 1e-4);
    }
}
