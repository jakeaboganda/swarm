use glam::Vec3;

/// The driver-actuator commands the FMU plant consumes. `steer` is a steering
/// angle in radians, positive = counter-clockwise about +Y (a left turn in a
/// right-handed, Y-up frame); a real FMU with the opposite convention gets the
/// sign flipped in its binding later. `throttle`/`brake` are `0.0..=1.0` and
/// never both nonzero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controls {
    pub steer: f32,
    pub throttle: f32,
    pub brake: f32,
}

/// The universal control target for one tick, translated from the server's
/// `DesiredVelocity` seam into Bevy-free terms.
#[derive(Debug, Clone, Copy)]
pub struct DriverInput {
    /// Desired world velocity: its horizontal magnitude is the target speed and
    /// its horizontal direction is where to aim.
    pub desired_velocity: Vec3,
    /// Pure-pursuit lookahead to the aim point (m). Zero means no steering
    /// target this tick.
    pub lookahead: f32,
    /// Current body heading, a horizontal unit vector.
    pub heading: Vec3,
    /// Current forward speed (m/s).
    pub speed: f32,
    /// A reflex (brake/stop) is driving the target -- command a full stop.
    pub urgent: bool,
}

/// Tunables for the driver. Defaults suit a ~2.5 m-wheelbase car.
#[derive(Debug, Clone, Copy)]
pub struct DriverConfig {
    /// Wheelbase (m), the pure-pursuit geometry length.
    pub wheelbase: f32,
    /// Steering angle clamp (rad).
    pub max_steer: f32,
    /// Longitudinal proportional gain.
    pub speed_kp: f32,
    /// Longitudinal integral gain.
    pub speed_ki: f32,
    /// Anti-windup clamp on the speed integrator.
    pub integral_limit: f32,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self {
            wheelbase: 2.5,
            max_steer: 0.6,
            speed_kp: 0.5,
            speed_ki: 0.1,
            integral_limit: 5.0,
        }
    }
}

/// Converts the universal `DesiredVelocity` target into plant pedals + steer.
/// Carries the longitudinal PI integrator, so it is stepped `&mut` per tick.
#[derive(Debug, Clone, Default)]
pub struct Driver {
    config: DriverConfig,
    speed_integral: f32,
    /// Last steer angle we commanded, held through a reflex stop (see `control`).
    last_steer: f32,
}

impl Driver {
    pub fn new(config: DriverConfig) -> Self {
        Self {
            config,
            speed_integral: 0.0,
            last_steer: 0.0,
        }
    }

    /// One tick: target velocity + body state -> controls.
    pub fn control(&mut self, input: DriverInput, dt: f32) -> Controls {
        if input.urgent {
            // Reflex stop: full brake and bleed the integrator so we do not
            // lunge forward when the reflex later clears. The wheel is *held*
            // at its last commanded angle, not centred: centring is a steering
            // input nobody asked for, and the brake still needs metres to stop,
            // so snapping straight mid-corner would leave the bend tangentially
            // (same reasoning as `RaycastVehicle`).
            self.speed_integral = 0.0;
            return Controls {
                steer: self.last_steer,
                throttle: 0.0,
                brake: 1.0,
            };
        }
        let steer = self.steer(&input);
        self.last_steer = steer;
        let (throttle, brake) = self.longitudinal(&input, dt);
        Controls {
            steer,
            throttle,
            brake,
        }
    }

    fn steer(&self, input: &DriverInput) -> f32 {
        let desired = horizontal(input.desired_velocity);
        let heading = horizontal(input.heading);
        if input.lookahead <= 0.0
            || desired.length_squared() < 1e-6
            || heading.length_squared() < 1e-6
        {
            return 0.0;
        }
        let desired = desired.normalize();
        let heading = heading.normalize();
        // Signed angle heading -> desired about +Y (right-hand rule).
        let alpha = heading.cross(desired).y.atan2(heading.dot(desired));
        // Pure pursuit: delta = atan2(2 * wheelbase * sin(alpha), lookahead).
        let steer = (2.0 * self.config.wheelbase * alpha.sin()).atan2(input.lookahead);
        steer.clamp(-self.config.max_steer, self.config.max_steer)
    }

    fn longitudinal(&mut self, input: &DriverInput, dt: f32) -> (f32, f32) {
        let target = horizontal(input.desired_velocity).length();
        let error = target - input.speed;
        self.speed_integral = (self.speed_integral + error * dt)
            .clamp(-self.config.integral_limit, self.config.integral_limit);
        let u = self.config.speed_kp * error + self.config.speed_ki * self.speed_integral;
        if u >= 0.0 {
            (u.min(1.0), 0.0)
        } else {
            (0.0, (-u).min(1.0))
        }
    }
}

fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 64.0;

    /// Facing -Z, aiming straight forward at `target_speed`, currently at
    /// `current`.
    fn straight(target_speed: f32, current: f32) -> DriverInput {
        DriverInput {
            desired_velocity: Vec3::new(0.0, 0.0, -target_speed),
            lookahead: 5.0,
            heading: Vec3::new(0.0, 0.0, -1.0),
            speed: current,
            urgent: false,
        }
    }

    #[test]
    fn straight_ahead_needs_no_steer() {
        let mut d = Driver::default();
        let c = d.control(straight(5.0, 5.0), DT);
        assert!(c.steer.abs() < 1e-5, "steer={}", c.steer);
    }

    #[test]
    fn under_target_speed_opens_throttle_not_brake() {
        let mut d = Driver::default();
        let c = d.control(straight(8.0, 2.0), DT);
        assert!(c.throttle > 0.0);
        assert_eq!(c.brake, 0.0);
    }

    #[test]
    fn over_target_speed_brakes_not_throttles() {
        let mut d = Driver::default();
        let c = d.control(straight(2.0, 8.0), DT);
        assert_eq!(c.throttle, 0.0);
        assert!(c.brake > 0.0);
    }

    #[test]
    fn urgent_full_brake_holds_the_wheel_rather_than_centring_it() {
        let mut d = Driver::default();
        // A non-urgent turning tick commands a nonzero steer...
        let turning = DriverInput {
            desired_velocity: Vec3::new(-1.0, 0.0, -1.0),
            lookahead: 5.0,
            heading: Vec3::new(0.0, 0.0, -1.0),
            speed: 4.0,
            urgent: false,
        };
        let turned = d.control(turning, DT);
        assert!(turned.steer > 0.0, "precondition: a turn is under way");
        // ...and an immediately following reflex stop holds that angle, not 0.
        let mut urgent = turning;
        urgent.urgent = true;
        let c = d.control(urgent, DT);
        assert_eq!(c.brake, 1.0);
        assert_eq!(c.throttle, 0.0);
        assert_eq!(
            c.steer, turned.steer,
            "urgent must hold the last wheel angle, not centre it"
        );
    }

    #[test]
    fn aiming_counter_clockwise_of_heading_steers_positive() {
        // Heading -Z; aim toward forward-and--X, which is CCW about +Y.
        let mut d = Driver::default();
        let input = DriverInput {
            desired_velocity: Vec3::new(-1.0, 0.0, -1.0),
            lookahead: 5.0,
            heading: Vec3::new(0.0, 0.0, -1.0),
            speed: 4.0,
            urgent: false,
        };
        let c = d.control(input, DT);
        assert!(c.steer > 0.0, "steer={}", c.steer);
    }

    /// A tight-lookahead aim in `dir` (world), heading -Z, that would demand a
    /// steer past the clamp.
    fn hard_turn(dir: Vec3) -> (Driver, DriverInput) {
        let d = Driver::new(DriverConfig {
            max_steer: 0.4,
            ..Default::default()
        });
        let input = DriverInput {
            desired_velocity: dir,
            lookahead: 0.5,
            heading: Vec3::new(0.0, 0.0, -1.0),
            speed: 1.0,
            urgent: false,
        };
        (d, input)
    }

    #[test]
    fn steer_clamps_on_the_positive_side() {
        // Aim forward-and--X (CCW of -Z heading) -> positive steer, saturated.
        let (mut d, input) = hard_turn(Vec3::new(-1.0, 0.0, -0.05));
        let c = d.control(input, DT);
        assert!((c.steer - 0.4).abs() < 1e-6, "steer={}", c.steer);
    }

    #[test]
    fn steer_clamps_on_the_negative_side() {
        // Aim forward-and-+X (CW of -Z heading) -> negative steer, saturated.
        let (mut d, input) = hard_turn(Vec3::new(1.0, 0.0, -0.05));
        let c = d.control(input, DT);
        assert!((c.steer + 0.4).abs() < 1e-6, "steer={}", c.steer);
    }

    #[test]
    fn speed_integrator_anti_windup_holds() {
        // Isolate the integral term (kp=0, ki=1) with a small integral_limit, so
        // the steady throttle equals ki*integral_limit and cannot wind past it.
        // Without the clamp, a persistent positive error drives throttle to 1.0.
        let mut d = Driver::new(DriverConfig {
            speed_kp: 0.0,
            speed_ki: 1.0,
            integral_limit: 0.5,
            ..Default::default()
        });
        let input = straight(10.0, 0.0); // large, persistent positive error
        let mut c = Controls {
            steer: 0.0,
            throttle: 0.0,
            brake: 0.0,
        };
        for _ in 0..2000 {
            c = d.control(input, DT);
        }
        assert!(
            (c.throttle - 0.5).abs() < 1e-6,
            "throttle wound past the clamp: {}",
            c.throttle
        );
    }
}
