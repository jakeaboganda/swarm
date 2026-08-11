use glam::Vec3;
use protocol::scenario::SensorSpec;

/// Deterministic, seedable PRNG (splitmix64) with a Box–Muller Gaussian.
/// Hand-rolled to keep the dependency set unchanged; the server seeds one per
/// `(scenario_seed, agent, tick)` so simulated perception is reproducible.
pub struct Rng(u64);

impl Rng {
    pub fn seed(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)` from the top 24 bits.
    fn next_f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// A sample from `N(0, sigma)`. Returns exactly 0 (consuming no
    /// randomness) when `sigma == 0`, so zero-noise perception is the identity
    /// and doesn't perturb the deterministic stream.
    pub fn gaussian(&mut self, sigma: f32) -> f32 {
        if sigma == 0.0 {
            return 0.0;
        }
        let u1 = self.next_f32().max(f32::MIN_POSITIVE);
        let u2 = self.next_f32();
        (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos() * sigma
    }
}

/// What an entity is, so an agent can tell a peer from a wall.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionKind {
    Agent,
    Static,
}

/// Ground-truth input to `perceive` — one entity as the sim actually knows it.
#[derive(Debug, Clone)]
pub struct PerceivedEntity {
    pub id: String,
    pub kind: DetectionKind,
    pub position: Vec3,
    pub velocity: Vec3,
}

/// One entity as an agent perceives it: identity + kind carried through, pose
/// degraded by noise. `distance` is the true (pre-noise) centre distance used
/// for range gating.
#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    pub id: String,
    pub kind: DetectionKind,
    pub position: Vec3,
    pub velocity: Vec3,
    pub distance: f32,
}

/// The impairment pipeline: cull `others` by range then field of view, then
/// perturb the survivors' pose with Gaussian noise. Pure — the caller owns
/// the RNG (and thus the seed) and gathers the ground-truth `others`.
pub fn perceive(
    spec: &SensorSpec,
    self_position: Vec3,
    self_heading: Vec3,
    others: &[PerceivedEntity],
    rng: &mut Rng,
) -> Vec<Detection> {
    let heading = flatten_xz(self_heading);
    let mut detections = Vec::new();
    for entity in others {
        let offset = entity.position - self_position;
        let distance = offset.length();
        if distance > spec.range {
            continue;
        }
        // Field of view: bearing from heading in the ground plane. Skipped for
        // a full-circle spec, or when the agent has no heading (no cone to
        // define) — a still agent senses all around.
        if spec.fov_half_angle < std::f32::consts::PI {
            if let (Some(h), Some(d)) = (heading, flatten_xz(offset)) {
                if h.dot(d).clamp(-1.0, 1.0).acos() > spec.fov_half_angle {
                    continue;
                }
            }
        }
        detections.push(Detection {
            id: entity.id.clone(),
            kind: entity.kind,
            position: entity.position + noise_vec(rng, spec.position_noise),
            velocity: entity.velocity + noise_vec(rng, spec.velocity_noise),
            distance,
        });
    }
    detections
}

/// Unit xz-plane direction, or `None` if the vector has no horizontal extent.
fn flatten_xz(v: Vec3) -> Option<Vec3> {
    let flat = Vec3::new(v.x, 0.0, v.z);
    (flat.length_squared() > 1e-12).then(|| flat.normalize())
}

fn noise_vec(rng: &mut Rng, sigma: f32) -> Vec3 {
    Vec3::new(
        rng.gaussian(sigma),
        rng.gaussian(sigma),
        rng.gaussian(sigma),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const FULL: f32 = std::f32::consts::PI;

    fn ent(id: &str, position: Vec3, velocity: Vec3) -> PerceivedEntity {
        PerceivedEntity {
            id: id.into(),
            kind: DetectionKind::Agent,
            position,
            velocity,
        }
    }

    fn spec(range: f32, fov: f32, position_noise: f32) -> SensorSpec {
        SensorSpec {
            range,
            fov_half_angle: fov,
            position_noise,
            velocity_noise: 0.0,
            latency_ticks: 0,
        }
    }

    #[test]
    fn out_of_range_is_culled() {
        let others = vec![
            ent("near", Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO),
            ent("far", Vec3::new(50.0, 0.0, 0.0), Vec3::ZERO),
        ];
        let d = perceive(
            &spec(10.0, FULL, 0.0),
            Vec3::ZERO,
            Vec3::X,
            &others,
            &mut Rng::seed(1),
        );
        assert_eq!(
            d.iter().map(|x| x.id.as_str()).collect::<Vec<_>>(),
            ["near"]
        );
    }

    #[test]
    fn behind_fov_culled_in_front_kept() {
        let others = vec![
            ent("front", Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO),
            ent("back", Vec3::new(-5.0, 0.0, 0.0), Vec3::ZERO),
        ];
        // heading +X, 90° half-angle cone.
        let d = perceive(
            &spec(100.0, std::f32::consts::FRAC_PI_2, 0.0),
            Vec3::ZERO,
            Vec3::X,
            &others,
            &mut Rng::seed(1),
        );
        assert_eq!(
            d.iter().map(|x| x.id.as_str()).collect::<Vec<_>>(),
            ["front"]
        );
    }

    #[test]
    fn full_fov_keeps_all_around() {
        let others = vec![
            ent("front", Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO),
            ent("back", Vec3::new(-5.0, 0.0, 0.0), Vec3::ZERO),
        ];
        let d = perceive(
            &spec(100.0, FULL, 0.0),
            Vec3::ZERO,
            Vec3::X,
            &others,
            &mut Rng::seed(1),
        );
        assert_eq!(d.len(), 2);
    }

    #[test]
    fn no_heading_skips_fov() {
        let others = vec![ent("back", Vec3::new(-5.0, 0.0, 0.0), Vec3::ZERO)];
        let d = perceive(
            &spec(100.0, 0.1, 0.0),
            Vec3::ZERO,
            Vec3::ZERO,
            &others,
            &mut Rng::seed(1),
        );
        assert_eq!(d.len(), 1);
    }

    #[test]
    fn zero_noise_is_identity() {
        let others = vec![ent("a", Vec3::new(3.0, 0.0, 4.0), Vec3::new(1.0, 0.0, 2.0))];
        let d = perceive(
            &spec(100.0, FULL, 0.0),
            Vec3::ZERO,
            Vec3::X,
            &others,
            &mut Rng::seed(42),
        );
        assert_eq!(d[0].position, Vec3::new(3.0, 0.0, 4.0));
        assert_eq!(d[0].velocity, Vec3::new(1.0, 0.0, 2.0));
        assert!((d[0].distance - 5.0).abs() < 1e-4);
    }

    #[test]
    fn same_seed_is_deterministic_and_actually_noisy() {
        let others = vec![ent("a", Vec3::new(3.0, 0.0, 4.0), Vec3::ZERO)];
        let s = spec(100.0, FULL, 0.5);
        let d1 = perceive(&s, Vec3::ZERO, Vec3::X, &others, &mut Rng::seed(7));
        let d2 = perceive(&s, Vec3::ZERO, Vec3::X, &others, &mut Rng::seed(7));
        assert_eq!(d1, d2);
        assert_ne!(d1[0].position, Vec3::new(3.0, 0.0, 4.0));
    }

    #[test]
    fn noise_is_roughly_unbiased() {
        let others = vec![ent("a", Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO)];
        let s = spec(100.0, FULL, 1.0);
        let mut rng = Rng::seed(123);
        let n = 2000;
        let mut sum = Vec3::ZERO;
        for _ in 0..n {
            sum += perceive(&s, Vec3::ZERO, Vec3::X, &others, &mut rng)[0].position;
        }
        let mean = sum / n as f32;
        assert!(
            (mean.x - 10.0).abs() < 0.2,
            "mean x {} should be ~10",
            mean.x
        );
    }
}
