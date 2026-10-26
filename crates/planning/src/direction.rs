//! Conservative empirical reserves over supplied world-velocity error discs.
use rustdriving_core::{ForecastEnvelopeConfig, Track, Vec2};
use std::collections::BTreeMap;

#[derive(Clone, Copy)]
pub(super) struct Envelope {
    velocity: Vec2,
    error: f64,
    acceleration: f64,
    age: f64,
}
impl Envelope {
    fn invalid() -> Self {
        Self {
            velocity: Vec2::default(),
            error: f64::INFINITY,
            acceleration: 0.,
            age: 0.,
        }
    }
    pub fn reserve(self, time: f64, tangent: Vec2) -> f64 {
        let error = self.error + self.acceleration * (self.age + time);
        let speed = self.velocity.x.hypot(self.velocity.y);
        let length = tangent.x.hypot(tangent.y);
        if !time.is_finite()
            || time < 0.
            || !error.is_finite()
            || error < 0.
            || !speed.is_finite()
            || !length.is_finite()
        {
            return f64::INFINITY;
        }
        let (fraction, transverse) = if speed == 0. && error == 0. {
            (0., 0.)
        } else if length <= 1e-6 {
            (1., speed + error)
        } else {
            let along = (self.velocity.x * (tangent.x / length)
                + self.velocity.y * (tangent.y / length))
                .abs();
            let across = (self.velocity.x * (tangent.y / length)
                - self.velocity.y * (tangent.x / length))
                .abs();
            let fraction = if along <= error || speed <= error {
                1.
            } else {
                let ratio = error / speed;
                ((across / speed) * (1. - ratio * ratio).max(0.).sqrt() + (along / speed) * ratio)
                    .min(1.)
            };
            (fraction, across + error)
        };
        (0.4 * transverse.min(1.)).max(0.7 * fraction)
    }
}

#[derive(Clone)]
pub(super) struct Context {
    valid: bool,
    entries: BTreeMap<u64, Envelope>,
}
impl Context {
    pub fn observe(bounds: ForecastEnvelopeConfig, tracks: &[Track], now: f64) -> Self {
        let mut result = Self {
            valid: bounds.validate().is_ok() && now.is_finite() && now >= 0.,
            entries: BTreeMap::new(),
        };
        for track in tracks {
            let age = now - track.last_seen;
            let valid = track.velocity.finite()
                && track.velocity.x.hypot(track.velocity.y).is_finite()
                && age.is_finite()
                && age >= 0.;
            result.valid &= valid;
            if result
                .entries
                .insert(
                    track.id,
                    Envelope {
                        velocity: track.velocity,
                        error: bounds.velocity_error_m_s,
                        acceleration: bounds.acceleration_error_m_s2,
                        age,
                    },
                )
                .is_some()
            {
                result.valid = false;
            }
        }
        result
    }
    pub fn for_prediction(&self, id: u64) -> Envelope {
        if self.valid {
            self.entries
                .get(&id)
                .copied()
                .unwrap_or_else(Envelope::invalid)
        } else {
            Envelope::invalid()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn envelope(velocity: Vec2, error: f64) -> Envelope {
        Envelope {
            velocity,
            error,
            acceleration: 0.,
            age: 0.,
        }
    }
    fn empirical(velocity: Vec2, tangent: Vec2) -> f64 {
        let speed = velocity.x.hypot(velocity.y);
        let length = tangent.x.hypot(tangent.y);
        if speed == 0. {
            return 0.;
        }
        let across = (velocity.x * tangent.y - velocity.y * tangent.x).abs() / length;
        (0.4 * across.min(1.)).max(0.7 * across / speed)
    }
    #[test]
    fn unresolved_small_velocities_retain_the_same_crossing_reserve() {
        for i in 0..128 {
            let angle = i as f64 * std::f64::consts::TAU / 128.;
            let model = envelope(Vec2::new(angle.cos() * 0.01, angle.sin() * 0.01), 0.3);
            assert_eq!(model.reserve(0., Vec2::new(1., 0.)), 0.7);
        }
        assert_eq!(
            envelope(Vec2::default(), 0.3).reserve(0., Vec2::new(1., 0.)),
            0.7
        );
    }
    #[test]
    fn independent_circle_samples_cannot_exceed_the_analytic_supremum() {
        for heading in [-2.3_f64, -0.4, 0.8, 2.6] {
            let tangent = Vec2::new(heading.cos(), heading.sin());
            for velocity in [
                Vec2::default(),
                Vec2::new(0.01, -0.02),
                Vec2::new(2., 0.1),
                Vec2::new(-4., 0.7),
            ] {
                for radius in [0., 0.05, 0.3, 1.] {
                    let model = envelope(velocity, radius);
                    let upper = model.reserve(0., tangent);
                    let mut sampled = empirical(velocity, tangent);
                    for i in 0..4096 {
                        let angle = i as f64 * std::f64::consts::TAU / 4096.;
                        let candidate =
                            velocity.plus(Vec2::new(angle.cos(), angle.sin()).scaled(radius));
                        sampled = sampled.max(empirical(candidate, tangent));
                    }
                    assert!(upper + 1e-12 >= sampled, "{upper} < {sampled}");
                    assert!(
                        upper - sampled < 0.002,
                        "{upper} is not attained near {sampled}"
                    );
                }
            }
        }
    }
    #[test]
    fn zero_error_keeps_the_observed_reserve_and_joint_rotation_is_invariant() {
        let velocity = Vec2::new(3., 0.7);
        let tangent = Vec2::new(0.8, 0.6);
        assert!(
            (envelope(velocity, 0.).reserve(0., tangent) - empirical(velocity, tangent)).abs()
                < 1e-12
        );
        let original = envelope(velocity, 0.3).reserve(0., tangent);
        for yaw in [-2.5, -0.2, 1.1, 2.8] {
            assert!(
                (envelope(velocity.rotated(yaw), 0.3).reserve(0., tangent.rotated(yaw)) - original)
                    .abs()
                    < 1e-12
            );
        }
    }
    #[test]
    fn acquisition_age_and_future_time_consume_the_same_error_budget() {
        let mut model = envelope(Vec2::new(4., 0.), 0.1);
        model.acceleration = 0.2;
        model.age = 0.5;
        let heading = Vec2::new(1., 0.);
        assert!((model.reserve(0., heading) - 0.4 * 0.2).abs() < 1e-12);
        assert!((model.reserve(1., heading) - 0.4 * 0.4).abs() < 1e-12);
        let later = Envelope { age: 1.5, ..model };
        assert!((later.reserve(0., heading) - model.reserve(1., heading)).abs() < 1e-12);
    }
    #[test]
    fn missing_duplicate_future_or_invalid_context_never_disables_the_guard() {
        let bounds = ForecastEnvelopeConfig {
            position_error_m: 0.2,
            velocity_error_m_s: 0.3,
            acceleration_error_m_s2: 0.1,
        };
        let track = Track {
            id: 9,
            position: Vec2::default(),
            velocity: Vec2::new(1., 0.),
            radius: 1.,
            last_seen: 1.,
        };
        let heading = Vec2::new(1., 0.);
        assert!(
            Context::observe(bounds, &[], 1.)
                .for_prediction(9)
                .reserve(0., heading)
                .is_infinite()
        );
        assert!(
            Context::observe(bounds, &[track.clone(), track.clone()], 1.)
                .for_prediction(9)
                .reserve(0., heading)
                .is_infinite()
        );
        assert!(
            Context::observe(bounds, std::slice::from_ref(&track), 0.9)
                .for_prediction(9)
                .reserve(0., heading)
                .is_infinite()
        );
        assert!(
            Context::observe(
                ForecastEnvelopeConfig {
                    velocity_error_m_s: f64::NAN,
                    ..bounds
                },
                &[track],
                1.
            )
            .for_prediction(9)
            .reserve(0., heading)
            .is_infinite()
        );
    }
}
